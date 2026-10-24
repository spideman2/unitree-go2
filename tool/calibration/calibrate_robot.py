#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0
"""Guide Go2 poseXYZ calibration, or process existing BRAKE captures offline."""

import argparse
import copy
import math
import os
import re
import statistics
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path

import yaml

JOINTS = [f"{leg}_{joint}" for leg in ("FR", "FL", "RR", "RL")
          for joint in ("hip", "thigh", "calf")]
RATIOS = [6.33, 6.33, 12.13] * 4
DIRECTIONS = [1, -1, 1, 1, 1, -1, -1, -1, 1, -1, 1, -1]
PHASES = {
    "poseX": [1, 2, 4, 5, 7, 8, 10, 11],
    "poseY": [3, 9],
    "poseZ": [0, 6],
}
REPLY = re.compile(r"\breply id=(\d+).*?\berror=(\d+).*?\bq=(\S+)")


def default_config(name):
    script = Path(__file__).resolve()
    candidates = [script.parents[2] / "config" / name,
                  script.parents[4] / "share/unitree_go2/config" / name]
    if name.startswith("calibration_pose_"):
        candidates.insert(0, script.with_name(name))
        candidates.insert(1, script.parents[4] / "share/unitree_go2/tool/calibration" / name)
    for source in candidates:
        if source.is_file():
            return source
    return candidates[0]


def load_config(path):
    """Resolve shared defaults and board wiring before applying local calibration."""
    local = yaml.safe_load(Path(path).read_text())
    shared = yaml.safe_load(default_config("robot.yaml").read_text())
    params = {**shared["defaults"], **local.get("defaults", {})}
    board = local.get("robot", {}).get("board_profile", "legacy_usb")
    profiles = {**shared["board_profiles"], **local.get("board_profiles", {})}
    if board not in profiles:
        raise ValueError("unknown board_profile: " + board)
    params.update(profiles[board]["go2"])
    driver = local.get("go2", {}).get("ros__parameters", {}).get("imu_driver", params["imu_driver"])
    params.update(profiles[board].get("imu_overrides", {}).get(driver, {}))
    params.update(local.get("go2", {}).get("ros__parameters", {}))
    params["board_profile"] = board
    for key in ("joint_zero_offsets_rad", "startup_reference_rotor_positions_rad"):
        if params[key] == []:
            params[key] = [0.0] * 12  # Uncalibrated template; all three phases must be captured.
        finite_vector(params[key], key)
    return {**local, "go2": {"ros__parameters": params}}


def finite_vector(value, name):
    if not isinstance(value, list) or len(value) != 12:
        raise ValueError(f"{name}: expected 12 values in FR/FL/RR/RL order")
    if any(isinstance(x, bool) or not isinstance(x, (float, int)) or
           not math.isfinite(x) for x in value):
        raise ValueError(f"{name}: values must be finite numbers")
    return value


def validate(config, reference):
    params = config["go2"]["ros__parameters"]
    for name, expected in (("motor_ids", [0, 1, 2] * 4),
                           ("gear_ratios", RATIOS),
                           ("joint_directions", DIRECTIONS)):
        if params[name] != expected:
            raise ValueError(f"{name}: does not match the fixed Go2 layout")
    if params.get("motor_driver") != "drv_485_m8010a":
        raise ValueError("calibration requires the Go2 direct UART driver")
    if params.get("board_profile", "legacy_usb") not in ("legacy_usb", "new_bigcore", "new_usbmotor"):
        raise ValueError("capture requires direct UART or USB motor devices")
    if "crc_seed" in params and params["crc_seed"] != 0x2CBB:
        raise ValueError("Go2 CRC seed must be 0x2CBB")
    if reference["joint_order"] != JOINTS:
        raise ValueError("reference joint_order must be FR/FL/RR/RL; hip/thigh/calf")
    finite_vector(params["startup_reference_joint_positions_rad"], "startup pose")
    finite_vector(reference["joint_positions_deg"], "XYZ reference joint positions")
    return params


def phase_rotor_means(directory, phase, samples):
    """Read and validate only the motors sampled for one pose."""
    if samples < 1:
        raise ValueError("samples must be positive")
    sdk = directory / "sdk" if (directory / "sdk").is_dir() else directory
    means = {}
    for index in PHASES[phase]:
        joint = JOINTS[index]
        path = sdk / f"{joint[:2]}_id{index % 3}.txt"
        text = path.read_text(encoding="ascii")
        replies = REPLY.findall(text)
        if len(replies) != samples or len(re.findall(r"\breply id=", text)) != samples:
            raise ValueError(f"{path}: expected exactly {samples} replies")
        values = []
        for motor_id, error, q in replies:
            if int(motor_id) != index % 3 or int(error) != 0:
                raise ValueError(f"{path}: wrong motor ID or motor error")
            value = float(q)
            if not math.isfinite(value):
                raise ValueError(f"{path}: non-finite rotor position")
            values.append(value)
        spread = math.degrees((max(values) - min(values)) / RATIOS[index])
        if spread > 0.1:
            raise ValueError(f"{path}: joint spread {spread:.4f} deg exceeds 0.1 deg")
        means[index] = statistics.fmean(values)
    return means


def phase_reference(reference, phase):
    values = finite_vector(reference["joint_positions_deg"], "joint positions")
    return {index: math.radians(values[index]) for index in PHASES[phase]}


def update_phase(config, phase, reference, rotor_means_by_index):
    """Update offsets and Pose A rotor anchors for this phase only."""
    result = copy.deepcopy(config)
    params = validate(result, reference)
    references = phase_reference(reference, phase)
    offsets = params["joint_zero_offsets_rad"]
    startup_joint = params["startup_reference_joint_positions_rad"]
    startup_rotor = params["startup_reference_rotor_positions_rad"]
    for index, rotor in rotor_means_by_index.items():
        offsets[index] = round(references[index] - DIRECTIONS[index] * rotor / RATIOS[index], 9)
        startup_rotor[index] = round(
            rotor + DIRECTIONS[index] * (startup_joint[index] - references[index]) * RATIOS[index], 9)
    return result


def capture_phase(directory, config, reference, tool, samples, phase):
    params = validate(config, reference)
    ports = params["serial_devices"]
    if (len(ports) != 4 or any(not isinstance(port, str) or not port.startswith("/dev/")
                               for port in ports) or
            len({Path(port).resolve() for port in ports}) != 4):
        raise ValueError("serial_devices must name four distinct device paths")
    if not tool.is_file():
        raise ValueError(f"BRAKE tool not found: {tool}; build unitree_go2 first")
    for port in ports:
        if not Path(port).exists():
            raise ValueError(f"missing motor port: {port}")
    busy = subprocess.run(["fuser", *ports], capture_output=True, text=True, check=False)
    if busy.returncode != 1 or busy.stdout.strip() or busy.stderr.strip():
        raise ValueError(f"motor ports busy or occupancy check failed: {busy.stdout}{busy.stderr}")
    directory.mkdir(parents=True, exist_ok=False)
    (directory / "sdk").mkdir()
    (directory / "base_config.yaml").write_text(yaml.safe_dump(config, sort_keys=False))
    (directory / "reference.yaml").write_text(yaml.safe_dump(reference, sort_keys=False))
    (directory / "phase.txt").write_text(phase + "\n")
    (directory / "samples.txt").write_text(str(samples) + "\n")
    for index in PHASES[phase]:
        joint = JOINTS[index]
        path = directory / "sdk" / f"{joint[:2]}_id{index % 3}.txt"
        with path.open("x", encoding="ascii") as log:
            subprocess.run([str(tool), ports[index // 3], str(index % 3), str(samples)],
                           stdout=log, stderr=subprocess.STDOUT, check=True, timeout=30)
        print(f"Captured {phase} {joint}: {samples} BRAKE replies", flush=True)


def combine_phases(directories, output=None):
    phases = {}
    base = reference = None
    for directory in map(Path, directories):
        phase = (directory / "phase.txt").read_text().strip()
        if phase not in PHASES or phase in phases:
            raise ValueError(f"unexpected or duplicate phase in {directory}: {phase}")
        phases[phase] = directory
        saved_base = yaml.safe_load((directory / "base_config.yaml").read_text())
        saved_reference = yaml.safe_load((directory / "reference.yaml").read_text())
        if base is None:
            base, reference = saved_base, saved_reference
        elif saved_base != base or saved_reference != reference:
            raise ValueError("phase captures use different base config or pose reference")
    if set(phases) != set(PHASES):
        raise ValueError("provide one capture directory each for poseX, poseY, and poseZ")
    result = base
    for phase in PHASES:
        directory = phases[phase]
        samples = int((directory / "samples.txt").read_text())
        result = update_phase(result, phase, reference,
                              phase_rotor_means(directory, phase, samples))
    if output is not None:
        Path(output).parent.mkdir(parents=True, exist_ok=True)
        with Path(output).open("x") as stream:
            yaml.safe_dump(result, stream, sort_keys=False)
        print(f"Generated {output}; updated only poseX/poseY/poseZ calibrated joints")
    return result


def apply_calibration(path, original_text, result):
    """Replace local YAML atomically, rejecting edits made during capture."""
    if path.read_text() != original_text:
        raise ValueError("本机配置在标定期间已变化，未覆盖；请检查生成结果")
    metadata = path.stat()
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=".calibration-", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            current = os.fstat(stream.fileno())
            if (current.st_uid, current.st_gid) != (metadata.st_uid, metadata.st_gid):
                os.fchown(stream.fileno(), metadata.st_uid, metadata.st_gid)
            os.fchmod(stream.fileno(), metadata.st_mode & 0o7777)
            yaml.safe_dump(result, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        if path.read_text() != original_text:
            raise ValueError("本机配置在标定期间已变化，未覆盖；请检查生成结果")
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def interactive_calibration(directory, base_path, reference_path, tool, samples, output):
    base_path = base_path.expanduser().resolve(strict=True)
    original_text = base_path.read_text()
    original = yaml.safe_load(original_text)
    config = load_config(base_path)
    reference = yaml.safe_load(reference_path.read_text())
    validate(config, reference)
    if output.exists() or output.resolve() == base_path:
        raise ValueError("结果文件已存在或与本机配置相同，请选择新的 --output 路径")
    directory.mkdir(parents=True, exist_ok=False)
    print("开始 poseXYZ 标定。请先停止 Go2 主程序和自启服务，支撑好机身。", flush=True)
    print(f"本机配置：{base_path}\n采集目录：{directory}", flush=True)
    captures = []
    descriptions = {"poseX": "四条腿的大腿和小腿", "poseY": "左前和左后髋关节", "poseZ": "右前和右后髋关节"}
    for phase in PHASES:
        attempt = 0
        while True:
            print(f"\n请摆到 {phase}：{descriptions[phase]}，按标定说明中的对应姿势图摆放。",
                  flush=True)
            answer = input("已摆好并保持不动后按回车，5 秒后开始记录；输入 q 退出：").strip().lower()
            if answer == "q":
                print("标定已取消，本机配置未修改。", flush=True)
                return
            if answer not in ("", "y", "yes", "确认", "已摆好"):
                continue
            for remaining in range(5, 0, -1):
                print(f"{phase} 采集倒计时：{remaining} 秒，请保持姿势。", flush=True)
                time.sleep(1)
            attempt += 1
            capture_directory = directory / (phase if attempt == 1 else f"{phase}-{attempt}")
            print(f"正在记录 {phase}，请继续保持不动……", flush=True)
            try:
                capture_phase(capture_directory, config, reference, tool, samples, phase)
                phase_rotor_means(capture_directory, phase, samples)
            except (OSError, ValueError, subprocess.SubprocessError) as error:
                print(f"{phase} 采集未通过：{error}\n请重新摆好本阶段姿势，确认后重采。", flush=True)
                continue
            captures.append(capture_directory)
            print(f"{phase} 记录完成，采样稳定性检查通过。", flush=True)
            break

    print("\nPose X/Y/Z 全部记录完成，可以松手、放稳机器狗；现在计算标定结果。", flush=True)
    combined = combine_phases(captures)["go2"]["ros__parameters"]
    result = copy.deepcopy(original)
    local_params = result.setdefault("go2", {}).setdefault("ros__parameters", {})
    for name in ("joint_zero_offsets_rad", "startup_reference_rotor_positions_rad"):
        local_params[name] = combined[name]
    previous = config["go2"]["ros__parameters"]["joint_zero_offsets_rad"]
    print("关节           原零偏(rad)    新零偏(rad)    Pose A 转子参考(rad)", flush=True)
    for index, joint in enumerate(JOINTS):
        print(f"{joint:12s} {previous[index]:13.6f} {combined['joint_zero_offsets_rad'][index]:13.6f} "
              f"{combined['startup_reference_rotor_positions_rad'][index]:20.6f}", flush=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as stream:
        yaml.safe_dump(result, stream, sort_keys=False, allow_unicode=True)
    print(f"结果已保存：{output}", flush=True)
    answer = input(f"输入“确认标定”写入 {base_path}；直接回车仅保留结果、不应用：").strip().lower()
    if answer not in ("确认标定", "确认", "y", "yes"):
        print("未应用标定，本机配置保持原样。", flush=True)
        return
    apply_calibration(base_path, original_text, result)
    print(f"标定已确认并写入 {base_path}。下次启动生效，启动前请恢复 Pose A。", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", type=Path, nargs="*")
    parser.add_argument("--interactive", action="store_true", help="guide all XYZ stages; default without paths")
    parser.add_argument("--phase", choices=tuple(PHASES),
                        help="phase to capture; omit when combining the three captures")
    parser.add_argument("--capture", action="store_true", help="sample BRAKE feedback for this phase")
    parser.add_argument("--base-config", type=Path)
    parser.add_argument("--reference", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--tool", type=Path,
                        default=Path(__file__).resolve().with_name("motor_brake_capture"))
    args = parser.parse_args()
    if not 1 <= args.samples <= 10000:
        parser.error("samples must be between 1 and 10000")
    if args.output is not None and args.output.exists():
        parser.error("output already exists; choose a new file")
    if args.interactive or not args.paths:
        if len(args.paths) > 1 or args.capture or args.phase:
            parser.error("interactive calibration uses poseXYZ with at most one new capture directory")
        directory = args.paths[0] if args.paths else (
            Path(os.environ.get("SROBOTIS_OUTPUT_ROOT", "output")) / "calibration" /
            datetime.now().strftime("poseXYZ-%Y%m%d-%H%M%S-%f"))
        interactive_calibration(
            directory, args.base_config or Path(os.environ.get("GO2_LOCAL_CONFIG", "/etc/unitree-go2/robot.yaml")),
            args.reference or default_config("calibration_pose_xyz.yaml"), args.tool, args.samples,
            args.output or directory / "calibrated.yaml")
        return
    if args.phase:
        if len(args.paths) != 1 or not args.capture:
            parser.error("phase capture requires CAPTURE_DIR --phase poseX|poseY|poseZ --capture")
        base_path = args.base_config or Path(os.environ.get("GO2_LOCAL_CONFIG", "/etc/unitree-go2/robot.yaml"))
        reference_path = args.reference or default_config("calibration_pose_xyz.yaml")
        config = load_config(base_path)
        reference = yaml.safe_load(reference_path.read_text())
        capture_phase(args.paths[0], config, reference, args.tool, args.samples, args.phase)
        return
    if args.capture:
        parser.error("combining takes three phase directories; omit --capture")
    if len(args.paths) != 3:
        parser.error("provide exactly three pose capture directories")
    if args.output is None:
        parser.error("combining phase captures requires --output")
    combine_phases(args.paths, args.output)


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print("\n标定已中止。", file=sys.stderr)
        sys.exit(130)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"标定失败：{error}", file=sys.stderr)
        sys.exit(1)
