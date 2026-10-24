#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0
"""Evaluate launch actions without executing or opening any hardware."""

import copy
import importlib.util
import os
import sys
import tempfile
from pathlib import Path
from unittest import TestCase
from unittest.mock import patch

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("go2_launch", root / "launch/go2.launch.py")
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
checks = TestCase()


# Evaluate the current source tree, without requiring an installed ROS package.
def source_share(package):
    assert package == "unitree_go2"
    return str(root.resolve())


m.get_package_share_directory = source_share


def launch_context(**overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in m.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def evaluate(context, include_domain=False):
    actions = m._make_node(context)
    for action in actions:
        if isinstance(action, SetEnvironmentVariable):
            action.execute(context)
    actions = [action for action in actions if isinstance(action, Node)]
    # ROS launch keeps normalized parameters on the Node action until execution.
    result = [evaluate_parameters(context, a._Node__parameters)[-1] for a in actions]
    return (context.environment["ROS_DOMAIN_ID"], result) if include_domain else result


def resolve(include_domain=False, **overrides):
    context = launch_context(**{
        "sbus": "true", "web": "true", "local_config": str(root / "test/nonexistent-local.yaml"), **overrides,
    })
    return evaluate(context, include_domain)


legacy = resolve()
assert legacy[0]["walking_tilt_protection"] is True
assert resolve(walking_tilt_protection="false")[0]["walking_tilt_protection"] is False
assert legacy[-1]["ap_band"] == "a" and legacy[-1]["ap_channel"] == 0
ap = resolve(ap_band="bg", ap_channel="6")[-1]
assert ap["ap_band"] == "bg" and ap["ap_channel"] == 6

reference_robot = yaml.safe_load((root / "config/robot.yaml").read_text())
reference_domain, reference_nodes = resolve(include_domain=True, local_config=str(root / "config/robot.yaml"),
                                        backend="serial")
assert reference_nodes[0]["robot"] == reference_robot["robot"]["id"]
assert reference_domain == str(reference_robot["robot"]["ros_domain_id"])
assert (list(reference_nodes[0]["joint_zero_offsets_rad"])
        == reference_robot["go2"]["ros__parameters"]["joint_zero_offsets_rad"])
assert reference_nodes[0]["imu_state_timeout_sec"] == reference_robot["defaults"]["imu_state_timeout_sec"]

# Run real launch argument defaults, without resolve() injecting a local_config.
# Model a board without its /etc file without changing the host's configuration.
path_is_file = Path.is_file
with patch.dict(os.environ, {}), patch.object(
        Path, "is_file", lambda path: False if str(path) == "/etc/unitree-go2/robot.yaml" else path_is_file(path)):
    os.environ.pop("GO2_LOCAL_CONFIG", None)
    context = launch_context()
    assert context.launch_configurations["local_config"] == "/etc/unitree-go2/robot.yaml"
    domain, nodes = evaluate(context, include_domain=True)
    assert domain == "217" and nodes[0]["robot"] == "go2"
    assert list(nodes[0]["joint_zero_offsets_rad"]) == [0.0] * 12
    assert list(nodes[0]["startup_reference_rotor_positions_rad"]) == [0.0] * 12
    assert list(nodes[0]["imu_to_body_quaternion_wxyz"]) == [1.0, 0.0, 0.0, 0.0]
    with checks.assertRaisesRegex(RuntimeError, "Missing board-local configuration: /etc/unitree-go2/robot.yaml"):
        evaluate(launch_context(backend="serial"))
    with tempfile.TemporaryDirectory() as temporary:
        shared_path = Path(temporary) / "reference.yaml"
        shared = copy.deepcopy(reference_robot)
        shared["robot"].update(id="reference-only", ros_domain_id=201)
        shared["defaults"]["imu_state_timeout_sec"] = 0.42
        shared_path.write_text(yaml.safe_dump(shared))
        context = launch_context(config=str(shared_path))
        assert context.launch_configurations["local_config"] == "/etc/unitree-go2/robot.yaml"
        domain, nodes = evaluate(context, include_domain=True)
        assert domain == "217" and nodes[0]["robot"] == "go2"
        assert nodes[0]["imu_state_timeout_sec"] == 0.42
        assert list(nodes[0]["joint_zero_offsets_rad"]) == [0.0] * 12
        with checks.assertRaises(RuntimeError):
            evaluate(launch_context(config=str(shared_path), backend="serial"))
        local_path = Path(temporary) / "board.yaml"
        local = {"robot": {"id": "environment-robot", "ros_domain_id": 209, "board_profile": "new_bigcore"},
                 "go2": {"ros__parameters": {"joint_zero_offsets_rad": [0.25] * 12,
                                            "startup_reference_rotor_positions_rad": [1.5] * 12}}}
        local_path.write_text(yaml.safe_dump(local))
        os.environ["GO2_LOCAL_CONFIG"] = str(local_path)
        context = launch_context(backend="serial")
        assert context.launch_configurations["local_config"] == str(local_path)
        domain, nodes = evaluate(context, include_domain=True)
        assert domain == "209" and nodes[0]["robot"] == "environment-robot"
        assert list(nodes[0]["joint_zero_offsets_rad"]) == [0.25] * 12
        # An explicit local_config may select a reference and overrides the environment.
        context = launch_context(local_config=str(root / "config/robot.yaml"), backend="serial")
        domain, nodes = evaluate(context, include_domain=True)
        assert domain == str(reference_robot["robot"]["ros_domain_id"])
        assert nodes[0]["robot"] == reference_robot["robot"]["id"]

# Exercise the merge through launch's evaluated ROS parameters, including sparse
# external files created before shared defaults were moved into robot.yaml.
with tempfile.TemporaryDirectory() as temporary:
    shared_path = Path(temporary) / "shared.yaml"
    local_path = Path(temporary) / "robot.yaml"
    shared = copy.deepcopy(reference_robot)
    shared["defaults"]["imu_state_timeout_sec"] = 0.42
    shared["defaults"]["maximum_motor_temperature_c"] = 80
    shared["board_profiles"]["new_bigcore"]["go2"]["maximum_motor_temperature_c"] = 70
    shared_path.write_text(yaml.safe_dump(shared))
    local = {"robot": {"id": "customer-robot", "ros_domain_id": 210, "board_profile": "new_bigcore"},
             "go2": {"ros__parameters": {"joint_zero_offsets_rad": [0.25] * 12,
                                        "startup_reference_rotor_positions_rad": [1.5] * 12}}}
    local_path.write_text(yaml.safe_dump(local))
    domain, nodes = resolve(config=str(shared_path), local_config=str(local_path), backend="serial",
                            include_domain=True)
    params = nodes[0]
    assert domain == "210" and params["robot"] == "customer-robot"
    assert params["imu_state_timeout_sec"] == 0.42
    assert params["maximum_motor_temperature_c"] == 70
    assert list(params["joint_zero_offsets_rad"]) == [0.25] * 12
    assert list(params["startup_reference_rotor_positions_rad"]) == [1.5] * 12
    assert list(params["imu_to_body_quaternion_wxyz"]) == [0.0, 0.0, 0.0, 1.0]
    assert "robot_name" not in params and "defaults" not in params and "board_profiles" not in params
    local["go2"]["ros__parameters"].update(maximum_motor_temperature_c=60, imu_device="/dev/customer-imu")
    local_path.write_text(yaml.safe_dump(local))
    params = resolve(config=str(shared_path), local_config=str(local_path), backend="serial")[0]
    assert params["maximum_motor_temperature_c"] == 60 and params["imu_device"] == "/dev/customer-imu"
    params = resolve(config=str(shared_path), local_config=str(local_path), backend="serial",
                     imu_device="/dev/launch-imu")[0]
    assert params["imu_device"] == "/dev/launch-imu"
    # A complete local config can carry its own shared settings as well.
    local["defaults"] = {"imu_state_timeout_sec": 0.51}
    local["board_profiles"] = {"new_bigcore": copy.deepcopy(shared["board_profiles"]["new_bigcore"])}
    local["board_profiles"]["new_bigcore"]["sbus_device"] = "/dev/customer-sbus"
    local_path.write_text(yaml.safe_dump(local))
    nodes = resolve(config=str(shared_path), local_config=str(local_path), backend="serial")
    assert nodes[0]["imu_state_timeout_sec"] == 0.51 and nodes[1]["device"] == "/dev/customer-sbus"
    del local["go2"]["ros__parameters"]["joint_zero_offsets_rad"]
    local_path.write_text(yaml.safe_dump(local))
    with checks.assertRaises(RuntimeError):
        resolve(config=str(shared_path), local_config=str(local_path), backend="serial")

# Temporary synthetic board profiles exercise identity and calibration isolation.
with tempfile.TemporaryDirectory() as temporary:
    local_path = Path(temporary) / "robot.yaml"
    for index in range(2):
        local = {"robot": {"id": f"robot-{index}", "name": f"Go2 {index}", "ros_domain_id": 211 + index,
                           "board_profile": "new_bigcore", "walking_tilt_protection": bool(index),
                           "wifi_connection": "Saved Wi-Fi", "ap_ssid": f"Go2-{index}"},
                 "go2": {"ros__parameters": {"joint_zero_offsets_rad": [0.25 + index] * 12,
                                            "startup_reference_rotor_positions_rad": [1.5 + index] * 12,
                                            "imu_to_body_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                                            "imu_to_body_matrix": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]}}}
        local_path.write_text(yaml.safe_dump(local))
        domain, nodes = resolve(include_domain=True,
                                local_config=str(local_path), backend="serial")
        assert domain == str(211 + index)
        assert nodes[0]["robot"] == f"robot-{index}"
        assert list(nodes[0]["joint_zero_offsets_rad"]) == [0.25 + index] * 12
        assert list(nodes[0]["startup_reference_rotor_positions_rad"]) == [1.5 + index] * 12
        assert list(nodes[0]["imu_to_body_quaternion_wxyz"]) == [1.0, 0.0, 0.0, 0.0]
        assert nodes[0]["walking_tilt_protection"] is bool(index)
        assert nodes[0]["motor_io_authorized"] is True
        assert nodes[-1]["robot_name"] == f"Go2 {index}"
        assert nodes[-1]["wifi_connection"] == "Saved Wi-Fi"
        assert nodes[-1]["ap_ssid"] == f"Go2-{index}"
        assert resolve(local_config=str(local_path))[0]["motor_io_authorized"] is False
        override = resolve(local_config=str(local_path), walking_tilt_protection="false")[0]
        assert override["walking_tilt_protection"] is False
        assert resolve(include_domain=True, local_config=str(local_path), ros_domain_id="197")[0] == "197"
    # A calibration produced by the tool contains shared defaults too. Launch
    # arguments must still select the actual backend, identity and board.
    local["go2"]["ros__parameters"].update(backend="mock", robot="ignored", board_profile="legacy_usb")
    local_path.write_text(yaml.safe_dump(local))
    node = resolve(local_config=str(local_path), backend="serial")[0]
    assert node["backend"] == "serial" and node["robot"] == "robot-1" and node["board_profile"] == "new_bigcore"
    local["robot"]["board_profile"] = "legacy_usb"
    local["go2"]["ros__parameters"].update(imu_driver="drv_spi_bmi270", imu_device="/dev/iio:device1")
    local_path.write_text(yaml.safe_dump(local))
    node = resolve(local_config=str(local_path))[0]
    assert node["imu_driver"] == "drv_spi_bmi270" and node["imu_device"] == "/dev/iio:device1"
    local["robot"]["board_profile"] = "new_bigcore"
    for invalid in ([], [0.0] * 11, [float("nan")] * 12, [True] * 12):
        local["go2"]["ros__parameters"]["joint_zero_offsets_rad"] = invalid
        local_path.write_text(yaml.safe_dump(local))
        with checks.assertRaises(RuntimeError):
            resolve(local_config=str(local_path), backend="serial")
with checks.assertRaises(RuntimeError):
    resolve(backend="serial")
assert resolve(include_domain=True, robot="any-new-robot")[0] == "217"
for domain in ("-1", "233", "not-a-number"):
    with checks.assertRaises((RuntimeError, ValueError)):
        resolve(ros_domain_id=domain)
for robot in ("../robot", "invalid robot", "x" * 65):
    with checks.assertRaises(RuntimeError):
        resolve(robot=robot)
assert legacy[0]["board_profile"] == "legacy_usb"
assert legacy[0]["imu_driver"] == "drv_uart_iwt603"
assert list(legacy[0]["serial_devices"]) == reference_robot["defaults"]["serial_devices"]
assert legacy[1]["device"] == "/dev/ttyS5"
bmi = resolve(imu="bmi270")[0]
assert bmi["imu_device"] == "/dev/bmi270"
assert resolve(imu="bmi270", imu_device="/dev/spidev0.0")[0]["imu_device"] == "/dev/spidev0.0"
mount_keys = ("imu_to_body_matrix", "imu_to_body_quaternion_wxyz", "imu_roll_offset_deg", "imu_pitch_offset_deg")
assert all(legacy[0][k] == tuple(reference_robot["defaults"][k]) for k in mount_keys[:2])
assert all(legacy[0][k] == reference_robot["defaults"][k] for k in mount_keys[2:])
assert "imu_gpiochip" not in bmi
assert "imu_gpio_line" not in bmi
assert "imu_gpiochip" not in legacy[0]
assert all(k in bmi for k in mount_keys)
explicit = resolve(imu_driver="drv_spi_bmi270")[0]
assert all(list(explicit[k]) == list(bmi[k]) for k in mount_keys[:2])
q = list(bmi["imu_to_body_quaternion_wxyz"])
matrix = list(bmi["imu_to_body_matrix"])
# Generic mounting is a nominal 180-degree yaw, without measured tilt.
assert q == [0.0, 0.0, 0.0, 1.0]
assert matrix == [-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0]
assert bmi["imu_roll_offset_deg"] == bmi["imu_pitch_offset_deg"] == 0.0
for board in ("new_smallcore", "new_bigcore", "new_usbmotor"):
    params = resolve(board_profile=board)
    expected_driver = "go2_smallcore" if board == "new_smallcore" else "drv_485_m8010a"
    assert params[0]["motor_driver"] == expected_driver
    assert params[0]["imu_gpiochip"] == ""
    assert params[0]["imu_gpio_line"] == 0
    assert params[0]["imu_device"] == "/dev/iio:device0"
    assert params[1]["device"] == "/dev/ttyS7"
    assert params[0]["imu_roll_offset_deg"] == params[0]["imu_pitch_offset_deg"] == 0.0
    assert params[0]["imu_driver"] == "drv_spi_bmi270"
    assert list(params[0]["imu_to_body_quaternion_wxyz"]) == q
    assert list(params[0]["imu_to_body_matrix"]) == matrix
    expected = ["go2-motor-" + leg for leg in ("rl", "rr", "fl", "fr")] if board == "new_smallcore" else (
        ["/dev/ttyS8", "/dev/ttyS5", "/dev/ttyS6", "/dev/ttyS3"] if board == "new_bigcore" else [
            "/dev/go2/motor-fr", "/dev/go2/motor-fl", "/dev/go2/motor-rr", "/dev/go2/motor-rl",
        ]
    )
    assert list(params[0]["serial_devices"]) == expected
    for wrong in ({"imu": "iwt603"}, {"imu_driver": "drv_uart_iwt603"}, {"motor_driver": "wrong"}):
        with checks.assertRaises(RuntimeError):
            resolve(board_profile=board, **wrong)
    overridden = resolve(board_profile=board, sbus_device="/dev/test-sbus")
    assert overridden[1]["device"] == "/dev/test-sbus"
# The status light follows the selected input, rather than zero Twist messages.
for sbus, web, source in (("true", "false", "sbus"), ("false", "true", "web"),
                          ("true", "true", "any"), ("false", "false", "none")):
    assert resolve(sbus=sbus, web=web)[0]["teleop_source"] == source
assert list(resolve(board_profile="new_bigcore")[0]["led_pwm_devices"]) == [
    "d4022400.pwm", "d4020c00.pwm", "d4022800.pwm", "d4022c00.pwm"]

print("PASS board profiles: wiring, IMU, input selection, PWM lights, overrides")
