# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0
import math
from pathlib import Path
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _make_node(context):
    backend = LaunchConfiguration("backend").perform(context)
    config_path = Path(LaunchConfiguration("config").perform(context)).expanduser()
    with config_path.open(encoding="utf-8") as stream:
        shared = yaml.safe_load(stream)
    local_path = Path(LaunchConfiguration("local_config").perform(context)).expanduser()
    local = {}
    if local_path.resolve() == config_path.resolve():
        local = shared
    elif local_path.is_file():
        with local_path.open(encoding="utf-8") as stream:
            local = yaml.safe_load(stream) or {}
    elif backend == "serial":
        raise RuntimeError(f"Missing board-local configuration: {local_path}")
    # Take only shared settings from the bundled file; its reference calibration must
    # never fill missing fields in an explicitly selected robot configuration.
    defaults = {**shared["defaults"], **local.get("defaults", {})}
    profiles = {**shared["board_profiles"], **local.get("board_profiles", {})}
    identity = local.get("robot", {})
    local_parameters = local.get("go2", {}).get("ros__parameters", {})
    board_profile = LaunchConfiguration("board_profile").perform(context) or identity.get("board_profile", "legacy_usb")
    robot = LaunchConfiguration("robot").perform(context) or identity.get("id", "go2")
    if not isinstance(robot, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", robot):
        raise RuntimeError("invalid robot ID")
    tilt_protection = LaunchConfiguration("walking_tilt_protection").perform(context)
    if tilt_protection not in ("", "true", "false"):
        raise RuntimeError("walking_tilt_protection must be true or false")
    tilt_protection = (
        identity.get("walking_tilt_protection", True) if not tilt_protection else tilt_protection == "true"
    )
    if not isinstance(tilt_protection, bool):
        raise RuntimeError("walking_tilt_protection must be a boolean")
    domain = LaunchConfiguration("ros_domain_id").perform(context)
    domain = int(domain) if domain else identity.get("ros_domain_id", 217)
    if isinstance(domain, bool) or not isinstance(domain, int) or not 0 <= domain <= 232:
        raise RuntimeError("ros_domain_id must be an integer between 0 and 232")
    if backend == "serial":
        if not identity.get("id") or "ros_domain_id" not in identity:
            raise RuntimeError("Board-local config requires robot.id and robot.ros_domain_id")
        for key in ("joint_zero_offsets_rad", "startup_reference_rotor_positions_rad"):
            values = local_parameters.get(key)
            if (not isinstance(values, list) or len(values) != 12
                    or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                           for v in values)):
                raise RuntimeError(f"Board-local config requires 12 finite values for {key}")
    model = LaunchConfiguration("model").perform(context)
    if (
        board_profile not in ("legacy_usb", "new_smallcore", "new_bigcore", "new_usbmotor")
        or board_profile not in profiles
    ):
        raise RuntimeError("unknown board_profile: " + board_profile)
    profile = profiles[board_profile]
    board_parameters = dict(profile["go2"])
    imu = LaunchConfiguration("imu").perform(context)
    imu_driver = LaunchConfiguration("imu_driver").perform(context)
    motor_driver = LaunchConfiguration("motor_driver").perform(context)
    if board_profile != "legacy_usb":
        if imu not in ("", "bmi270") or imu_driver not in ("", "drv_spi_bmi270"):
            raise RuntimeError("new-board profiles require the fixed BMI270 IMU")
        imu_driver = "drv_spi_bmi270"
        if motor_driver and motor_driver != board_parameters["motor_driver"]:
            raise RuntimeError("motor_driver conflicts with board_profile")
    else:
        imu_driver = imu_driver or (
            {"iwt603": "drv_uart_iwt603", "bmi270": "drv_spi_bmi270"}[imu] if imu
            else local_parameters.get("imu_driver", defaults.get("imu_driver", "drv_uart_iwt603"))
        )
        motor_driver = motor_driver or local_parameters.get(
            "motor_driver", defaults.get("motor_driver", "drv_485_m8010a")
        )
        board_parameters["motor_driver"] = motor_driver
    # Mounting rotations apply consistently to attitude and sensor vectors.
    # A per-robot absolute calibration replaces the fallback, without applying it twice.
    board_parameters.update(profile.get("imu_overrides", {}).get(imu_driver, {}))
    board_parameters.update(local_parameters)
    if board_profile != "legacy_usb" and (
        board_parameters.get("imu_driver", imu_driver) != imu_driver
        or board_parameters.get("motor_driver") != profile["go2"]["motor_driver"]
    ):
        raise RuntimeError("Board-local driver conflicts with board_profile")
    imu_device = LaunchConfiguration("imu_device").perform(context)
    board_parameters["imu_driver"] = imu_driver
    if board_profile == "legacy_usb":
        board_parameters["motor_driver"] = motor_driver
    if imu_device:
        board_parameters["imu_device"] = imu_device
    elif board_profile == "legacy_usb" and imu_driver == "drv_spi_bmi270" and "imu_device" not in board_parameters:
        board_parameters["imu_device"] = "/dev/bmi270"
    sbus_device = (
        LaunchConfiguration("sbus_device").perform(context) or identity.get("sbus_device", profile["sbus_device"])
    )
    # Leave shared/local YAML values intact unless the caller explicitly overrides them.
    # Only the serial backend opens motor devices; mock runs clear all approvals.
    sbus_enabled = LaunchConfiguration("sbus").perform(context).lower() in ("true", "1", "yes")
    web_enabled = LaunchConfiguration("web").perform(context).lower() in ("true", "1", "yes")
    teleop_source = (
        "any" if sbus_enabled and web_enabled else "sbus" if sbus_enabled else "web" if web_enabled else "none"
    )
    approved = backend == "serial"
    actions = [
        Node(
            package="unitree_go2",
            executable="go2_node",
            name="go2",
            output="screen",
            parameters=[
                {
                    **defaults,
                    **board_parameters,
                    "robot": ParameterValue(robot, value_type=str),
                    "teleop_source": teleop_source,
                    "sbus_connection_timeout_sec": ParameterValue(
                        LaunchConfiguration("sbus_connection_timeout_sec"), value_type=float
                    ),
                    "motion_log_path": ParameterValue(LaunchConfiguration("motion_log_path"), value_type=str),
                    "backend": backend,
                    "board_profile": board_profile,
                    "model_path": model,
                    "policy_config": LaunchConfiguration("policy_config"),
                    "walking_tilt_protection": tilt_protection,
                    "motor_io_authorized": approved,
                    "actuation_enabled": approved,
                    "hardware_configuration_verified": approved,
                    "motor_ids_verified": approved,
                    "joint_calibration_verified": approved,
                    "suspended_test_verified": approved,
                },
            ],
        )
    ]
    if sbus_enabled:
        actions.append(
            Node(
                package="unitree_go2",
                executable="sbus_teleop_node",
                name="go2_sbus",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {
                        "device": sbus_device,
                        "shared_control": sbus_enabled and web_enabled,
                        "connection_timeout_sec": ParameterValue(
                            LaunchConfiguration("sbus_connection_timeout_sec"), value_type=float
                        ),
                        "accept_sbus2_footer": ParameterValue(
                            LaunchConfiguration("sbus_accept_sbus2_footer"), value_type=bool
                        ),
                        "forward_channel": ParameterValue(LaunchConfiguration("sbus_forward_channel"), value_type=int),
                        "lateral_channel": ParameterValue(LaunchConfiguration("sbus_lateral_channel"), value_type=int),
                        "yaw_channel": ParameterValue(LaunchConfiguration("sbus_yaw_channel"), value_type=int),
                        "right_vertical_channel": ParameterValue(
                            LaunchConfiguration("sbus_right_vertical_channel"), value_type=int
                        ),
                        "e_channel": ParameterValue(LaunchConfiguration("sbus_e_channel"), value_type=int),
                        "f_channel": ParameterValue(LaunchConfiguration("sbus_f_channel"), value_type=int),
                        "b_channel": ParameterValue(LaunchConfiguration("sbus_b_channel"), value_type=int),
                        "c_channel": ParameterValue(LaunchConfiguration("sbus_c_channel"), value_type=int),
                        "d_channel": ParameterValue(LaunchConfiguration("sbus_d_channel"), value_type=int),
                        "g_channel": ParameterValue(LaunchConfiguration("sbus_g_channel"), value_type=int),
                        "h_channel": ParameterValue(LaunchConfiguration("sbus_h_channel"), value_type=int),
                        "channel_min": ParameterValue(LaunchConfiguration("sbus_channel_min"), value_type=int),
                        "channel_center": ParameterValue(LaunchConfiguration("sbus_channel_center"), value_type=int),
                        "channel_max": ParameterValue(LaunchConfiguration("sbus_channel_max"), value_type=int),
                        "switch_low": ParameterValue(LaunchConfiguration("sbus_switch_low"), value_type=int),
                        "switch_high": ParameterValue(LaunchConfiguration("sbus_switch_high"), value_type=int),
                        "wheel_extreme_margin": ParameterValue(
                            LaunchConfiguration("sbus_wheel_extreme_margin"), value_type=int
                        ),
                        "deadband": ParameterValue(LaunchConfiguration("sbus_deadband"), value_type=float),
                        "frame_timeout_sec": ParameterValue(
                            LaunchConfiguration("sbus_frame_timeout_sec"), value_type=float
                        ),
                        "edge_debounce_sec": ParameterValue(
                            LaunchConfiguration("sbus_edge_debounce_sec"), value_type=float
                        ),
                        "invert_forward": ParameterValue(LaunchConfiguration("sbus_invert_forward"), value_type=bool),
                        "invert_lateral": ParameterValue(LaunchConfiguration("sbus_invert_lateral"), value_type=bool),
                        "invert_yaw": ParameterValue(LaunchConfiguration("sbus_invert_yaw"), value_type=bool),
                    }
                ],
            )
        )
    if web_enabled:
        actions.append(
            Node(
                package="unitree_go2",
                executable="web_teleop_server.py",
                name="go2_web",
                output="screen",
                sigterm_timeout="35",
                parameters=[
                    {
                        "web_port": int(LaunchConfiguration("web_port").perform(context)),
                        "shared_control": sbus_enabled and web_enabled,
                        "web_root": str(Path(get_package_share_directory("unitree_go2")) / "web"),
                        "robot": ParameterValue(robot, value_type=str),
                        "network_mode": LaunchConfiguration("network_mode"),
                        "robot_name": ParameterValue(str(identity.get("name", robot)), value_type=str),
                        "ap_ssid": ParameterValue(str(identity.get("ap_ssid", "")), value_type=str),
                        "wifi_connection": ParameterValue(
                            LaunchConfiguration("wifi_connection").perform(context)
                            or str(identity.get("wifi_connection", "")), value_type=str
                        ),
                        "ap_address": LaunchConfiguration("ap_address"),
                        "ap_password": LaunchConfiguration("ap_password"),
                        "ap_band": LaunchConfiguration("ap_band"),
                        "ap_channel": int(LaunchConfiguration("ap_channel").perform(context)),
                    }
                ],
            )
        )
    # Set the domain before starting every control, input and peripheral node.
    # Do not inherit a common shell domain when multiple robots are online.
    critical_nodes = [actions[0]]
    if web_enabled:
        critical_nodes.append(actions[-1])
    exit_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[EmitEvent(event=Shutdown(reason="Go2 control or web node exited"))],
            )
        )
        for node in critical_nodes
    ]
    return [SetEnvironmentVariable("ROS_DOMAIN_ID", str(domain)), *exit_handlers, *actions]


def generate_launch_description():
    share = Path(get_package_share_directory("unitree_go2"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config", default_value=str(share / "config/robot.yaml"),
                description="Shared runtime defaults and board profiles",
            ),
            DeclareLaunchArgument("robot", default_value="", description="Override the board-local robot ID"),
            DeclareLaunchArgument("ros_domain_id", default_value="", description="Override the board-local DDS domain"),
            DeclareLaunchArgument(
                "local_config",
                default_value=EnvironmentVariable("GO2_LOCAL_CONFIG", default_value="/etc/unitree-go2/robot.yaml"),
                description="Board-local robot identity and calibration",
            ),
            DeclareLaunchArgument(
                "motion_log_path",
                default_value="/tmp/go2_motion.csv",
                description="Motion CSV path; empty disables recording",
            ),
            DeclareLaunchArgument("backend", default_value="mock", choices=["mock", "serial"]),
            DeclareLaunchArgument("motor_driver", default_value=""),
            DeclareLaunchArgument(
                "board_profile",
                default_value="",
                choices=["", "legacy_usb", "new_smallcore", "new_bigcore", "new_usbmotor"],
            ),
            # Empty keeps the legacy IWT603 default; new boards use fixed BMI270.
            DeclareLaunchArgument("imu", default_value="", choices=["", "iwt603", "bmi270"]),
            DeclareLaunchArgument("imu_driver", default_value="", description="Optional low-level driver override"),
            DeclareLaunchArgument("imu_device", default_value="", description="Override selected IMU device path"),
            DeclareLaunchArgument(
                "walking_tilt_protection", default_value="", choices=["", "true", "false"],
                description="Walking tilt protection from the board-local config; "
                    "crouch then passive when |roll| > 40 deg or |pitch| > 60 deg",
            ),
            DeclareLaunchArgument("model", default_value=str(share / "policy")),
            DeclareLaunchArgument("policy_config", default_value=str(share / "config/policies.yaml")),
            DeclareLaunchArgument("web", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("web_port", default_value="8082"),
            DeclareLaunchArgument("network_mode", default_value="wifi", choices=["wifi", "ap"]),
            DeclareLaunchArgument("wifi_connection", default_value=""),
            DeclareLaunchArgument("ap_address", default_value="10.0.91.250/23"),
            DeclareLaunchArgument("ap_password", default_value="go2control"),
            DeclareLaunchArgument("ap_band", default_value="a", choices=["a", "bg"]),
            DeclareLaunchArgument("ap_channel", default_value="0"),
            DeclareLaunchArgument("sbus", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("sbus_device", default_value=""),
            DeclareLaunchArgument("sbus_accept_sbus2_footer", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("sbus_forward_channel", default_value="3"),
            DeclareLaunchArgument("sbus_lateral_channel", default_value="4"),
            DeclareLaunchArgument("sbus_yaw_channel", default_value="1"),
            DeclareLaunchArgument("sbus_right_vertical_channel", default_value="2"),
            DeclareLaunchArgument("sbus_e_channel", default_value="5"),
            DeclareLaunchArgument("sbus_f_channel", default_value="6"),
            DeclareLaunchArgument("sbus_b_channel", default_value="8"),
            DeclareLaunchArgument("sbus_c_channel", default_value="9"),
            DeclareLaunchArgument("sbus_d_channel", default_value="10"),
            DeclareLaunchArgument("sbus_g_channel", default_value="11"),
            DeclareLaunchArgument("sbus_h_channel", default_value="12"),
            DeclareLaunchArgument("sbus_channel_min", default_value="282"),
            DeclareLaunchArgument("sbus_channel_center", default_value="1002"),
            DeclareLaunchArgument("sbus_channel_max", default_value="1722"),
            DeclareLaunchArgument("sbus_switch_low", default_value="700"),
            DeclareLaunchArgument("sbus_switch_high", default_value="1300"),
            DeclareLaunchArgument("sbus_wheel_extreme_margin", default_value="100"),
            DeclareLaunchArgument("sbus_deadband", default_value="0.05"),
            DeclareLaunchArgument("sbus_frame_timeout_sec", default_value="0.25"),
            DeclareLaunchArgument("sbus_connection_timeout_sec", default_value="1.0"),
            DeclareLaunchArgument("sbus_edge_debounce_sec", default_value="0.08"),
            DeclareLaunchArgument("sbus_invert_forward", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("sbus_invert_lateral", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("sbus_invert_yaw", default_value="true", choices=["true", "false"]),
            OpaqueFunction(function=_make_node),
        ]
    )
