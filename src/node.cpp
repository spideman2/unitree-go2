/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "node.hpp"

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <cerrno>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <nlohmann/json.hpp>

namespace go2 {
namespace {
const char* mode_display(Mode mode) {
    switch (mode) {
        case Mode::kPassive:
            return "被动";
        case Mode::kCrouch:
            return "蹲下";
        case Mode::kSit:
            return "坐下";
        case Mode::kForwardJump:
            return "前跳";
        case Mode::kStand:
            return "站立";
        case Mode::kWalk:
            return "行走";
        case Mode::kWave:
            return "招手";
        case Mode::kTerrain:
            return "地形";
        case Mode::kRollover:
            return "倒地自起";
    }
    return "未知";
}

std::string debug_reason(const std::string& reason) {
    if (reason.empty()) return "无";
    if (reason.find("FixStand") != std::string::npos) return "正在先到初始站立姿态";
    if (reason.find("fresh body IMU") != std::string::npos || reason.find("IMU") != std::string::npos)
        return "等待有效IMU数据";
    if (reason.find("motor") != std::string::npos || reason.find("feedback") != std::string::npos)
        return "等待有效电机反馈";
    if (reason.find("no timely policy") != std::string::npos) return "策略推理没有及时返回";
    if (reason.find("stale") != std::string::npos) return "传感器反馈超时";
    if (reason.find("phase complete") != std::string::npos) return "动作阶段完成";
    return reason;
}

template <class T>
T parameter(rclcpp::Node& node, const char* name, T value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return node.declare_parameter<T>(name, value, descriptor);
}
template <std::size_t N>
std::array<float, N> floats(rclcpp::Node& node, const char* name, const std::array<float, N>& defaults) {
    const auto values = parameter(node, name, std::vector<double>(defaults.begin(), defaults.end()));
    if (values.size() != N) throw std::invalid_argument(std::string(name) + " has wrong length");
    std::array<float, N> result;
    for (std::size_t i = 0; i < N; ++i) {
        result[i] = static_cast<float>(values[i]);
        if (!std::isfinite(result[i])) throw std::invalid_argument(std::string(name) + " must be finite");
    }
    return result;
}

std::string joints_text(const Commands& commands) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    for (std::size_t i = 0; i < commands.size(); ++i) {
        if (i) out << ',';
        out << commands[i].position_rad;
    }
    return out.str();
}

// Same projection as the policy frames use, kept here so the diagnostics can
// print the gravity term each attitude source would produce.
std::array<float, 3> projected_gravity(const std::array<float, 4>& quaternion) {
    float norm = 0.0F;
    for (float value : quaternion) norm += value * value;
    if (!(norm > 0.0F)) return {0.0F, 0.0F, -1.0F};
    const float inverse = 1.0F / std::sqrt(norm);
    const float w = quaternion[0] * inverse;
    const float x = quaternion[1] * inverse;
    const float y = quaternion[2] * inverse;
    const float z = quaternion[3] * inverse;
    return {2.0F * (w * y - x * z), -2.0F * (y * z + w * x), -(1.0F - 2.0F * (x * x + y * y))};
}

}  // namespace

Node::Node() : rclcpp::Node("go2") {
    HardwareConfig h;
    robot_ = parameter(*this, "robot", std::string("go2"));
    const auto board_profile = parameter(*this, "board_profile", std::string("legacy_usb"));
    board_profile_ = board_profile;
    if (board_profile == "legacy_usb") {
        h.board_profile = BoardProfile::kLegacyUsb;
    } else if (board_profile == "new_smallcore") {
        h.board_profile = BoardProfile::kNewSmallcore;
    } else if (board_profile == "new_bigcore") {
        h.board_profile = BoardProfile::kNewBigcore;
    } else if (board_profile == "new_usbmotor") {
        h.board_profile = BoardProfile::kNewUsbMotor;
    } else {
        throw std::invalid_argument(
            "board_profile must be legacy_usb, new_smallcore, new_bigcore or new_usbmotor");
    }
    RCLCPP_INFO(get_logger(), "Board profile: %s", board_profile.c_str());
    const auto backend = parameter(*this, "backend", std::string("mock"));
    if (backend != "mock" && backend != "serial") throw std::invalid_argument("backend must be mock or serial");
    h.backend = backend == "mock" ? Backend::kMock : Backend::kSerial;
    native_uart_debug_ = h.backend == Backend::kSerial && h.board_profile == BoardProfile::kNewBigcore;
    h.motor_io_authorized = parameter(*this, "motor_io_authorized", false);
    h.actuation_enabled = parameter(*this, "actuation_enabled", false);
    h.hardware_configuration_verified = parameter(*this, "hardware_configuration_verified", false);
    h.motor_ids_verified = parameter(*this, "motor_ids_verified", false);
    h.joint_calibration_verified = parameter(*this, "joint_calibration_verified", false);
    h.suspended_test_verified = parameter(*this, "suspended_test_verified", false);
    h.imu_device = parameter(*this, "imu_device", std::string(""));
    h.imu_driver = parameter(*this, "imu_driver", h.imu_driver);
    h.imu_baud = parameter(*this, "imu_baud", 921600);
    h.imu_gpiochip = parameter(*this, "imu_gpiochip", std::string(""));
    h.imu_gpio_line = parameter(*this, "imu_gpio_line", 0);
    h.imu_to_body_matrix = floats(*this, "imu_to_body_matrix", h.imu_to_body_matrix);
    h.imu_to_body_quaternion = floats(*this, "imu_to_body_quaternion_wxyz", h.imu_to_body_quaternion);
    constexpr double radians = 3.14159265358979323846 / 180.;
    h.imu_roll_offset_rad = parameter(*this, "imu_roll_offset_deg", 0.) * radians;
    h.imu_pitch_offset_rad = parameter(*this, "imu_pitch_offset_deg", 0.) * radians;
    imu_frame_ = parameter(*this, "body_imu_frame", std::string("go2_body_imu"));
    orientation_covariance_ = parameter(*this, "body_imu_orientation_covariance", .05);
    angular_covariance_ = parameter(*this, "body_imu_angular_velocity_covariance", .01);
    acceleration_covariance_ = parameter(*this, "body_imu_linear_acceleration_covariance", .1);
    if (imu_frame_.empty()) throw std::invalid_argument("body IMU frame must not be empty");
    for (double c : {orientation_covariance_, angular_covariance_, acceleration_covariance_}) {
        if (!std::isfinite(c) || c < 0) throw std::invalid_argument("invalid IMU covariance");
    }

    ControllerConfig c;
    c.rr_uart6_degraded = h.board_profile == BoardProfile::kNewBigcore ||
        h.board_profile == BoardProfile::kNewSmallcore;
    c.motor_soft_timeout = parameter(*this, "motor_state_timeout_sec", c.motor_soft_timeout);
    c.motor_hard_timeout = parameter(*this, "motor_state_hard_timeout_sec", c.motor_hard_timeout);
    c.rr_motor_soft_timeout = parameter(*this, "rr_motor_state_timeout_sec", c.rr_motor_soft_timeout);
    c.rr_motor_hard_timeout = parameter(*this, "rr_motor_state_hard_timeout_sec", c.rr_motor_hard_timeout);
    c.imu_soft_timeout = parameter(*this, "imu_state_timeout_sec", c.imu_soft_timeout);
    c.imu_hard_timeout = parameter(*this, "imu_state_hard_timeout_sec", c.imu_hard_timeout);
    c.command_timeout = parameter(*this, "command_timeout_sec", c.command_timeout);
    c.walking_tilt_protection = parameter(*this, "walking_tilt_protection", c.walking_tilt_protection);
    c.stand_duration = parameter(*this, "stand_duration_sec", c.stand_duration);
    c.crouch_pose_duration = parameter(*this, "crouch_pose_duration_sec", c.crouch_pose_duration);
    c.crouch_duration = parameter(*this, "protective_crouch_duration_sec", c.crouch_duration);
    c.move_linear_acceleration = parameter(*this, "move_linear_acceleration_mps2", c.move_linear_acceleration);
    c.move_yaw_acceleration = parameter(*this, "move_yaw_acceleration_rad_s2", c.move_yaw_acceleration);
    c.move_linear_deceleration = parameter(*this, "move_linear_deceleration_mps2", c.move_linear_deceleration);
    c.move_yaw_deceleration = parameter(*this, "move_yaw_deceleration_rad_s2", c.move_yaw_deceleration);
    c.maximum_temperature = parameter(*this, "maximum_motor_temperature_c", c.maximum_temperature);
    c.maximum_position =
        parameter(*this, "maximum_absolute_joint_position_rad", static_cast<double>(c.maximum_position));
    c.maximum_velocity =
        parameter(*this, "maximum_absolute_joint_velocity_rad_s", static_cast<double>(c.maximum_velocity));
    c.crouch_pose = floats(*this, "startup_reference_joint_positions_rad", c.crouch_pose);

    const auto devices = parameter(*this, "serial_devices", std::vector<std::string>(4, ""));
    const auto ids = parameter(*this, "motor_ids", std::vector<std::int64_t>{0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2});
    const auto gears = floats<12>(
        *this, "gear_ratios", {6.33F, 6.33F, 12.13F, 6.33F, 6.33F, 12.13F, 6.33F, 6.33F, 12.13F, 6.33F, 6.33F, 12.13F});
    const auto directions = floats<12>(*this, "joint_directions", {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
    const auto offsets = floats<12>(*this, "joint_zero_offsets_rad", {});
    const auto references = floats<12>(*this, "startup_reference_rotor_positions_rad", {});
    const auto tolerances = floats<12>(*this, "startup_anchor_joint_tolerances_rad",
        {.174532925F, .174532925F, .174532925F, .174532925F, .174532925F, .174532925F, .174532925F, .174532925F,
            .174532925F, .174532925F, .174532925F, .174532925F});
    const auto motor_driver = parameter(*this, "motor_driver", std::string("drv_485_m8010a"));
    const int attempts = parameter(*this, "startup_attempts", 5);
    if (devices.size() != 4 || ids.size() != 12 || attempts < 1 || attempts > 100) {
        throw std::invalid_argument("invalid motor bus configuration");
    }
    for (std::size_t leg = 0; leg < 4; ++leg) {
        h.legs[leg].device = devices[leg];
        native_uart_devices_[leg] = devices[leg];
        h.legs[leg].driver = motor_driver;
        h.legs[leg].startup_attempts = attempts;
        for (std::size_t local = 0; local < 3; ++local) {
            const auto i = leg * 3 + local;
            if (ids[i] < 0 || ids[i] > 14) throw std::invalid_argument("motor ID must be in [0,14]");
            h.legs[leg].calibration[local] = {static_cast<std::uint8_t>(ids[i]), gears[i], directions[i], offsets[i]};
            h.legs[leg].session_reference[local] = {c.crouch_pose[i], references[i]};
            h.legs[leg].startup_joint_tolerances_rad[local] = tolerances[i];
        }
    }
    // Validate every parameter and load the model before constructing any I/O thread.
    const auto share = ament_index_cpp::get_package_share_directory("unitree_go2");
    const auto model_path = parameter(*this, "model_path", share + "/policy");
    const auto policy_config = parameter(*this, "policy_config", share + "/config/policies.yaml");
    c.sit_reference = load_sit_reference(share + "/resources/motions/go2_sit.motion");
    c.policy_adapter = std::make_shared<PolicyAdapter>(policy_config, model_path);
    controller_ = std::make_unique<Controller>(c);
    hardware_ = std::make_unique<Hardware>(h);
    input_source_ = parameter(*this, "teleop_source", std::string("none"));
    if (input_source_ != "none" && input_source_ != "sbus" && input_source_ != "web" && input_source_ != "any")
        throw std::invalid_argument("teleop_source must be none, sbus, web or any");
    rc_control_ = input_source_ == "sbus";
    sbus_connection_timeout_ = parameter(*this, "sbus_connection_timeout_sec", 1.0);
    if (!std::isfinite(sbus_connection_timeout_) || sbus_connection_timeout_ < .25 ||
        sbus_connection_timeout_ > 10.0)
        throw std::invalid_argument("invalid SBUS connection timeout");
    light_imu_timeout_ = c.imu_hard_timeout;
    light_motor_timeout_ = c.rr_uart6_degraded ? c.rr_motor_hard_timeout : c.motor_hard_timeout;
    const auto light_devices = parameter(*this, "led_pwm_devices", std::vector<std::string>{});
    if (!light_devices.empty() && light_devices.size() != 4)
        throw std::invalid_argument("led_pwm_devices must contain white, orange, red and green devices");
    if (h.backend == Backend::kSerial && !light_devices.empty()) {
        led_pwm_config config{};
        config.channel_count = light_devices.size();
        for (std::size_t i = 0; i < light_devices.size(); ++i)
            config.channels[i] = {light_devices[i].c_str(), 0, 5000000};
        leds_.reset(led_alloc_pwm("drv_pwm_sysfs", "go2-lights", &config));
        const int result = leds_ ? led_init(leds_.get()) : -ENODEV;
        if (result != 0) {
            RCLCPP_ERROR(get_logger(), "LED 初始化失败：%d", result);
            leds_.reset();
        }
    }
    // Optional 200 Hz motion recorder; on by default so a hardware run always
    // leaves data behind.  Pass motion_log_path:="" to disable.
    const auto motion_log_path = parameter(*this, "motion_log_path", std::string("/tmp/go2_motion.csv"));
    if (!motion_log_path.empty()) {
        motion_log_.open(motion_log_path, std::ios::out | std::ios::trunc);
        if (!motion_log_) {
            throw std::runtime_error("cannot open motion_log_path: " + motion_log_path);
        }
        motion_log_ << "t_s,mode";
        for (int i = 0; i < static_cast<int>(kNumJoints); ++i) motion_log_ << ",target_" << kJointNames[i];
        for (int i = 0; i < static_cast<int>(kNumJoints); ++i) motion_log_ << ",pos_" << kJointNames[i];
        for (int i = 0; i < static_cast<int>(kNumJoints); ++i) motion_log_ << ",vel_" << kJointNames[i];
        motion_log_ << ",gyro_x,gyro_y,gyro_z,gx_used,gy_used,gz_used";
        if (native_uart_debug_) {
            for (const auto* name : kJointNames)
                motion_log_ << ",age_ms_" << name;
            for (const auto* leg : kGo2LegNames) {
                for (const auto* field : {"stage", "motor_id", "command_mode", "stage_age_ms",
                        "transaction_ms", "last_result", "transactions", "failures"})
                    motion_log_ << ',' << leg << '_' << field;
            }
            for (const auto* name : kJointNames)
                for (const auto* field : {"tx_age_ms", "crouch_tx_sec", "crouch_tx_id", "crouch_final_sent"})
                    motion_log_ << ',' << field << '_' << name;
        }
        motion_log_ << '\n';
        RCLCPP_INFO(get_logger(), "运动录波已开启：%s（约 200 Hz，关节顺序 FR,FL,RR,RL × 髋/大腿/小腿）",
            motion_log_path.c_str());
    }
    policy_ = std::make_unique<Policy>(model_path, policy_config, c.policy_adapter);
    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    policy_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = control_group_;
    // Keep source topics observable, and forward only the current owner to
    // /cmd_vel_direct. The public direct input remains available to terminal tools.
    direct_velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_direct", 10);
    velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_direct", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
            try {
                controller_->set_velocity({static_cast<float>(msg->linear.x), static_cast<float>(msg->linear.y),
                                                static_cast<float>(msg->angular.z)},
                    SteadyClock::now());
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "收到无效速度指令：%s", e.what());
            }
        },
        subscription_options);
    web_velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_web", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
            if (rc_control_.load() || navigation_mode_.load() || ai_agent_mode_.load()) return;
            if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) || !std::isfinite(msg->angular.z))
                return;
            direct_velocity_pub_->publish(*msg);
        }, subscription_options);
    sbus_slot_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/go2/sbus_slot", 1,
        [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
            if (msg->data < 0 || msg->data > 18) return;
            const int previous_slot = sbus_slot_.exchange(msg->data);
            sbus_slot_at_ = SteadyClock::now();
            // Entering slot 4 explicitly hands velocity control to the web.
            // Its subsequent heartbeats must still allow web navigation requests.
            if (input_source_ == "any" && msg->data == 4 && previous_slot != 4) {
                const bool navigation = navigation_mode_.exchange(false);
                const bool ai_agent = ai_agent_mode_.exchange(false);
                if (navigation || ai_agent) {
                    controller_->set_velocity({}, sbus_slot_at_);
                    direct_velocity_pub_->publish(geometry_msgs::msg::Twist{});
                }
            }
            update_input_control(sbus_slot_at_);
        }, subscription_options);
    rc_velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_rc", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
            if ((input_source_ == "any" && !rc_control_.load()) ||
                navigation_mode_.load(std::memory_order_relaxed) || ai_agent_mode_.load(std::memory_order_relaxed))
                return;
            if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) || !std::isfinite(msg->angular.z)) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "遥控器速度指令无效，已忽略");
                return;
            }
            direct_velocity_pub_->publish(*msg);
        },
        subscription_options);
    navigation_velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
            if (!navigation_mode_.load(std::memory_order_relaxed)) return;
            if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) || !std::isfinite(msg->angular.z)) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "导航速度指令无效，已忽略");
                return;
            }
            direct_velocity_pub_->publish(*msg);
        },
        subscription_options);
    ai_agent_velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_ai", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
            if (!ai_agent_mode_.load(std::memory_order_relaxed)) return;
            if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) || !std::isfinite(msg->angular.z)) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "AI智能体速度指令无效，已忽略");
                return;
            }
            direct_velocity_pub_->publish(*msg);
        },
        subscription_options);
    const auto mode_callback = [this](const char* source) {
        return [this, source](const std::shared_ptr<unitree_go2::srv::SetMode::Request> request,
            std::shared_ptr<unitree_go2::srv::SetMode::Response> response) {
            try {
                update_input_control(SteadyClock::now());
                if (request->mode != "passive_direct" && input_source_ != "none" &&
                    ((std::string_view(source) == "web" && rc_control_.load()) ||
                        (std::string_view(source) == "sbus" && !rc_control_.load()))) {
                    response->accepted = false;
                    response->message = rc_control_.load() ? "遥控器控制中；切到4号位或关闭遥控器后使用网页" :
                        "网页控制中；遥控器切出4号位后使用遥控器";
                    return;
                }
                const auto state = hardware_->read();
                const auto now = SteadyClock::now();
                const bool rollover_step = request->mode == "rollover_step";
                const auto requested_mode = rollover_step ? Mode::kRollover :
                    parse_mode(request->mode == "passive_direct" ? "passive" : request->mode);
                if (!policy_->load_error(requested_mode).empty())
                    throw std::runtime_error(
                        std::string(mode_name(requested_mode)) + "策略不可用：" + policy_->load_error(requested_mode));
                const auto previous_mode = controller_->status().mode;
                const auto result = rollover_step ? controller_->step_rollover(state, now) :
                    controller_->set_mode(requested_mode, state, now, request->mode == "passive");
                // A model request may be queued behind FixStand.  Reset its ONNX
                // state at request time so the first inference after the transition
                // still starts from frame zero (the controller blocks inference
                // while it is in FixStand).
                if (result.accepted && requested_mode != previous_mode && mode_uses_policy(requested_mode)) {
                    policy_->reset_mode(requested_mode, &state);
                }
                if (result.accepted) {
                    const bool navigation = request->mode == "navigation";
                    const bool ai_agent = request->mode == "ai_agent";
                    const bool navigation_changed =
                        navigation_mode_.exchange(navigation, std::memory_order_relaxed) != navigation;
                    const bool ai_agent_changed =
                        ai_agent_mode_.exchange(ai_agent, std::memory_order_relaxed) != ai_agent;
                    if (navigation_changed || ai_agent_changed) {
                        // Walk/navigation/AI-agent share one policy, so set_mode alone
                        // does not clear the old source's cached velocity when ownership
                        // changes.
                        controller_->set_velocity({}, now);
                        direct_velocity_pub_->publish(geometry_msgs::msg::Twist{});
                    }
                }
                RCLCPP_INFO(get_logger(), "模式请求：%s；结果：%s；说明：%s", request->mode.c_str(),
                    result.accepted ? "已接受" : "已拒绝",
                    (native_uart_debug_ ? result.message : debug_reason(result.message)).c_str());
                response->accepted = result.accepted;
                response->message = result.message;
            } catch (const std::exception& e) {
                response->accepted = false;
                RCLCPP_ERROR(get_logger(), "模式请求处理失败：%s", e.what());
                response->message = e.what();
            }
        };
    };
    mode_service_ = create_service<unitree_go2::srv::SetMode>(
        "/go2/set_mode", mode_callback("terminal"), rmw_qos_profile_services_default, control_group_);
    rc_mode_service_ = create_service<unitree_go2::srv::SetMode>(
        "/go2/set_mode_rc", mode_callback("sbus"), rmw_qos_profile_services_default, control_group_);
    web_mode_service_ = create_service<unitree_go2::srv::SetMode>(
        "/go2/set_mode_web", mode_callback("web"), rmw_qos_profile_services_default, control_group_);
    calibration_service_ = create_service<unitree_go2::srv::Calibration>(
        "/go2/calibration",
        [this](const std::shared_ptr<unitree_go2::srv::Calibration::Request> request,
            std::shared_ptr<unitree_go2::srv::Calibration::Response> response) {
            try {
                if (request->operation == "prepare") {
                    if (calibration_active_.exchange(true))
                        throw std::runtime_error("calibration is already active");
                    if (controller_->status().mode != Mode::kPassive) {
                        calibration_active_.store(false);
                        throw std::runtime_error("calibration requires passive mode");
                    }
                    hardware_->stop();
                    response->accepted = true;
                    response->message = "calibration hardware prepared";
                    return;
                }
                if (request->operation == "abort" || request->operation == "resume") {
                    if (!calibration_active_.load()) throw std::runtime_error("no active calibration");
                    hardware_->start();
                    calibration_active_.store(false);
                    response->accepted = true;
                    response->message = request->operation == "abort" ? "calibration aborted" : "calibration resumed";
                    return;
                }
                if (request->operation != "finish") throw std::runtime_error("unknown calibration operation");
                if (!calibration_active_.load()) throw std::runtime_error("no active calibration");
                if (request->joint_zero_offsets_rad.size() != kNumGo2Motors ||
                    request->startup_reference_rotor_positions_rad.size() != kNumGo2Motors)
                    throw std::runtime_error("calibration arrays must contain 12 values");
                std::array<float, kNumGo2Motors> offsets{}, rotors{};
                for (std::size_t i = 0; i < kNumGo2Motors; ++i) {
                    offsets[i] = request->joint_zero_offsets_rad[i];
                    rotors[i] = request->startup_reference_rotor_positions_rad[i];
                    if (!std::isfinite(offsets[i]) || !std::isfinite(rotors[i]))
                        throw std::runtime_error("calibration values must be finite");
                }
                hardware_->replace_calibration(offsets, rotors);
                hardware_->start();
                calibration_active_.store(false);
                response->accepted = true;
                response->message = "calibration applied";
            } catch (const std::exception& e) {
                response->accepted = false;
                response->message = e.what();
                RCLCPP_WARN(get_logger(), "标定请求失败：%s", e.what());
                if (request->operation == "prepare") calibration_active_.store(false);
            }
        },
        rmw_qos_profile_services_default, control_group_);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/go2/body_imu", rclcpp::SensorDataQoS());
    joints_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());
    backend_ = backend;
    status_pub_ = create_publisher<std_msgs::msg::String>("/go2/status", rclcpp::QoS(1).transient_local());
    sbus_alive_sub_ = create_subscription<std_msgs::msg::Bool>("/go2/sbus_alive", 1,
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
            sbus_alive_ = msg->data;
            sbus_report_at_ = SteadyClock::now();
        });
    web_alive_sub_ = create_subscription<std_msgs::msg::Bool>("/go2/web_alive", 1,
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
            web_alive_ = msg->data;
            web_report_at_ = SteadyClock::now();
        });
    white_light_service_ = create_service<std_srvs::srv::Trigger>("/go2/toggle_white_light",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            white_light_enabled_ = !white_light_enabled_;
            response->success = apply_lights();
            if (!response->success) {
                white_light_enabled_ = !white_light_enabled_;
                response->message = "white light hardware unavailable";
                return;
            }
            response->message = white_light_enabled_ ? "on" : "off";
            RCLCPP_INFO(get_logger(), "白灯：%s", white_light_enabled_ ? "开" : "关");
        });
    light_timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&Node::update_lights, this));
    control_timer_ =
        create_wall_timer(std::chrono::milliseconds(1), std::bind(&Node::control_tick, this), control_group_);
    policy_timer_ =
        create_wall_timer(std::chrono::milliseconds(20), std::bind(&Node::policy_tick, this), policy_group_);
    imu_timer_ = create_wall_timer(std::chrono::milliseconds(1), std::bind(&Node::publish_imu, this));
    joints_timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&Node::publish_joints, this));
    status_timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&Node::publish_status, this));
    hardware_->start();
    last_report_ = SteadyClock::now();
    RCLCPP_INFO(get_logger(), "Go2已启动：后端=%s，初始模式=被动，控制周期=1毫秒，策略周期=20毫秒", backend.c_str());
    if (h.backend == Backend::kMock) RCLCPP_WARN(get_logger(), "当前是模拟后端：不会打开真实电机和IMU串口");
}
Node::~Node() {
    if (hardware_) hardware_->stop();
}

bool Node::apply_lights() {
    if (!leds_)
        return backend_ == "mock";
    std::array<std::uint64_t, 4> duty{white_light_enabled_ ? 200000U : 0U, 0, 0, 0};
    const auto channel = status_light_ == StatusLight::kRed ? 2U :
        (status_light_ == StatusLight::kOrange ? 1U : 3U);
    duty[channel] = 5000000;
    const int result = led_set_duty_ns(leds_.get(), duty.data(), duty.size());
    if (result != 0)
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "LED 输出失败：%d", result);
    return result == 0;
}

void Node::update_lights() {
    const auto state = hardware_->read();
    const auto now = SteadyClock::now();
    const bool sbus = sbus_alive_ && seconds(now - sbus_report_at_) <= .30;
    const bool web = web_alive_ && seconds(now - web_report_at_) <= .30;
    const bool input = input_source_ == "none" || (rc_control_.load() ? sbus : web);
    const auto light = status_light_state(controller_->status().mode, state, now, input,
        light_imu_timeout_, light_motor_timeout_);
    status_light_ = light.color;
    light_fault_ = std::string(light.fault) == "teleop_no_data" ?
        (rc_control_.load() ? "sbus_no_data" : "web_no_data") : light.fault;
    (void)apply_lights();
}

void Node::update_input_control(TimePoint now) {
    const int slot = sbus_slot_.load();
    const bool rc = input_source_ == "sbus" || (input_source_ == "any" && slot > 0 && slot != 4 &&
        sbus_slot_at_ != TimePoint{} && seconds(now - sbus_slot_at_) <= sbus_connection_timeout_);
    // A receiver dropout/reconnect changes the manual endpoint, not the
    // navigation/AI velocity owner. Slot selection and accepted mode requests
    // explicitly update that ownership instead.
    if (rc_control_.exchange(rc) == rc) return;
    controller_->set_velocity({}, now);
    direct_velocity_pub_->publish(geometry_msgs::msg::Twist{});
    RCLCPP_INFO(get_logger(), "输入控制切换：%s", rc ? "遥控器" : "网页");
}

void Node::control_tick() {
    update_input_control(SteadyClock::now());
    if (calibration_active_.load(std::memory_order_relaxed)) return;
    // Take the hardware snapshot before sampling the evaluation time.  Serial
    // workers stamp feedback when a reply arrives; sampling `now` first can
    // make a perfectly fresh reply look as if it came from the future.
    const auto state = hardware_->read();
    const auto now = SteadyClock::now();
    if (last_control_ != TimePoint{}) {
        const double ms = seconds(now - last_control_) * 1000;
        period_sum_ms_ += ms;
        period_max_ms_ = std::max(period_max_ms_, ms);
        ++period_count_;
    }
    last_control_ = now;
    const auto commands = controller_->tick(state, now);
    hardware_->write(commands);
    ++debug_control_ticks_;
    if (motion_log_.is_open() && (++motion_log_tick_ % 5U) == 0U) {
        if (motion_log_start_ == TimePoint{}) motion_log_start_ = now;
        motion_log_ << std::fixed << std::setprecision(4) << seconds(now - motion_log_start_) << ','
                    << static_cast<int>(controller_->status().mode);
        for (const auto& command : commands) motion_log_ << ',' << command.position_rad;
        for (const auto& joint : state.motors.joints) motion_log_ << ',' << joint.feedback.position_rad;
        for (const auto& joint : state.motors.joints) motion_log_ << ',' << joint.feedback.velocity_rad_s;
        for (float value : state.imu.angular_velocity) motion_log_ << ',' << value;
        const auto gravity = projected_gravity(state.imu.quaternion);
        for (float value : gravity) motion_log_ << ',' << value;
        if (native_uart_debug_) {
            for (const auto& joint : state.motors.joints)
                motion_log_ << ',' << (joint.valid ? seconds(now - joint.received_at) * 1000 : -1);
            for (const auto& trace : state.motors.native_uart) {
                motion_log_ << ',' << trace.stage << ',' << trace.motor_id << ',' << trace.command_mode
                            << ',' << trace.stage_age_ms
                            << ',' << trace.last_transaction_ms << ',' << trace.last_result
                            << ',' << trace.transactions << ',' << trace.failures;
            }
            for (const auto& sent : state.motors.transmissions) {
                motion_log_ << ',' << (sent.sent_at == TimePoint{} ? -1 : seconds(now - sent.sent_at) * 1000)
                            << ',' << sent.protective_elapsed_sec << ',' << sent.protective_crouch_id
                            << ',' << sent.protective_final;
            }
        }
        motion_log_ << '\n';
    }
    const auto s = controller_->status();
    const bool native_protection_event = native_uart_debug_ &&
        ((s.protective_crouch && !last_native_protective_) ||
        (s.mode == Mode::kPassive && last_control_mode_ != Mode::kPassive));
    last_control_mode_ = s.mode;
    if (s.mode == Mode::kPassive) {
        navigation_mode_.store(false, std::memory_order_relaxed);
        ai_agent_mode_.store(false, std::memory_order_relaxed);
    }
    const std::string status =
        std::string(mode_name(s.mode)) + (s.protective_crouch ? " (crouching) " : " ") + debug_reason(s.reason);
    for (std::size_t leg = 0; leg < state.motors.leg_errors.size(); ++leg) {
        const auto& error = state.motors.leg_errors[leg];
        if (error == last_leg_errors_[leg]) continue;
        if (!error.empty()) {
            const auto& timing = state.motors.leg_timing[leg];
            RCLCPP_ERROR(get_logger(), "电机腿%s通信异常：%s（失败次数=%lu，连续失败=%lu，重连次数=%lu）", kGo2LegNames[leg],
                error.c_str(), static_cast<unsigned long>(timing.transaction_failures),
                static_cast<unsigned long>(timing.consecutive_failures),
                static_cast<unsigned long>(timing.reconnect_attempts));
        } else if (!last_leg_errors_[leg].empty()) {
            RCLCPP_INFO(get_logger(), "电机腿%s通信已恢复", kGo2LegNames[leg]);
        }
        last_leg_errors_[leg] = error;
    }
    if (status != last_status_) {
        const bool first_status = last_status_.empty();
        const bool status_log_due = first_status || native_protection_event || seconds(now - last_status_report_) >= 1.0;
        if (status_log_due) {
            RCLCPP_INFO(get_logger(), "控制状态：模式=%s，保护蹲下=%s，原因=%s", mode_display(s.mode),
                s.protective_crouch ? "是" : "否",
                (native_uart_debug_ ? s.reason : debug_reason(s.reason)).c_str());
            last_status_report_ = now;
        }
        last_status_ = status;
    }
    if (native_uart_debug_) {
        bool stalled = false;
        for (const auto& trace : state.motors.native_uart)
            stalled = stalled || (std::string_view(trace.stage) != "inactive" && trace.stage_age_ms >= 20);
        for (const auto& joint : state.motors.joints)
            stalled = stalled || (joint.valid && seconds(now - joint.received_at) >= .020);
        const double interval = stalled ? .10 : 1.0;
        if (native_protection_event || seconds(now - last_native_uart_report_) >= interval) {
            std::ostringstream report;
            report << std::fixed << std::setprecision(2);
            for (std::size_t leg = 0; leg < kNumGo2Legs; ++leg) {
                const auto& trace = state.motors.native_uart[leg];
                report << " | " << kGo2LegNames[leg] << '/' << native_uart_devices_[leg]
                        << " id=" << trace.motor_id << " mode=" << trace.command_mode << " stage=" << trace.stage
                        << " stage_age=" << trace.stage_age_ms << "ms sdk=" << trace.last_transaction_ms
                        << "ms ret=" << trace.last_result
                        << " calls=" << trace.transactions << " failures=" << trace.failures << " ages=";
                for (std::size_t id = 0; id < kMotorsPerLeg; ++id) {
                    const auto& joint = state.motors.joints[leg * kMotorsPerLeg + id];
                    if (id) report << '/';
                    report << (joint.valid ? seconds(now - joint.received_at) * 1000 : -1);
                }
            }
            report << " | mode=" << mode_name(s.mode) << " protective=" << s.protective_crouch
                    << " reason=" << s.reason << " control_max=" << period_max_ms_ << "ms";
            if (stalled || native_protection_event)
                RCLCPP_WARN(get_logger(), "原生串口诊断：%s", report.str().c_str());
            else
                RCLCPP_INFO(get_logger(), "原生串口诊断：%s", report.str().c_str());
            if (native_protection_event && motion_log_.is_open())
                motion_log_.flush();
            last_native_uart_report_ = now;
        }
        last_native_protective_ = s.protective_crouch;
    }
    if (seconds(now - last_report_) >= 1.0) {
        std::size_t motor_valid = 0;
        std::size_t motor_anchored = 0;
        std::size_t foc_commands = 0;
        for (const auto& joint : state.motors.joints) {
            motor_valid += joint.valid ? 1U : 0U;
            motor_anchored += joint.valid && joint.feedback.position_anchored ? 1U : 0U;
        }
        for (const auto& command : commands) foc_commands += command.mode == MotorMode::kFoc ? 1U : 0U;
        const bool imu_received = state.imu.valid && state.imu.received_at != TimePoint{};
        const bool imu_fresh =
            imu_received && now >= state.imu.received_at && seconds(now - state.imu.received_at) <= 0.30;
        const auto status_snapshot = controller_->status();
        RCLCPP_INFO(get_logger(),
            "调试摘要：遥控器状态见go2_sbus；IMU数据=%s（新鲜=%s）；电机反馈=%s（%zu/12）；"
            "初始启动范围=%s（%zu/12）；电机命令=%s（FOC=%zu/12）；模式=%s；"
            "策略请求/完成/失败=%lu/%lu/%lu；推理耗时=%.2f毫秒",
            imu_received ? "有" : "无", imu_fresh ? "是" : "否", motor_valid == kNumGo2Motors ? "齐全" : "不全",
            motor_valid, motor_anchored == kNumGo2Motors ? "已通过" : "未通过", motor_anchored,
            foc_commands > 0 ? "已发送" : "未发送", foc_commands, mode_display(status_snapshot.mode),
            static_cast<unsigned long>(debug_policy_requests_.exchange(0, std::memory_order_relaxed)),
            static_cast<unsigned long>(debug_policy_completed_.exchange(0, std::memory_order_relaxed)),
            static_cast<unsigned long>(debug_policy_failed_.exchange(0, std::memory_order_relaxed)), inference_ms_.load());
        last_report_ = now;
        period_sum_ms_ = period_max_ms_ = 0;
        period_count_ = 0;
    }
}
void Node::policy_tick() {
    if (calibration_active_.load(std::memory_order_relaxed)) return;
    const auto state = hardware_->read();
    const auto begin = SteadyClock::now();
    const auto request = controller_->prepare_policy(state, begin);
    if (!request) return;
    debug_policy_requests_.fetch_add(1, std::memory_order_relaxed);
    try {
        const Action action = policy_->infer(
            request->mode, request->observation, &request->state, request->policy_frame);

        const auto end = SteadyClock::now();
        inference_ms_.store(seconds(end - begin) * 1000);
        controller_->complete_policy(*request, action, end);
        debug_policy_completed_.fetch_add(1, std::memory_order_relaxed);
    } catch (const std::exception& e) {
        debug_policy_failed_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_ERROR(get_logger(), "策略推理失败（模式=%s）：%s", mode_display(request->mode), e.what());
        controller_->fail_policy(*request, e.what());
    }
}
void Node::publish_imu() {
    if (calibration_active_.load(std::memory_order_relaxed)) return;
    const auto imu = hardware_->read().imu;
    if (!imu.valid || imu.sequence == imu_sequence_) return;
    imu_sequence_ = imu.sequence;
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = imu_frame_;
    msg.orientation.w = imu.quaternion[0];
    msg.orientation.x = imu.quaternion[1];
    msg.orientation.y = imu.quaternion[2];
    msg.orientation.z = imu.quaternion[3];
    msg.angular_velocity.x = imu.angular_velocity[0];
    msg.angular_velocity.y = imu.angular_velocity[1];
    msg.angular_velocity.z = imu.angular_velocity[2];
    msg.linear_acceleration.x = imu.linear_acceleration[0];
    msg.linear_acceleration.y = imu.linear_acceleration[1];
    msg.linear_acceleration.z = imu.linear_acceleration[2];
    for (const auto i : {0, 4, 8}) {
        msg.orientation_covariance[i] = orientation_covariance_;
        msg.angular_velocity_covariance[i] = angular_covariance_;
        msg.linear_acceleration_covariance[i] = acceleration_covariance_;
    }
    imu_pub_->publish(msg);
}
void Node::publish_joints() {
    if (calibration_active_.load(std::memory_order_relaxed)) return;
    const auto state = hardware_->read();
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        const auto& j = state.motors.joints[i];
        if (!j.valid || !j.feedback.position_anchored) return;
        msg.name.emplace_back(kJointNames[i]);
        msg.position.push_back(j.feedback.position_rad);
        msg.velocity.push_back(j.feedback.velocity_rad_s);
        msg.effort.push_back(j.feedback.torque_nm);
    }
    joints_pub_->publish(msg);
}

void Node::publish_status() {
    const auto state = hardware_->read();
    const auto sampled_at = SteadyClock::now();
    const auto status = controller_->status();
    const bool sbus_connected = sbus_alive_ && seconds(sampled_at - sbus_report_at_) <= .30;
    // Serialize outside the 1 ms control callback. JSON handles arbitrary
    // fault strings and lets consumers read named fields without parsing logs.
    nlohmann::json data = {
        {"robot", robot_},
        {"ros_domain_id", get_node_base_interface()->get_context()->get_domain_id()},
        {"board_profile", board_profile_}, {"backend", backend_},
        {"mode", mode_name(status.mode)}, {"protective_crouch", status.protective_crouch},
        {"reason", status.reason},
        {"rollover_segments", status.rollover_segments},
        {"rollover_step", status.rollover_step}, {"rollover_step_running", status.rollover_step_running},
        {"navigation_enabled", navigation_mode_.load(std::memory_order_relaxed)},
        {"ai_agent_enabled", ai_agent_mode_.load(std::memory_order_relaxed)},
        {"calibration_active", calibration_active_.load(std::memory_order_relaxed)},
        {"white_light_enabled", white_light_enabled_},
        {"white_light_available", leds_ != nullptr || backend_ == "mock"},
        {"status_light", status_light_ == StatusLight::kRed ? "red" :
            (status_light_ == StatusLight::kOrange ? "orange" : "green")},
        {"light_fault", light_fault_}, {"teleop_source", input_source_},
        {"control_source", input_source_ == "none" ? "terminal" : (rc_control_.load() ? "sbus" : "web")},
        {"sbus_slot", sbus_connected ? sbus_slot_.load() : 0}, {"sbus_connected", sbus_connected},
        {"startup_pose_invalid", state.motors.startup_pose_invalid},
        {"imu_valid", state.imu.valid},
        {"imu_age_ms", state.imu.valid ? std::max(0.0, seconds(sampled_at - state.imu.received_at) * 1000) : -1.0},
        {"joints", nlohmann::json::array()}, {"leg_errors", state.motors.leg_errors}};
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        const auto& joint = state.motors.joints[i];
        data["joints"].push_back({
            {"name", kJointNames[i]}, {"valid", joint.valid},
            {"position_anchored", joint.feedback.position_anchored},
            {"feedback_age_ms", joint.valid ? std::max(0.0, seconds(sampled_at - joint.received_at) * 1000) : -1.0}});
    }
    std_msgs::msg::String message;
    message.data = data.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace);
    status_pub_->publish(message);
}
}  // namespace go2
