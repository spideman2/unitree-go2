/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <rc_receiver.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <future>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <string>
#include <unitree_go2/srv/set_mode.hpp>

namespace {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
constexpr std::uint8_t kGo2EffectiveChannelCount = 12U;
constexpr std::uint32_t kSbusChannelValueMax = 2047U;

// Protocol-specific ex_args consumed by drv_uart_sbus.
struct SbusReceiverConfig {
    std::uint8_t accept_sbus2_footer;
};

template <typename T>
T parameter(rclcpp::Node& node, const char* name, T value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return node.declare_parameter<T>(name, value, descriptor);
}

int channel_parameter(rclcpp::Node& node, const char* name, int value) {
    const int channel = parameter(node, name, value);
    if (channel < 1 || channel > static_cast<int>(kGo2EffectiveChannelCount)) {
        throw std::invalid_argument(std::string(name) + " must be an effective channel in [1,12]");
    }
    return channel;
}

double seconds_since(TimePoint then, TimePoint now) { return std::chrono::duration<double>(now - then).count(); }

enum class SwitchPosition { kLow, kMiddle, kHigh };

const char* switch_mode_display(const std::string& mode) {
    if (mode == "passive") return "正常休眠";
    if (mode == "passive_direct") return "紧急制动";
    if (mode == "stand") return "站立";
    if (mode == "crouch") return "蹲下";
    if (mode == "sit") return "坐下";
    if (mode == "forward_jump") return "前跳";
    if (mode == "walk") return "平地行走";
    if (mode == "wave") return "招手";
    if (mode == "navigation") return "导航";
    if (mode == "ai_agent") return "AI模式";
    if (mode == "terrain") return "综合地形";
    return mode.c_str();
}

// Each policy is driven with the command limits used during its training.
// Terrain policy (CTS comprehensive-terrain export): deployed physical envelope
// in m/s and rad/s, i.e. exactly what /cmd_vel_rc carries.  Training sampled
// vx=[-2,2] m/s, vy=[-1,1] m/s, yaw=[-2,2] rad/s; translation is deployed at
// half of that, yaw at the full trained range.
constexpr double kV6TerrainForwardMps = 1.00;
constexpr double kV6TerrainLateralMps = 0.50;
constexpr double kV6TerrainYawRadS = 2.00;
constexpr double kActionForwardMps = 0.35;
constexpr double kActionLateralMps = 0.25;
constexpr double kActionYawRadS = 0.50;
// Jump actor: the historical teleop envelope used +/-1.0 rad/s; large live yaw
// makes the hopping gait fight its own heading, so it is deployed at half.

// UART worker only copies frames. ROS executor owns all control decisions.
class SbusFrameEvent final : public rclcpp::Waitable {
    public:
    struct Sample {
        rc_receiver_frame_t frame;
        TimePoint received;
    };
    struct Batch {
        std::array<Sample, 64> samples{};
        std::size_t count{0};
        bool overflow{false};
    };
    explicit SbusFrameEvent(std::function<void(const Batch&)> handler) : handler_(std::move(handler)) {}

    void push(const rc_receiver_frame_t& frame) noexcept {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (pending_.count == pending_.samples.size()) pending_.overflow = true;
            if (!pending_.overflow) pending_.samples[pending_.count++] = {frame, Clock::now()};
        }
        try {
            guard_.trigger();
        } catch (...) { /* Shutdown can invalidate the ROS context. */
        }
    }
    size_t get_number_of_ready_guard_conditions() override { return 1; }
    void add_to_wait_set(rcl_wait_set_t* set) override {
        if (rcl_wait_set_add_guard_condition(set, &guard_.get_rcl_guard_condition(), &index_) != RCL_RET_OK)
            throw std::runtime_error("failed to add SBUS guard condition");
    }
    bool is_ready(rcl_wait_set_t* set) override {
        return index_ < set->size_of_guard_conditions && set->guard_conditions[index_] != nullptr;
    }
    std::shared_ptr<void> take_data() override {
        auto batch = std::make_shared<Batch>();
        std::lock_guard<std::mutex> lock(mutex_);
        *batch = pending_;
        pending_ = {};
        return batch;
    }
    void execute(std::shared_ptr<void>& data) override { handler_(*std::static_pointer_cast<Batch>(data)); }

    private:
    rclcpp::GuardCondition guard_;
    std::size_t index_{0};
    std::mutex mutex_;
    Batch pending_;
    std::function<void(const Batch&)> handler_;
};

class SbusTeleopNode final : public rclcpp::Node {
    public:
    SbusTeleopNode() : SbusTeleopNode(true) {}

    private:
    friend struct SbusTeleopTestAccess;
    explicit SbusTeleopNode(bool start_receiver) : rclcpp::Node("go2_sbus") {
        device_ = parameter(*this, "device", std::string("/dev/ttyS5"));
        if (device_.empty()) throw std::invalid_argument("device must not be empty");
        accept_sbus2_footer_ = parameter(*this, "accept_sbus2_footer", true);

        forward_channel_ = channel_parameter(*this, "forward_channel", 3);  // CH3, left stick vertical
        lateral_channel_ = channel_parameter(*this, "lateral_channel", 4);  // CH4, left stick horizontal
        yaw_channel_ = channel_parameter(*this, "yaw_channel", 1);          // CH1, right stick horizontal
        e_channel_ = channel_parameter(*this, "e_channel", 5);              // CH5
        f_channel_ = channel_parameter(*this, "f_channel", 6);              // CH6
        b_channel_ = channel_parameter(*this, "b_channel", 8);              // CH8
        c_channel_ = channel_parameter(*this, "c_channel", 9);              // CH9
        d_channel_ = channel_parameter(*this, "d_channel", 10);             // CH10
        g_channel_ = channel_parameter(*this, "g_channel", 11);             // CH11
        h_channel_ = channel_parameter(*this, "h_channel", 12);             // CH12

        channel_min_ = parameter(*this, "channel_min", 282);
        channel_center_ = parameter(*this, "channel_center", 1002);
        channel_max_ = parameter(*this, "channel_max", 1722);
        switch_low_ = parameter(*this, "switch_low", 700);
        switch_high_ = parameter(*this, "switch_high", 1300);
        wheel_extreme_margin_ = parameter(*this, "wheel_extreme_margin", 100);
        deadband_ = parameter(*this, "deadband", 0.01);
        frame_timeout_ = parameter(*this, "frame_timeout_sec", 0.25);
        shared_control_ = parameter(*this, "shared_control", false);
        connection_timeout_ = parameter(*this, "connection_timeout_sec", 1.0);
        edge_debounce_ = parameter(*this, "edge_debounce_sec", 0.08);
        // CH3 is oriented opposite to CH2: 1722 is left-stick up, so no
        // inversion is needed for positive linear.x = forward.
        invert_forward_ = parameter(*this, "invert_forward", false);
        invert_lateral_ = parameter(*this, "invert_lateral", true);
        invert_yaw_ = parameter(*this, "invert_yaw", true);

        if (channel_min_ < 0 || channel_center_ <= channel_min_ || channel_max_ <= channel_center_ ||
            channel_max_ > 2047 || switch_low_ < channel_min_ || switch_low_ >= switch_high_ ||
            switch_high_ > channel_max_ || wheel_extreme_margin_ < 0 ||
            2 * wheel_extreme_margin_ >= channel_max_ - channel_min_ || !std::isfinite(deadband_) || deadband_ < 0.0 ||
            deadband_ >= 0.5 || !std::isfinite(frame_timeout_) || frame_timeout_ <= 0.05 || frame_timeout_ > 1.0 ||
            !std::isfinite(connection_timeout_) || connection_timeout_ < frame_timeout_ || connection_timeout_ > 10.0 ||
            !std::isfinite(edge_debounce_) || edge_debounce_ < 0.0 || edge_debounce_ > 1.0) {
            throw std::invalid_argument("invalid SBUS configuration");
        }

        if (start_receiver) {
            SbusReceiverConfig receiver_config{static_cast<std::uint8_t>(accept_sbus2_footer_ ? 1U : 0U)};
            receiver_.reset(rc_receiver_alloc_uart("sbus:go2", device_.c_str(), 0U, &receiver_config));
            if (receiver_ == nullptr) {
                throw std::runtime_error("failed to allocate RC receiver");
            }
            if (rc_receiver_init(receiver_.get()) != 0) {
                throw std::runtime_error("failed to initialize RC receiver");
            }
        }

        velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_rc", 10);
        alive_pub_ = create_publisher<std_msgs::msg::Bool>("/go2/sbus_alive", 10);
        slot_pub_ = create_publisher<std_msgs::msg::Int32>("/go2/sbus_slot", 1);
        light_client_ = create_client<std_srvs::srv::Trigger>("/go2/toggle_white_light");
        mode_client_ = create_client<unitree_go2::srv::SetMode>(
            shared_control_ ? "/go2/set_mode_rc" : "/go2/set_mode");
        frame_event_ = std::make_shared<SbusFrameEvent>([this](const SbusFrameEvent::Batch& batch) {
            if (batch.overflow) {
                have_frame_ = false;
                g_hold_started_ = {};
                h_hold_started_ = {};
                publish_zero();
                select_mode("passive_direct");
                RCLCPP_ERROR(get_logger(), "SBUS event queue overflow; discarded batch and requested passive");
                return;
            }
            for (std::size_t i = 0; i < batch.count; ++i) {
                const auto& sample = batch.samples[i];
                if (seconds_since(sample.received, Clock::now()) <= frame_timeout_)
                    handle_frame(sample.frame, sample.received);
            }
        });
        get_node_waitables_interface()->add_waitable(frame_event_, nullptr);
        publish_timer_ =
            create_wall_timer(std::chrono::milliseconds(20), std::bind(&SbusTeleopNode::publish_command, this));

        RCLCPP_INFO(get_logger(), "遥控器节点已启动：设备=%s；等待有效SBUS帧；初始模式=被动", device_.c_str());
        if (start_receiver && rc_receiver_set_callback(
                receiver_.get(),
                [](rc_receiver_t*, const rc_receiver_frame_t* frame, void* context) {
                    static_cast<SbusFrameEvent*>(context)->push(*frame);
                },
                frame_event_.get()) != 0) {
            throw std::runtime_error("failed to start SBUS UART event callback: " +
                                        std::to_string(rc_receiver_last_error(receiver_.get())));
        }
    }

    public:
    ~SbusTeleopNode() override {
        receiver_.reset();
        get_node_waitables_interface()->remove_waitable(frame_event_, nullptr);
        if (velocity_pub_) publish_zero();
    }

    private:
    std::uint16_t channel_value(const rc_receiver_frame_t& frame, int channel) const noexcept {
        const auto normalized = static_cast<std::uint32_t>(frame.channels[static_cast<std::size_t>(channel - 1)]);
        // Preserve the existing Go2 calibration and launch parameters, which are
        // expressed in native 11-bit SBUS units, while the generic receiver API
        // exposes protocol-neutral 0..65535 values.
        return static_cast<std::uint16_t>((normalized * kSbusChannelValueMax + RC_RECEIVER_CHANNEL_MAX / 2U) /
                                            RC_RECEIVER_CHANNEL_MAX);
    }

    SwitchPosition switch_position(const rc_receiver_frame_t& frame, int channel) const noexcept {
        const int value = static_cast<int>(channel_value(frame, channel));
        if (value <= switch_low_) return SwitchPosition::kLow;
        if (value >= switch_high_) return SwitchPosition::kHigh;
        return SwitchPosition::kMiddle;
    }

    bool switch_high(const rc_receiver_frame_t& frame, int channel) const noexcept {
        return switch_position(frame, channel) == SwitchPosition::kHigh;
    }

    SwitchPosition wheel_position(const rc_receiver_frame_t& frame, int channel) const noexcept {
        const int value = static_cast<int>(channel_value(frame, channel));
        if (value <= channel_min_ + wheel_extreme_margin_) return SwitchPosition::kLow;
        if (value >= channel_max_ - wheel_extreme_margin_) return SwitchPosition::kHigh;
        return SwitchPosition::kMiddle;
    }

    float axis(const rc_receiver_frame_t& frame, int channel, bool invert) const noexcept {
        double value = raw_axis(frame, channel, invert);
        const double magnitude = std::abs(value);
        if (magnitude <= deadband_) return 0.0F;
        value = std::copysign((magnitude - deadband_) / (1.0 - deadband_), value);
        return static_cast<float>(value);
    }

    float raw_axis(const rc_receiver_frame_t& frame, int channel, bool invert) const noexcept {
        const int raw = static_cast<int>(channel_value(frame, channel));
        double value =
            raw >= channel_center_
                ? static_cast<double>(raw - channel_center_) / static_cast<double>(channel_max_ - channel_center_)
                : static_cast<double>(raw - channel_center_) / static_cast<double>(channel_center_ - channel_min_);
        value = std::clamp(value, -1.0, 1.0);
        if (invert) value = -value;
        return static_cast<float>(value);
    }

    int selected_slot(const rc_receiver_frame_t& frame) const {
        const auto b = switch_high(frame, b_channel_);
        const auto f = switch_position(frame, f_channel_);
        const auto e = switch_position(frame, e_channel_);

        const int group = f == SwitchPosition::kLow ? 0 : f == SwitchPosition::kMiddle ? 1 : 2;
        const int position = e == SwitchPosition::kLow ? 0 : e == SwitchPosition::kMiddle ? 1 : 2;
        return (b ? 9 : 0) + group * 3 + position + 1;
    }

    std::string mode_for_slot(int slot) const {
        switch (slot) {
            case 1:
                return "terrain";
            case 2:
                return "wave";
            case 3:
                return "sit";
            case 4:
                return "web";
            case 5:
                return "navigation";
            case 6:
                return "ai_agent";
            case 7:
                return "forward_jump";
            case 9:
                return "walk";
            default:
                return "";
        }
    }

    bool is_joystick_mode(const std::string& mode) const {
        return mode == "walk" || mode == "terrain";
    }

    bool button_edge(bool current, bool& previous, TimePoint& last_edge, TimePoint now) {
        if (!buttons_initialized_) {
            previous = current;
            return false;
        }
        if (current == previous) return false;
        previous = current;
        if (last_edge != TimePoint{} && seconds_since(last_edge, now) < edge_debounce_) {
            return false;
        }
        last_edge = now;
        return true;
    }

    bool request_mode(const std::string& mode, int slot) {
        const auto now = Clock::now();
        if (mode == pending_mode_ && last_mode_request_ != TimePoint{} && seconds_since(last_mode_request_, now) < 1.0)
            return false;
        const bool safety_pose = mode == "passive_direct" || mode == "passive" || mode == "stand" || mode == "crouch";
        if (!safety_pose && mode == last_sent_mode_ && last_mode_request_ != TimePoint{} &&
            seconds_since(last_mode_request_, now) < 1.0) {
            return false;
        }
        if (!safety_pose && last_mode_request_ != TimePoint{} && seconds_since(last_mode_request_, now) < 0.2) {
            return false;
        }
        if (!mode_client_->service_is_ready() && !mode_client_->wait_for_service(std::chrono::milliseconds(20))) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "模式服务不可用，遥控器请求未发送：%s",
                                    mode.c_str());
            return false;
        }
        last_mode_request_ = now;
        last_sent_mode_ = mode;
        const auto generation = ++mode_request_generation_;
        pending_mode_ = mode;
        auto request = std::make_shared<unitree_go2::srv::SetMode::Request>();
        request->mode = mode;
        mode_client_->async_send_request(
            request, [this, mode, slot, generation](rclcpp::Client<unitree_go2::srv::SetMode>::SharedFuture future) {
                try {
                    const auto response = future.get();
                    if (generation != mode_request_generation_) return;
                    pending_mode_.clear();
                    if (!response->accepted) {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "遥控器模式被拒绝：%s；原因：%s",
                                                switch_mode_display(mode), response->message.c_str());
                    } else {
                        current_mode_ = mode == "passive_direct" ? "passive" : mode;
                        current_slot_ = slot;
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "遥控器模式已接受：%s；说明：%s",
                                                switch_mode_display(mode), response->message.c_str());
                    }
                } catch (const std::exception& error) {
                    if (generation != mode_request_generation_) return;
                    pending_mode_.clear();
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "遥控器模式请求失败：%s；原因：%s",
                                            switch_mode_display(mode), error.what());
                }
            });
        return true;
    }

    bool select_mode(const std::string& requested_mode, int slot = 0) {
        std::string mode = requested_mode;
        // The controller may enter Passive on its own after an IMU or motor fault.
        // Always resend pose requests so a later button press can recover it.
        // External ownership can also change through slot 4 or another client.
        if (pending_mode_.empty() && mode == current_mode_ && mode != "rollover" && mode != "stand" &&
            mode != "crouch" && mode != "passive_direct" && mode != "passive" && mode != "forward_jump" &&
            mode != "sit" && mode != "navigation" && mode != "ai_agent")
            return true;
        if (!request_mode(mode, slot)) return false;
        pending_slot_ = 0;
        RCLCPP_INFO(get_logger(), "遥控器请求模式：%s", switch_mode_display(mode));
        return true;
    }

    bool handle_h(const rc_receiver_frame_t& frame, TimePoint now) {
        const auto position = wheel_position(frame, h_channel_);
        if (position != SwitchPosition::kLow) {
            h_hold_started_ = {};
            h_hold_fired_ = false;
            return false;
        }
        if (h_hold_fired_) return false;
        if (h_hold_started_ == TimePoint{}) h_hold_started_ = now;
        if (seconds_since(h_hold_started_, now) < 0.5) return false;
        h_hold_fired_ = true;
        if (current_mode_ == "passive" || current_mode_ == "crouch") return select_mode("rollover");
        return false;
    }

    bool handle_g(const rc_receiver_frame_t& frame, TimePoint now) {
        const auto position = wheel_position(frame, g_channel_);
        if (position != g_position_) {
            g_position_ = position;
            g_hold_started_ = now;
            g_hold_fired_ = false;
        }
        if (position == SwitchPosition::kMiddle || g_hold_fired_) return false;
        if (g_hold_started_ == TimePoint{}) g_hold_started_ = now;
        if (seconds_since(g_hold_started_, now) < 0.5) return false;
        g_hold_fired_ = true;
        select_mode(position == SwitchPosition::kHigh ? "stand" : "crouch");
        return true;
    }

    void handle_c(const rc_receiver_frame_t& frame) {
        const int slot = selected_slot(frame);
        const auto mode = mode_for_slot(slot);
        if (mode.empty()) {
            RCLCPP_INFO(get_logger(), "槽位 %d 暂空，保持当前模式", slot);
            return;
        }
        select_mode(mode, slot);
    }

    void handle_frame(const rc_receiver_frame_t& frame, TimePoint received) {
        if (frame.channel_count < kGo2EffectiveChannelCount) {
            g_hold_started_ = {};
            h_hold_started_ = {};
            ++invalid_frame_count_;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "遥控器帧无效：通道数不足（需要至少%u）",
                                    static_cast<unsigned>(kGo2EffectiveChannelCount));
            return;
        }
        latest_frame_ = frame;
        have_frame_ = true;

        // A receiver frame marked lost/failsafe is not a valid heartbeat.  Keep
        // `last_frame_` pinned to the last valid frame so a continuous stream of
        // failsafe frames still reaches the normal frame-timeout passive request.
        // The command publisher also sees the flags above and sends zero velocity
        // while the timeout window is running.
        if ((frame.flags & (RC_RECEIVER_FLAG_FRAME_LOST | RC_RECEIVER_FLAG_FAILSAFE)) != 0U) {
            g_hold_started_ = {};
            h_hold_started_ = {};
            ++lost_frame_count_;
            return;
        }

        ++valid_frame_count_;
        valid_frame_received_ = true;
        if (last_frame_ != TimePoint{} && seconds_since(last_frame_, Clock::now()) > frame_timeout_) {
            g_hold_started_ = {};
            h_hold_started_ = {};
        }
        last_frame_ = received;
        input_slot_ = selected_slot(frame);
        publish_slot(received);
        passive_requested_ = false;

        if (!buttons_initialized_) {
            previous_c_ = switch_high(frame, c_channel_);
            previous_d_ = switch_high(frame, d_channel_);
            buttons_initialized_ = true;
        }

        const auto now = received;
        const bool c_edge = button_edge(switch_high(frame, c_channel_), previous_c_, last_c_edge_, now);
        const bool d_edge = button_edge(switch_high(frame, d_channel_), previous_d_, last_d_edge_, now);
        const bool h_high = wheel_position(frame, h_channel_) == SwitchPosition::kHigh;
        // Emergency stop remains available when slot 4 hands control to the web.
        if (d_edge && h_high) {
            g_position_ = wheel_position(frame, g_channel_);
            g_hold_fired_ = true;
            h_hold_fired_ = false;
            select_mode("passive_direct");
            return;
        }
        if (c_edge && h_high) {
            if (light_client_->service_is_ready()) {
                light_client_->async_send_request(
                    std::make_shared<std_srvs::srv::Trigger::Request>(),
                    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                        const auto response = future.get();
                        if (response->success)
                            RCLCPP_INFO(get_logger(), "白灯已%s", response->message == "on" ? "打开" : "关闭");
                        else
                            RCLCPP_WARN(get_logger(), "白灯切换失败：%s", response->message.c_str());
                    });
            } else
                RCLCPP_WARN(get_logger(), "白灯服务尚未就绪");
        }
        if (input_slot_ == 4) {
            g_hold_started_ = {};
            h_hold_started_ = {};
            g_position_ = wheel_position(frame, g_channel_);
            g_hold_fired_ = false;
            h_hold_fired_ = false;
            return;
        }
        // Safety/pose control wins over model selection if a receiver sends
        // several switch changes in the same SBUS frame.
        if (d_edge) {
            g_position_ = wheel_position(frame, g_channel_);
            g_hold_fired_ = true;
            h_hold_fired_ = wheel_position(frame, h_channel_) == SwitchPosition::kLow;
            select_mode("passive");
        } else if (handle_g(frame, now)) {
            // A completed pose hold takes priority over C in the same frame.
        } else if (c_edge && !h_high) {
            h_hold_fired_ = wheel_position(frame, h_channel_) == SwitchPosition::kLow;
            handle_c(frame);
        } else {
            handle_h(frame, now);
        }
    }

    void check_receiver() {
        const auto now = Clock::now();
        if (last_debug_report_ == TimePoint{} || seconds_since(last_debug_report_, now) >= 1.0) {
            const bool frame_fresh = valid_frame_received_ && seconds_since(last_frame_, now) <= frame_timeout_;
            RCLCPP_INFO(get_logger(),
                        "遥控器调试：有效帧=%s；当前帧新鲜=%s；"
                        "有效/丢失/无效=%lu/%lu/%lu；模式=%s",
                        valid_frame_received_ ? "有" : "无", frame_fresh ? "是" : "否",
                        static_cast<unsigned long>(valid_frame_count_), static_cast<unsigned long>(lost_frame_count_),
                        static_cast<unsigned long>(invalid_frame_count_), switch_mode_display(current_mode_));
            last_debug_report_ = now;
            valid_frame_count_ = lost_frame_count_ = invalid_frame_count_ = 0;
        }
        // In shared mode, a sustained loss hands ownership to the web. Brief
        // gaps only zero stale sticks; they must not request passive or change owner.
        if (shared_control_) return;
        if (valid_frame_received_ && seconds_since(last_frame_, now) > frame_timeout_) {
            // Navigation owns an independent /cmd_vel stream.  Once selected, an
            // SBUS dropout must not stop a long-range autonomous mission; the
            // navigation producer is responsible for its own command watchdog.
            if (current_mode_ == "navigation") {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
                                        "遥控器帧超时，但当前是导航模式，继续由导航速度控制");
                return;
            }
            // Let one-shot action actors keep ownership through their motion when a
            // receiver dropout is caused by vibration; an explicit D-button request
            // still stops them when valid frames return.
            if (current_mode_ == "forward_jump" ||
                current_mode_ == "sit" || current_mode_ == "wave") {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "遥控器帧超时，但当前动作策略继续运行");
                return;
            }
            if (!passive_requested_ || current_mode_ != "passive") {
                select_mode("passive_direct");
                // Keep retrying if the mode service was temporarily unavailable.
                // An accepted response changes current_mode_ to passive.
                passive_requested_ = current_mode_ == "passive";
            }
        }
    }

    void publish_zero() {
        if (!velocity_pub_) return;
        geometry_msgs::msg::Twist message;
        velocity_pub_->publish(message);
    }

    bool publish_slot(TimePoint now) {
        const bool connected = valid_frame_received_ &&
            seconds_since(last_frame_, now) <= (shared_control_ ? connection_timeout_ : frame_timeout_);
        std_msgs::msg::Int32 slot;
        slot.data = connected ? input_slot_ : 0;
        slot_pub_->publish(slot);
        return connected;
    }

    void publish_command() {
        check_receiver();
        const bool fresh = valid_frame_received_ && seconds_since(last_frame_, Clock::now()) <= frame_timeout_;
        std_msgs::msg::Bool alive;
        alive.data = shared_control_ ? publish_slot(Clock::now()) : fresh;
        alive_pub_->publish(alive);
        // Keep publishing live sticks in navigation for guidance consumers.
        // go2_node gates /cmd_vel_rc forwarding, not this telemetry stream.
        if (input_slot_ == 4 || !pending_mode_.empty() || !have_frame_ ||
            seconds_since(last_frame_, Clock::now()) > frame_timeout_ ||
            (latest_frame_.flags & (RC_RECEIVER_FLAG_FRAME_LOST | RC_RECEIVER_FLAG_FAILSAFE)) != 0U) {
            publish_zero();
            return;
        }
        if (!is_joystick_mode(current_mode_) && current_mode_ != "navigation") {
            publish_zero();
            return;
        }
        double max_forward;
        double max_lateral;
        double max_yaw;
        if (current_mode_ == "terrain") {
            // The terrain export consumes a physical command; these are real m/s.
            max_forward = kV6TerrainForwardMps;
            max_lateral = kV6TerrainLateralMps;
            max_yaw = kV6TerrainYawRadS;
        } else if (current_mode_ == "walk" || current_mode_ == "navigation") {
            max_forward = 1.0;
            max_lateral = 0.4;
            max_yaw = 1.0;
        } else {
            // Other joystick actions keep the conservative action envelope.
            max_forward = kActionForwardMps;
            max_lateral = kActionLateralMps;
            max_yaw = kActionYawRadS;
        }
        geometry_msgs::msg::Twist message;
        float forward_axis = axis(latest_frame_, forward_channel_, invert_forward_);
        float lateral_axis = axis(latest_frame_, lateral_channel_, invert_lateral_);
        float yaw_axis = axis(latest_frame_, yaw_channel_, invert_yaw_);

        message.linear.x = max_forward * forward_axis;
        message.linear.y = max_lateral * lateral_axis;
        message.angular.z = max_yaw * yaw_axis;

        velocity_pub_->publish(message);
    }

    std::string device_;
    bool accept_sbus2_footer_{true};
    int forward_channel_{3}, lateral_channel_{4}, yaw_channel_{1};
    int e_channel_{5}, f_channel_{6}, b_channel_{8};
    int c_channel_{9}, d_channel_{10}, g_channel_{11}, h_channel_{12};
    int channel_min_{282}, channel_center_{1002}, channel_max_{1722};
    int switch_low_{700}, switch_high_{1300}, wheel_extreme_margin_{100};
    double deadband_{0.01};
    double frame_timeout_{0.25}, edge_debounce_{0.08}, connection_timeout_{1.0};
    bool shared_control_{false};
    int input_slot_{0};
    // 原厂 SportModeFsm::GetJoystick 对横移和偏航都取负号：vy=-CH4，yaw=-CH1。
    // 因此当前遥控器默认保持这两个反向映射；不要把入口站立摆动误判成 yaw 符号问题。
    bool invert_forward_{false}, invert_lateral_{true}, invert_yaw_{true};

    std::shared_ptr<SbusFrameEvent> frame_event_;
    std::unique_ptr<rc_receiver_t, decltype(&rc_receiver_free)> receiver_{nullptr, rc_receiver_free};
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alive_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr slot_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr light_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Client<unitree_go2::srv::SetMode>::SharedPtr mode_client_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    rc_receiver_frame_t latest_frame_{};
    bool have_frame_{false}, valid_frame_received_{false}, passive_requested_{false};
    bool buttons_initialized_{false};
    bool previous_c_{false}, previous_d_{false};
    int current_slot_{0};
    int pending_slot_{0};
    SwitchPosition g_position_{SwitchPosition::kMiddle};
    TimePoint g_hold_started_{};
    bool g_hold_fired_{false};
    TimePoint h_hold_started_{};
    bool h_hold_fired_{false};
    TimePoint last_frame_{};
    TimePoint last_c_edge_{}, last_d_edge_{};
    TimePoint last_mode_request_{};
    TimePoint last_debug_report_{};
    std::uint64_t valid_frame_count_{0}, lost_frame_count_{0};
    std::uint64_t invalid_frame_count_{0};
    std::string last_sent_mode_;
    std::string current_mode_{"passive"};
    std::uint64_t mode_request_generation_{0};
    std::string pending_mode_;

};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SbusTeleopNode>());
    } catch (const std::exception& error) {
        fprintf(stderr, "遥控器节点启动或运行失败：%s\n", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
