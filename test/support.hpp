/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SUPPORT_HPP
#define SUPPORT_HPP
#include "policy_adapter.hpp"
#include "controller.hpp"
#include <cassert>
#include <cmath>
#include <stdexcept>

inline go2::TimePoint at(double sec) {
    return go2::TimePoint{} +
        std::chrono::duration_cast<go2::SteadyClock::duration>(std::chrono::duration<double>(sec));
}
inline go2::RobotState fresh(double sec, const go2::Joints& pose = go2::kStandPose) {
    go2::RobotState state;
    for (std::size_t i = 0; i < go2::kNumJoints; ++i) {
        auto& j = state.motors.joints[i];
        j.valid = j.feedback.position_anchored = true;
        j.received_at = at(sec);
        j.feedback.position_rad = pose[i];
        j.feedback.temperature_c = 30;
    }
    state.imu.valid = true;
    state.imu.received_at = at(sec);
    return state;
}
inline void near(float a, float b, float tolerance = 1e-5F) { assert(std::fabs(a - b) < tolerance); }
template <class F>
void rejects(F function) {
    bool failed = false;
    try {
        function();
    } catch (const std::exception&) {
        failed = true;
    }
    assert(failed);
}
inline void brake(const go2::Commands& commands) {
    for (const auto& c : commands) assert(c.mode == go2::MotorMode::kBrake);
}

inline std::shared_ptr<const go2::PolicyAdapter> test_adapter_ptr() {
    static const auto adapter = std::make_shared<go2::PolicyAdapter>(
        std::string(GO2_TEST_APP) + "/config/policies.yaml", std::string(GO2_TEST_MODEL_DIR));
    return adapter;
}
inline const go2::PolicyAdapter& test_adapter() { return *test_adapter_ptr(); }
inline go2::ControllerConfig test_controller_config() {
    go2::ControllerConfig config;
    config.policy_adapter = test_adapter_ptr();
    config.sit_reference = go2::load_sit_reference(std::string(GO2_TEST_APP) + "/resources/motions/go2_sit.motion");
    return config;
}

// Policy contract references, deliberately independent of runtime YAML.
namespace go2 {
constexpr Joints kIsaacGymDefaultJointPos{.1F, .8F, -1.5F, -.1F, .8F, -1.5F, .1F, 1.F, -1.5F, -.1F, 1.F, -1.5F};
}  // namespace go2

#endif  // SUPPORT_HPP
