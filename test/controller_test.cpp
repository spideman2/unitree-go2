/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "controller.hpp"
#include "policy.hpp"
#include "support.hpp"
#include <algorithm>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

void stand(go2::Controller &c, double start = 1) {
    assert(c.set_mode(go2::Mode::kStand, fresh(start), at(start)).accepted);
    c.tick(fresh(start + 1), at(start + 1));
}
void terrain(go2::Controller &c) {
    stand(c);
    assert(c.set_mode(go2::Mode::kTerrain, fresh(2), at(2)).accepted);
}

int main() {
    using go2::Mode;
    {
        const auto tilted_state = [](double time, float roll, float pitch) {
            auto state = fresh(time);
            constexpr float radians = 3.14159265358979323846F / 180.F;
            const float cr = std::cos(roll * radians / 2.F), sr = std::sin(roll * radians / 2.F);
            const float cp = std::cos(pitch * radians / 2.F), sp = std::sin(pitch * radians / 2.F);
            state.imu.quaternion = {cr * cp, sr * cp, cr * sp, -sr * sp};
            state.imu.six_axis_quaternion = state.imu.quaternion;
            return state;
        };
        const auto start_mode = [](go2::Controller &controller, Mode mode) {
            stand(controller);
            assert(controller.set_mode(mode, fresh(2), at(2)).accepted);
            double time = 2.01;
            controller.tick(fresh(time), at(time));
            if (controller.status().mode == Mode::kStand) {
                time = 3.01;
                controller.tick(fresh(time), at(time));
            }
            assert(controller.status().mode == mode);
            return time + .01;
        };
        for (bool enabled : {false, true}) {
            auto config = test_controller_config();
            config.walking_tilt_protection = enabled;
            config.crouch_duration = .2;
            for (const auto mode : {Mode::kWalk, Mode::kTerrain}) {
                for (const auto angles : {std::array<float, 2>{41.F, 0.F}, {-41.F, 0.F},
                        {0.F, 61.F}, {0.F, -61.F}}) {
                    for (bool policy_first : {false, true}) {
                        go2::Controller controller(config);
                        const double time = start_mode(controller, mode);
                        const auto state = tilted_state(time, angles[0], angles[1]);
                        if (policy_first) {
                            const auto request = controller.prepare_policy(state, at(time));
                            assert(request.has_value() != enabled);
                        }
                        const auto commands = controller.tick(state, at(time));
                        assert(controller.status().protective_crouch == enabled);
                        if (enabled) {
                            assert(controller.status().reason.find("walking tilt exceeded") != std::string::npos);
                            for (const auto &command : commands) {
                                assert(command.mode == go2::MotorMode::kFoc && command.protective_crouch);
                            }
                            assert(!controller.prepare_policy(state, at(time)));
                            assert(!controller.set_mode(mode, state, at(time)).accepted);
                            brake(controller.tick(fresh(time + .21), at(time + .21)));
                            assert(controller.status().mode == Mode::kPassive);
                        } else {
                            assert(controller.status().mode == mode);
                        }
                    }
                }
            }
        }
        auto config = test_controller_config();
        config.walking_tilt_protection = true;
        for (const float sign : {-1.F, 1.F}) {
            go2::Controller controller(config);
            const double time = start_mode(controller, Mode::kTerrain);
            controller.tick(tilted_state(time, sign * 39.99F, sign * 59.99F), at(time));
            assert(!controller.status().protective_crouch);
        }
        for (const auto mode : {Mode::kStand, Mode::kCrouch,
                Mode::kForwardJump, Mode::kSit}) {
            go2::Controller controller(config);
            const double time = start_mode(controller, mode);
            const auto state = tilted_state(time, 41.F, 61.F);
            controller.prepare_policy(state, at(time));
            controller.tick(state, at(time));
            assert(!controller.status().protective_crouch);
        }
        go2::Controller passive_controller(config);
        const auto state = tilted_state(1, 75.F, 0.F);
        brake(passive_controller.tick(state, at(1)));
        assert(!passive_controller.status().protective_crouch);
        assert(passive_controller.set_mode(Mode::kRollover, state, at(1)).accepted);
        passive_controller.tick(state, at(1));
        assert(!passive_controller.status().protective_crouch);
    }
    {
        // Ordinary standing still enforces the configured limit on every joint.
        for (std::size_t index = 0; index < go2::kNumJoints; ++index) {
            for (const float sign : {-1.F, 1.F}) {
                go2::Controller c(test_controller_config());
                stand(c);
                auto state = fresh(2.01);
                state.motors.joints[index].feedback.velocity_rad_s = sign * 50.F;
                c.tick(state, at(2.01));
                assert(!c.status().protective_crouch);
                state = fresh(2.02);
                state.motors.joints[index].feedback.velocity_rad_s = sign * 51.F;
                assert(c.tick(state, at(2.02))[index].protective_crouch);
                assert(c.status().reason.find("velocity=") != std::string::npos);
            }
        }
    }
    {
        // Skill/running speed exemptions do not suppress other motor faults.
        auto config = test_controller_config();
        for (const auto mode : {Mode::kForwardJump, Mode::kSit, Mode::kWave}) {
            for (unsigned fault = 0; fault < 4; ++fault) {
                go2::Controller c(config);
                stand(c);
                assert(c.set_mode(mode, fresh(2), at(2)).accepted);
                double time = 2.01;
                c.tick(fresh(time), at(time));
                if (c.status().mode == Mode::kStand) {
                    time = 3.01;
                    c.tick(fresh(time), at(time));
                }
                assert(c.status().mode == mode);
                for (const float speed : {-60.F, 60.F}) {
                    time += .001;
                    auto state = fresh(time);
                    for (auto& joint : state.motors.joints) {
                        joint.feedback.velocity_rad_s = speed;
                    }
                    c.tick(state, at(time));
                    assert(c.status().mode == mode && !c.status().protective_crouch);
                }
                time += .001;
                auto state = fresh(time);
                auto& joint = state.motors.joints[1];
                switch (fault) {
                case 0:
                    joint.feedback.velocity_rad_s = std::numeric_limits<float>::infinity();
                    break;
                case 1:
                    joint.feedback.error = 1;
                    break;
                case 2:
                    joint.feedback.temperature_c = 78;
                    break;
                case 3:
                    joint.received_at = at(time - 1);
                    break;
                }
                const auto stopped = c.tick(state, at(time));
                assert(stopped[1].protective_crouch);
            }
        }
    }
    {
        using go2::StatusLight;
        auto state = fresh(2);
        for (auto mode : {Mode::kPassive, Mode::kStand, Mode::kCrouch, Mode::kRollover})
            assert(go2::status_light_state(mode, state, at(2), true).color == StatusLight::kGreen);
        for (auto mode : {Mode::kTerrain, Mode::kWave, Mode::kSit})
            assert(go2::status_light_state(mode, state, at(2), true).color == StatusLight::kOrange);
        // Repeated transaction errors and one missing motor do not turn the light red.
        state.motors.leg_errors[0] = "reply timeout";
        state.motors.joints[0].valid = false;
        assert(go2::status_light_state(Mode::kTerrain, state, at(2), true).color == StatusLight::kOrange);
        assert(go2::status_light_state(Mode::kTerrain, state, at(2), false).color == StatusLight::kRed);
        state.imu.valid = false;
        assert(std::string(go2::status_light_state(Mode::kStand, state, at(2), true).fault) == "imu_no_data");
        state = fresh(2);
        for (auto& joint : state.motors.joints) joint.received_at = at(1);
        assert(std::string(go2::status_light_state(Mode::kTerrain, state, at(2), true).fault) == "all_motors_offline");
        state = fresh(2);
        state.motors.startup_pose_invalid = true;
        assert(std::string(go2::status_light_state(Mode::kTerrain, state, at(2), true).fault) == "startup_pose_invalid");
    }
    {
        auto config = test_controller_config();
        config.crouch_duration = .2;
        go2::Controller c(config);
        stand(c);
        auto state = fresh(2.1);
        state.motors.transmissions_tracked = true;
        assert(c.set_mode(Mode::kPassive, state, at(2.1)).accepted);
        const auto initial = c.tick(state, at(2.1));
        const auto id = initial[0].protective_crouch_id;
        assert(id != 0);
        // No sends for 1.44s: do not consume the crouch or send BRAKE.
        const auto stalled = c.tick(state, at(3.54));
        for (unsigned i = 0; i < go2::kNumJoints; ++i) {
            assert(stalled[i].protective_crouch);
            near(stalled[i].position_rad, initial[i].position_rad);
            state.motors.transmissions[i] = {at(3.55), id, .1, false, true};
            state.motors.joints[i].received_at = at(1); // All reply frames may be missing.
        }
        state.motors.transmissions[8].protective_elapsed_sec = 0;
        near(c.tick(state, at(3.56))[0].position_rad, initial[0].position_rad);
        state.motors.transmissions[8].protective_elapsed_sec = .1;
        const auto midway = c.tick(state, at(3.57));
        for (unsigned i = 0; i < go2::kNumJoints; ++i)
            near(midway[i].position_rad, initial[i].position_rad * .5F + config.crouch_pose[i] * .5F);
        const auto interrupted = c.tick(state, at(5.01));
        near(interrupted[0].position_rad, midway[0].position_rad);
        for (auto &sent : state.motors.transmissions)
            sent.protective_elapsed_sec = .2;
        const auto final = c.tick(state, at(5.02));
        for (unsigned i = 0; i < go2::kNumJoints; ++i) {
            assert(final[i].mode == go2::MotorMode::kFoc && final[i].protective_crouch_final);
            near(final[i].position_rad, config.crouch_pose[i]);
            state.motors.transmissions[i].protective_final = true;
        }
        state.motors.transmissions[11].protective_final = false;
        assert(c.tick(state, at(5.03))[0].protective_crouch);
        state.motors.transmissions[11].protective_final = true;
        brake(c.tick(state, at(5.04)));
        assert(c.status().mode == Mode::kPassive);
        // A new protection episode cannot inherit the old completed sends.
        state = fresh(6);
        state.motors.transmissions_tracked = true;
        for (auto &sent : state.motors.transmissions)
            sent = {at(5.04), id, .2, true, true};
        assert(c.set_mode(Mode::kStand, state, at(6)).accepted);
        c.tick(state, at(6.01));
        assert(c.set_mode(Mode::kPassive, state, at(6.02)).accepted);
        assert(c.tick(state, at(8))[0].protective_crouch);
    }


    for (const auto mode : {Mode::kStand, Mode::kTerrain}) {
        go2::Controller c(test_controller_config());
        stand(c);
        assert(c.set_mode(mode, fresh(2), at(2)).accepted);
        assert(c.set_mode(Mode::kPassive, fresh(2.01), at(2.01), true).accepted);
        assert(c.status().protective_crouch);
        brake(c.tick(fresh(3.02), at(3.02)));
        assert(c.status().mode == Mode::kPassive);
    }

    for (bool native_uart : {false, true}) {
        auto config = test_controller_config();
        config.rr_uart6_degraded = native_uart;
        go2::Controller c(config);
        // No request: partial startup feedback and a persistent anchor fault must
        // never start FOC, even over several protective-crouch durations.
        auto waiting = fresh(1);
        waiting.motors.joints[0].valid = false;
        for (double t : {1.0, 1.1, 2.2, 3.4}) {
            brake(c.tick(waiting, at(t)));
            assert(c.status().mode == Mode::kPassive);
            assert(!c.status().protective_crouch);
        }
        waiting = fresh(4);
        waiting.motors.session_anchor_faulted = true;
        brake(c.tick(waiting, at(4)));
        brake(c.tick(fresh(5), at(5)));
        assert(c.status().mode == Mode::kPassive);
        assert(!c.status().protective_crouch);
    }
    {
        // On new_bigcore, a timed-out transaction on any UART keeps that motor's
        // last verified sample without forcing a protective crouch or dropping
        // the active stand command.
        auto config = test_controller_config();
        config.rr_uart6_degraded = true;
        config.rr_motor_soft_timeout = .01;
        config.rr_motor_hard_timeout = .20;
        go2::Controller c(config);
        stand(c);
        auto state = fresh(2.1);
        for (auto &joint : state.motors.joints)
            joint.received_at = at(2.0);
        const auto commands = c.tick(state, at(2.1));
        assert(commands[0].mode == go2::MotorMode::kFoc);
        assert(c.status().mode == Mode::kStand);
        assert(!c.status().protective_crouch);
    }
    for (bool native_uart : {false, true}) {
        // Failed transaction counts cannot override fresh per-motor samples.
        // Verify both native and USB paths still accept valid current feedback.
        auto config = test_controller_config();
        config.rr_uart6_degraded = native_uart;
        go2::Controller c(config);
        stand(c);
        auto state = fresh(2.1);
        state.motors.leg_timing[2].consecutive_failures = 100;
        const auto commands = c.tick(state, at(2.1));
        assert(commands[0].mode == go2::MotorMode::kFoc);
        assert(!c.status().protective_crouch);
        state.motors.joints[6].received_at = at(1.7);
        c.tick(state, at(2.1));
        assert(c.status().protective_crouch);
        brake(c.tick(state, at(3.11)));
        assert(c.status().mode == Mode::kPassive);
    }
    {
        go2::Controller c(test_controller_config());
        c.set_velocity({1, 0, 0}, at(1));
        brake(c.tick(fresh(1), at(1)));
        assert(!c.prepare_policy(fresh(1), at(1)));
        const auto queued = c.set_mode(Mode::kTerrain, fresh(1), at(1));
        assert(queued.accepted);
        assert(c.status().mode == Mode::kStand);
        auto bad = fresh(1);
        bad.imu.valid = false;
        assert(!c.set_mode(Mode::kStand, bad, at(1)).accepted);
        brake(c.tick(bad, at(1)));
        assert(!c.status().protective_crouch);
    }
    {
        go2::ControllerConfig config = test_controller_config();
        go2::Controller c(config);
        assert(c.set_mode(Mode::kStand, fresh(1, config.crouch_pose), at(1)).accepted);
        auto output = c.tick(fresh(1.5), at(1.5));
        for (int i = 0; i < 12; ++i) {
            near(output[i].position_rad, (config.crouch_pose[i] + go2::kStandPose[i]) * .5F);
            near(output[i].kp, go2::kStandKp[i]);
            near(output[i].kd, go2::kStandKd[i]);
        }
        assert(c.set_mode(Mode::kTerrain, fresh(1.5), at(1.5)).accepted);
        assert(c.status().mode == Mode::kStand);
        output = c.tick(fresh(2), at(2));
        for (int i = 0; i < 12; ++i)
            near(output[i].position_rad, go2::kStandPose[i]);
        assert(c.set_mode(Mode::kCrouch, fresh(2), at(2)).accepted);
        assert(!c.prepare_policy(fresh(2), at(2)));
        output = c.tick(fresh(4), at(4));
        for (int i = 0; i < 12; ++i)
            near(output[i].position_rad, config.crouch_pose[i]);
    }
    {
        // A model request from Passive first enters FixStand, then switches to
        // inference only after the one-second stand transition completes.
        go2::Controller c(test_controller_config());
        const auto queued = c.set_mode(Mode::kTerrain, fresh(1), at(1));
        assert(queued.accepted);
        assert(c.status().mode == Mode::kStand);
        c.tick(fresh(1.5), at(1.5));
        assert(c.status().mode == Mode::kStand);
        c.tick(fresh(2.01), at(2.01));
        assert(c.status().mode == Mode::kTerrain);
    }

    {
        // Only Passive -> Crouch may recover a finite, anchored and fresh
        // joint whose feedback position is outside the normal operating limit.
        go2::Controller c(test_controller_config());
        auto outside = fresh(1);
        outside.motors.joints[0].feedback.position_rad = 4.1F;
        const auto recovery = c.set_mode(Mode::kCrouch, outside, at(1));
        assert(recovery.accepted);
        assert(recovery.message == "crouch out-of-range recovery started");
        auto output = c.tick(outside, at(1.01));
        assert(output[0].mode == go2::MotorMode::kFoc);
        output = c.tick(fresh(2), at(2));
        assert(output[0].mode == go2::MotorMode::kFoc);

        go2::Controller stand_rejected(test_controller_config());
        assert(!stand_rejected.set_mode(Mode::kStand, outside, at(1)).accepted);

        for (int fault = 0; fault < 6; ++fault) {
            go2::Controller rejected(test_controller_config());
            auto unsafe = outside;
            switch (fault) {
            case 0:
                unsafe.motors.session_anchor_faulted = true;
                break;
            case 1:
                unsafe.motors.joints[0].valid = false;
                break;
            case 2:
                unsafe.motors.joints[0].feedback.position_anchored = false;
                break;
            case 3:
                unsafe.motors.joints[0].feedback.error = 1;
                break;
            case 4:
                unsafe.motors.joints[0].feedback.velocity_rad_s = 51;
                break;
            case 5:
                unsafe.motors.joints[0].feedback.temperature_c = 78;
                break;
            }
            assert(!rejected.set_mode(Mode::kCrouch, unsafe, at(1)).accepted);
        }

        go2::Controller stale(test_controller_config());
        assert(stale.set_mode(Mode::kCrouch, outside, at(1)).accepted);
        outside.motors.joints[0].received_at = at(.6);
        assert(stale.tick(outside, at(1.01))[0].protective_crouch);
        assert(stale.status().protective_crouch);
        brake(stale.tick(outside, at(2.02)));
    }
    {
        go2::Controller c(test_controller_config());
        terrain(c);
        c.set_velocity({.3F, .2F, .1F}, at(2));
        auto request = *c.prepare_policy(fresh(2), at(2));
        near(request.observation[42], 0);
        for (int i = 213; i < 225; ++i)
            near(request.observation[i], 0);
        go2::Action action{};
        action.fill(.1F);
        c.complete_policy(request, action, at(2.01));
        auto output = c.tick(fresh(2.02), at(2.02));
        const auto expected = test_adapter().action_to_hardware_targets(action, Mode::kTerrain);
        for (int i = 0; i < 12; ++i) {
            near(output[i].position_rad, expected[i]);
            near(output[i].kp, 20);
            near(output[i].kd, .5F);
        }
        auto next = *c.prepare_policy(fresh(2.02), at(2.02));
        for (int i = 213; i < 225; ++i)
            near(next.observation[i], .1F);
        next = *c.prepare_policy(fresh(2.31), at(2.31));
        near(next.observation[42], 0);
        near(next.observation[43], 0);
        near(next.observation[44], 0);
        assert(c.set_mode(Mode::kStand, fresh(2.1), at(2.1)).accepted);
        c.complete_policy(request, action, at(2.11)); // Result from previous mode discarded.
        c.tick(fresh(3.1), at(3.1));
        assert(c.set_mode(Mode::kTerrain, fresh(3.1), at(3.1)).accepted);
        c.complete_policy(request, action, at(3.11)); // Even after re-entering terrain.
        c.fail_policy(request, "obsolete error");
        next = *c.prepare_policy(fresh(3.11), at(3.11));
        for (int i = 213; i < 225; ++i)
            near(next.observation[i], 0);
        output = c.tick(fresh(3.11), at(3.11));
        near(output[0].kp, 60); // Still standing while waiting for the new inference.
        assert(!c.status().protective_crouch);
        c.complete_policy(next, action, at(3.12));
        assert(c.set_mode(Mode::kPassive, fresh(3.13), at(3.13)).accepted);
        assert(c.status().protective_crouch);
        assert(!c.set_mode(Mode::kTerrain, fresh(3.14), at(3.14)).accepted);
        brake(c.tick(fresh(4.14), at(4.14)));
    }

    // Check acceleration, direction changes and timeout at the terrain policy input.
    {
        go2::Controller c(test_controller_config());
        stand(c);
        assert(c.set_mode(Mode::kTerrain, fresh(2), at(2)).accepted);
        const auto physical_vx = [](const go2::PolicyRequest &request) {
            return request.observation[42] / 2.F;
        };
        c.set_velocity({1.F, .4F, 1.F}, at(2));
        auto request = c.prepare_policy(fresh(2), at(2));
        assert(request);
        near(physical_vx(*request), 0.F);
        c.set_velocity({1.F, .4F, 1.F}, at(2.02));
        request = c.prepare_policy(fresh(2.02), at(2.02));
        assert(request);
        near(physical_vx(*request), .08F);
        request = c.prepare_policy(fresh(2.12), at(2.12));
        assert(request);
        near(physical_vx(*request), .48F);
        c.set_velocity({}, at(2.12));
        request = c.prepare_policy(fresh(2.14), at(2.14));
        assert(request);
        near(physical_vx(*request), .36F);
        c.set_velocity({-1.F, 0.F, 0.F}, at(2.14));
        request = c.prepare_policy(fresh(2.20), at(2.20));
        assert(request);
        near(physical_vx(*request), 0.F);
        request = c.prepare_policy(fresh(2.22), at(2.22));
        assert(request);
        near(physical_vx(*request), -.08F);
        request = c.prepare_policy(fresh(2.45), at(2.45));
        assert(request);
        near(physical_vx(*request), 0.F); // A lost producer still stops immediately.
        c.set_velocity({1.F, 0.F, 0.F}, at(2.46));
        request = c.prepare_policy(fresh(2.46), at(2.46));
        assert(request);
        near(physical_vx(*request), 0.F); // Reconnect does not reuse the old ramp.
        assert(c.set_mode(Mode::kPassive, fresh(2.47), at(2.47)).accepted);
        assert(c.tick(fresh(2.48), at(2.48))[0].protective_crouch);
    }
    {
        go2::Controller c(test_controller_config());
        terrain(c);
        const auto old = *c.prepare_policy(fresh(2), at(2));
        auto s = fresh(2.1);
        s.motors.joints[0].received_at = at(1.85);
        auto held = c.tick(s, at(2.1));
        assert(held[0].mode == go2::MotorMode::kFoc);
        assert(c.status().mode == Mode::kTerrain);
        c.complete_policy(old, {}, at(2.12));
        c.tick(fresh(2.12), at(2.12));
        assert(c.set_mode(Mode::kTerrain, fresh(2.13), at(2.13)).accepted); // Reseeded stand; queue FixStand.
        s = fresh(2.2);
        s.motors.joints[0].received_at = at(1.89);
        s.imu.received_at = at(1.95); // Motor hard fault must win over IMU soft hold.
        assert(c.tick(s, at(2.2))[0].protective_crouch);
        assert(c.status().protective_crouch);
        brake(c.tick(s, at(3.21)));
    }
    {
        go2::Controller c(test_controller_config());
        stand(c);
        auto s = fresh(2.1);
        s.imu.received_at = at(1.85);
        auto held = c.tick(s, at(2.1));
        assert(held[0].mode == go2::MotorMode::kFoc);
        assert(!c.status().protective_crouch);
        s = fresh(2.2);
        s.imu.received_at = at(1.85);
        c.tick(s, at(2.2));
        assert(c.status().protective_crouch);
        s = fresh(3.21);
        s.imu.received_at = at(1.85);
        brake(c.tick(s, at(3.21)));
        brake(c.tick(s, at(3.22)));
        assert(!c.status().protective_crouch);
        assert(!c.set_mode(Mode::kStand, fresh(3.23), at(3.23)).accepted);
        c.tick(fresh(3.23), at(3.23));
        assert(c.set_mode(Mode::kStand, fresh(3.24), at(3.24)).accepted);
    }
    for (bool native_uart : {false, true})
        for (int fault = 0; fault < 10; ++fault) {
            auto config = test_controller_config();
            config.rr_uart6_degraded = native_uart;
            go2::Controller c(config);
            stand(c);
            auto s = fresh(2.1);
            switch (fault) {
            case 0:
                s.motors.session_anchor_faulted = true;
                break;
            case 1:
                s.motors.joints[0].valid = false;
                break;
            case 2:
                s.motors.joints[0].feedback.position_anchored = false;
                break;
            case 3:
                s.motors.joints[0].feedback.error = 1;
                break;
            case 4:
                s.motors.joints[0].feedback.position_rad = std::numeric_limits<float>::quiet_NaN();
                break;
            case 5:
                s.motors.joints[0].feedback.velocity_rad_s = 51;
                break;
            case 6:
                s.motors.joints[7].feedback.position_rad = -1;
                break;
            case 7:
                s.motors.joints[0].received_at = at(1);
                break;
            case 8:
                s.motors.joints[0].received_at = at(3);
                break;
            case 9:
                s.motors.joints[0].feedback.velocity_rad_s = std::numeric_limits<float>::infinity();
                break;
            }
            s.motors.joints[1].feedback.temperature_c = 80;
            const auto stopped = c.tick(s, at(2.1));
            assert(c.status().protective_crouch && c.status().mode != Mode::kPassive);
            const auto cause = c.status().reason;
            for (const auto &command : stopped) {
                assert(command.mode == go2::MotorMode::kFoc && command.protective_crouch);
                assert(std::isfinite(command.position_rad));
            }
            const auto midway = c.tick(s, at(2.6));
            assert(c.status().protective_crouch && c.status().reason == cause);
            for (unsigned i = 0; i < midway.size(); ++i)
                near(midway[i].position_rad, (stopped[i].position_rad + config.crouch_pose[i]) * .5F);
            brake(c.tick(s, at(3.11)));
            assert(c.status().mode == Mode::kPassive && c.status().reason.find(cause) != std::string::npos);
            brake(c.tick(s, at(4.2))); // A persistent fault cannot restart the crouch.
            go2::Controller requested(config);
            stand(requested);
            assert(requested.set_mode(Mode::kPassive, s, at(2.1)).accepted);
            assert(requested.tick(s, at(2.6))[0].protective_crouch);
            brake(requested.tick(s, at(3.11)));
        }
    {
        go2::Controller c(test_controller_config());
        stand(c);
        auto s = fresh(2.1);
        s.motors.joints[0].feedback.temperature_c = 78;
        c.tick(s, at(2.1));
        assert(c.status().protective_crouch);
        brake(c.tick(fresh(3.11), at(3.11)));
    }
    {
        // A position-limit violation with otherwise fresh, anchored and bounded
        // feedback is recoverable: use the same protective crouch as overheating
        // instead of dropping straight to BRAKE.  Mixed position+temperature
        // remains fail-closed (covered by the fatal-fault loop above).
        go2::Controller c(test_controller_config());
        stand(c);
        auto s = fresh(2.1);
        s.motors.joints[7].feedback.position_rad = -1.0F;
        auto output = c.tick(s, at(2.1));
        assert(c.status().protective_crouch);
        assert(output[0].mode == go2::MotorMode::kFoc);
        brake(c.tick(fresh(3.11), at(3.11)));

        go2::Controller requested(test_controller_config());
        stand(requested);
        assert(requested.set_mode(Mode::kPassive, s, at(2.1)).accepted);
        assert(requested.status().protective_crouch);
    }
    for (int fault = 0; fault < 4; ++fault) {
        go2::Controller c(test_controller_config());
        terrain(c);
        const auto request = *c.prepare_policy(fresh(2), at(2));
        if (fault == 0)
            c.fail_policy(request, "inference failed");
        if (fault == 1)
            c.complete_policy(request, {}, at(2.23));
        if (fault == 2) {
            go2::Action a{};
            a[1] = std::numeric_limits<float>::infinity();
            c.complete_policy(request, a, at(2.01));
        }
        c.tick(fresh(2.31), at(2.31)); // fault==3: inference never returns.
        assert(c.status().protective_crouch);
        assert(!c.status().reason.empty());
    }
    {
        // The selected rollover sequence avoids inference and returns to Pose A.
        go2::ControllerConfig config = test_controller_config();
        go2::Controller c(config);
        const int segments = c.status().rollover_segments;
        assert(segments == 9);
        stand(c);
        assert(!c.set_mode(Mode::kRollover, fresh(2), at(2)).accepted);
        auto tilted = fresh(2);
        tilted.imu.quaternion = {0.819152F, 0.573576F, 0.0F, 0.0F}; // roll = 70 deg
        tilted.imu.six_axis_quaternion = tilted.imu.quaternion;
        assert(c.set_mode(Mode::kRollover, tilted, at(2)).accepted);
        assert(!c.prepare_policy(tilted, at(2.01)));
        auto output = c.tick(tilted, at(2.01));
        assert(output[0].mode == go2::MotorMode::kFoc);
        const go2::Joints last_recorded_degrees{22.38F, 29.45F, -123.36F, -16.07F, 149.03F, -156.68F,
            22.38F, 29.45F, -123.36F, -16.07F, 149.03F, -156.68F};
        const double midpoint = 2 + (segments - .5) * .8;
        output = c.tick(fresh(midpoint), at(midpoint));
        for (std::size_t i = 0; i < go2::kNumJoints; ++i)
            near(output[i].position_rad, .5F * (last_recorded_degrees[i] * 3.14159265358979323846F / 180.F +
                config.crouch_pose[i]));
        const double completion = 2 + segments * .8 + .01;
        output = c.tick(fresh(completion), at(completion));
        for (std::size_t i = 0; i < go2::kNumJoints; ++i)
            near(output[i].position_rad, config.crouch_pose[i]);
    }
    {
        // Accept rollover directly from Pose A; it is not gated on a prior
        // completed stand transition.
        go2::Controller c(test_controller_config());
        assert(c.set_mode(Mode::kCrouch, fresh(1), at(1)).accepted);
        c.tick(fresh(3), at(3));
        auto pitched = fresh(3);
        pitched.imu.quaternion = {0.819152F, 0.0F, 0.573576F, 0.0F}; // pitch = 70 deg
        pitched.imu.six_axis_quaternion = pitched.imu.quaternion;
        assert(c.set_mode(Mode::kRollover, pitched, at(3)).accepted);
    }
    {
        // Manual steps match automatic targets, but wait indefinitely between
        // presses. After entry, becoming upright must not block later steps.
        auto config = test_controller_config();
        go2::Controller manual(config), automatic(config);
        const int segments = manual.status().rollover_segments;
        assert(!manual.step_rollover(fresh(1), at(1)).accepted);
        auto tilted = fresh(1);
        tilted.imu.quaternion = {0.819152F, 0.573576F, 0.0F, 0.0F};
        tilted.imu.six_axis_quaternion = tilted.imu.quaternion;
        assert(automatic.set_mode(Mode::kRollover, tilted, at(1)).accepted);
        assert(!automatic.step_rollover(tilted, at(1)).accepted);
        for (std::size_t step = 0; step < static_cast<std::size_t>(segments); ++step) {
            const double start = 1 + step * 4.0;
            assert(manual.step_rollover(step == 0 ? tilted : fresh(start), at(start)).accepted);
            assert(manual.status().rollover_step == step + 1);
            assert(manual.status().rollover_step_running);
            assert(!manual.step_rollover(fresh(start + .1), at(start + .1)).accepted);
            assert(!manual.prepare_policy(fresh(start + .4), at(start + .4)));
            const auto midway = manual.tick(fresh(start + .4), at(start + .4));
            const double reference_mid = 1 + step * .8 + .4;
            const auto expected_mid = automatic.tick(fresh(reference_mid), at(reference_mid));
            const auto end = manual.tick(fresh(start + .81), at(start + .81));
            assert(!manual.status().rollover_step_running);
            const auto held = manual.tick(fresh(start + 3), at(start + 3));
            const double reference_end = 1 + (step + 1) * .8;
            const auto expected_end = automatic.tick(fresh(reference_end), at(reference_end));
            for (std::size_t joint = 0; joint < go2::kNumJoints; ++joint) {
                near(midway[joint].position_rad, expected_mid[joint].position_rad);
                near(end[joint].position_rad, expected_end[joint].position_rad);
                near(held[joint].position_rad, end[joint].position_rad);
            }
        }
        assert(!manual.step_rollover(fresh(40), at(40)).accepted);
        assert(manual.set_mode(Mode::kPassive, fresh(40), at(40), true).accepted);
        manual.tick(fresh(40.01), at(40.01));
        assert(manual.status().protective_crouch);
        assert(manual.status().rollover_step == 0);
        assert(!manual.step_rollover(fresh(40.02), at(40.02)).accepted);
        manual.tick(fresh(41.1), at(41.1));
        assert(manual.status().mode == Mode::kPassive);
        tilted = fresh(42);
        tilted.imu.quaternion = {0.819152F, 0.573576F, 0.0F, 0.0F};
        tilted.imu.six_axis_quaternion = tilted.imu.quaternion;
        assert(manual.step_rollover(tilted, at(42)).accepted);
        assert(manual.status().rollover_step == 1);
        auto stale = fresh(43);
        stale.imu.received_at = at(42);
        assert(!manual.step_rollover(stale, at(43)).accepted);
        manual.tick(stale, at(43));
        assert(manual.status().protective_crouch);
    }
    for (const auto &[name, mode] : std::vector<std::pair<const char *, Mode>>{
            {"navigation", Mode::kWalk}, {"ai_agent", Mode::kWalk}, {"walk", Mode::kWalk},
            {"rollover", Mode::kRollover}, {"crouch", Mode::kCrouch}, {"wave", Mode::kWave},
            {"sit", Mode::kSit}, {"terrain", Mode::kTerrain}, {"forward_jump", Mode::kForwardJump}})
        assert(go2::parse_mode(name) == mode);
    assert(std::string(go2::mode_name(Mode::kCrouch)) == "crouch");
    for (const auto name : {"", "unknown", "run"})
        rejects([name] { go2::parse_mode(name); });
    go2::ControllerConfig bad = test_controller_config();
    bad.motor_hard_timeout = .1;
    rejects([&] { go2::Controller c(bad); });
    std::cout << "PASS: modes, interpolation, command timeout, stale results, motor/IMU/policy faults\n";
}
