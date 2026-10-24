/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "hardware.hpp"
#include "support.hpp"
#include <atomic>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

int main() {
    // A mounting correction changes all sensor-frame quantities consistently.
    // Gravity from either attitude must equal the same rotation of raw gravity,
    // including nonzero yaw, vertical poses and inverted poses.
    {
        go2::HardwareConfig mounting;
        mounting.imu_to_body_quaternion = {0.000010623207F, 0.013525885071F, -0.000785326235F, 0.999908212579F};
        mounting.imu_to_body_matrix = {-0.999634100640F, 0.F, 0.027049303815F, -0.000042488930F,
            -0.999998766300F, -0.001570220928F, 0.027049270445F, -0.001570795681F, 0.999632867391F};
        const auto gravity = [](const std::array<float, 4> &q) {
            const auto [w, x, y, z] = q;
            return std::array<float, 3>{2 * (w * y - x * z), -2 * (y * z + w * x), -(1 - 2 * (x * x + y * y))};
        };
        for (float roll : {-.040650F, .4F, 1.5707963F, 3.14159265F}) {
            for (float pitch : {-1.5709F, -1.5707963F, -1.5707F, -.8F, 0.F,
                    1.5707F, 1.5707963F, 1.5709F, 3.14159265F}) {
                for (float yaw : {-2.F, 0.F, 1.2F}) {
                    const float cr = std::cos(roll / 2), sr = std::sin(roll / 2);
                    const float cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
                    const float cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
                    go2::ImuSample raw;
                    raw.quaternion = {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                        cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
                    raw.six_axis_quaternion = raw.quaternion;
                    raw.angular_velocity = {.12F, -.34F, .56F};
                    const auto g = gravity(raw.quaternion);
                    for (int i = 0; i < 3; ++i)
                        raw.linear_acceleration[i] = -9.81F * g[i];
                    raw.sequence = 42;
                    raw.valid = true;
                    raw.received_at = at(1);
                    const auto corrected = go2::imu_sample_to_body(raw, mounting);
                    const auto expected_g = go2::rotate_vector(mounting.imu_to_body_matrix, g);
                    const auto corrected_g = gravity(corrected.quaternion);
                    const auto six_g = gravity(corrected.six_axis_quaternion);
                    const auto gyro = go2::rotate_vector(mounting.imu_to_body_matrix, raw.angular_velocity);
                    for (int i = 0; i < 3; ++i) {
                        near(corrected_g[i], expected_g[i]);
                        near(six_g[i], expected_g[i]);
                        near(corrected.linear_acceleration[i], -9.81F * expected_g[i]);
                        near(corrected.angular_velocity[i], gyro[i]);
                    }
                    assert(corrected.sequence == 42 && corrected.valid && corrected.received_at == at(1));
                    const auto unchanged = go2::imu_sample_to_body(raw, go2::HardwareConfig{});
                    for (int i = 0; i < 4; ++i)
                        near(unchanged.quaternion[i], raw.quaternion[i]);
                    for (int i = 0; i < 3; ++i)
                        near(unchanged.linear_acceleration[i], raw.linear_acceleration[i]);
                }
            }
        }
    }
    // Crossing either Euler pitch singularity must preserve quaternion continuity.
    {
        go2::HardwareConfig mounting;
        mounting.imu_to_body_quaternion = {0.000010623207F, 0.013525885071F, -0.000785326235F, 0.999908212579F};
        for (float center : {-1.5707963F, 1.5707963F}) {
            std::array<float, 4> previous{};
            for (int step = -10; step <= 10; ++step) {
                const float pitch = center + step * 0.0001F;
                go2::ImuSample sensor;
                sensor.quaternion = {std::cos(pitch / 2), 0, std::sin(pitch / 2), 0};
                sensor.six_axis_quaternion = sensor.quaternion;
                const auto body = go2::imu_sample_to_body(sensor, mounting);
                if (step > -10) {
                    float dot = 0;
                    for (int i = 0; i < 4; ++i) {
                        dot += previous[i] * body.quaternion[i];
                        near(body.quaternion[i], body.six_axis_quaternion[i]);
                    }
                    assert(dot > 0.999999F);
                }
                previous = body.quaternion;
            }
        }
    }
    for (float ratio : {6.33F, 12.13F})
        for (float direction : {-1.F, 1.F}) {
            go2::JointCalibration cal{2, ratio, direction, .3F};
            go2::JointCommand joint{go2::MotorMode::kFoc, 2, 3, 1, 25, .5F};
            const auto rotor = go2::joint_to_rotor_command(joint, cal);
            near(rotor.position_rad, direction * .7F * ratio);
            near(rotor.velocity_rad_s, direction * 3 * ratio);
            near(rotor.torque_nm, direction * 2 / ratio);
            near(rotor.kp, 25 / (ratio * ratio));
            near(rotor.kd, .5F / (ratio * ratio));
            // Backflip produces joint-side direct torque.  The same hardware boundary
            // converts it to the motor rotor side before writing the serial command.
            go2::JointCommand backflip{};
            backflip.mode = go2::MotorMode::kFoc;
            backflip.torque_nm = 20.2F;
            const auto backflip_rotor = go2::joint_to_rotor_command(backflip, cal);
            near(backflip_rotor.torque_nm, direction * 20.2F / ratio);
            near(backflip_rotor.kp, 0.0F);
            near(backflip_rotor.kd, 0.0F);
            go2::RotorFeedback feedback{2, rotor.torque_nm, rotor.velocity_rad_s, rotor.position_rad, 31, 0, 123};
            const auto decoded = go2::rotor_to_joint_feedback(feedback, cal);
            near(decoded.position_rad, 1);
            near(decoded.velocity_rad_s, 3);
            near(decoded.torque_nm, 2);
            assert(decoded.position_anchored && decoded.foot_force_raw == 123);
            for (int turn : {-10, 0, 10}) {
                go2::JointSessionAnchor anchor;
                const float current = .7F + turn * go2::kTwoPi;
                anchor.initialize(current, {1, .7F}, cal, .1F);
                feedback.position_rad = current + .05F * ratio * direction;
                near(anchor.to_joint_feedback(feedback, cal).position_rad, 1.05F);
                near(anchor.to_rotor_command(joint, cal).position_rad, current);
                feedback.position_rad += go2::kTwoPi;
                rejects([&] { anchor.to_joint_feedback(feedback, cal); });
                assert(!anchor.valid());
                rejects([&] { anchor.to_rotor_command(joint, cal); });
                auto stop = joint;
                stop.protective_crouch = true;
                near(anchor.to_rotor_command(stop, cal).position_rad, current);
                assert(!anchor.valid()); // Stopping cannot re-arm ordinary FOC.
                go2::JointSessionAnchor never_initialized;
                rejects([&] { never_initialized.to_rotor_command(stop, cal); });
                joint.mode = go2::MotorMode::kBrake;
                assert(anchor.to_rotor_command(joint, cal).mode == go2::MotorMode::kBrake);
                joint.mode = go2::MotorMode::kFoc;
            }
            // Backflip direct torque tolerates a large inter-sample rotor step without
            // invalidating the multi-turn Pose A anchor.  Normal modes remain strict.
            go2::JointSessionAnchor backflip_anchor;
            backflip_anchor.initialize(.7F, {1, .7F}, cal, .1F);
            feedback.position_rad = .7F + go2::kTwoPi;
            const auto backflip_feedback = backflip_anchor.to_joint_feedback(feedback, cal, false);
            assert(backflip_anchor.valid());
            near(backflip_feedback.position_rad, 1.0F + direction * go2::kTwoPi / ratio);
            go2::JointSessionAnchor anchor;
            rejects([&] { anchor.initialize(2, {1, .7F}, cal, .01F); });
            rejects([&] { anchor.initialize(.7F, {1, .7F}, cal, go2::kTwoPi / ratio); });
            joint.position_rad = std::numeric_limits<float>::quiet_NaN();
            rejects([&] { go2::joint_to_rotor_command(joint, cal); });
        }
    // Native buses can miss multiple replies during normal motion. USB must
    // still reject exactly the same half-turn steps as before.
    for (bool native : {false, true}) {
        for (float ratio : {6.33F, 12.13F}) {
            for (float direction : {-1.F, 1.F}) {
                const go2::JointCalibration cal{2, ratio, direction, 0};
                for (auto motion : {std::array<float, 2>{6.F, .050F}, {14.F, .019F}}) {
                    go2::JointSessionAnchor anchor;
                    anchor.initialize(.7F, {0, .7F}, cal, .1F);
                    go2::RotorFeedback feedback;
                    feedback.motor_id = 2;
                    feedback.position_rad = .7F;
                    feedback.velocity_rad_s = direction * ratio * motion[0];
                    const auto decode = [&](double time) {
                        return anchor.to_joint_feedback(feedback, cal, true, native ? at(time) : go2::TimePoint{});
                    };
                    decode(1);
                    feedback.position_rad += feedback.velocity_rad_s * motion[1];
                    if (!native && std::fabs(feedback.position_rad - .7F) >= go2::kTwoPi / 2) {
                        rejects([&] { decode(1 + motion[1]); });
                        assert(!anchor.valid());
                    } else {
                        near(decode(1 + motion[1]).position_rad, motion[0] * motion[1]);
                        // Re-reading a cached sample cannot advance its timestamp.
                        decode(1 + motion[1]);
                        feedback.position_rad += feedback.velocity_rad_s * motion[1];
                        near(decode(1 + 2 * motion[1]).position_rad, 2 * motion[0] * motion[1]);
                    }
                }
                for (double gap : {.001, .050, .299}) {
                    go2::JointSessionAnchor reset;
                    reset.initialize(.7F, {0, .7F}, cal, .1F);
                    go2::RotorFeedback feedback;
                    feedback.motor_id = 2;
                    feedback.position_rad = .7F;
                    reset.to_joint_feedback(feedback, cal, true, native ? at(1) : go2::TimePoint{});
                    feedback.position_rad += go2::kTwoPi;
                    rejects([&] {
                        reset.to_joint_feedback(feedback, cal, true, native ? at(1 + gap) : go2::TimePoint{});
                    });
                    assert(!reset.valid());
                }
                if (native) {
                    go2::JointSessionAnchor jitter;
                    jitter.initialize(.7F, {0, .7F}, cal, .1F);
                    go2::RotorFeedback feedback;
                    feedback.motor_id = 2;
                    feedback.position_rad = .7F;
                    feedback.velocity_rad_s = direction * ratio * 14;
                    jitter.to_joint_feedback(feedback, cal, true, at(1));
                    // Estimated sample time regresses: do not pair its new
                    // position with the previous sample's time and velocity.
                    feedback.position_rad += direction * 1.4F;
                    jitter.to_joint_feedback(feedback, cal, true, at(.999));
                    feedback.position_rad = .7F + feedback.velocity_rad_s * .050F;
                    near(jitter.to_joint_feedback(feedback, cal, true, at(1.05)).position_rad, .7F);
                    assert(jitter.valid());
                    go2::JointSessionAnchor lost;
                    lost.initialize(.7F, {0, .7F}, cal, .1F);
                    feedback = {};
                    feedback.motor_id = 2;
                    feedback.position_rad = .7F;
                    feedback.velocity_rad_s = direction * ratio * 6;
                    lost.to_joint_feedback(feedback, cal, true, at(1));
                    feedback.position_rad += feedback.velocity_rad_s * .31F;
                    rejects([&] { lost.to_joint_feedback(feedback, cal, true, at(1.31)); });
                    assert(!lost.valid());
                }
            }
        }
    }
    go2::JointCalibration bad;
    bad.direction = 0;
    rejects([&] { go2::validate_calibration(bad); });
    go2::HardwareConfig config;
    assert(config.board_profile == go2::BoardProfile::kLegacyUsb);
    // Poisoned device paths: mock must not attempt to open any of them.
    config.imu_device = "/dev/serial/by-id/MUST_NOT_OPEN_IMU";
    for (auto &leg : config.legs)
        leg.device = "/dev/serial/by-id/MUST_NOT_OPEN_MOTOR";
    go2::Hardware mock(config);
    mock.start();
    auto state = mock.read(at(10));
    assert(state.imu.valid);
    for (const auto &j : state.motors.joints)
        assert(j.valid && j.feedback.position_anchored);
    go2::Commands commands{};
    commands[0].mode = go2::MotorMode::kFoc;
    commands[0].position_rad = .2F;
    mock.write(commands);
    near(mock.read(at(11)).motors.joints[0].feedback.position_rad, .2F);
    mock.stop();
    mock.stop();
    // Calibration can stop/restart hardware while ROS callbacks read and write.
    // A command that arrives during a stopped interval must be rejected cleanly.
    {
        go2::Hardware concurrent(config);
        concurrent.start();
        std::atomic_bool running{true};
        std::atomic<unsigned> reads{0}, writes{0};
        std::thread reader([&] {
            while (running.load()) {
                assert(concurrent.read().imu.valid);
                ++reads;
                std::this_thread::yield();
            }
        });
        std::thread writer([&] {
            while (running.load()) {
                try {
                    concurrent.write(go2::Commands{});
                    ++writes;
                } catch (const std::logic_error&) {
                    // stop() may acquire the lifecycle lock before this write.
                }
                std::this_thread::yield();
            }
        });
        while (reads.load() == 0 || writes.load() == 0) {
            std::this_thread::yield();
        }
        for (unsigned i = 0; i < 100; ++i) {
            concurrent.stop();
            concurrent.replace_calibration({}, {});
            concurrent.start();
        }
        running = false;
        reader.join();
        writer.join();
        concurrent.stop();
    }
    config.backend = go2::Backend::kSerial;
    rejects([&] { go2::Hardware unapproved(config); });
    static_assert(go2::kMotorBaudrate == 4000000U);
    go2::HardwareConfig serial_config;
    serial_config.backend = go2::Backend::kSerial;
    serial_config.motor_io_authorized = true;
    serial_config.actuation_enabled = true;
    serial_config.hardware_configuration_verified = true;
    serial_config.motor_ids_verified = true;
    serial_config.joint_calibration_verified = true;
    serial_config.suspended_test_verified = true;
    serial_config.imu_device = "/dev/serial/by-path/imu";
    for (std::size_t i = 0; i < serial_config.legs.size(); ++i) {
        serial_config.legs[i].device = "/dev/serial/by-path/motor-" + std::to_string(i);
        for (std::size_t local = 0; local < go2::kMotorsPerLeg; ++local) {
            serial_config.legs[i].calibration[local].motor_id = static_cast<std::uint8_t>(local);
        }
    }
    go2::validate_hardware_config(serial_config);
    serial_config.imu_driver = "drv_spi_bmi270";
    serial_config.imu_device = "/dev/bmi270";
    serial_config.imu_baud = 0; // SPI does not require a UART baudrate.
    go2::validate_hardware_config(serial_config);
    {
        auto irq = serial_config;
        irq.imu_gpiochip = "/dev/gpiochip0";
        irq.imu_gpio_line = 28;
        go2::validate_hardware_config(irq);
        irq.imu_gpio_line = -1;
        rejects([&] { go2::validate_hardware_config(irq); });
        irq.imu_gpio_line = 28;
        irq.imu_gpiochip = "not-a-device";
        rejects([&] { go2::validate_hardware_config(irq); });
        irq.imu_gpiochip = "/dev/gpiochip0";
        irq.imu_driver = "drv_uart_iwt603";
        rejects([&] { go2::validate_hardware_config(irq); });
    }
    serial_config.imu_device = "/dev/spidev0.0";
    go2::validate_hardware_config(serial_config);
    auto new_config = serial_config;
    new_config.board_profile = go2::BoardProfile::kNewBigcore;
    new_config.imu_device = "/dev/iio:device0";
    new_config.imu_gpiochip.clear();
    new_config.imu_gpio_line = 0;
    const std::array<std::string, 4> names{"rl", "rr", "fl", "fr"};
    const std::array<std::string, 4> uart_paths{"/dev/ttyS8", "/dev/ttyS5", "/dev/ttyS6", "/dev/ttyS3"};
    for (std::size_t i = 0; i < 4; i++)
        new_config.legs[i].device = uart_paths[i];
    go2::validate_hardware_config(new_config);
    auto usb_motor_config = new_config;
    usb_motor_config.board_profile = go2::BoardProfile::kNewUsbMotor;
    const std::array<std::string, 4> usb_motor_paths{
        "/dev/go2/motor-fr", "/dev/go2/motor-fl", "/dev/go2/motor-rr", "/dev/go2/motor-rl"};
    for (std::size_t i = 0; i < 4; i++)
        usb_motor_config.legs[i].device = usb_motor_paths[i];
    go2::validate_hardware_config(usb_motor_config);
    auto alias_config = usb_motor_config;
    alias_config.board_profile = go2::BoardProfile::kLegacyUsb;
    alias_config.imu_driver = "drv_uart_iwt603";
    alias_config.imu_device = "/dev/go2/imu";
    alias_config.imu_baud = 921600;
    go2::validate_hardware_config(alias_config);
    alias_config.legs[0].device = "/dev/go2/unknown";
    rejects([&] { go2::validate_hardware_config(alias_config); });
    new_config.imu_driver = "drv_uart_iwt603";
    rejects([&] { go2::Hardware unsupported(new_config); });
    new_config.imu_driver = "drv_spi_bmi270";
    new_config.board_profile = go2::BoardProfile::kNewSmallcore;
    rejects([&] { go2::validate_hardware_config(new_config); });
    for (std::size_t i = 0; i < 4; i++) {
        new_config.legs[i].device = "go2-motor-" + names[i];
        new_config.legs[i].driver = "go2_smallcore";
    }
    go2::validate_hardware_config(new_config);
    std::swap(new_config.legs[0].device, new_config.legs[1].device);
    rejects([&] { go2::validate_hardware_config(new_config); });
    std::swap(new_config.legs[0].device, new_config.legs[1].device);

    serial_config.imu_device = "/dev/serial/by-path/imu";
    rejects([&] { go2::validate_hardware_config(serial_config); });
    serial_config.imu_driver = "unknown";
    rejects([&] { go2::validate_hardware_config(serial_config); });
    config.backend = go2::Backend::kMock;
    config.motor_io_authorized = true;
    rejects([&] { go2::Hardware ambiguous(config); });
    std::cout << "PASS: gear/sign/gain transforms, Pose A anchors, discontinuity, mock isolation\n";
}
