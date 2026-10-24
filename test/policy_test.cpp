/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "controller.hpp"
#include "policy.hpp"
#include "rl_service.h"
#include "support.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <filesystem> // NOLINT(build/c++17): this package requires C++17.
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

namespace fs = std::filesystem;

namespace {

using go2::Mode;

void seed_policy_state(go2::RobotState& state, go2::Action& previous) {
    state.imu.angular_velocity = {1.0F, -2.0F, 3.0F};
    for (std::size_t hardware_index = 0; hardware_index < go2::kNumJoints;
                ++hardware_index) {
        state.motors.joints[hardware_index].feedback.position_rad =
                0.05F * static_cast<float>(hardware_index);
        state.motors.joints[hardware_index].feedback.velocity_rad_s =
                static_cast<float>(hardware_index);
        previous[hardware_index] = -0.03F * static_cast<float>(hardware_index);
    }
}

void check_proprioception(const std::vector<float>& observation,
                                                    std::size_t offset, const go2::RobotState& state,
                                                    const go2::Action& previous, go2::Mode mode,
                                                    const go2::Joints& defaults) {
    const auto& map = test_adapter().policy_to_hardware(mode);
    for (std::size_t policy_index = 0; policy_index < go2::kNumJoints;
                ++policy_index) {
        const auto hardware_index = map[policy_index];
        near(observation[offset + policy_index],
                    state.motors.joints[hardware_index].feedback.position_rad -
                            defaults[policy_index]);
        near(observation[offset + 12U + policy_index],
                    state.motors.joints[hardware_index].feedback.velocity_rad_s * 0.05F);
        near(observation[offset + 24U + policy_index], previous[policy_index]);
    }
}

go2::Quaternion yaw(float radians) {
    return {std::cos(0.5F * radians), 0.0F, 0.0F,
                    std::sin(0.5F * radians)};
}

struct TemporaryConfig {
    fs::path directory;
    TemporaryConfig() {
        auto pattern = (fs::temp_directory_path() / "go2-policy-XXXXXX").string();
        const auto path = mkdtemp(pattern.data());
        if (!path)
            throw std::runtime_error("cannot create temporary config directory");
        directory = path;
    }
    ~TemporaryConfig() {
        std::error_code error;
        fs::remove_all(directory, error);
    }
    std::string write(const YAML::Node &config) const {
        const auto path = directory / "policies.yaml";
        std::ofstream stream(path);
        stream << config;
        if (!stream)
            throw std::runtime_error("cannot write test configuration");
        return path.string();
    }
};

YAML::Node policies(YAML::Node document) {
    return document["rl_policy"]["onnx_infer"]["policies"];
}

void check_policy_contracts(const std::string& models) {
    auto state = fresh(1.0);
    go2::Action previous{};
    seed_policy_state(state, previous);

    const auto walking = test_adapter().build_observation(state, {9.F, -3.F, 4.F}, previous, Mode::kWalk);
    assert(walking.size() == 45U);
    near(walking[0], .2F);
    near(walking[1], -.4F);
    near(walking[2], .6F);
    near(walking[6], 1.F);
    near(walking[7], -.4F);
    near(walking[8], 1.F);
    const go2::Joints walking_default{.1F, -.1F, .1F, -.1F, .8F, .8F, 1.F, 1.F,
        -1.5F, -1.5F, -1.5F, -1.5F};
    check_proprioception(walking, 9U, state, previous, Mode::kWalk, walking_default);
    for (float gain : test_adapter().policy_gains(Mode::kWalk).kp) near(gain, 25.F);
    for (float gain : test_adapter().policy_gains(Mode::kWalk).kd) near(gain, .5F);

    const auto waving = test_adapter().build_observation(state, {}, previous, Mode::kWave, 2.F);
    assert(waving.size() == 56U);
    near(waving[0], .25F);
    near(waving[1], -.5F);
    near(waving[2], .75F);
    const auto& wave_spec = test_adapter().spec_for(Mode::kWave);
    check_proprioception(waving, 6U, state, previous, Mode::kWave, wave_spec.default_pos);
    const auto reference = go2::wave_reference(2.F);
    for (unsigned i = 0; i < 12; ++i)
        near(waving[42+i], reference.position[i] - wave_spec.default_pos[i]);
    go2::Action wave_action{};
    wave_action.fill(1.F);
    const auto wave_target = test_adapter().action_to_hardware_targets(wave_action, Mode::kWave, 2.F);
    near(wave_target[wave_spec.mapping[3]], reference.position[3]+.1F);
    near(wave_target[wave_spec.mapping[4]], reference.position[4]);
    near(wave_target[wave_spec.mapping[5]], reference.position[5]);
    // Terrain holds the command in physical units and normalizes it once with the
    // trained [2, 2, 0.25] scale; {9, -3, 4} clamps to {1.0, -0.5, 2.0}.
    const auto terrain =
            test_adapter().build_observation(state, {9.0F, -3.0F, 4.0F}, previous,
                                                            Mode::kTerrain);
    assert(terrain.size() == 45U);
    near(terrain[0], 0.25F);
    near(terrain[1], -0.5F);
    near(terrain[2], 0.75F);
    near(terrain[6], 2.0F);
    near(terrain[7], -1.0F);
    near(terrain[8], 0.5F);
    check_proprioception(terrain, 9U, state, previous, Mode::kTerrain,
                                                go2::kIsaacGymDefaultJointPos);

    auto motor_state = fresh(1.0);
    go2::Joints far_target{};
    far_target.fill(100.0F);
    for (auto& joint : motor_state.motors.joints) joint.feedback.velocity_rad_s = 15.0F;
    for (float torque : test_adapter().policy_torques(far_target, motor_state, Mode::kWave)) near(torque, 11.75F);
    for (auto& joint : motor_state.motors.joints) joint.feedback.velocity_rad_s = 45.0F;
    for (float torque : test_adapter().policy_torques(far_target, motor_state, Mode::kWave)) near(torque, -11.75F);
    far_target.fill(-100.0F);
    for (auto& joint : motor_state.motors.joints) joint.feedback.velocity_rad_s = -45.0F;
    for (float torque : test_adapter().policy_torques(far_target, motor_state, Mode::kWave)) near(torque, 11.75F);

    const go2::Joints reference_position{};
    const go2::Joints reference_velocity{};
    const auto alignment = go2::motion_yaw_alignment(yaw(0.7F), yaw(0.2F));
    auto right_state = state;
    right_state.imu.six_axis_quaternion = yaw(0.7F);
    const auto right = test_adapter().build_motion_observation(
            Mode::kForwardJump, right_state, reference_position, reference_velocity,
            yaw(0.2F), previous, alignment);
    assert(right.size() == 69U);
    near(right[24], 1.0F);
    near(right[25], 0.0F);
    near(right[26], 0.0F);
    near(right[27], 1.0F);
    near(right[28], 0.0F);
    near(right[29], 0.0F);

    go2::Action zero{};
    const auto target = test_adapter().action_to_hardware_targets(zero, Mode::kTerrain);
    const auto& map = test_adapter().policy_to_hardware(Mode::kTerrain);
    for (std::size_t policy_index = 0; policy_index < 12U; ++policy_index)
        near(target[map[policy_index]], go2::kIsaacGymDefaultJointPos[policy_index]);
    for (float gain : test_adapter().policy_gains(Mode::kTerrain).kp) near(gain, 20.0F);
    for (float gain : test_adapter().policy_gains(Mode::kTerrain).kd) near(gain, 0.5F);

    rejects([&] {
        test_adapter().build_observation(state,
                {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F}, {}, Mode::kTerrain);
    });
    auto invalid_action = zero;
    invalid_action[0] = std::numeric_limits<float>::infinity();
    rejects([&] { test_adapter().action_to_hardware_targets(invalid_action, Mode::kWave); });

    go2::Policy policy(models, GO2_TEST_POLICY_CONFIG, test_adapter_ptr());
    struct SmokeCase {
        Mode mode;
        std::size_t dimension;
    };
    const std::array<SmokeCase, 3> cases{{
        {Mode::kWalk, 45U}, {Mode::kTerrain, 225U}, {Mode::kWave, 56U},
    }};
    for (const auto& test : cases) {
        std::cout << "smoke " << go2::mode_name(test.mode) << std::endl;
        policy.reset_mode(test.mode, &state);
        std::vector<float> observation(test.dimension, 0.0F);
        const auto action = policy.infer(test.mode, observation, &state, 0U);
        for (float value : action) assert(std::isfinite(value));
    }
    // Inferred results may be rejected by the controller. Assembly must use the
    // supplied accepted action, including zero after a mode reset, in every frame.
    for (const float accepted : {.37F, -.21F, 0.F}) {
        go2::Action accepted_action{};
        accepted_action.fill(accepted);
        const auto obs = test_adapter().build_observation(state, {}, accepted_action, Mode::kTerrain);
        check_proprioception(obs, 9U, state, accepted_action, Mode::kTerrain,
            go2::kIsaacGymDefaultJointPos);
    }
    {
        const auto sit_trajectory = go2::load_sit_reference(
            std::string(GO2_TEST_APP) + "/resources/motions/go2_sit.motion");
        // Exercise real ONNX files, including all jump reference frames and reset.
        auto state = fresh(1);
        for (auto mode : {Mode::kForwardJump, Mode::kSit}) {
            go2::Action first{}, previous{};
            for (unsigned episode = 0; episode < 2; ++episode) {
                previous = {};
                policy.reset_mode(mode, &state);
                for (unsigned n = 0; n < (mode == Mode::kSit ? 600U : 120U); ++n) {
                    auto obs = mode == Mode::kSit
                        ? test_adapter().build_sit_observation(sit_trajectory, state, previous, n * .02F, 6.F)
                        : test_adapter().build_observation(state, {}, previous, mode, n * .02F);
                    auto action = policy.infer(mode, obs, &state, n);
                    for (float v : action)
                        assert(std::isfinite(v));
                    if (n == 0) {
                        if (!episode)
                            first = action;
                        else
                            for (unsigned i = 0; i < 12; ++i)
                                near(action[i], first[i]);
                    }
                    previous = action;
                }
            }
        }
        // A nonzero relative yaw catches row/column and inverse-rotation mistakes.
        const auto obs = test_adapter().build_motion_observation(
            Mode::kForwardJump, state, {}, {}, {std::cos(.3F), 0, 0, std::sin(.3F)}, {});
        near(obs[24], std::cos(.6F));
        near(obs[25], -std::sin(.6F));
        near(obs[26], std::sin(.6F));
        near(obs[27], std::cos(.6F));
        near(obs[28], 0);
        near(obs[29], 0);
    }
    far_target.fill(100.F);
    for (float value : test_adapter().policy_torques(far_target, fresh(1), Mode::kSit)) near(value, 23.5F);
    auto bad_observation = std::vector<float>(225U, 0.0F);
    bad_observation[4] = std::numeric_limits<float>::quiet_NaN();
    rejects([&] { policy.infer(Mode::kTerrain, bad_observation); });

}

void check_config_overrides(const YAML::Node &original, const fs::path &models, const TemporaryConfig &temp) {
    // Prove configuration changes reach both observation and motor command paths.
    {
        auto document = YAML::Clone(original);
        auto terrain = policies(document)["terrain"];
        terrain["observation"]["segment_0_terms"] =
            YAML::Load("[command, gravity, ang_vel, dof_pos, dof_vel, last_action]");
        terrain["observation"]["ang_vel_scale"] = .7;
        terrain["observation"]["dof_pos_scale"] = 2.;
        terrain["observation"]["dof_vel_scale"] = .3;
        terrain["command"]["scale"] = YAML::Load("[3, 4, 5]");
        terrain["policy_adapter"]["command_limits"] = YAML::Load("[-.2, .2, -.1, .1, -.3, .3]");
        terrain["rl_default_pos"][3] = .4;
        terrain["action_scale"] = .6;
        terrain["kp"][0] = 33.;
        terrain["kd"][0] = 1.7;
        const auto adapter = std::make_shared<go2::PolicyAdapter>(temp.write(document), models.string());
        auto state = fresh(2.02);
        state.imu.angular_velocity = {1, 2, 3};
        state.motors.joints[3].feedback.position_rad = .5;
        state.motors.joints[3].feedback.velocity_rad_s = 2.;
        const auto obs = adapter->build_observation(state, {1, 1, 1}, {}, go2::Mode::kTerrain);
        near(obs[0], .6);
        near(obs[1], .4);
        near(obs[2], 1.5);
        near(obs[6], .7);
        near(obs[7], 1.4);
        near(obs[8], 2.1);
        near(obs[9], .2);
        near(obs[21], .6);
        go2::Action action{};
        action[0] = 1;
        const auto target = adapter->action_to_hardware_targets(action, go2::Mode::kTerrain);
        near(target[3], 1.);
        near(adapter->policy_gains(go2::Mode::kTerrain).kp[0], 33.);
        // No process-global mutation: the original adapter retains the original defaults.
        near(test_adapter().action_to_hardware_targets(action, go2::Mode::kTerrain)[3], .35);
        auto config = test_controller_config();
        config.policy_adapter = adapter;
        go2::Controller controller(config);
        assert(controller.set_mode(go2::Mode::kStand, fresh(1), at(1)).accepted);
        controller.tick(fresh(2), at(2));
        assert(controller.set_mode(go2::Mode::kTerrain, state, at(2.02)).accepted);
        controller.set_velocity({1, 1, 1}, at(2.02));
        const auto initial_request = controller.prepare_policy(state, at(2.02));
        assert(initial_request);
        near(initial_request->observation[0], 0.);
        const auto request = controller.prepare_policy(state, at(2.12));
        assert(request);
        near(request->observation[12], .6);
        controller.complete_policy(*request, action, at(2.12));
        const auto commands = controller.tick(state, at(2.12));
        near(commands[3].position_rad, 1.);
        near(commands[3].kp, 33.);
        near(commands[3].kd, 1.7);
    }
}

void check_invalid_config(const YAML::Node &original, const fs::path &models, const TemporaryConfig &temp) {
    const auto load = [&](const YAML::Node &document, const std::string &name) {
        return go2::LoadPolicyConfigFromYaml(temp.write(document), name, models.string());
    };
    for (const auto &[key, value] : std::vector<std::pair<std::string, YAML::Node>>{
            {"action_joint_index", YAML::Load("[0,0,2,3,4,5,6,7,8,9,10,11]")}, {"rl_default_pos", YAML::Load("[0,0]")},
            {"action_scale", YAML::Load("[.2,.3]")}, {"kp", YAML::Load("[-1,1,1,1,1,1,1,1,1,1,1,1]")},
            {"kd", YAML::Load("[.nan,1,1,1,1,1,1,1,1,1,1,1]")}}) {
        auto document = YAML::Clone(original);
        policies(document)["terrain"][key] = value;
        rejects([&] { go2::PolicyAdapter adapter(temp.write(document), models.string()); });
    }
    for (const auto &[key, value] : std::vector<std::pair<std::string, YAML::Node>>{
            {"backend", YAML::Node("mnn")},
            {"model_path", YAML::Node("missing.onnx")},
            {"runtime", YAML::Load("{threads: 0}")}, {"model_io", YAML::Load("{inputs: [], outputs: []}")}}) {
        auto document = YAML::Clone(original);
        policies(document)["terrain"][key] = value;
        rejects([&] { (void)load(document, "terrain"); });
    }
    {
        auto document = YAML::Clone(original);
        policies(document)["terrain"]["observation"]["segment_0_terms"] =
            YAML::Load("[ang_vel, gravity, command, dof_pos, dof_vel]");
        const auto invalid = load(document, "terrain");
        rl_policy::PolicyExecutor executor;
        rejects([&] { executor.Init(invalid.exec_cfg); });
    }
    {
        auto document = YAML::Clone(original);
        document["go2_modes"].remove("terrain");
        rejects([&] { go2::Policy policy(models.string(), temp.write(document)); });
    }
    {
        auto document = YAML::Clone(original);
        document["rl_policy"]["rl_dt"] = .01;
        rejects([&] { go2::Policy policy(models.string(), temp.write(document)); });
    }

    {
        auto document = YAML::Clone(original);
        policies(document)["terrain"]["model_path"] = "missing.onnx";
        rejects([&] { go2::Policy policy(models.string(), temp.write(document)); });
    }
    {
        auto document = YAML::Clone(original);
        document["go2_modes"]["terrain"]["models"] = YAML::Load("[terrain, terrain]");
        rejects([&] { go2::Policy policy(models.string(), temp.write(document)); });
    }
}

} // namespace

int main(int argc, char **argv) {
    assert(argc == 2);
    const auto models = fs::absolute(argv[1]);
    const auto source = fs::path(GO2_TEST_POLICY_CONFIG);
    const auto original = YAML::LoadFile(source.string());
    // Temporary configurations keep references to the application's resources.
    for (const auto &entry : original["go2_modes"]) {
        if (auto motion = entry.second["motion"])
            motion["path"] = (source.parent_path() / motion["path"].as<std::string>()).lexically_normal().string();
    }
    TemporaryConfig temp;

    const auto load = [&](const YAML::Node &document, const std::string &name) {
        return go2::LoadPolicyConfigFromYaml(temp.write(document), name, models.string());
    };
    auto terrain = load(original, "terrain");
    assert(terrain.exec_cfg.runtime.provider == "cpu" && terrain.exec_cfg.runtime.threads == 1);
    assert(terrain.exec_cfg.model_path == (models / "go2_terrain.onnx").string());
    assert(terrain.exec_cfg.obs_segments.front().terms.size() == 6);
    assert(terrain.exec_cfg.clip_actions.empty());

    check_config_overrides(original, models, temp);
    check_invalid_config(original, models, temp);
    check_policy_contracts(models.string());
    std::cout << "PASS: model contracts, mappings, ONNX episodes, YAML overrides and invalid inputs\n";
}
