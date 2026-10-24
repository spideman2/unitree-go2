/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "hardware.hpp"
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/random.h>
#include <unistd.h>
#include <cstdlib>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>

#include <cmath>
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <pthread.h>
#include <sched.h>
#include <shared_mutex>
#include <stdexcept>
#include <string>

namespace {

/* G2MR v2 wire helpers are private to the Go2 RPMsg transport. */
constexpr std::size_t GO2_RPC_SIZE = 152U;
constexpr std::uint8_t GO2_RPC_VERSION = 2U;
constexpr std::uint8_t GO2_RPC_HELLO = 1U;
constexpr std::uint8_t GO2_RPC_SET = 2U;
constexpr std::uint8_t GO2_RPC_STOP = 3U;
constexpr std::uint8_t GO2_RPC_GET = 4U;
constexpr std::uint32_t GO2_RPC_FAULT = 1U;
constexpr std::uint32_t GO2_RPC_STOPPED = 2U;
constexpr std::uint32_t GO2_RPC_RCPU_BASE = 1100U;
constexpr std::uint32_t GO2_RPC_LINUX_BASE = 1200U;

std::uint32_t go2_get32(const std::uint8_t *p) {
    return static_cast<std::uint32_t>(p[0]) |
        static_cast<std::uint32_t>(p[1]) << 8 |
        static_cast<std::uint32_t>(p[2]) << 16 |
        static_cast<std::uint32_t>(p[3]) << 24;
}

void go2_put32(std::uint8_t *p, std::uint32_t n) {
    p[0] = static_cast<std::uint8_t>(n);
    p[1] = static_cast<std::uint8_t>(n >> 8);
    p[2] = static_cast<std::uint8_t>(n >> 16);
    p[3] = static_cast<std::uint8_t>(n >> 24);
}

std::uint64_t go2_get64(const std::uint8_t *p) {
    return go2_get32(p) | (static_cast<std::uint64_t>(go2_get32(p + 4)) << 32);
}

void go2_put64(std::uint8_t *p, std::uint64_t n) {
    go2_put32(p, static_cast<std::uint32_t>(n));
    go2_put32(p + 4, static_cast<std::uint32_t>(n >> 32));
}

float go2_get_float(const std::uint8_t *p) {
    const std::uint32_t bits = go2_get32(p);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

void go2_put_float(std::uint8_t *p, float value) {
    std::uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    go2_put32(p, bits);
}

void go2_rpc_header(std::uint8_t *p, std::uint8_t op, std::uint8_t leg,
    std::uint32_t seq, std::uint64_t session) {
    std::memset(p, 0, GO2_RPC_SIZE);
    std::memcpy(p, "G2MR", 4);
    p[4] = GO2_RPC_VERSION;
    p[5] = op;
    p[6] = leg;
    go2_put32(p + 8, seq);
    go2_put64(p + 12, session);
}

bool go2_rpc_valid(const std::uint8_t *p, std::size_t n) {
    return n == GO2_RPC_SIZE && !std::memcmp(p, "G2MR", 4) &&
        p[4] == GO2_RPC_VERSION && p[6] < 4;
}

} // namespace

namespace go2 {

namespace {

bool finite(float value) { return std::isfinite(value); }

void validate_feedback(const RotorFeedback &feedback, const JointCalibration &calibration) {
    validate_calibration(calibration);
    if (feedback.motor_id != calibration.motor_id) {
        throw std::runtime_error("SDK feedback motor ID does not match calibration");
    }
    if (!finite(feedback.torque_nm) || !finite(feedback.velocity_rad_s) || !finite(feedback.position_rad)) {
        throw std::runtime_error("SDK feedback contains NaN or infinity");
    }
}

JointFeedback copy_feedback_diagnostics(const RotorFeedback &feedback) {
    JointFeedback joint;
    joint.rotor_position_rad = feedback.position_rad;
    joint.rotor_velocity_rad_s = feedback.velocity_rad_s;
    joint.rotor_torque_nm = feedback.torque_nm;
    joint.temperature_c = feedback.temperature_c;
    joint.error = feedback.error;
    joint.foot_force_raw = feedback.foot_force_raw;
    return joint;
}

} // namespace

void validate_calibration(const JointCalibration &calibration) {
    if (calibration.motor_id > 14U) {
        throw std::invalid_argument("motor ID must be in [0, 14]");
    }
    if (!finite(calibration.gear_ratio) || calibration.gear_ratio <= 0.0F) {
        throw std::invalid_argument("gear ratio must be finite and positive");
    }
    if (calibration.direction != 1.0F && calibration.direction != -1.0F) {
        throw std::invalid_argument("joint direction must be +1 or -1");
    }
    if (!finite(calibration.joint_zero_offset_rad)) {
        throw std::invalid_argument("joint zero offset must be finite");
    }
}

RotorCommand joint_to_rotor_command(const JointCommand &command, const JointCalibration &calibration) {
    validate_calibration(calibration);
    RotorCommand rotor;
    rotor.motor_id = calibration.motor_id;
    rotor.mode = command.mode;
    if (command.mode == MotorMode::kBrake)
        return rotor;

    if (!finite(command.torque_nm) || !finite(command.velocity_rad_s) || !finite(command.position_rad) ||
        !finite(command.kp) || !finite(command.kd) || command.kp < 0.0F || command.kd < 0.0F) {
        throw std::invalid_argument("FOC command contains invalid values");
    }

    const float ratio = calibration.gear_ratio;
    const float direction = calibration.direction;
    rotor.position_rad = direction * (command.position_rad - calibration.joint_zero_offset_rad) * ratio;
    rotor.velocity_rad_s = direction * command.velocity_rad_s * ratio;
    rotor.torque_nm = direction * command.torque_nm / ratio;
    rotor.kp = command.kp / (ratio * ratio);
    rotor.kd = command.kd / (ratio * ratio);
    return rotor;
}

JointFeedback rotor_to_joint_feedback(const RotorFeedback &feedback, const JointCalibration &calibration) {
    validate_feedback(feedback, calibration);

    const float ratio = calibration.gear_ratio;
    const float direction = calibration.direction;
    JointFeedback joint = copy_feedback_diagnostics(feedback);
    joint.position_rad = direction * feedback.position_rad / ratio + calibration.joint_zero_offset_rad;
    joint.velocity_rad_s = direction * feedback.velocity_rad_s / ratio;
    joint.torque_nm = direction * feedback.torque_nm * ratio;
    joint.position_anchored = true;
    return joint;
}

JointFeedback rotor_to_unanchored_feedback(const RotorFeedback &feedback, const JointCalibration &calibration) {
    validate_feedback(feedback, calibration);
    return copy_feedback_diagnostics(feedback);
}

void JointSessionAnchor::initialize(float current_rotor_position_rad, const JointSessionReference &reference,
    const JointCalibration &calibration, float maximum_joint_error_rad) {
    validate_calibration(calibration);
    if (!finite(current_rotor_position_rad) || !finite(reference.joint_position_rad) ||
        !finite(reference.rotor_position_rad) || !finite(maximum_joint_error_rad) || maximum_joint_error_rad <= 0.0F) {
        throw std::invalid_argument("invalid session anchor configuration");
    }
    const float maximum_phase_error_rad = maximum_joint_error_rad * calibration.gear_ratio;
    if (!finite(maximum_phase_error_rad) || maximum_phase_error_rad >= kTwoPi * 0.5F) {
        throw std::invalid_argument("startup joint tolerance must map inside half a rotor turn");
    }

    const float phase_error = std::remainder(current_rotor_position_rad - reference.rotor_position_rad, kTwoPi);
    if (std::fabs(phase_error) > maximum_phase_error_rad) {
        const float joint_error_rad = calibration.direction * phase_error / calibration.gear_ratio;
        const float radians_to_degrees = 360.0F / kTwoPi;
        throw std::runtime_error("startup joint position is outside the Pose A tolerance: "
                                "joint_error_rad=" +
            std::to_string(joint_error_rad) +
            " joint_error_deg=" + std::to_string(joint_error_rad * radians_to_degrees) +
            " tolerance_rad=" + std::to_string(maximum_joint_error_rad) +
            " tolerance_deg=" + std::to_string(maximum_joint_error_rad * radians_to_degrees) +
            " current_rotor_rad=" + std::to_string(current_rotor_position_rad) +
            " reference_rotor_rad=" + std::to_string(reference.rotor_position_rad));
    }

    reference_ = reference;
    session_reference_rotor_position_rad_ = current_rotor_position_rad - phase_error;
    last_rotor_position_rad_ = current_rotor_position_rad;
    last_rotor_velocity_rad_s_ = 0.0F;
    last_sampled_at_ = {};
    valid_ = true;
    reference_initialized_ = true;
}

void JointSessionAnchor::invalidate() { valid_ = false; }

JointFeedback JointSessionAnchor::to_joint_feedback(
    const RotorFeedback &feedback, const JointCalibration &calibration, bool enforce_step_continuity,
    TimePoint sampled_at) {
    if (!valid_) {
        throw std::runtime_error("joint session anchor is not valid");
    }
    validate_feedback(feedback, calibration);
    const float rotor_step = feedback.position_rad - last_rotor_position_rad_;
    bool discontinuity = std::fabs(rotor_step) >= kTwoPi * 0.5F;
    double sample_gap = 0.0;
    double expected_step = 0.0;
    // Only native callers provide sample times. USB keeps the exact half-turn
    // rule. A lost reply can span multiple rotor turns without losing the
    // absolute position reference: accept it only when endpoint velocities
    // explain the displacement inside the existing 300ms freshness bound.
    if (sampled_at != TimePoint{} && last_sampled_at_ != TimePoint{} && sampled_at > last_sampled_at_) {
        sample_gap = seconds(sampled_at - last_sampled_at_);
        expected_step = 0.5 * (last_rotor_velocity_rad_s_ + feedback.velocity_rad_s) * sample_gap;
        // Joint-space margin covers quantisation/timestamp jitter. Velocity
        // changes widen it, capped below half a turn so an unexplained counter
        // reset never becomes acceptable merely because several replies died.
        const double tolerance = std::min(static_cast<double>(kTwoPi) / 4.0,
            0.05 * calibration.gear_ratio +
                0.5 * std::fabs(feedback.velocity_rad_s - last_rotor_velocity_rad_s_) * sample_gap);
        if (discontinuity && sample_gap < 0.30 && std::fabs(rotor_step - expected_step) <= tolerance)
            discontinuity = false;
    }
    if (enforce_step_continuity && discontinuity) {
        invalidate();
        std::string reason = "motor position discontinuity invalidated the session anchor";
        if (sampled_at != TimePoint{})
            reason += "; rotor_step_rad=" + std::to_string(rotor_step) +
                " sample_gap_sec=" + std::to_string(sample_gap) +
                " expected_step_rad=" + std::to_string(expected_step);
        throw std::runtime_error(reason);
    }
    // Keep the position, velocity and time baseline from the same sample.
    // RPMsg age estimation can make adjacent timestamps equal or regress.
    if (sampled_at == TimePoint{} || sampled_at > last_sampled_at_) {
        last_rotor_position_rad_ = feedback.position_rad;
        if (sampled_at != TimePoint{}) {
            last_rotor_velocity_rad_s_ = feedback.velocity_rad_s;
            last_sampled_at_ = sampled_at;
        }
    }
    JointFeedback joint = copy_feedback_diagnostics(feedback);
    joint.position_rad = reference_.joint_position_rad +
        calibration.direction * (feedback.position_rad - session_reference_rotor_position_rad_) /
            calibration.gear_ratio;
    joint.velocity_rad_s = calibration.direction * feedback.velocity_rad_s / calibration.gear_ratio;
    joint.torque_nm = calibration.direction * feedback.torque_nm * calibration.gear_ratio;
    joint.position_anchored = true;
    return joint;
}

RotorCommand JointSessionAnchor::to_rotor_command(
    const JointCommand &command, const JointCalibration &calibration) const {
    if (command.mode == MotorMode::kBrake) {
        return joint_to_rotor_command(command, calibration);
    }
    if (!can_command(command)) {
        throw std::runtime_error("FOC command requires a valid session anchor");
    }
    RotorCommand rotor = joint_to_rotor_command(command, calibration);
    rotor.position_rad = session_reference_rotor_position_rad_ +
        calibration.direction * (command.position_rad - reference_.joint_position_rad) * calibration.gear_ratio;
    return rotor;
}

} // namespace go2

#include <atomic>
#include <memory>
#include "imu.h"

#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "motor.h"

namespace go2 {
class MotorBank {
    public:
    explicit MotorBank(std::array<LegConfig, kNumGo2Legs> configs, bool command_watchdog = false,
        bool native_uart = false);
    MotorBank(const MotorBank &) = delete;
    MotorBank &operator=(const MotorBank &) = delete;
    ~MotorBank();

    void start();
    void stop();
    bool running() const { return running_.load(); }

    void set_commands(const std::array<JointCommand, kNumGo2Motors> &commands);
    MotorSnapshot snapshot() const;

    private:
    struct LegRuntime;
    void run_leg(std::size_t leg_index);
    void run_rpmsg_leg(std::size_t leg_index);

    std::array<LegConfig, kNumGo2Legs> configs_;
    std::array<std::unique_ptr<LegRuntime>, kNumGo2Legs> legs_;
    std::atomic_bool running_{false};
    bool command_watchdog_{false};
    bool native_uart_{false};
    std::atomic<std::int64_t> last_command_ns_{0};
    // Before the first FOC command, every motor is held in BRAKE at the
    // configured Pose A. A transient serial failure is therefore recoverable by
    // reconnecting and validating Pose A again. Once active control has begun,
    // the session anchor remains fail-closed for the rest of the process.
    std::atomic_bool active_control_started_{false};
    std::atomic_bool session_anchor_faulted_{false};
};

namespace {

struct RpmsgSnapshot {
    std::array<RotorFeedback, kMotorsPerLeg> motors{};
    uint32_t age_ms[3]{}, sample[3]{}, applied_sequence[3]{};
    uint8_t valid[3]{}, mode[3]{};
    uint32_t flags{}, accepted_sequence{}, sweeps{};
};

struct RpmsgSession {
    int fd;
    uint8_t leg;
    uint32_t seq;
    uint64_t session;
    int failed;
    TimePoint command_sent_at{};
};
static int64_t rpmsg_milliseconds(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}
static int rpmsg_endpoint_address(unsigned index, const char *attribute, unsigned expected) {
    char path[128], value[32] = {0}, *end;
    snprintf(path, sizeof(path), "/sys/class/rpmsg/rpmsg%u/%s", index, attribute);
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0)
        return 0;
    ssize_t n = read(fd, value, sizeof(value) - 1);
    close(fd);
    if (n <= 0)
        return 0;
    unsigned long actual = strtoul(value, &end, 10);
    return end != value && (*end == '\n' || *end == '\0') && actual == expected;
}
static int rpmsg_endpoint(const char *service, unsigned leg) {
    for (unsigned i = 0; i < 256; i++) {
        char path[128], name[64] = {0};
        snprintf(path, sizeof(path), "/sys/class/rpmsg/rpmsg%u/name", i);
        int f = open(path, O_RDONLY | O_CLOEXEC);
        if (f < 0)
            continue;
        ssize_t n = read(f, name, sizeof(name) - 1);
        close(f);
        if (n <= 0)
            continue;
        name[strcspn(name, "\n")] = 0;
        if (strcmp(name, service))
            continue;
        if (!rpmsg_endpoint_address(i, "src", GO2_RPC_LINUX_BASE + leg) ||
            !rpmsg_endpoint_address(i, "dst", GO2_RPC_RCPU_BASE + leg))
            continue;
        snprintf(path, sizeof(path), "/dev/rpmsg%u", i);
        return open(path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    }
    return -1;
}
static int rpmsg_exchange(RpmsgSession *p, uint8_t op, const uint8_t *frame, uint8_t *reply) {
    uint8_t q[GO2_RPC_SIZE], r[GO2_RPC_SIZE + 1];
    if (p->seq == UINT32_MAX)
        return -EOVERFLOW;
    go2_rpc_header(q, op, p->leg, ++p->seq, p->session);
    if (frame)
        memcpy(q + 32, frame, 120);
    int64_t deadline = rpmsg_milliseconds() + 100;
    for (;;) {
        ssize_t n = write(p->fd, q, sizeof(q));
        if (n == (ssize_t)sizeof(q)) {
            if (op == GO2_RPC_SET)
                p->command_sent_at = SteadyClock::now();
            break;
        }
        if (n >= 0)
            return -EIO;
        if (errno != EINTR && errno != EAGAIN)
            return -errno;
        if (rpmsg_milliseconds() >= deadline)
            return -ETIMEDOUT;
        struct pollfd f = {p->fd, POLLOUT, 0};
        poll(&f, 1, 1);
    }
    while (rpmsg_milliseconds() < deadline) {
        struct pollfd f = {p->fd, POLLIN, 0};
        int64_t left = deadline - rpmsg_milliseconds();
        if (left <= 0)
            return -ETIMEDOUT;
        int ready = poll(&f, 1, (int)left);
        if (ready < 0 && errno == EINTR)
            continue;
        if (ready <= 0)
            return ready < 0 ? -errno : -ETIMEDOUT;
        if (f.revents & (POLLERR | POLLHUP | POLLNVAL))
            return -EIO;
        ssize_t n = read(p->fd, r, sizeof(r));
        if (n < 0 && (errno == EAGAIN || errno == EINTR))
            continue;
        if (n <= 0)
            return -EIO;
        if (n != GO2_RPC_SIZE)
            return -EPROTO;
        if (go2_get64(r + 12) != p->session || go2_get32(r + 8) != p->seq)
            continue;
        if (!go2_rpc_valid(r, (size_t)n))
            return -EPROTO;
        if (r[5] != (op | 0x80) || r[6] != p->leg)
            return -EPROTO;
        if (r[7])
            return -EREMOTEIO;
        if (reply)
            memcpy(reply, r, GO2_RPC_SIZE);
        return 0;
    }
    return -ETIMEDOUT;
}
RpmsgSession *rpmsg_open(const char *service) {
    static const char *names[] = {"go2-motor-fr", "go2-motor-fl", "go2-motor-rr", "go2-motor-rl"};
    if (!service) {
        errno = EINVAL;
        return NULL;
    }
    unsigned leg;
    int ret;
    uint8_t stale[512];
    for (leg = 0; leg < 4; leg++)
        if (!strcmp(names[leg], service))
            break;
    if (leg == 4) {
        errno = EINVAL;
        return NULL;
    }
    auto *p = new (std::nothrow) RpmsgSession{};
    if (!p)
        return NULL;
    p->leg = (uint8_t)leg;
    p->fd = rpmsg_endpoint(service, leg);
    if (p->fd < 0) {
        struct endpoint_info {
            char name[32];
            uint32_t src, dst;
        } info{};
        strcpy(info.name, service);
        info.src = GO2_RPC_LINUX_BASE + leg;
        info.dst = GO2_RPC_RCPU_BASE + leg;
        int ctrl = open("/dev/rpmsg_ctrl0", O_RDWR | O_CLOEXEC);
        if (ctrl < 0)
            goto fail;
        int ret = ioctl(ctrl, _IOW(0xb5, 1, struct endpoint_info), &info);
        int err = errno;
        close(ctrl);
        if (ret < 0 && err != EEXIST) {
            errno = err;
            goto fail;
        }
        for (int attempt = 0; attempt < 50 && p->fd < 0; attempt++) {
            usleep(20000);
            p->fd = rpmsg_endpoint(service, leg);
        }
    }
    if (p->fd < 0 || flock(p->fd, LOCK_EX | LOCK_NB) < 0)
        goto fail;
    if (getrandom(&p->session, sizeof(p->session), 0) != sizeof(p->session) || !p->session)
        goto fail;
    for (unsigned i = 0; i < 256; i++)
        if (read(p->fd, stale, sizeof(stale)) <= 0)
            break;
    ret = rpmsg_exchange(p, GO2_RPC_HELLO, NULL, NULL);
    if (ret < 0) {
        errno = -ret;
        goto fail;
    }
    return p;
fail:
    {
        int saved = errno;
        if (p->fd >= 0)
            close(p->fd);
        delete p;
        errno = saved;
        return NULL;
    }
}

int rpmsg_set(RpmsgSession *p, const RotorCommand commands[3]) {
    uint8_t records[120] = {0};
    if (!p || !commands)
        return -EINVAL;
    if (p->failed)
        return -EIO;
    for (unsigned id = 0; id < 3; ++id) {
        const RotorCommand *c = &commands[id];
        if (c->mode != MotorMode::kBrake && c->mode != MotorMode::kFoc)
            return -EINVAL;
        if (c->mode == MotorMode::kBrake)
            continue;
        if (!std::isfinite(c->torque_nm) || !std::isfinite(c->velocity_rad_s) || !std::isfinite(c->position_rad) ||
            !std::isfinite(c->kp) || !std::isfinite(c->kd) || c->kp < 0 || c->kd < 0)
            return -EINVAL;
        uint8_t *r = records + id * 40;
        go2_put32(r, 1);
        go2_put_float(r + 4, c->torque_nm);
        go2_put_float(r + 8, c->velocity_rad_s);
        go2_put_float(r + 12, c->position_rad);
        go2_put_float(r + 16, c->kp);
        go2_put_float(r + 20, c->kd);
    }
    int result = rpmsg_exchange(p, GO2_RPC_SET, records, NULL);
    if (result < 0 && result != -ETIMEDOUT)
        p->failed = 1;
    return result;
}
int rpmsg_get(RpmsgSession *p, RpmsgSnapshot *out) {
    uint8_t response[GO2_RPC_SIZE];
    if (!out)
        return -EINVAL;
    *out = {};
    if (!p || p->failed)
        return -EIO;
    int64_t start = rpmsg_milliseconds();
    int result = rpmsg_exchange(p, GO2_RPC_GET, NULL, response);
    if (result < 0) {
        if (result != -ETIMEDOUT)
            p->failed = 1;
        return result;
    }
    RpmsgSnapshot snapshot{};
    snapshot.flags = go2_get32(response + 20);
    snapshot.accepted_sequence = go2_get32(response + 24);
    snapshot.sweeps = go2_get32(response + 28);
    for (unsigned id = 0; id < 3; ++id) {
        const uint8_t *r = response + 32 + id * 40;
        if (r[0] > 1 || r[1] > 1 || r[2] != id || go2_get32(r + 36)) {
            p->failed = 1;
            return -EPROTO;
        }
        snapshot.valid[id] = r[0];
        snapshot.mode[id] = r[1];
        snapshot.motors[id].torque_nm = go2_get_float(r + 4);
        snapshot.motors[id].velocity_rad_s = go2_get_float(r + 8);
        snapshot.motors[id].position_rad = go2_get_float(r + 12);
        const float temperature = go2_get_float(r + 16);
        if (!std::isfinite(snapshot.motors[id].torque_nm) || !std::isfinite(snapshot.motors[id].velocity_rad_s) ||
            !std::isfinite(snapshot.motors[id].position_rad) || !std::isfinite(temperature) || temperature < -128 ||
            temperature > 127) {
            p->failed = 1;
            return -EPROTO;
        }
        snapshot.motors[id].motor_id = static_cast<uint8_t>(id);
        snapshot.motors[id].temperature_c = static_cast<int>(temperature);
        snapshot.motors[id].error = r[3];
        snapshot.motors[id].foot_force_raw = (uint16_t)go2_get32(r + 20);
        uint64_t age = (uint64_t)go2_get32(r + 24) + (uint64_t)(rpmsg_milliseconds() - start) + 1;
        snapshot.age_ms[id] = age > UINT32_MAX ? UINT32_MAX : (uint32_t)age;
        snapshot.sample[id] = go2_get32(r + 28);
        snapshot.applied_sequence[id] = go2_get32(r + 32);
    }
    *out = snapshot;
    return 0;
}
int rpmsg_stop(RpmsgSession *p) {
    if (!p)
        return -EINVAL;
    uint8_t r[GO2_RPC_SIZE];
    int result = rpmsg_exchange(p, GO2_RPC_STOP, NULL, r);
    if (result < 0)
        return result;
    int64_t deadline = rpmsg_milliseconds() + 100;
    do {
        if (go2_get32(r + 20) & GO2_RPC_STOPPED)
            return 0;
        usleep(1000);
        result = rpmsg_exchange(p, GO2_RPC_GET, NULL, r);
        if (result < 0)
            return result;
    } while (rpmsg_milliseconds() < deadline);
    return -ETIMEDOUT;
}
void rpmsg_close(RpmsgSession *p) {
    if (!p)
        return;
    (void)rpmsg_stop(p);
    close(p->fd);
    delete p;
}

class MotorTransportError final : public std::runtime_error {
    public:
    explicit MotorTransportError(std::string message, bool retryable = false)
        : std::runtime_error(std::move(message)), retryable_(retryable) {}

    bool retryable() const noexcept { return retryable_; }

    private:
    bool retryable_;
};

std::int64_t native_uart_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(SteadyClock::now().time_since_epoch()).count();
}

// One writer per leg. Diagnostic readers never wait for a stalled SDK call.
struct NativeUartProgress {
    std::atomic<const char*> stage{"inactive"};
    std::atomic<int> motor_id{-1}, command_mode{-1}, last_result{0};
    std::atomic<std::int64_t> stage_at_ns{0}, transaction_ns{0};
    std::atomic<std::uint64_t> transactions{0}, failures{0};

    void mark(const char* value) {
        stage_at_ns.store(native_uart_now_ns(), std::memory_order_relaxed);
        stage.store(value, std::memory_order_release);
    }

    NativeUartDiagnostics snapshot() const {
        NativeUartDiagnostics result;
        result.stage = stage.load(std::memory_order_acquire);
        const auto started = stage_at_ns.load(std::memory_order_relaxed);
        result.stage_age_ms = started ? std::max<std::int64_t>(0, native_uart_now_ns() - started) / 1e6 : 0;
        result.motor_id = motor_id.load(std::memory_order_relaxed);
        result.command_mode = command_mode.load(std::memory_order_relaxed);
        result.last_result = last_result.load(std::memory_order_relaxed);
        result.last_transaction_ms = transaction_ns.load(std::memory_order_relaxed) / 1e6;
        result.transactions = transactions.load(std::memory_order_relaxed);
        result.failures = failures.load(std::memory_order_relaxed);
        return result;
    }
};

RotorFeedback transact(struct motor_dev *device, const JointCommand &joint_command, const JointCalibration &calibration,
    const JointSessionAnchor *session_anchor = nullptr, bool native_uart = false,
    NativeUartProgress *progress = nullptr,
    std::uint64_t *command_sent_us = nullptr) {
    if (progress)
        progress->motor_id.store(calibration.motor_id, std::memory_order_relaxed);
    const RotorCommand rotor_command = session_anchor == nullptr ?
        joint_to_rotor_command(joint_command, calibration) :
        session_anchor->to_rotor_command(joint_command, calibration);
    struct motor_cmd command{};
    command.mode = rotor_command.mode == MotorMode::kBrake ? MOTOR_MODE_IDLE : MOTOR_MODE_HYBRID;
    command.trq_des = rotor_command.torque_nm;
    command.vel_des = rotor_command.velocity_rad_s;
    command.pos_des = rotor_command.position_rad;
    command.kp = rotor_command.kp;
    command.kd = rotor_command.kd;
    struct motor_state feedback{};
    std::uint64_t previous_sent_us = 0;
    if (command_sent_us) {
        *command_sent_us = 0;
        if (motor_get_paras(device, "command_sent_us", &previous_sent_us, sizeof(previous_sent_us)) < 0)
            throw MotorTransportError("SDK command transmit timestamp unavailable");
    }
    if (progress) {
        progress->command_mode.store(static_cast<int>(rotor_command.mode));
        progress->mark("set_cmd_send_recv");
    }
    const auto transaction_begin = native_uart || progress ? native_uart_now_ns() : 0;
    int transport_result = motor_set_cmd_one(device, &command);
    // Keep the native-UART deadline check local to this leg's SDK call.
    if (native_uart && native_uart_now_ns() - transaction_begin >= 10000000)
        transport_result = -ETIMEDOUT;
    if (command_sent_us) {
        std::uint64_t sent_us = 0;
        if (motor_get_paras(device, "command_sent_us", &sent_us, sizeof(sent_us)) < 0)
            throw MotorTransportError("SDK command transmit timestamp unavailable");
        // A failed write leaves the SDK's earlier BRAKE/FOC timestamp intact.
        // Attribute a send to this command only when that timestamp advances.
        if (sent_us > previous_sent_us)
            *command_sent_us = sent_us;
    }
    if (progress) {
        progress->transaction_ns.store(native_uart_now_ns() - transaction_begin);
        progress->last_result.store(transport_result);
        ++progress->transactions;
        if (transport_result < 0)
            ++progress->failures;
    }
    if (transport_result < 0) {
        throw MotorTransportError(
            "SDK motor " + std::to_string(calibration.motor_id) + " did not return a valid reply (" +
                std::strerror(-transport_result) + ")",
            transport_result == -ETIMEDOUT ||
                (native_uart && (transport_result == -EBADMSG || transport_result == -ENOMSG)));
    }
    if (progress)
        progress->mark("get_state");
    transport_result = motor_get_state_one(device, &feedback);
    if (transport_result < 0) {
        throw MotorTransportError(
            "SDK motor " + std::to_string(calibration.motor_id) + " did not return a valid reply (" +
                std::strerror(-transport_result) + ")",
            transport_result == -ETIMEDOUT ||
                (native_uart && (transport_result == -EBADMSG || transport_result == -ENOMSG)));
    }
    std::uint8_t feedback_id = 0;
    std::uint16_t foot_force_raw = 0;
    if (progress)
        progress->mark("get_paras");
    if (motor_get_paras(device, "feedback_id", &feedback_id, sizeof(feedback_id)) < 0 ||
        feedback_id != calibration.motor_id) {
        throw MotorTransportError("SDK feedback motor ID does not match the request");
    }
    static_cast<void>(motor_get_paras(device, "foot_force_raw", &foot_force_raw, sizeof(foot_force_raw)));
    RotorFeedback result;
    result.motor_id = feedback_id;
    result.torque_nm = feedback.trq;
    result.velocity_rad_s = feedback.vel;
    result.position_rad = feedback.pos;
    result.temperature_c = feedback.temp;
    result.error = static_cast<std::uint8_t>(feedback.err);
    result.foot_force_raw = foot_force_raw;
    if (progress)
        progress->mark("get_timestamp");
    if (native_uart && motor_get_feedback_timestamps(&device, &result.received_us, 1) < 0)
        throw MotorTransportError("SDK motor feedback timestamp unavailable");
    if (progress)
        progress->mark("decode_feedback");
    return result;
}

class LegDevices final {
    public:
    explicit LegDevices(const LegConfig &config) {
        try {
            for (std::size_t index = 0; index < devices_.size(); ++index) {
                devices_[index] = motor_alloc_uart(config.driver.c_str(), config.device.c_str(), kMotorBaudrate,
                    config.calibration[index].motor_id, nullptr);
                if (!devices_[index]) {
                    throw MotorTransportError("failed to allocate motor driver");
                }
            }
            if (motor_init(devices_.data(), devices_.size()) < 0) {
                throw MotorTransportError("failed to initialize motor UART devices");
            }
        } catch (...) {
            motor_free(devices_.data(), devices_.size());
            devices_.fill(nullptr);
            throw;
        }
    }

    ~LegDevices() { motor_free(devices_.data(), devices_.size()); }
    LegDevices(const LegDevices &) = delete;
    LegDevices &operator=(const LegDevices &) = delete;

    struct motor_dev *at(std::size_t index) const { return devices_.at(index); }

    private:
    std::array<struct motor_dev *, kMotorsPerLeg> devices_{};
};

JointCommand brake_command() {
    JointCommand command;
    command.mode = MotorMode::kBrake;
    return command;
}

void record_transmission(JointTransmission &previous, const JointCommand &command, TimePoint sent_at) {
    if (sent_at <= previous.sent_at)
        return;
    const auto id = command.mode == MotorMode::kFoc && command.protective_crouch ? command.protective_crouch_id : 0;
    if (id == 0) {
        // A watchdog BRAKE interrupts execution but must not erase the already
        // transmitted part of this episode or rewind its interpolation.
        previous.sent_at = sent_at;
        previous.protective_active = previous.protective_final = false;
        return;
    }
    double elapsed = previous.protective_crouch_id == id ? previous.protective_elapsed_sec : 0;
    const double interval = seconds(sent_at - previous.sent_at);
    // Command watchdog coverage is 100ms. A longer gap is an interrupted
    // transmission interval, not time spent executing the crouch.
    if (previous.protective_active && previous.protective_crouch_id == id && interval <= .100)
        elapsed += interval;
    previous = {sent_at, id, elapsed, command.protective_crouch_final, true};
}

} // namespace

struct MotorBank::LegRuntime {
    NativeUartProgress native_uart;
    mutable std::mutex mutex;
    std::array<JointCommand, kMotorsPerLeg> commands{};
    std::array<TimedJointFeedback, kMotorsPerLeg> feedback{};
    std::array<JointTransmission, kMotorsPerLeg> transmissions{};
    std::array<JointSessionAnchor, kMotorsPerLeg> anchors{};
    std::uint64_t command_generation{0};
    SteadyClock::time_point command_at{};
    std::uint64_t completed_sweeps{0};
    std::uint64_t period_samples{0};
    std::uint64_t transaction_failures{0};
    std::uint64_t reconnect_attempts{0};
    std::uint64_t consecutive_failures{0};
    double mean_period_ms{0.0};
    double period_m2_ms2{0.0};
    double minimum_period_ms{std::numeric_limits<double>::infinity()};
    double maximum_period_ms{0.0};
    std::chrono::steady_clock::time_point previous_sweep_completed_at{};
    std::string error;
    bool startup_pose_invalid{false};
    std::thread thread;
};

MotorBank::MotorBank(std::array<LegConfig, kNumGo2Legs> configs, bool command_watchdog, bool native_uart)
    : configs_(std::move(configs)), command_watchdog_(command_watchdog), native_uart_(native_uart) {
    for (std::size_t i = 0; i < legs_.size(); ++i) {
        legs_[i] = std::make_unique<LegRuntime>();
    }
}

MotorBank::~MotorBank() { stop(); }

void MotorBank::start() {
    if (running_.load())
        return;
    for (std::size_t i = 0; i < configs_.size(); ++i) {
        if (configs_[i].device.empty()) {
            throw std::invalid_argument("all four SDK serial devices are required");
        }
        if (configs_[i].startup_attempts == 0U) {
            throw std::invalid_argument("SDK startup attempts must be positive");
        }
        for (std::size_t local_index = 0; local_index < kMotorsPerLeg; ++local_index) {
            const float tolerance = configs_[i].startup_joint_tolerances_rad[local_index];
            if (!std::isfinite(tolerance) || tolerance <= 0.0F) {
                throw std::invalid_argument("SDK startup joint tolerance must be finite and positive");
            }
            const auto &calibration = configs_[i].calibration[local_index];
            validate_calibration(calibration);
            const float maximum_phase_error_rad = tolerance * calibration.gear_ratio;
            if (!std::isfinite(maximum_phase_error_rad) || maximum_phase_error_rad >= kTwoPi * 0.5F) {
                throw std::invalid_argument("SDK startup joint tolerance must map inside half a rotor turn");
            }
        }
        for (std::size_t j = i + 1; j < configs_.size(); ++j) {
            if (configs_[i].device == configs_[j].device) {
                throw std::invalid_argument("each leg must use a distinct SDK serial device");
            }
        }
    }

    active_control_started_.store(false);
    session_anchor_faulted_.store(false);
    for (auto &leg : legs_) {
        std::lock_guard<std::mutex> lock(leg->mutex);
        leg->error.clear();
        leg->completed_sweeps = 0;
        leg->command_generation = 0;
        leg->command_at = {};
        for (auto &command : leg->commands)
            command = brake_command();
        leg->period_samples = 0;
        leg->transaction_failures = 0;
        leg->reconnect_attempts = 0;
        leg->consecutive_failures = 0;
        leg->mean_period_ms = 0.0;
        leg->period_m2_ms2 = 0.0;
        leg->minimum_period_ms = std::numeric_limits<double>::infinity();
        leg->maximum_period_ms = 0.0;
        leg->previous_sweep_completed_at = {};
        leg->startup_pose_invalid = false;
        for (auto &feedback : leg->feedback)
            feedback = {};
        leg->transmissions.fill({});
        for (auto &anchor : leg->anchors)
            anchor = JointSessionAnchor{};
    }
    running_.store(true);
    try {
        for (std::size_t i = 0; i < legs_.size(); ++i) {
            legs_[i]->thread = std::thread(&MotorBank::run_leg, this, i);
        }
    } catch (...) {
        stop();
        throw;
    }
}

void MotorBank::stop() {
    running_.store(false);
    for (auto &leg : legs_) {
        if (leg && leg->thread.joinable())
            leg->thread.join();
    }
}

void MotorBank::set_commands(const std::array<JointCommand, kNumGo2Motors> &commands) {
    last_command_ns_.store(
        std::chrono::duration_cast<std::chrono::nanoseconds>(SteadyClock::now().time_since_epoch()).count());
    for (std::size_t leg_index = 0; leg_index < kNumGo2Legs; ++leg_index) {
        auto &leg = *legs_[leg_index];
        std::lock_guard<std::mutex> lock(leg.mutex);
        ++leg.command_generation;
        leg.command_at = SteadyClock::now();
        for (std::size_t local_index = 0; local_index < kMotorsPerLeg; ++local_index) {
            leg.commands[local_index] = commands[leg_index * kMotorsPerLeg + local_index];
        }
    }
}

MotorSnapshot MotorBank::snapshot() const {
    MotorSnapshot result;
    result.transmissions_tracked = true;
    result.session_anchor_faulted = session_anchor_faulted_.load();
    for (std::size_t leg_index = 0; leg_index < kNumGo2Legs; ++leg_index) {
        const auto &leg = *legs_[leg_index];
        if (native_uart_)
            result.native_uart[leg_index] = leg.native_uart.snapshot();
        std::lock_guard<std::mutex> lock(leg.mutex);
        result.leg_errors[leg_index] = leg.error;
        result.startup_pose_invalid = result.startup_pose_invalid || leg.startup_pose_invalid;
        auto &timing = result.leg_timing[leg_index];
        timing.completed_sweeps = leg.completed_sweeps;
        timing.period_samples = leg.period_samples;
        timing.transaction_failures = leg.transaction_failures;
        timing.reconnect_attempts = leg.reconnect_attempts;
        timing.consecutive_failures = leg.consecutive_failures;
        timing.mean_period_ms = leg.mean_period_ms;
        timing.minimum_period_ms = std::isfinite(leg.minimum_period_ms) ? leg.minimum_period_ms : 0.0;
        timing.maximum_period_ms = leg.maximum_period_ms;
        timing.standard_deviation_ms =
            leg.period_samples > 1U ? std::sqrt(leg.period_m2_ms2 / static_cast<double>(leg.period_samples - 1U)) : 0.0;
        for (std::size_t local_index = 0; local_index < kMotorsPerLeg; ++local_index) {
            result.joints[leg_index * kMotorsPerLeg + local_index] = leg.feedback[local_index];
            result.transmissions[leg_index * kMotorsPerLeg + local_index] = leg.transmissions[local_index];
        }
    }
    return result;
}

void MotorBank::run_leg(std::size_t leg_index) {
    if (configs_[leg_index].driver == "go2_smallcore") {
        run_rpmsg_leg(leg_index);
        return;
    }
    auto &runtime = *legs_[leg_index];
    const auto &config = configs_[leg_index];
    auto *progress = native_uart_ ? &runtime.native_uart : nullptr;
    if (native_uart_) {
        // Native UART workers outrank normal AI/camera work, but stay below
        // threaded IRQs (priority 50). RR also bounds equal-priority CPU use.
        const struct sched_param scheduling{10};
        const int result = pthread_setschedparam(pthread_self(), SCHED_RR, &scheduling);
        if (result != 0) {
            std::fprintf(stderr, "native UART %s: SCHED_RR priority 10 unavailable: %s\n",
                config.device.c_str(), std::strerror(result));
        }
    }

    while (running_.load()) {
        bool startup_complete = false;
        try {
            if (progress)
                progress->mark("initialize");
            LegDevices devices(config);

            for (std::size_t local_index = 0; local_index < kMotorsPerLeg; ++local_index) {
                bool initialized = false;
                std::string last_error;
                for (unsigned int attempt = 0; attempt < config.startup_attempts && running_.load(); ++attempt) {
                    try {
                        const RotorFeedback rotor =
                            transact(devices.at(local_index), brake_command(), config.calibration[local_index], nullptr,
                                native_uart_, progress);
                        auto &anchor = runtime.anchors[local_index];
                        if (config.startup_pose_verified && !session_anchor_faulted_.load()) {
                            anchor.initialize(rotor.position_rad, config.session_reference[local_index],
                                config.calibration[local_index], config.startup_joint_tolerances_rad[local_index]);
                        }
                        const JointFeedback joint = anchor.valid() ?
                            anchor.to_joint_feedback(rotor, config.calibration[local_index], true,
                                native_uart_ ? TimePoint(std::chrono::duration_cast<SteadyClock::duration>(
                                    std::chrono::microseconds(rotor.received_us))) : TimePoint{}) :
                            rotor_to_unanchored_feedback(rotor, config.calibration[local_index]);
                        std::lock_guard<std::mutex> lock(runtime.mutex);
                        runtime.feedback[local_index].feedback = joint;
                        runtime.feedback[local_index].received_at = std::chrono::steady_clock::now();
                        runtime.feedback[local_index].valid = true;
                        initialized = true;
                        break;
                    } catch (const MotorTransportError &error) {
                        last_error = error.what();
                    } catch (const std::exception &error) {
                        const bool fail_closed = active_control_started_.load();
                        if (fail_closed) {
                            session_anchor_faulted_.store(true);
                        }
                        for (auto &anchor : runtime.anchors)
                            anchor.invalidate();
                        {
                            std::lock_guard<std::mutex> lock(runtime.mutex);
                            runtime.startup_pose_invalid = true;
                        }
                        throw std::runtime_error("SDK startup anchor failed on motor " +
                            std::to_string(config.calibration[local_index].motor_id) + ": " + error.what());
                    }
                }
                if (!running_.load())
                    break;
                if (!initialized) {
                    throw std::runtime_error("SDK startup failed on motor " +
                        std::to_string(config.calibration[local_index].motor_id) + ": " + last_error);
                }
            }

            {
                std::lock_guard<std::mutex> lock(runtime.mutex);
                if (!session_anchor_faulted_.load())
                    runtime.error.clear();
                runtime.startup_pose_invalid = false;
            }
            startup_complete = true;

            auto next_sweep = SteadyClock::now();
            while (running_.load()) {
                const auto sweep_started_at = SteadyClock::now();
                bool sweep_failed = false;
                for (std::size_t local_index = 0; local_index < kMotorsPerLeg && running_.load(); ++local_index) {
                    if (!running_.load())
                        break;
                    if (progress)
                        progress->mark("prepare_command");
                    // Read the newest target before this leg's transaction.
                    JointCommand command;
                    {
                        std::lock_guard<std::mutex> lock(runtime.mutex);
                        command = runtime.commands[local_index];
                    }
                    if (command_watchdog_) {
                        const auto now_ns =
                            std::chrono::duration_cast<std::chrono::nanoseconds>(SteadyClock::now().time_since_epoch())
                                .count();
                        if (now_ns - last_command_ns_.load() > 100000000)
                            command = brake_command();
                    }
                    if (native_uart_ && !command.protective_crouch) {
                        std::lock_guard<std::mutex> lock(runtime.mutex);
                        const auto &last = runtime.feedback[local_index];
                        if (last.valid && SteadyClock::now() - last.received_at >= std::chrono::milliseconds(300))
                            command = brake_command();
                    }
                    auto &anchor = runtime.anchors[local_index];
                    if (session_anchor_faulted_.load()) {
                        for (auto &item : runtime.anchors)
                            item.invalidate();
                    }
                    if (!anchor.can_command(command))
                        command = brake_command();
                    std::uint64_t command_sent_us = 0;
                    try {
                        // Only a FOC frame that reaches the worker is evidence that
                        // active control has begun.  A command queued by the controller
                        // can otherwise race with passive startup/reconnect handling.
                        if (command.mode == MotorMode::kFoc) {
                            active_control_started_.store(true);
                        }
                        const RotorFeedback rotor =
                            transact(devices.at(local_index), command, config.calibration[local_index], &anchor,
                                native_uart_, progress, &command_sent_us);
                        const JointFeedback joint = anchor.valid() ?
                            anchor.to_joint_feedback(
                                rotor, config.calibration[local_index], !command.backflip_control,
                                native_uart_ ? TimePoint(std::chrono::duration_cast<SteadyClock::duration>(
                                    std::chrono::microseconds(rotor.received_us))) : TimePoint{}) :
                            rotor_to_unanchored_feedback(rotor, config.calibration[local_index]);
                        if (progress)
                            progress->mark("publish_feedback");
                        std::lock_guard<std::mutex> lock(runtime.mutex);
                        runtime.feedback[local_index].feedback = joint;
                        runtime.feedback[local_index].received_at = native_uart_ ?
                            TimePoint(std::chrono::duration_cast<SteadyClock::duration>(
                                std::chrono::microseconds(rotor.received_us))) : SteadyClock::now();
                        runtime.feedback[local_index].valid = true;
                        if (session_anchor_faulted_.load()) {
                            runtime.error = "session anchor invalidated; restart to rebuild anchors";
                        } else {
                            runtime.error.clear();
                        }
                    } catch (const MotorTransportError &error) {
                        if (progress)
                            progress->mark("handle_error");
                        sweep_failed = true;
                        // A single dropped USB-RS485 reply used to poison every session
                        // anchor after FOC had started.  During a backflip that instantly
                        // changed all twelve outputs to BRAKE in mid-air.  Keep the last
                        // good sample and retry on the next sweep for this actor only.
                        const bool fail_closed = active_control_started_.load();
                        // Retry-and-hold is the native new_bigcore policy. USB
                        // keeps the original fail-closed transaction behavior.
                        const bool transient_timeout = native_uart_ && error.retryable();
                        if (!transient_timeout && !command.backflip_control && fail_closed) {
                            session_anchor_faulted_.store(true);
                            for (auto &item : runtime.anchors)
                                item.invalidate();
                        }
                        {
                            std::lock_guard<std::mutex> lock(runtime.mutex);
                            ++runtime.transaction_failures;
                            ++runtime.consecutive_failures;
                            if (!transient_timeout && !command.backflip_control) {
                                runtime.feedback[local_index].valid = false;
                            }
                            runtime.error = error.what();
                        }
                        if (!transient_timeout && !command.backflip_control && !fail_closed) {
                            throw MotorTransportError(
                                "passive startup transport interrupted; retrying: " + std::string(error.what()));
                        }
                        // A native timeout releases the transaction gate before
                        // this handler. Retain feedback and immediately yield
                        // the next slot to another waiting leg; no retry sleep.
                    } catch (const std::exception &error) {
                        if (progress)
                            progress->mark("handle_error");
                        sweep_failed = true;
                        const bool fail_closed = active_control_started_.load();
                        if (fail_closed) {
                            session_anchor_faulted_.store(true);
                            for (auto &item : runtime.anchors)
                                item.invalidate();
                        }
                        {
                            std::lock_guard<std::mutex> lock(runtime.mutex);
                            ++runtime.transaction_failures;
                            ++runtime.consecutive_failures;
                            runtime.feedback[local_index].valid = false;
                            runtime.error = error.what();
                        }
                        if (!fail_closed) {
                            // No FOC command has ever been sent in this process, so the leg
                            // is still at the required startup pose under BRAKE. Reopen the
                            // bus and rebuild this leg's anchors instead of poisoning all
                            // four legs because of one cold-start packet loss.
                            throw MotorTransportError(
                                "passive startup transport interrupted; retrying: " + std::string(error.what()));
                        }
                    }
                    if (command_sent_us != 0) {
                        std::lock_guard<std::mutex> lock(runtime.mutex);
                        record_transmission(runtime.transmissions[local_index], command,
                            TimePoint(std::chrono::duration_cast<SteadyClock::duration>(
                                std::chrono::microseconds(command_sent_us))));
                    }
                }
                if (!running_.load())
                    break;
                const auto completed_at = std::chrono::steady_clock::now();
                bool flip_uart_pacing = false;
                {
                    std::lock_guard<std::mutex> lock(runtime.mutex);
                    flip_uart_pacing = runtime.commands[0].flip_uart_pacing;
                    ++runtime.completed_sweeps;
                    if (!sweep_failed) {
                        runtime.consecutive_failures = 0;
                        if (!session_anchor_faulted_.load())
                            runtime.error.clear();
                    }
                    if (runtime.previous_sweep_completed_at != std::chrono::steady_clock::time_point{}) {
                        const double period_ms =
                            std::chrono::duration<double, std::milli>(completed_at - runtime.previous_sweep_completed_at)
                                .count();
                        ++runtime.period_samples;
                        const double delta = period_ms - runtime.mean_period_ms;
                        runtime.mean_period_ms += delta / static_cast<double>(runtime.period_samples);
                        const double delta_after = period_ms - runtime.mean_period_ms;
                        runtime.period_m2_ms2 += delta * delta_after;
                        runtime.minimum_period_ms = std::min(runtime.minimum_period_ms, period_ms);
                        runtime.maximum_period_ms = std::max(runtime.maximum_period_ms, period_ms);
                    }
                    runtime.previous_sweep_completed_at = completed_at;
                }

                // Normal native-UART sweeps cover all three motors at most once
                // per millisecond. Keep flip pacing and other backends unchanged.
                const bool pace_native = native_uart_ && !flip_uart_pacing;
                const auto sweep_period = pace_native ? std::chrono::milliseconds(1) :
                    (command_watchdog_ ? std::chrono::milliseconds(native_uart_ ? 5 : 10) :
                        std::chrono::milliseconds(0));
                if (sweep_period.count() > 0) {
                    // Anchor to the actual start so a late wakeup cannot cause
                    // a shorter catch-up sweep on the following iteration.
                    next_sweep = pace_native ? sweep_started_at + sweep_period : next_sweep + sweep_period;
                    const auto now = SteadyClock::now();
                    if (next_sweep < now)
                        next_sweep = now;
                    std::this_thread::sleep_until(next_sweep);
                } else {
                    next_sweep = SteadyClock::now();
                }
            }

            for (std::size_t local_index = 0; local_index < kMotorsPerLeg; ++local_index) {
                try {
                    static_cast<void>(
                        transact(devices.at(local_index), brake_command(), config.calibration[local_index], nullptr,
                            native_uart_, progress));
                } catch (...) {
                }
            }
            if (progress)
                progress->mark("close_devices");
        } catch (const std::exception &error) {
            if (progress)
                progress->mark("reconnect");
            // Opening a bus or collecting its first three BRAKE replies may fail
            // transiently while the four workers start. Retry that leg without
            // poisoning anchors already established by the other buses. Once a leg
            // has completed startup, any unexpected escape remains a hard session
            // fault; anchor-validation failures also set the flag above.
            if (startup_complete && active_control_started_.load()) {
                session_anchor_faulted_.store(true);
            }
            {
                std::lock_guard<std::mutex> lock(runtime.mutex);
                ++runtime.reconnect_attempts;
                runtime.error = error.what();
                for (auto &feedback : runtime.feedback)
                    feedback.valid = false;
                for (auto &anchor : runtime.anchors)
                    anchor.invalidate();
            }
            const auto retry_at = std::chrono::steady_clock::now() + config.reconnect_delay;
            while (running_.load() && std::chrono::steady_clock::now() < retry_at) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
    if (progress)
        progress->mark("stopped");
}

void MotorBank::run_rpmsg_leg(std::size_t leg_index) {
    auto &runtime = *legs_[leg_index];
    const auto &config = configs_[leg_index];
    while (running_.load()) {
        bool startup_complete = false;
        try {
            std::unique_ptr<RpmsgSession, decltype(&rpmsg_close)> channel(
                rpmsg_open(config.device.c_str()), &rpmsg_close);
            if (!channel)
                throw MotorTransportError("failed to open G2MR v2 motor service");
            RpmsgSnapshot state{};
            bool initialized = false;
            for (unsigned attempt = 0; attempt < config.startup_attempts && running_.load(); ++attempt) {
                if (rpmsg_get(channel.get(), &state) < 0)
                    throw MotorTransportError("failed to read smallcore startup state");
                initialized = (state.flags & GO2_RPC_STOPPED) != 0;
                for (std::size_t id = 0; id < kMotorsPerLeg; ++id)
                    initialized = initialized && state.valid[id] && state.age_ms[id] < 100 && !state.motors[id].error;
                if (initialized)
                    break;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            if (!running_.load())
                break;
            if (!initialized)
                throw MotorTransportError("smallcore has no fresh BRAKE feedback for all motors");
            const auto startup_received = SteadyClock::now();
            for (std::size_t id = 0; id < kMotorsPerLeg; ++id) {
                if (config.calibration[id].motor_id != id)
                    throw std::invalid_argument("smallcore motor order must be 0,1,2");
                if (config.startup_pose_verified && !session_anchor_faulted_.load()) {
                    try {
                        runtime.anchors[id].initialize(state.motors[id].position_rad, config.session_reference[id],
                            config.calibration[id], config.startup_joint_tolerances_rad[id]);
                        static_cast<void>(runtime.anchors[id].to_joint_feedback(state.motors[id], config.calibration[id],
                            true, startup_received - std::chrono::milliseconds(state.age_ms[id])));
                    } catch (...) {
                        std::lock_guard<std::mutex> lock(runtime.mutex);
                        runtime.startup_pose_invalid = true;
                        throw;
                    }
                }
            }
            {
                std::lock_guard<std::mutex> lock(runtime.mutex);
                runtime.startup_pose_invalid = false;
            }
            std::uint64_t sent_generation = std::numeric_limits<std::uint64_t>::max();
            startup_complete = true;
            std::array<std::uint32_t, kMotorsPerLeg> previous_samples{};
            bool previously_forced = false;
            bool previously_expired = false;
            while (running_.load()) {
                std::array<JointCommand, kMotorsPerLeg> commands;
                std::uint64_t generation;
                SteadyClock::time_point command_at;
                {
                    std::lock_guard<std::mutex> lock(runtime.mutex);
                    commands = runtime.commands;
                    generation = runtime.command_generation;
                    command_at = runtime.command_at;
                }
                const bool forced =
                    session_anchor_faulted_.load() || SteadyClock::now() - command_at >= std::chrono::milliseconds(100);
                const bool command_expired = SteadyClock::now() - command_at >= std::chrono::milliseconds(100);
                if (session_anchor_faulted_.load())
                    for (auto &anchor : runtime.anchors)
                        anchor.invalidate();
                std::array<RotorCommand, kMotorsPerLeg> targets{};
                for (std::size_t id = 0; id < kMotorsPerLeg; ++id) {
                    auto &anchor = runtime.anchors[id];
                    if (command_expired || !anchor.can_command(commands[id]) ||
                        (session_anchor_faulted_.load() && !commands[id].protective_crouch))
                        commands[id] = brake_command();
                    const auto rotor = anchor.to_rotor_command(commands[id], config.calibration[id]);
                    targets[id] = rotor;
                }
                // Reading snapshots does not retransmit targets or keep an old FOC alive.
                if (generation != sent_generation || forced != previously_forced ||
                    command_expired != previously_expired) {
                    for (const auto &target : targets)
                        if (target.mode == MotorMode::kFoc)
                            active_control_started_.store(true);
                    const auto previous_sent_at = channel->command_sent_at;
                    const int set_result = rpmsg_set(channel.get(), targets.data());
                    if ((set_result == 0 || set_result == -ETIMEDOUT) &&
                        channel->command_sent_at > previous_sent_at) {
                        std::lock_guard<std::mutex> lock(runtime.mutex);
                        for (std::size_t id = 0; id < kMotorsPerLeg; ++id)
                            record_transmission(runtime.transmissions[id], commands[id], channel->command_sent_at);
                    }
                    if (set_result == -ETIMEDOUT) {
                        std::lock_guard<std::mutex> lock(runtime.mutex);
                        ++runtime.transaction_failures;
                        runtime.error = "smallcore SET reply timeout; retrying latest targets";
                        continue;
                    }
                    if (set_result < 0)
                        throw MotorTransportError("smallcore rejected motor targets");
                    sent_generation = generation;
                    previously_forced = forced;
                    previously_expired = command_expired;
                }
                const int get_result = rpmsg_get(channel.get(), &state);
                if (get_result == -ETIMEDOUT) {
                    std::lock_guard<std::mutex> lock(runtime.mutex);
                    ++runtime.transaction_failures;
                    runtime.error = "smallcore GET reply timeout; retaining verified feedback";
                    continue;
                }
                if (get_result < 0 || (state.flags & GO2_RPC_FAULT))
                    throw MotorTransportError("smallcore transport, watchdog or motor fault");
                const auto received = SteadyClock::now();
                for (std::size_t id = 0; id < kMotorsPerLeg; ++id) {
                    if (!state.valid[id] || state.age_ms[id] >= 300 || state.motors[id].error)
                        throw MotorTransportError("smallcore motor feedback missing, stale or faulted");
                    {
                        std::lock_guard<std::mutex> lock(runtime.mutex);
                        if (runtime.feedback[id].valid && previous_samples[id] == state.sample[id])
                            continue;
                    }
                    const auto sampled_at = received - std::chrono::milliseconds(state.age_ms[id]);
                    const auto &rotor = state.motors[id];
                    auto &anchor = runtime.anchors[id];
                    const auto joint = anchor.valid() ?
                        anchor.to_joint_feedback(rotor, config.calibration[id], !commands[id].backflip_control,
                            sampled_at) :
                        rotor_to_unanchored_feedback(rotor, config.calibration[id]);
                    std::lock_guard<std::mutex> lock(runtime.mutex);
                    auto &feedback = runtime.feedback[id];
                    feedback.feedback = joint;
                    feedback.received_at = sampled_at;
                    feedback.valid = true;
                    previous_samples[id] = state.sample[id];
                }
                {
                    std::lock_guard<std::mutex> lock(runtime.mutex);
                    ++runtime.completed_sweeps;
                    if (runtime.previous_sweep_completed_at != SteadyClock::time_point{}) {
                        const double period =
                            std::chrono::duration<double, std::milli>(received - runtime.previous_sweep_completed_at)
                                .count();
                        ++runtime.period_samples;
                        const double delta = period - runtime.mean_period_ms;
                        runtime.mean_period_ms += delta / static_cast<double>(runtime.period_samples);
                        runtime.period_m2_ms2 += delta * (period - runtime.mean_period_ms);
                        runtime.minimum_period_ms = std::min(runtime.minimum_period_ms, period);
                        runtime.maximum_period_ms = std::max(runtime.maximum_period_ms, period);
                    }
                    runtime.previous_sweep_completed_at = received;
                    runtime.error =
                        session_anchor_faulted_.load() ? "session anchor invalidated; restart to rebuild anchors" : "";
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        } catch (const std::exception &error) {
            // Firmware faults are sticky. Never silently resume active control
            // after a new RPMsg session, including during backflip control.
            if (startup_complete && active_control_started_.load())
                session_anchor_faulted_.store(true);
            {
                std::lock_guard<std::mutex> lock(runtime.mutex);
                ++runtime.transaction_failures;
                ++runtime.reconnect_attempts;
                runtime.error = error.what();
                for (auto &feedback : runtime.feedback)
                    feedback.valid = false;
                for (auto &anchor : runtime.anchors)
                    anchor.invalidate();
            }
            const auto retry_at = SteadyClock::now() + config.reconnect_delay;
            while (running_.load() && SteadyClock::now() < retry_at)
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

} // namespace go2

namespace go2 {
/** Rotates a three-axis sensor vector into the configured body frame. */
std::array<float, 3> rotate_vector(const std::array<float, 9> &matrix, const std::array<float, 3> &value) {
    return {matrix[0] * value[0] + matrix[1] * value[1] + matrix[2] * value[2],
        matrix[3] * value[0] + matrix[4] * value[1] + matrix[5] * value[2],
        matrix[6] * value[0] + matrix[7] * value[1] + matrix[8] * value[2]};
}

/** Composes and normalizes two wxyz quaternions. */
std::array<float, 4> multiply_quaternion(const std::array<float, 4> &left, const std::array<float, 4> &right) {
    const auto [lw, lx, ly, lz] = left;
    const auto [rw, rx, ry, rz] = right;
    std::array<float, 4> result{lw * rw - lx * rx - ly * ry - lz * rz, lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx, lw * rz + lx * ry - ly * rx + lz * rw};
    const float norm =
        std::sqrt(result[0] * result[0] + result[1] * result[1] + result[2] * result[2] + result[3] * result[3]);
    if (norm > 1e-6F) {
        for (float &component : result)
            component /= norm;
    }
    return result;
}

/** Applies configured static body-frame roll and pitch corrections. */
std::array<float, 4> add_roll_pitch_offsets(
    const std::array<float, 4> &quaternion, float roll_offset_radians, float pitch_offset_radians) {
    // Preserve the sensor quaternion across vertical/inverted attitudes.
    // Euler reconstruction is only needed when a real static offset is set.
    if (std::fabs(roll_offset_radians) <= 1.0e-7F && std::fabs(pitch_offset_radians) <= 1.0e-7F) {
        return quaternion;
    }
    const auto [w, x, y, z] = quaternion;
    const float roll = std::atan2(2.0F * (w * x + y * z), 1.0F - 2.0F * (x * x + y * y)) + roll_offset_radians;
    const float pitch = std::asin(std::clamp(2.0F * (w * y - z * x), -1.0F, 1.0F)) + pitch_offset_radians;
    const float yaw = std::atan2(2.0F * (w * z + x * y), 1.0F - 2.0F * (y * y + z * z));

    const float cr = std::cos(roll * 0.5F);
    const float sr = std::sin(roll * 0.5F);
    const float cp = std::cos(pitch * 0.5F);
    const float sp = std::sin(pitch * 0.5F);
    const float cy = std::cos(yaw * 0.5F);
    const float sy = std::sin(yaw * 0.5F);
    return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy};
}

ImuSample imu_sample_to_body(const ImuSample &sensor, const HardwareConfig &config) {
    ImuSample body = sensor;
    body.angular_velocity = rotate_vector(config.imu_to_body_matrix, sensor.angular_velocity);
    body.linear_acceleration = rotate_vector(config.imu_to_body_matrix, sensor.linear_acceleration);
    body.quaternion = add_roll_pitch_offsets(multiply_quaternion(sensor.quaternion, config.imu_to_body_quaternion),
        config.imu_roll_offset_rad, config.imu_pitch_offset_rad);
    body.six_axis_quaternion =
        add_roll_pitch_offsets(multiply_quaternion(sensor.six_axis_quaternion, config.imu_to_body_quaternion),
            config.imu_roll_offset_rad, config.imu_pitch_offset_rad);
    return body;
}

void validate_hardware_config(const HardwareConfig &c) {
    if (c.board_profile != BoardProfile::kLegacyUsb && c.board_profile != BoardProfile::kNewBigcore &&
        c.board_profile != BoardProfile::kNewSmallcore && c.board_profile != BoardProfile::kNewUsbMotor)
        throw std::invalid_argument("unknown board profile");
    if (c.board_profile != BoardProfile::kLegacyUsb && c.imu_driver != "drv_spi_bmi270")
        throw std::invalid_argument("new boards require BMI270");
    for (float v : c.imu_to_body_matrix) {
        if (!std::isfinite(v))
            throw std::invalid_argument("invalid IMU rotation matrix");
    }
    float norm = 0;
    for (float v : c.imu_to_body_quaternion)
        norm += v * v;
    if (!std::isfinite(norm) || norm < 1e-12F || !std::isfinite(c.imu_roll_offset_rad) ||
        !std::isfinite(c.imu_pitch_offset_rad)) {
        throw std::invalid_argument("invalid IMU orientation correction");
    }
    if (c.backend == Backend::kMock) {
        if (c.motor_io_authorized || c.actuation_enabled) {
            throw std::invalid_argument("mock cannot authorize serial I/O or actuation");
        }
        return;
    }
    if (!c.motor_io_authorized || !c.actuation_enabled || !c.hardware_configuration_verified || !c.motor_ids_verified ||
        !c.joint_calibration_verified || !c.suspended_test_verified) {
        throw std::invalid_argument("serial requires all hardware and actuation approvals");
    }
    const auto stable_path = [](const std::string &path) {
        return path.rfind("/dev/serial/by-id/", 0) == 0 || path.rfind("/dev/serial/by-path/", 0) == 0;
    };
    const bool spi_imu = c.imu_driver == "drv_spi_bmi270";
    if (c.imu_gpio_line < 0 || (!c.imu_gpiochip.empty() && (!spi_imu || c.imu_gpiochip.rfind("/dev/", 0) != 0))) {
        throw std::invalid_argument("invalid IMU GPIO configuration");
    }
    const bool valid_imu = spi_imu ?
        (c.imu_device == "/dev/go2/imu" || c.imu_device == "/dev/bmi270" ||
            c.imu_device.rfind("/dev/spidev", 0) == 0 ||
            c.imu_device.rfind("/dev/iio:device", 0) == 0) :
        (c.imu_driver == "drv_uart_iwt603" &&
            (stable_path(c.imu_device) || c.imu_device == "/dev/go2/imu") && c.imu_baud > 0);
    if (!valid_imu) {
        throw std::invalid_argument("invalid IMU driver, device path or UART baudrate");
    }
    for (std::size_t i = 0; i < c.legs.size(); ++i) {
        const auto &leg = c.legs[i];
        const std::array<std::string, 4> services{"go2-motor-rl", "go2-motor-rr", "go2-motor-fl", "go2-motor-fr"};
        const bool rpc = c.board_profile == BoardProfile::kNewSmallcore;
        const bool board_uart = c.board_profile == BoardProfile::kNewBigcore;
        const std::array<std::string, 4> aliases{
            "/dev/go2/motor-fr", "/dev/go2/motor-fl", "/dev/go2/motor-rr", "/dev/go2/motor-rl"};
        const bool device_valid = rpc ?
            leg.device == services[i] :
            (stable_path(leg.device) || leg.device == aliases[i] ||
                (board_uart && (leg.device.rfind("/dev/go2/motor-", 0) == 0 || leg.device.rfind("/dev/ttyS", 0) == 0)));
        if ((rpc && leg.driver != "go2_smallcore") || (!rpc && leg.driver == "go2_smallcore"))
            throw std::invalid_argument("motor driver does not match board profile");
        if (leg.driver.empty() || !device_valid || leg.device == c.imu_device || !leg.startup_pose_verified ||
            leg.startup_attempts == 0 || leg.startup_attempts > 100) {
            throw std::invalid_argument("each motor bus requires a stable path and Pose A anchoring");
        }
        for (std::size_t j = i + 1; j < c.legs.size(); ++j) {
            if (leg.device == c.legs[j].device)
                throw std::invalid_argument("duplicate motor bus");
        }
        for (std::size_t j = 0; j < kMotorsPerLeg; ++j) {
            const auto &cal = leg.calibration[j];
            const auto &ref = leg.session_reference[j];
            validate_calibration(cal);
            if (rpc && cal.motor_id != j)
                throw std::invalid_argument("RCPU service requires motor IDs 0,1,2 per leg");
            const float tolerance = leg.startup_joint_tolerances_rad[j];
            if (!std::isfinite(tolerance) || tolerance <= 0 || tolerance * cal.gear_ratio >= kTwoPi * .5F ||
                !std::isfinite(ref.joint_position_rad) || !std::isfinite(ref.rotor_position_rad)) {
                throw std::invalid_argument("invalid Pose A session reference or tolerance");
            }
            for (std::size_t k = j + 1; k < kMotorsPerLeg; ++k) {
                if (cal.motor_id == leg.calibration[k].motor_id) {
                    throw std::invalid_argument("duplicate motor ID on a leg bus");
                }
            }
        }
    }
}

namespace {

struct ImuDeleter {
    void operator()(struct imu_dev *dev) const { imu_free(dev); }
};

} // namespace

struct Hardware::Impl {
    explicit Impl(HardwareConfig value) : config(std::move(value)) {
        validate_hardware_config(config);
        for (std::size_t i = 0; i < kNumJoints; ++i) {
            auto &f = mock.motors.joints[i];
            f.valid = true;
            f.feedback.position_rad = kStandPose[i];
            f.feedback.position_anchored = true;
            f.feedback.temperature_c = 25;
        }
        mock.imu.valid = true;
        mock.imu.linear_acceleration = {0, 0, 9.81F};
    }
    HardwareConfig config;
    std::unique_ptr<MotorBank> motors;
    std::unique_ptr<struct imu_dev, ImuDeleter> imu;
    mutable std::shared_mutex lifecycle_mutex;
    mutable std::mutex mutex;
    mutable std::mutex imu_mutex;
    ImuSample imu_sample{};
    std::atomic_bool imu_polling{false};
    std::thread imu_poll_thread;
    RobotState mock{};
    bool started{false};

    static void receive_imu(struct imu_dev *, const struct imu_data *data, void *context) noexcept {
        static_cast<Impl *>(context)->update_imu(*data);
    }

    void update_imu(const struct imu_data &data) noexcept {
        try {
            ImuSample sample;
            sample.angular_velocity = {data.gyro[0], data.gyro[1], data.gyro[2]};
            sample.linear_acceleration = {data.acc[0], data.acc[1], data.acc[2]};
            sample.quaternion = {data.quat[0], data.quat[1], data.quat[2], data.quat[3]};
            sample.six_axis_quaternion = sample.quaternion;
            float quaternion_norm_squared = 0;
            for (float component : sample.quaternion)
                quaternion_norm_squared += component * component;
            const auto timestamp_ns = data.timestamp_us * 1000ULL;
            sample.received_at =
                TimePoint(std::chrono::duration_cast<SteadyClock::duration>(std::chrono::nanoseconds(timestamp_ns)));
            sample.sequence = timestamp_ns;
            sample.valid = data.timestamp_us != 0 && std::isfinite(quaternion_norm_squared) &&
                quaternion_norm_squared > 1.0e-12F;
            std::lock_guard<std::mutex> lock(imu_mutex);
            imu_sample = imu_sample_to_body(sample, config);
        } catch (const std::exception &) {
            std::lock_guard<std::mutex> lock(imu_mutex);
            imu_sample.valid = false;
        }
    }

    void start_imu() {
        const bool polling = config.imu_driver == "drv_spi_bmi270" && config.imu_gpiochip.empty();
        if (!polling)
            imu_set_callback(imu.get(), &Impl::receive_imu, this);
        const int result = imu_init(imu.get(), nullptr);
        if (result != 0)
            throw std::runtime_error("failed to initialize IMU/event source: " + std::to_string(result));
        if (!polling) {
            std::fprintf(stderr, "go2: IMU event callback active (%s)\n", config.imu_driver.c_str());
            return;
        }
        std::fprintf(stderr, "go2: BMI270 has no DRDY event source; using application synchronous polling\n");
        imu_polling = true;
        try {
            imu_poll_thread = std::thread([this] {
                while (imu_polling.load()) {
                    struct imu_data data{};
                    if (imu_read(imu.get(), &data) == 0)
                        update_imu(data);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            });
        } catch (...) {
            imu_polling = false;
            throw;
        }
    }

    void stop_imu() {
        imu_polling = false;
        if (imu_poll_thread.joinable())
            imu_poll_thread.join();
        if (imu)
            imu_set_callback(imu.get(), nullptr, nullptr);
        std::lock_guard<std::mutex> lock(imu_mutex);
        imu_sample.valid = false;
    }

    void stop() {
        if (motors) {
            motors->set_commands(Commands{});
            motors->stop();
        }
        stop_imu();
        imu.reset();
        started = false;
    }
};

Hardware::Hardware(HardwareConfig config) : impl_(std::make_unique<Impl>(std::move(config))) {}
Hardware::~Hardware() { stop(); }

void Hardware::start() {
    auto &h = *impl_;
    std::unique_lock<std::shared_mutex> lock(h.lifecycle_mutex);
    if (h.started)
        return;
    if (h.config.backend == Backend::kSerial) {
        // Each leg uses its own driver-side serial lock, on both native UART
        // and USB. Native boards also retain their command watchdogs.
        const bool command_watchdog = h.config.board_profile == BoardProfile::kNewBigcore ||
            h.config.board_profile == BoardProfile::kNewSmallcore;
        const bool native_uart = h.config.board_profile == BoardProfile::kNewBigcore;
        h.motors = std::make_unique<MotorBank>(h.config.legs, command_watchdog, native_uart);
        if (h.config.imu_driver == "drv_spi_bmi270") {
            struct imu_spi_config spi{};
            spi.bits_per_word = 8;
            spi.speed_hz = 1000000;
            // An empty GPIO path means the DTB exposes IIO only; pass NULL so
            // the BMI270 driver selects synchronous polling instead of
            // rejecting an empty GPIO device path.
            spi.gpiochip_path = h.config.imu_gpiochip.empty() ? nullptr : h.config.imu_gpiochip.c_str();
            spi.gpio_line = static_cast<std::uint32_t>(h.config.imu_gpio_line);
            h.imu.reset(imu_alloc_spi(h.config.imu_driver.c_str(), h.config.imu_device.c_str(), 0, &spi));
        } else {
            h.imu.reset(imu_alloc_uart(h.config.imu_driver.c_str(), h.config.imu_device.c_str(),
                static_cast<std::uint32_t>(h.config.imu_baud), nullptr));
        }
        if (!h.imu) {
            throw std::runtime_error("failed to initialize IMU device");
        }
        try {
            h.start_imu();
            h.motors->start();
        } catch (...) {
            h.stop();
            throw;
        }
    }
    h.started = true;
}

void Hardware::stop() {
    auto &h = *impl_;
    std::unique_lock<std::shared_mutex> lock(h.lifecycle_mutex);
    // Each worker sends best-effort BRAKE before its devices are released.
    h.stop();
}

void Hardware::replace_calibration(
    const std::array<float, kNumGo2Motors>& offsets,
    const std::array<float, kNumGo2Motors>& startup_rotors) {
    auto& h = *impl_;
    std::unique_lock<std::shared_mutex> lock(h.lifecycle_mutex);
    if (h.started) throw std::runtime_error("replace_calibration requires stopped hardware");
    for (std::size_t leg = 0; leg < kNumGo2Legs; ++leg) {
        for (std::size_t local = 0; local < kMotorsPerLeg; ++local) {
            const auto index = leg * kMotorsPerLeg + local;
            h.config.legs[leg].calibration[local].joint_zero_offset_rad = offsets[index];
            h.config.legs[leg].session_reference[local].rotor_position_rad = startup_rotors[index];
        }
    }
}

RobotState Hardware::read(TimePoint now) const {
    auto &h = *impl_;
    std::shared_lock<std::shared_mutex> lock(h.lifecycle_mutex);
    if (h.config.backend == Backend::kMock) {
        std::lock_guard<std::mutex> lock(h.mutex);
        RobotState state = h.mock;
        for (auto &joint : state.motors.joints)
            joint.received_at = now;
        state.imu.received_at = now;
        state.imu.sequence = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        return state;
    }
    RobotState result;
    if (!h.motors || !h.imu)
        return result;
    result.motors = h.motors->snapshot();
    {
        std::lock_guard<std::mutex> lock(h.imu_mutex);
        result.imu = h.imu_sample;
    }
    return result;
}

void Hardware::write(const Commands &commands) {
    auto &h = *impl_;
    std::shared_lock<std::shared_mutex> lock(h.lifecycle_mutex);
    if (!h.started)
        throw std::logic_error("hardware has not started");
    if (h.config.backend == Backend::kSerial) {
        h.motors->set_commands(commands);
    } else {
        std::lock_guard<std::mutex> lock(h.mutex);
        h.mock.motors.transmissions_tracked = true;
        const auto sent_at = SteadyClock::now();
        for (std::size_t i = 0; i < commands.size(); ++i) {
            record_transmission(h.mock.motors.transmissions[i], commands[i], sent_at);
            if (commands[i].mode == MotorMode::kFoc) {
                h.mock.motors.joints[i].feedback.position_rad = commands[i].position_rad;
                h.mock.motors.joints[i].feedback.velocity_rad_s = 0;
            }
        }
    }
}

} // namespace go2
