/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
// Exercise the private event bridge without opening UARTs or creating teleop.
#define main sbus_teleop_entry
#include "../src/sbus_teleop_node.cpp"
#undef main

#include <cassert>
#include <thread>

namespace {

struct SbusTeleopTestAccess {
    static void check_mode_mapping() {
        auto teleop = std::shared_ptr<SbusTeleopNode>(new SbusTeleopNode(false));
        const std::array<std::string, 18> modes = {
            "terrain", "wave", "sit", "web", "navigation", "ai_agent",
            "forward_jump", "", "walk", "", "", "", "", "", "", "", "", ""};
        rc_receiver_frame_t frame{};
        for (int slot = 1; slot <= 18; ++slot) {
            const int group = (slot - 1) % 9;
            frame.channels[7] = slot >= 10 ? RC_RECEIVER_CHANNEL_MAX : 0;  // B
            frame.channels[5] = (group / 3) * RC_RECEIVER_CHANNEL_MAX / 2;  // F
            frame.channels[4] = (group % 3) * RC_RECEIVER_CHANNEL_MAX / 2;  // E
            assert(teleop->selected_slot(frame) == slot);
            assert(teleop->mode_for_slot(slot) == modes[slot - 1]);
        }
    }
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sbus_event_test");
    const auto executor_thread = std::this_thread::get_id();
    unsigned calls = 0;
    auto event = std::make_shared<SbusFrameEvent>([&](const SbusFrameEvent::Batch& batch) {
        assert(std::this_thread::get_id() == executor_thread);
        assert(!batch.overflow && batch.count == 2);
        assert(batch.samples[0].frame.channels[0] == 101);
        assert(batch.samples[1].frame.channels[0] == 202);
        assert(batch.samples[0].received <= batch.samples[1].received);
        ++calls;
    });
    node->get_node_waitables_interface()->add_waitable(event, nullptr);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin_some();
    assert(calls == 0);
    std::thread producer([&] {
        rc_receiver_frame_t frame{};
        frame.channels[0] = 101;
        event->push(frame);
        frame.channels[0] = 202;
        event->push(frame);
    });
    producer.join();
    for (int attempt = 0; attempt < 100 && calls == 0; ++attempt) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    assert(calls == 1);
    executor.spin_some();
    assert(calls == 1);
    rc_receiver_frame_t frame{};
    for (int i = 0; i < 65; ++i) event->push(frame);
    auto data = event->take_data();
    auto batch = std::static_pointer_cast<SbusFrameEvent::Batch>(data);
    assert(batch->overflow && batch->count == 64);
    data = event->take_data();
    batch = std::static_pointer_cast<SbusFrameEvent::Batch>(data);
    assert(!batch->overflow && batch->count == 0);
    node->get_node_waitables_interface()->remove_waitable(event, nullptr);
    executor.remove_node(node);
    SbusTeleopTestAccess::check_mode_mapping();
    rclcpp::shutdown();
}
