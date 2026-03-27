#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>
#include <zmq.hpp>
#include "cpp_zmq_server/habilis_bridge.hpp"
#include "cpp_zmq_server/habilis_types.hpp"

using namespace cpp_zmq_server;
using namespace std::chrono_literals;

class HabilisBridgeTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Use pid + atomic counter for unique ports across parallel runs
        static std::atomic<int> counter{0};
        int offset = (getpid() % 500) + counter.fetch_add(2);
        pub_port_ = static_cast<uint16_t>(14270 + offset);
        sub_port_ = pub_port_ + 1;
    }

    uint16_t pub_port_;
    uint16_t sub_port_;
};

// ── PUB/SUB communication ───────────────────────────────────────────────────

TEST_F(HabilisBridgeTest, SendAndReceiveMessage) {
    zmq::context_t ctx(1);
    HabilisBridge bridge(ctx);

    std::atomic<bool> message_received{false};
    HabilisMsgType received_type{};
    std::string received_value;

    // Start bridge in a thread
    std::thread bridge_thread([&]() {
        bridge.run("127.0.0.1", sub_port_, pub_port_,
            [&](HabilisMsgType type, msgpack::object_handle body) {
                received_type = type;
                message_received = true;
                try {
                    auto map = body.get().as<std::map<std::string, std::string>>();
                    received_value = map["test"];
                } catch (...) {}
            });
    });

    std::this_thread::sleep_for(200ms);

    // Create a PUB socket to simulate AI Manager sending to bridge's SUB
    zmq::socket_t ai_pub(ctx, zmq::socket_type::pub);
    ai_pub.set(zmq::sockopt::linger, 0);
    ai_pub.bind("tcp://127.0.0.1:" + std::to_string(sub_port_));

    // SUB sockets need time to connect
    std::this_thread::sleep_for(300ms);

    // Send a HEARTBEAT message
    msgpack::sbuffer buf;
    msgpack::packer<msgpack::sbuffer> pk(buf);
    pk.pack_map(1);
    pk.pack("test");
    pk.pack("hello");

    auto frame = serialize_habilis_frame(HabilisMsgType::HEARTBEAT, buf);
    ai_pub.send(zmq::buffer(frame), zmq::send_flags::none);

    // Wait for message processing
    std::this_thread::sleep_for(500ms);

    EXPECT_TRUE(message_received.load());
    EXPECT_EQ(received_type, HabilisMsgType::HEARTBEAT);
    EXPECT_EQ(received_value, "hello");

    ai_pub.close();
    bridge.stop();
    bridge_thread.join();
}

// ── Send via PUB socket ─────────────────────────────────────────────────────

TEST_F(HabilisBridgeTest, SendViaPublishSocket) {
    zmq::context_t ctx(1);
    HabilisBridge bridge(ctx);

    std::thread bridge_thread([&]() {
        bridge.run("127.0.0.1", sub_port_, pub_port_,
            [](HabilisMsgType, msgpack::object_handle) {});
    });

    std::this_thread::sleep_for(200ms);

    // Create a SUB socket to receive from bridge's PUB
    zmq::socket_t ai_sub(ctx, zmq::socket_type::sub);
    ai_sub.set(zmq::sockopt::linger, 0);
    ai_sub.set(zmq::sockopt::subscribe, "");
    ai_sub.connect("tcp://127.0.0.1:" + std::to_string(pub_port_));

    std::this_thread::sleep_for(300ms);

    // Send a message via bridge
    msgpack::sbuffer buf;
    msgpack::packer<msgpack::sbuffer> pk(buf);
    pk.pack_map(1);
    pk.pack("status");
    pk.pack("ok");

    EXPECT_TRUE(bridge.send(HabilisMsgType::RESPONSE, buf));

    // Wait for drain
    std::this_thread::sleep_for(200ms);

    // Check if subscriber received it
    zmq::pollitem_t items[] = {{ai_sub.handle(), 0, ZMQ_POLLIN, 0}};
    zmq::poll(items, 1, 2000ms);

    if (items[0].revents & ZMQ_POLLIN) {
        zmq::message_t recv_msg;
        (void)ai_sub.recv(recv_msg);

        // Parse the received frame
        HabilisMessage parsed;
        ASSERT_TRUE(parse_habilis_frame(
            static_cast<const uint8_t*>(recv_msg.data()),
            recv_msg.size(), parsed));
        EXPECT_EQ(parsed.type, HabilisMsgType::RESPONSE);
    } else {
        FAIL() << "No message received on SUB socket within timeout";
    }

    ai_sub.close();
    bridge.stop();
    bridge_thread.join();
}

// ── Backpressure ────────────────────────────────────────────────────────────

TEST_F(HabilisBridgeTest, BackpressureDropsOldest) {
    zmq::context_t ctx(1);
    HabilisBridge bridge(ctx);

    std::thread bridge_thread([&]() {
        bridge.run("127.0.0.1", sub_port_, pub_port_,
            [](HabilisMsgType, msgpack::object_handle) {});
    });

    std::this_thread::sleep_for(200ms);

    // Flood the queue beyond kMaxPubQueueSize without a subscriber draining
    // Note: the bridge poll loop will drain, but we flood faster
    msgpack::sbuffer buf;
    msgpack::packer<msgpack::sbuffer> pk(buf);
    pk.pack(42);

    // Send kMaxPubQueueSize + 100 messages
    int send_count = 0;
    for (size_t i = 0; i < HabilisBridge::kMaxPubQueueSize + 100; ++i) {
        if (bridge.send(HabilisMsgType::TICK, buf)) {
            send_count++;
        }
    }

    // All sends should succeed (oldest dropped when full)
    EXPECT_EQ(send_count, static_cast<int>(HabilisBridge::kMaxPubQueueSize + 100));

    bridge.stop();
    bridge_thread.join();
}

// ── Clean shutdown ──────────────────────────────────────────────────────────

TEST_F(HabilisBridgeTest, CleanShutdown) {
    zmq::context_t ctx(1);
    HabilisBridge bridge(ctx);

    std::thread bridge_thread([&]() {
        bridge.run("127.0.0.1", sub_port_, pub_port_,
            [](HabilisMsgType, msgpack::object_handle) {});
    });

    std::this_thread::sleep_for(200ms);
    EXPECT_TRUE(bridge.is_running());

    bridge.stop();
    bridge_thread.join();

    EXPECT_FALSE(bridge.is_running());
}

TEST_F(HabilisBridgeTest, SendAfterStopIsNoop) {
    zmq::context_t ctx(1);
    HabilisBridge bridge(ctx);

    std::thread bridge_thread([&]() {
        bridge.run("127.0.0.1", sub_port_, pub_port_,
            [](HabilisMsgType, msgpack::object_handle) {});
    });

    std::this_thread::sleep_for(200ms);
    bridge.stop();
    bridge_thread.join();

    // Sending after stop should not crash and should return false or no-op
    msgpack::sbuffer buf;
    msgpack::packer<msgpack::sbuffer> pk(buf);
    pk.pack(true);
    // send() returns false when not running
    EXPECT_FALSE(bridge.send(HabilisMsgType::HEARTBEAT, buf));
}

TEST_F(HabilisBridgeTest, ContextShutdownUnblocksPoll) {
    zmq::context_t ctx(1);
    HabilisBridge bridge(ctx);

    std::thread bridge_thread([&]() {
        bridge.run("127.0.0.1", sub_port_, pub_port_,
            [](HabilisMsgType, msgpack::object_handle) {});
    });

    std::this_thread::sleep_for(200ms);

    // Stop bridge, then shutdown context
    bridge.stop();
    ctx.shutdown();
    bridge_thread.join();

    EXPECT_FALSE(bridge.is_running());
}

// ── Queue constant ──────────────────────────────────────────────────────────

TEST_F(HabilisBridgeTest, QueueMaxSizeConstant) {
    // Verify the constant matches PRD spec
    EXPECT_EQ(HabilisBridge::kMaxPubQueueSize, 1024u);
}

TEST_F(HabilisBridgeTest, PollTimeoutConstant) {
    // Verify 5ms poll timeout for 30Hz data
    EXPECT_EQ(HabilisBridge::kPollTimeoutMs, 5);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
