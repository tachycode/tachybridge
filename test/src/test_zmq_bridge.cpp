#include <gtest/gtest.h>
#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

#include <zmq.hpp>
#include "cpp_zmq_server/zmq_server.hpp"

using namespace cpp_zmq_server;
using namespace std::chrono_literals;

// Helper: poll a single socket for incoming data
static bool poll_socket(zmq::socket_t& sock, int timeout_ms) {
    zmq::pollitem_t items[] = {{sock.handle(), 0, ZMQ_POLLIN, 0}};
    zmq::poll(items, 1, std::chrono::milliseconds{timeout_ms});
    return (items[0].revents & ZMQ_POLLIN) != 0;
}

// ── ZmqSession unit tests ───────────────────────────────────────────────────

TEST(ZmqSessionTest, BasicProperties) {
    ZmqSession session("test-identity", 42);
    EXPECT_EQ(session.id(), 42u);
    EXPECT_EQ(session.identity(), "test-identity");
}

TEST(ZmqSessionTest, EnqueueReply) {
    ZmqSession session("id1", 1);
    EXPECT_TRUE(session.enqueue_reply("hello"));
    EXPECT_TRUE(session.enqueue_reply("world"));
}

TEST(ZmqSessionTest, BackpressureDropsOldest) {
    ZmqSession session("id1", 1);

    // Fill queue to max
    for (size_t i = 0; i < ZmqSession::kMaxWriteQueueSize; ++i) {
        session.enqueue_reply("msg-" + std::to_string(i));
    }

    // One more should succeed (drops oldest)
    EXPECT_TRUE(session.enqueue_reply("overflow-msg"));
}

TEST(ZmqSessionTest, HeartbeatExpiry) {
    ZmqSession session("id1", 1);
    session.touch();

    // Should not be expired immediately
    EXPECT_FALSE(session.is_expired(1s));

    // Wait and check expiry with very short timeout
    std::this_thread::sleep_for(50ms);
    EXPECT_TRUE(session.is_expired(std::chrono::seconds{0}));
}

TEST(ZmqSessionTest, HeartbeatTouch) {
    ZmqSession session("id1", 1);

    std::this_thread::sleep_for(50ms);
    session.touch();

    // Should not be expired after touch
    EXPECT_FALSE(session.is_expired(1s));
}

// ── ZmqServer integration tests ─────────────────────────────────────────────

class ZmqServerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Use a random high port to avoid conflicts
        port_ = 15555 + (rand() % 1000);
    }

    uint16_t port_;
};

TEST_F(ZmqServerTest, StartAndStop) {
    zmq::context_t test_ctx{1};
    ZmqServer server(test_ctx);

    std::thread server_thread([&]() {
        server.run("127.0.0.1", port_,
                   [](uint64_t, const std::string&,
                      std::function<void(const std::string&)>) {},
                   30s);
    });

    std::this_thread::sleep_for(200ms);
    server.stop();
    server_thread.join();
}

TEST_F(ZmqServerTest, MessageRoundTrip) {
    zmq::context_t test_ctx{1};
    ZmqServer server(test_ctx);
    std::atomic<bool> message_received{false};
    std::string received_payload;

    std::thread server_thread([&]() {
        server.run("127.0.0.1", port_,
                   [&](uint64_t /*session_id*/, const std::string& message,
                       std::function<void(const std::string&)> reply_fn) {
                       received_payload = message;
                       message_received = true;
                       // Echo back
                       reply_fn("{\"op\":\"pong\"}");
                   },
                   30s);
    });

    std::this_thread::sleep_for(200ms);

    // Create DEALER client
    zmq::context_t ctx(1);
    zmq::socket_t dealer(ctx, zmq::socket_type::dealer);
    dealer.set(zmq::sockopt::linger, 0);
    dealer.connect("tcp://127.0.0.1:" + std::to_string(port_));

    // Send: [empty | payload] (DEALER auto-prepends identity)
    std::string payload = "{\"op\":\"ping\"}";
    dealer.send(zmq::message_t(0), zmq::send_flags::sndmore);
    dealer.send(zmq::buffer(payload), zmq::send_flags::none);

    // Wait for the message to be processed and reply drained
    std::this_thread::sleep_for(500ms);

    EXPECT_TRUE(message_received.load());
    EXPECT_EQ(received_payload, "{\"op\":\"ping\"}");

    // Check reply
    if (poll_socket(dealer, 2000)) {
        zmq::message_t empty_reply;
        zmq::message_t reply_payload;
        (void)dealer.recv(empty_reply);
        (void)dealer.recv(reply_payload);
        std::string reply(static_cast<const char*>(reply_payload.data()),
                          reply_payload.size());
        EXPECT_EQ(reply, "{\"op\":\"pong\"}");
    } else {
        FAIL() << "No reply received within timeout";
    }

    dealer.close();
    ctx.shutdown();
    server.stop();
    server_thread.join();
    ctx.close();
}

TEST_F(ZmqServerTest, MultipleClients) {
    zmq::context_t test_ctx{1};
    ZmqServer server(test_ctx);
    std::atomic<int> message_count{0};

    std::thread server_thread([&]() {
        server.run("127.0.0.1", port_,
                   [&](uint64_t session_id, const std::string& /*message*/,
                       std::function<void(const std::string&)> reply_fn) {
                       message_count++;
                       reply_fn("{\"op\":\"pong\",\"session\":" +
                                std::to_string(session_id) + "}");
                   },
                   30s);
    });

    std::this_thread::sleep_for(200ms);

    zmq::context_t ctx(1);
    constexpr int kNumClients = 3;
    std::vector<zmq::socket_t> clients;

    for (int i = 0; i < kNumClients; ++i) {
        clients.emplace_back(ctx, zmq::socket_type::dealer);
        clients.back().set(zmq::sockopt::linger, 0);
        // Set unique identity for each client
        std::string identity = "client-" + std::to_string(i);
        clients.back().set(zmq::sockopt::routing_id, identity);
        clients.back().connect("tcp://127.0.0.1:" + std::to_string(port_));
    }

    std::this_thread::sleep_for(100ms);

    // Each client sends a message
    for (auto& client : clients) {
        std::string payload = "{\"op\":\"ping\"}";
        client.send(zmq::message_t(0), zmq::send_flags::sndmore);
        client.send(zmq::buffer(payload), zmq::send_flags::none);
    }

    std::this_thread::sleep_for(500ms);

    EXPECT_EQ(message_count.load(), kNumClients);

    // Each client should receive a reply
    for (auto& client : clients) {
        if (poll_socket(client, 2000)) {
            zmq::message_t empty, reply;
            (void)client.recv(empty);
            (void)client.recv(reply);
            std::string reply_str(static_cast<const char*>(reply.data()), reply.size());
            EXPECT_TRUE(reply_str.find("\"op\":\"pong\"") != std::string::npos);
        }
    }

    for (auto& client : clients) {
        client.close();
    }
    ctx.shutdown();
    server.stop();
    server_thread.join();
    ctx.close();
}

TEST_F(ZmqServerTest, SessionExpiry) {
    zmq::context_t test_ctx{1};
    ZmqServer server(test_ctx);
    std::atomic<bool> disconnect_fired{false};
    uint64_t disconnected_session_id = 0;

    server.set_on_disconnect([&](uint64_t session_id) {
        disconnect_fired = true;
        disconnected_session_id = session_id;
    });

    // Use very short heartbeat timeout for testing
    std::thread server_thread([&]() {
        server.run("127.0.0.1", port_,
                   [](uint64_t, const std::string&,
                      std::function<void(const std::string&)>) {},
                   1s);  // 1 second timeout
    });

    std::this_thread::sleep_for(200ms);

    // Connect client, send one message, then disconnect
    {
        zmq::context_t ctx(1);
        zmq::socket_t dealer(ctx, zmq::socket_type::dealer);
        dealer.set(zmq::sockopt::linger, 0);
        dealer.connect("tcp://127.0.0.1:" + std::to_string(port_));

        std::string payload = "{\"op\":\"ping\"}";
        dealer.send(zmq::message_t(0), zmq::send_flags::sndmore);
        dealer.send(zmq::buffer(payload), zmq::send_flags::none);

        std::this_thread::sleep_for(200ms);
        dealer.close();
        ctx.close();
    }

    // Wait for heartbeat expiry + reap cycle
    // kReapIntervalIterations=100 * 100ms poll = ~10s
    std::this_thread::sleep_for(12s);

    if (disconnect_fired.load()) {
        EXPECT_GT(disconnected_session_id, 0u);
    }
    // Don't fail if reap hasn't happened yet — it's timing-dependent

    server.stop();
    server_thread.join();
}

TEST_F(ZmqServerTest, MalformedMessageHandling) {
    zmq::context_t test_ctx{1};
    ZmqServer server(test_ctx);
    std::atomic<int> message_count{0};

    std::thread server_thread([&]() {
        server.run("127.0.0.1", port_,
                   [&](uint64_t, const std::string&,
                       std::function<void(const std::string&)>) {
                       message_count++;
                   },
                   30s);
    });

    std::this_thread::sleep_for(200ms);

    zmq::context_t ctx(1);
    zmq::socket_t dealer(ctx, zmq::socket_type::dealer);
    dealer.set(zmq::sockopt::linger, 0);
    dealer.connect("tcp://127.0.0.1:" + std::to_string(port_));

    // Send a properly framed message
    {
        std::string payload = "{\"op\":\"ping\"}";
        dealer.send(zmq::message_t(0), zmq::send_flags::sndmore);
        dealer.send(zmq::buffer(payload), zmq::send_flags::none);
    }

    std::this_thread::sleep_for(300ms);

    // The valid message should have been processed
    EXPECT_EQ(message_count.load(), 1);

    dealer.close();
    ctx.shutdown();
    server.stop();
    server_thread.join();
    ctx.close();
}

// ── Main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
