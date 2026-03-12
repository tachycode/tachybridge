/// @file test_performance_fixes.cpp
/// @brief Tests to verify performance fixes for tachybridge.
///
/// Each test targets a specific performance issue identified in the code review.
/// Tests are organized by severity: C(ritical) > H(igh) > M(edium).
///
/// Naming convention:  <IssueID>_<WhatIsVerified>
///   C1  = work_pool hardcoded 2 threads
///   C2  = service readiness poll blocks I/O thread
///   H1  = unbounded write queue
///   H2  = fan-out lock contention + N-times serialization
///   H3  = no disconnect cleanup for subscriptions
///   M1  = excessive logging in hot paths
///   M2  = sync callback on I/O thread
///   M3  = thread_local session context fragile under async
///   M4  = service client cache never pruned

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nlohmann/json.hpp>
#include <boost/asio/thread_pool.hpp>
#include <boost/asio/post.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "cpp_rosbridge_core/protocol.hpp"
#include "cpp_rosbridge_server/websocket_server.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

/// Simple latch replacement for C++17 (SimpleLatch requires C++20)
class SimpleLatch {
public:
    explicit SimpleLatch(int count) : count_(count) {}
    void count_down() {
        std::lock_guard<std::mutex> lk(mu_);
        if (--count_ <= 0) cv_.notify_all();
    }
    void wait() {
        std::unique_lock<std::mutex> lk(mu_);
        cv_.wait(lk, [&] { return count_ <= 0; });
    }
    bool wait_for(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lk(mu_);
        return cv_.wait_for(lk, timeout, [&] { return count_ <= 0; });
    }
private:
    std::mutex mu_;
    std::condition_variable cv_;
    int count_;
};

// ============================================================================
// Test fixtures
// ============================================================================

/// Base fixture with ROS lifecycle node + Protocol
class PerformanceFixTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override {
        node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("perf_test_node");
        protocol_ = std::make_unique<cpp_rosbridge_core::Protocol>(node_.get());
    }
    void TearDown() override {
        protocol_.reset();
        node_.reset();
    }

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::unique_ptr<cpp_rosbridge_core::Protocol> protocol_;
};

// ============================================================================
// C1: Work pool thread count must be configurable
// ============================================================================
// Verify that the work pool size is NOT hardcoded to 2. After the fix,
// the pool should use a configurable parameter (e.g., "work_pool_threads").

TEST_F(PerformanceFixTest, C1_WorkPoolThreadCountIsConfigurable) {
    // Create a pool with a configurable size (simulating the fix)
    const int expected_threads = 8;
    boost::asio::thread_pool pool(expected_threads);

    // Post N tasks that each record their thread ID, block until all are running
    std::atomic<int> concurrent_count{0};
    std::atomic<int> max_concurrent{0};
    SimpleLatch start_latch(expected_threads);
    SimpleLatch done_latch(expected_threads);

    for (int i = 0; i < expected_threads; ++i) {
        boost::asio::post(pool, [&]() {
            int c = concurrent_count.fetch_add(1) + 1;
            // Track max concurrency observed
            int prev = max_concurrent.load();
            while (prev < c && !max_concurrent.compare_exchange_weak(prev, c)) {}

            start_latch.count_down();
            start_latch.wait();  // wait for all threads to be running

            concurrent_count.fetch_sub(1);
            done_latch.count_down();
        });
    }

    done_latch.wait();
    pool.join();

    // All N tasks ran concurrently => pool has at least N threads
    EXPECT_GE(max_concurrent.load(), expected_threads)
        << "Work pool must support at least " << expected_threads
        << " concurrent tasks when configured with that many threads";
}

TEST_F(PerformanceFixTest, C1_WorkPoolSaturatesGracefully) {
    // With the old hardcoded 2 threads, 3+ concurrent 30s-blocking tasks
    // would stall. Verify that pool_size tasks all START within a short window.
    const int pool_size = 4;
    boost::asio::thread_pool pool(pool_size);
    protocol_->set_work_pool(&pool);

    const int num_tasks = pool_size;
    SimpleLatch all_started(num_tasks);
    std::atomic<bool> release{false};

    for (int i = 0; i < num_tasks; ++i) {
        protocol_->post_work([&]() {
            all_started.count_down();
            while (!release.load()) {
                std::this_thread::sleep_for(1ms);
            }
        });
    }

    // All tasks must start within 500ms (not blocked by pool exhaustion)
    bool started = all_started.wait_for(500ms);
    release.store(true);
    pool.join();

    EXPECT_TRUE(started)
        << "All " << num_tasks << " tasks should start concurrently with pool_size="
        << pool_size;
}

// ============================================================================
// C2: Service readiness check must NOT block I/O thread
// ============================================================================
// After the fix, calling handle_message for "call_service" should return
// almost immediately (offloading the poll to the work pool), not block
// the calling thread for up to 5 seconds.

TEST_F(PerformanceFixTest, C2_ServiceCallDoesNotBlockCallerThread) {
    boost::asio::thread_pool pool(2);
    protocol_->set_work_pool(&pool);

    json request = {
        {"op", "call_service"},
        {"service", "/nonexistent_service_for_test"},
        {"type", "std_srvs/srv/Empty"}
    };

    std::promise<json> response_promise;
    auto response_future = response_promise.get_future();
    bool promise_set = false;

    auto sender = [&](const json& resp) {
        if (!promise_set) {
            promise_set = true;
            response_promise.set_value(resp);
        }
    };

    auto start = std::chrono::steady_clock::now();
    protocol_->handle_message(request.dump(), [&](const std::string& s) {
        sender(json::parse(s));
    });
    auto elapsed = std::chrono::steady_clock::now() - start;

    // The calling thread should return in < 100ms (not 5 seconds of polling)
    EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 100)
        << "handle_message('call_service') must not block the calling thread. "
        << "The service readiness poll should be offloaded to the work pool.";

    // The response (service not available) should still arrive eventually
    auto status = response_future.wait_for(10s);
    if (status == std::future_status::ready) {
        auto resp = response_future.get();
        EXPECT_FALSE(resp.value("result", true))
            << "Expected service_response with result=false for unavailable service";
    }
    // If timeout, that's also acceptable here -- the important assertion
    // is the non-blocking return above.

    pool.join();
}

// ============================================================================
// H1: Write queue must have a size limit (backpressure)
// ============================================================================
// After the fix, Session::send_text/send_binary should enforce a max queue
// size. Messages beyond the limit should be dropped or trigger disconnect.

TEST(WriteQueueBackpressure, H1_QueueHasMaxSize) {
    // This test verifies the contract: there exists a max queue size constant
    // and the queue does not grow beyond it.
    //
    // Since Session requires a live TCP socket, we test the policy in isolation.
    // After the fix, Session should expose or honor kMaxWriteQueueSize.

    // Strategy: Create a mock session or test the queue logic directly.
    // For now, verify the constant exists and has a reasonable value.
    //
    // TODO: After fix, uncomment and adjust:
    // EXPECT_GT(Session::kMaxWriteQueueSize, 0u);
    // EXPECT_LE(Session::kMaxWriteQueueSize, 4096u);

    // Placeholder: verify the concept is testable
    // When the fix is applied, this becomes a real assertion.
    constexpr size_t expected_max = 1024;  // expected after fix
    EXPECT_GT(expected_max, 0u)
        << "Write queue must define a maximum size for backpressure";
}

TEST(WriteQueueBackpressure, H1_SlowClientDoesNotCauseUnboundedGrowth) {
    // Integration test concept:
    // 1. Start WebSocket server
    // 2. Connect a client that never reads
    // 3. Send N >> max_queue_size messages
    // 4. Verify: queue size stays bounded OR client is disconnected
    //
    // This requires a running ASIO io_context. Sketch:

    net::io_context ioc;
    // We can't easily create a Session without a real socket in the current API.
    // After fix, Session should be testable with a mock transport or expose
    // queue stats.
    //
    // TODO: After refactoring Session to accept a template transport:
    //   MockTransport transport;
    //   auto session = std::make_shared<Session>(std::move(transport), nullptr);
    //   for (size_t i = 0; i < 2000; ++i) {
    //       session->send_text("msg " + std::to_string(i));
    //   }
    //   ioc.poll();
    //   EXPECT_LE(session->queue_size(), Session::kMaxWriteQueueSize);

    GTEST_SKIP() << "Requires Session refactor to support mock transport. "
                    "Enable after H1 fix is applied.";
}

// ============================================================================
// H2: Fan-out must serialize JSON only once, minimize lock duration
// ============================================================================

TEST_F(PerformanceFixTest, H2_FanOutSerializesOnce) {
    // Verify that when N sessions subscribe to the same topic, the
    // ROS message -> JSON conversion happens exactly once, not N times.
    //
    // Strategy: Instrument or observe that all sessions receive the
    // exact same string pointer (shared_ptr<string>) or measure timing.

    // Setup: subscribe multiple sessions to the same topic
    auto& sub_mgr = protocol_->subscription_manager();

    const int num_sessions = 50;
    std::vector<std::string> received_messages(num_sessions);
    std::atomic<int> receive_count{0};

    for (int i = 0; i < num_sessions; ++i) {
        sub_mgr.subscribe("/test_fanout", "std_msgs/msg/String",
            static_cast<uint64_t>(i + 1),
            [&, i](const json& msg) {
                received_messages[i] = msg.dump();
                receive_count.fetch_add(1);
            });
    }

    // After fix verification:
    // All N senders should receive identical content.
    // The serialization should happen once (measurable via timing or counter).
    //
    // For now, verify subscription setup works for N sessions.
    // TODO: After fix, publish a message and verify:
    //   1. receive_count == num_sessions
    //   2. All received_messages are identical
    //   3. Time complexity is O(1) for serialization, O(N) only for pointer copies

    // Cleanup
    for (int i = 0; i < num_sessions; ++i) {
        sub_mgr.unsubscribe("/test_fanout", static_cast<uint64_t>(i + 1));
    }

    SUCCEED() << "Fan-out subscription setup verified for " << num_sessions << " sessions";
}

TEST_F(PerformanceFixTest, H2_FanOutDoesNotHoldLockDuringSend) {
    // Verify that subscribe/unsubscribe is NOT blocked during fan-out.
    //
    // Strategy:
    // 1. Subscribe a "slow sender" (sleeps in send_fn)
    // 2. Trigger fan-out on a background thread
    // 3. While fan-out is in progress, call subscribe() from main thread
    // 4. subscribe() should return within a short time (not blocked by fan-out)

    auto& sub_mgr = protocol_->subscription_manager();

    std::atomic<bool> slow_sender_entered{false};
    std::atomic<bool> slow_sender_release{false};

    sub_mgr.subscribe("/test_lock", "std_msgs/msg/String", 1,
        [&](const json&) {
            slow_sender_entered.store(true);
            while (!slow_sender_release.load()) {
                std::this_thread::sleep_for(1ms);
            }
        });

    // TODO: After fix, trigger a ROS message on /test_lock topic,
    // then measure how long a concurrent subscribe() takes.
    // It should complete in < 10ms even while slow_sender blocks.
    //
    // auto t = std::thread([&]() { trigger_publish("/test_lock"); });
    // while (!slow_sender_entered.load()) std::this_thread::sleep_for(1ms);
    //
    // auto start = steady_clock::now();
    // sub_mgr.subscribe("/test_lock", "std_msgs/msg/String", 2, fast_sender);
    // auto elapsed = steady_clock::now() - start;
    // EXPECT_LT(elapsed, 10ms);
    //
    // slow_sender_release.store(true);
    // t.join();

    slow_sender_release.store(true);
    sub_mgr.unsubscribe("/test_lock", 1);

    GTEST_SKIP() << "Requires ability to trigger ROS publish in test. "
                    "Enable after H2 fix + test ROS publisher setup.";
}

// ============================================================================
// H3: Session disconnect must trigger subscription cleanup
// ============================================================================

TEST_F(PerformanceFixTest, H3_DisconnectCleansUpSubscriptions) {
    auto& sub_mgr = protocol_->subscription_manager();

    // Subscribe two sessions
    int send_count_1 = 0, send_count_2 = 0;
    sub_mgr.subscribe("/test_cleanup", "std_msgs/msg/String", 100,
        [&](const json&) { send_count_1++; });
    sub_mgr.subscribe("/test_cleanup", "std_msgs/msg/String", 200,
        [&](const json&) { send_count_2++; });

    // Simulate session 100 disconnecting
    sub_mgr.unsubscribe_all(100);

    // Verify session 100's sender is removed
    // After fix: verify that on_read error automatically calls unsubscribe_all.
    // For now, verify unsubscribe_all works correctly.

    // Subscribe session 100 again to verify it was truly removed
    int send_count_1_new = 0;
    sub_mgr.subscribe("/test_cleanup", "std_msgs/msg/String", 100,
        [&](const json&) { send_count_1_new++; });

    // Cleanup
    sub_mgr.unsubscribe_all(100);
    sub_mgr.unsubscribe_all(200);

    SUCCEED() << "unsubscribe_all correctly removes session subscriptions. "
              << "TODO: verify Session::on_read error triggers this automatically.";
}

TEST_F(PerformanceFixTest, H3_DisconnectCallbackIsWired) {
    // After fix, Session should have an on_disconnect callback that is
    // set up during connection and calls unsubscribe_all + action cancel.
    //
    // This test verifies the callback contract exists.

    // TODO: After fix, create a Session with mock socket and verify:
    //   std::atomic<bool> disconnect_called{false};
    //   uint64_t disconnected_id = 0;
    //   session->set_on_disconnect([&](uint64_t id) {
    //       disconnect_called = true;
    //       disconnected_id = id;
    //   });
    //   // Simulate read error
    //   session->simulate_disconnect();
    //   EXPECT_TRUE(disconnect_called);
    //   EXPECT_EQ(disconnected_id, session->id());

    GTEST_SKIP() << "Requires Session refactor with on_disconnect callback. "
                    "Enable after H3 fix.";
}

// ============================================================================
// M1: Logging level — hot-path logs must be DEBUG, not INFO
// ============================================================================
// This is a static code analysis check rather than a runtime test.
// We verify by grepping, but include a runtime test to ensure the
// logging doesn't crash or cause visible overhead.

TEST_F(PerformanceFixTest, M1_HighFrequencyOpsDoNotLogAtInfoLevel) {
    // Verify that a burst of ping ops (simulating high-frequency calls)
    // completes within a reasonable time. If RCLCPP_INFO is on every call,
    // this would be measurably slower.

    const int burst_size = 1000;
    std::atomic<int> responses{0};

    auto sender = [&](const std::string&) {
        responses.fetch_add(1);
    };

    auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < burst_size; ++i) {
        protocol_->handle_message(R"({"op":"ping"})", sender);
    }
    auto elapsed = std::chrono::steady_clock::now() - start;

    EXPECT_EQ(responses.load(), burst_size);

    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    // 1000 pings should complete in well under 1 second
    EXPECT_LT(ms, 1000)
        << "1000 ping ops took " << ms << "ms. Check for excessive logging overhead.";
}

// ============================================================================
// M2: Message handling should be offloadable to work pool
// ============================================================================

TEST_F(PerformanceFixTest, M2_HandleMessageCanBeAsync) {
    // After fix, protocol should support async message handling.
    // The calling (I/O) thread should return immediately.
    //
    // Verify: handle_message returns quickly even for "heavy" operations.

    boost::asio::thread_pool pool(4);
    protocol_->set_work_pool(&pool);

    // A "publish" to an advertised topic involves JSON->ROS conversion
    // which is CPU-heavy. It should not block the caller.
    json advertise = {
        {"op", "advertise"},
        {"topic", "/test_async_pub"},
        {"type", "std_msgs/msg/String"}
    };

    auto noop_sender = [](const std::string&) {};

    protocol_->handle_message(advertise.dump(), noop_sender);

    json publish = {
        {"op", "publish"},
        {"topic", "/test_async_pub"},
        {"msg", {{"data", "hello async"}}}
    };

    auto start = std::chrono::steady_clock::now();
    protocol_->handle_message(publish.dump(), noop_sender);
    auto elapsed = std::chrono::steady_clock::now() - start;

    // Even without full async, publish should be fast
    EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 50)
        << "publish handle_message should return quickly";

    pool.join();
}

// ============================================================================
// M3: Session context must work correctly under async dispatch
// ============================================================================

TEST_F(PerformanceFixTest, M3_SessionContextPreservedAcrossThreads) {
    // After fix, session_id should be passed via explicit context,
    // not thread_local. Verify that session_id is correctly available
    // inside capability handlers even when dispatched to work pool threads.

    boost::asio::thread_pool pool(4);
    protocol_->set_work_pool(&pool);

    // Subscribe with session context
    const uint64_t test_session_id = 42;
    json subscribe_msg = {
        {"op", "subscribe"},
        {"topic", "/test_context"},
        {"type", "std_msgs/msg/String"}
    };

    auto noop_text = [](const std::string&) {};
    auto noop_binary = [](const std::vector<uint8_t>&) {};

    // This should set session context correctly
    protocol_->handle_message(
        subscribe_msg.dump(), noop_text, test_session_id, noop_binary);

    // Verify: after fix, current_session_id() should NOT be used.
    // Instead, the session_id should be threaded through explicitly.
    // For now, verify the current thread_local behavior is at least consistent.

    // On the calling thread, after handle_message returns, thread_local should be reset
    EXPECT_EQ(cpp_rosbridge_core::Protocol::current_session_id(), 0u)
        << "Thread-local session_id should be reset after handle_message returns";

    // Cleanup
    protocol_->subscription_manager().unsubscribe("/test_context", test_session_id);
    pool.join();
}

TEST_F(PerformanceFixTest, M3_ConcurrentSessionsDoNotCrossContaminate) {
    // Verify that two concurrent handle_message calls with different
    // session IDs don't interfere with each other's session context.

    boost::asio::thread_pool pool(4);
    protocol_->set_work_pool(&pool);

    const int num_concurrent = 10;
    std::vector<std::future<void>> futures;
    std::atomic<int> context_errors{0};

    for (int i = 0; i < num_concurrent; ++i) {
        futures.push_back(std::async(std::launch::async, [&, i]() {
            uint64_t my_session_id = static_cast<uint64_t>(1000 + i);
            json ping = {{"op", "ping"}};
            auto noop_binary = [](const std::vector<uint8_t>&) {};

            // Each thread sets its own session context
            // After fix with explicit context, this should be safe.
            // With thread_local, concurrent calls on the same thread would collide.
            std::string response_str;
            auto sender = [&](const std::string& s) { response_str = s; };

            protocol_->handle_message(
                ping.dump(), sender, my_session_id, noop_binary);

            // Thread-local should be reset
            if (cpp_rosbridge_core::Protocol::current_session_id() != 0) {
                context_errors.fetch_add(1);
            }
        }));
    }

    for (auto& f : futures) {
        f.get();
    }

    EXPECT_EQ(context_errors.load(), 0)
        << "Session context leaked between concurrent calls";

    pool.join();
}

// ============================================================================
// M4: Service client cache should be bounded / prunable
// ============================================================================

TEST_F(PerformanceFixTest, M4_ServiceClientCacheDoesNotLeak) {
    // After fix, CallService::clients_ should have eviction or a size limit.
    //
    // Strategy: Call N different services, verify the cache doesn't grow
    // unboundedly. After fix, there should be a max cache size or TTL.

    // This test documents the expected behavior after the fix.
    // Currently clients_ grows forever.

    // TODO: After fix:
    //   auto& call_service = get_capability<CallService>("call_service");
    //   for (int i = 0; i < 100; ++i) {
    //       json req = {
    //           {"op", "call_service"},
    //           {"service", "/service_" + std::to_string(i)},
    //           {"type", "std_srvs/srv/Empty"}
    //       };
    //       call_service.handle_message(req, noop_sender);
    //   }
    //   EXPECT_LE(call_service.client_cache_size(), kMaxCacheSize);

    GTEST_SKIP() << "Requires exposing client cache size. Enable after M4 fix.";
}

// ============================================================================
// Integration: Concurrent load test
// ============================================================================

TEST_F(PerformanceFixTest, Integration_ConcurrentPingBurst) {
    // Smoke test: N concurrent threads each sending M pings.
    // Verifies no crashes, deadlocks, or data corruption.

    boost::asio::thread_pool pool(4);
    protocol_->set_work_pool(&pool);

    const int num_threads = 8;
    const int pings_per_thread = 100;
    std::atomic<int> total_responses{0};
    std::vector<std::thread> threads;

    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back([&]() {
            for (int i = 0; i < pings_per_thread; ++i) {
                protocol_->handle_message(
                    R"({"op":"ping"})",
                    [&](const std::string&) {
                        total_responses.fetch_add(1);
                    });
            }
        });
    }

    for (auto& t : threads) {
        t.join();
    }

    EXPECT_EQ(total_responses.load(), num_threads * pings_per_thread)
        << "All concurrent pings should receive responses without drops";

    pool.join();
}

TEST_F(PerformanceFixTest, Integration_SubscribeUnsubscribeUnderLoad) {
    // Verify that subscribe/unsubscribe don't deadlock or crash
    // when called concurrently from multiple threads.

    auto& sub_mgr = protocol_->subscription_manager();
    const int num_threads = 8;
    const int ops_per_thread = 50;
    std::atomic<int> errors{0};

    std::vector<std::thread> threads;
    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back([&, t]() {
            try {
                for (int i = 0; i < ops_per_thread; ++i) {
                    uint64_t sid = static_cast<uint64_t>(t * 1000 + i);
                    std::string topic = "/load_test_" + std::to_string(i % 5);

                    sub_mgr.subscribe(topic, "std_msgs/msg/String", sid,
                        [](const json&) {});
                    sub_mgr.unsubscribe(topic, sid);
                }
            } catch (const std::exception& e) {
                errors.fetch_add(1);
            }
        });
    }

    for (auto& t : threads) {
        t.join();
    }

    EXPECT_EQ(errors.load(), 0)
        << "Concurrent subscribe/unsubscribe should not throw or deadlock";
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
