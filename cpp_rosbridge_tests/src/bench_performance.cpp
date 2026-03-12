/// @file bench_performance.cpp
/// @brief Performance benchmarks for before/after comparison of tachybridge fixes.

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
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "cpp_rosbridge_core/protocol.hpp"

using json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

class PerfBench : public ::testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override {
        node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("bench_node");
        protocol_ = std::make_unique<cpp_rosbridge_core::Protocol>(node_.get());
        pool_ = std::make_unique<boost::asio::thread_pool>(4);
        protocol_->set_work_pool(pool_.get());
    }
    void TearDown() override {
        protocol_.reset();
        pool_->join();
        pool_.reset();
        node_.reset();
    }

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::unique_ptr<cpp_rosbridge_core::Protocol> protocol_;
    std::unique_ptr<boost::asio::thread_pool> pool_;
};

TEST_F(PerfBench, C2_ServiceCallCallerBlockingTime) {
    json request = {
        {"op", "call_service"},
        {"service", "/bench_nonexistent_service"},
        {"type", "std_srvs/srv/Empty"}
    };

    auto start = Clock::now();
    protocol_->handle_message(request.dump(), [](const std::string&) {});
    auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
        Clock::now() - start).count();

    double elapsed_ms = elapsed_us / 1000.0;
    std::cout << "[BENCH C2] Caller thread blocking time: "
              << elapsed_ms << " ms" << std::endl;
    RecordProperty("C2_caller_block_ms", elapsed_ms);
}

TEST_F(PerfBench, PingThroughput) {
    const int num_threads = 8;
    const int pings_per_thread = 5000;
    std::atomic<int> total_responses{0};

    auto start = Clock::now();
    std::vector<std::thread> threads;
    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back([&]() {
            for (int i = 0; i < pings_per_thread; ++i) {
                protocol_->handle_message(
                    R"({"op":"ping"})",
                    [&](const std::string&) { total_responses.fetch_add(1); });
            }
        });
    }
    for (auto& t : threads) t.join();

    auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
        Clock::now() - start).count();
    int total = num_threads * pings_per_thread;
    double elapsed_s = elapsed_us / 1e6;
    double ops_per_sec = total / elapsed_s;

    std::cout << "[BENCH Ping] " << total << " pings in " << elapsed_s << "s = "
              << static_cast<int>(ops_per_sec) << " ops/sec" << std::endl;
    EXPECT_EQ(total_responses.load(), total);
    RecordProperty("ping_ops_per_sec", ops_per_sec);
}

TEST_F(PerfBench, SubscriptionThroughput) {
    auto& sub_mgr = protocol_->subscription_manager();
    const int num_threads = 4;
    const int ops_per_thread = 200;
    std::atomic<int> completed{0};

    auto start = Clock::now();
    std::vector<std::thread> threads;
    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back([&, t]() {
            for (int i = 0; i < ops_per_thread; ++i) {
                uint64_t sid = static_cast<uint64_t>(t * 10000 + i);
                std::string topic = "/bench_topic_" + std::to_string(i % 10);
                sub_mgr.subscribe(topic, "std_msgs/msg/String", sid,
                    [](const json&) {});
                sub_mgr.unsubscribe(topic, sid);
                completed.fetch_add(1);
            }
        });
    }
    for (auto& t : threads) t.join();

    auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
        Clock::now() - start).count();
    int total = num_threads * ops_per_thread;
    double elapsed_s = elapsed_us / 1e6;
    double ops_per_sec = total / elapsed_s;

    std::cout << "[BENCH Sub] " << total << " sub+unsub pairs in "
              << elapsed_s << "s = " << static_cast<int>(ops_per_sec)
              << " ops/sec" << std::endl;
    EXPECT_EQ(completed.load(), total);
    RecordProperty("sub_ops_per_sec", ops_per_sec);
}

TEST_F(PerfBench, FanOutLatency) {
    auto& sub_mgr = protocol_->subscription_manager();
    const int num_sessions = 100;
    std::atomic<int> receive_count{0};

    for (int i = 0; i < num_sessions; ++i) {
        sub_mgr.subscribe("/bench_fanout", "std_msgs/msg/String",
            static_cast<uint64_t>(i + 1),
            [&](const json&) { receive_count.fetch_add(1); });
    }

    const int measure_count = 100;
    std::vector<double> latencies_us;
    latencies_us.reserve(measure_count);

    for (int i = 0; i < measure_count; ++i) {
        uint64_t sid = static_cast<uint64_t>(10000 + i);
        auto start = Clock::now();
        sub_mgr.subscribe("/bench_fanout", "std_msgs/msg/String", sid,
            [](const json&) {});
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            Clock::now() - start).count();
        latencies_us.push_back(static_cast<double>(elapsed));
        sub_mgr.unsubscribe("/bench_fanout", sid);
    }

    for (int i = 0; i < num_sessions; ++i) {
        sub_mgr.unsubscribe("/bench_fanout", static_cast<uint64_t>(i + 1));
    }

    double avg = std::accumulate(latencies_us.begin(), latencies_us.end(), 0.0)
                 / latencies_us.size();
    double max_val = *std::max_element(latencies_us.begin(), latencies_us.end());

    std::cout << "[BENCH Fan] Subscribe latency with " << num_sessions
              << " existing sessions: avg=" << avg << "us, max=" << max_val
              << "us" << std::endl;
    RecordProperty("fanout_subscribe_avg_us", avg);
    RecordProperty("fanout_subscribe_max_us", max_val);
}

TEST_F(PerfBench, WorkPoolConcurrency) {
    const int num_tasks = 4;
    std::atomic<int> running{0};
    std::atomic<int> max_running{0};
    std::atomic<bool> release{false};
    std::atomic<int> finished{0};

    for (int i = 0; i < num_tasks; ++i) {
        protocol_->post_work([&]() {
            int r = running.fetch_add(1) + 1;
            int prev = max_running.load();
            while (prev < r && !max_running.compare_exchange_weak(prev, r)) {}

            while (running.load() < num_tasks && !release.load()) {
                std::this_thread::sleep_for(1ms);
            }
            std::this_thread::sleep_for(10ms);

            running.fetch_sub(1);
            finished.fetch_add(1);
            release.store(true);
        });
    }

    auto deadline = Clock::now() + 5s;
    while (finished.load() < num_tasks && Clock::now() < deadline) {
        std::this_thread::sleep_for(10ms);
    }

    std::cout << "[BENCH Pool] Max concurrent tasks: " << max_running.load()
              << " / " << num_tasks << std::endl;
    EXPECT_EQ(finished.load(), num_tasks);
    RecordProperty("work_pool_max_concurrent", max_running.load());
}

TEST_F(PerfBench, LoggingOverhead) {
    const int burst = 10000;
    std::atomic<int> responses{0};

    auto start = Clock::now();
    for (int i = 0; i < burst; ++i) {
        protocol_->handle_message(R"({"op":"ping"})",
            [&](const std::string&) { responses.fetch_add(1); });
    }
    auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
        Clock::now() - start).count();

    double elapsed_ms = elapsed_us / 1000.0;
    double per_msg_us = static_cast<double>(elapsed_us) / burst;

    std::cout << "[BENCH Log] " << burst << " pings: " << elapsed_ms
              << "ms total, " << per_msg_us << "us/msg" << std::endl;
    EXPECT_EQ(responses.load(), burst);
    RecordProperty("logging_total_ms", elapsed_ms);
    RecordProperty("logging_per_msg_us", per_msg_us);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
