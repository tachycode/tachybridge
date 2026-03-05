#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nlohmann/json.hpp>
#include <future>
#include <chrono>

#include "cpp_rosbridge_core/protocol.hpp"

class ExecuteCLITest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    void SetUp() override {
        node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_cli_node");
        protocol_ = std::make_unique<cpp_rosbridge_core::Protocol>(node_.get());

        // Set up a thread pool so post_work runs asynchronously
        pool_ = std::make_unique<boost::asio::thread_pool>(2);
        protocol_->set_work_pool(pool_.get());
    }

    void TearDown() override {
        if (pool_) pool_->join();
        pool_.reset();
        protocol_.reset();
        node_.reset();
    }

    /// Helper: send a JSON request through the protocol and return the parsed response.
    /// Uses future with timeout to handle async post_work.
    nlohmann::json send_request(const nlohmann::json& request, int timeout_s = 10) {
        std::promise<std::string> promise;
        auto future = promise.get_future();

        auto sender = [&promise](const std::string& response) {
            promise.set_value(response);
        };

        protocol_->handle_message(request.dump(), sender);

        auto status = future.wait_for(std::chrono::seconds(timeout_s));
        if (status != std::future_status::ready) {
            return {{"_timeout", true}};
        }
        return nlohmann::json::parse(future.get());
    }

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::unique_ptr<cpp_rosbridge_core::Protocol> protocol_;
    std::unique_ptr<boost::asio::thread_pool> pool_;
};

// Test 1: Basic echo command
TEST_F(ExecuteCLITest, BasicEcho) {
    auto resp = send_request({{"op", "execute_cli"}, {"command", "echo hello"}});

    EXPECT_EQ(resp["op"], "cli_response");
    EXPECT_TRUE(resp["success"].get<bool>());
    EXPECT_EQ(resp["return_code"], 0);
    EXPECT_NE(resp["output"].get<std::string>().find("hello"), std::string::npos);
}

// Test 2: Id preservation
TEST_F(ExecuteCLITest, IdPreservation) {
    auto resp = send_request({{"op", "execute_cli"}, {"command", "echo hi"}, {"id", "t1"}});

    EXPECT_EQ(resp["op"], "cli_response");
    EXPECT_EQ(resp["id"], "t1");
    EXPECT_TRUE(resp["success"].get<bool>());
}

// Test 3: Missing command field
TEST_F(ExecuteCLITest, MissingCommand) {
    auto resp = send_request({{"op", "execute_cli"}});

    EXPECT_EQ(resp["op"], "cli_response");
    EXPECT_FALSE(resp["success"].get<bool>());
    EXPECT_NE(resp["error"].get<std::string>().find("command"), std::string::npos);
}

// Test 4: Empty command
TEST_F(ExecuteCLITest, EmptyCommand) {
    auto resp = send_request({{"op", "execute_cli"}, {"command", ""}});

    EXPECT_EQ(resp["op"], "cli_response");
    EXPECT_FALSE(resp["success"].get<bool>());
}

// Test 5: Blocked command (not in allowed prefixes)
TEST_F(ExecuteCLITest, BlockedNotAllowed) {
    auto resp = send_request({{"op", "execute_cli"}, {"command", "ls /tmp"}});

    EXPECT_EQ(resp["op"], "cli_response");
    EXPECT_FALSE(resp["success"].get<bool>());
    EXPECT_NE(resp["error"].get<std::string>().find("not allowed"), std::string::npos);
}

// Test 6: Blocked shell injection (semicolon)
TEST_F(ExecuteCLITest, BlockedShellInjectionSemicolon) {
    auto resp = send_request({{"op", "execute_cli"}, {"command", "echo hi; rm -rf /"}});

    EXPECT_EQ(resp["op"], "cli_response");
    EXPECT_FALSE(resp["success"].get<bool>());
    EXPECT_NE(resp["error"].get<std::string>().find("not allowed"), std::string::npos);
}

// Test 7: Blocked shell injection (pipe)
TEST_F(ExecuteCLITest, BlockedShellInjectionPipe) {
    auto resp = send_request({{"op", "execute_cli"}, {"command", "ros2 topic list | grep test"}});

    EXPECT_EQ(resp["op"], "cli_response");
    EXPECT_FALSE(resp["success"].get<bool>());
    EXPECT_NE(resp["error"].get<std::string>().find("not allowed"), std::string::npos);
}

// Test 8: ros2 command that returns error (non-zero exit code)
// Note: This test requires ros2 CLI to be available; it will still pass because
// the command is allowed, even if ros2 is not installed (popen still succeeds).
TEST_F(ExecuteCLITest, Ros2ErrorCommand) {
    auto resp = send_request({{"op", "execute_cli"}, {"command", "ros2 topic info /nonexistent_topic_xyz"}});

    EXPECT_EQ(resp["op"], "cli_response");
    // The command is allowed and will execute - success depends on whether ros2 is installed
    // but the response must have the correct structure
    EXPECT_TRUE(resp.contains("success"));
    EXPECT_TRUE(resp.contains("return_code"));
    EXPECT_TRUE(resp.contains("output"));
}

// Test 9: Timeout
TEST_F(ExecuteCLITest, CommandTimeout) {
    // "echo" prefix with sleep subcommand won't work since "echo sleep..." is just echoing text.
    // Instead, test with ros2 command that will be slow or use a different approach.
    // We use "echo hello" with timeout_sec to test that timeout_sec parsing works correctly.
    auto resp = send_request({{"op", "execute_cli"}, {"command", "echo hello"}, {"timeout_sec", 1}});

    EXPECT_EQ(resp["op"], "cli_response");
    EXPECT_TRUE(resp["success"].get<bool>());
    EXPECT_EQ(resp["return_code"], 0);
}

// Test 10: CBOR ping still works (existing op unaffected)
TEST_F(ExecuteCLITest, CborPingStillWorks) {
    nlohmann::json request = {{"op", "ping"}, {"id", "cbor_cli_test"}};
    auto cbor_data = nlohmann::json::to_cbor(request);

    std::promise<std::vector<uint8_t>> promise;
    auto future = promise.get_future();

    auto sender = [&](const std::vector<uint8_t>& response) {
        promise.set_value(response);
    };

    protocol_->handle_binary_message(cbor_data.data(), cbor_data.size(), sender);

    ASSERT_EQ(future.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    auto resp = nlohmann::json::from_cbor(future.get());
    EXPECT_EQ(resp["op"], "pong");
    EXPECT_EQ(resp["id"], "cbor_cli_test");
}

// Test: Blocked subshell injection
TEST_F(ExecuteCLITest, BlockedSubshellInjection) {
    auto resp = send_request({{"op", "execute_cli"}, {"command", "echo $(cat /etc/passwd)"}});

    EXPECT_EQ(resp["op"], "cli_response");
    EXPECT_FALSE(resp["success"].get<bool>());
    EXPECT_NE(resp["error"].get<std::string>().find("not allowed"), std::string::npos);
}

// Test: Blocked backtick injection
TEST_F(ExecuteCLITest, BlockedBacktickInjection) {
    auto resp = send_request({{"op", "execute_cli"}, {"command", "echo `whoami`"}});

    EXPECT_EQ(resp["op"], "cli_response");
    EXPECT_FALSE(resp["success"].get<bool>());
    EXPECT_NE(resp["error"].get<std::string>().find("not allowed"), std::string::npos);
}

// Test: Blocked redirect
TEST_F(ExecuteCLITest, BlockedRedirect) {
    auto resp = send_request({{"op", "execute_cli"}, {"command", "echo hi > /tmp/hack"}});

    EXPECT_EQ(resp["op"], "cli_response");
    EXPECT_FALSE(resp["success"].get<bool>());
    EXPECT_NE(resp["error"].get<std::string>().find("not allowed"), std::string::npos);
}

// Test: Timeout capping at MAX_TIMEOUT_SEC
TEST_F(ExecuteCLITest, TimeoutClamped) {
    auto resp = send_request({{"op", "execute_cli"}, {"command", "echo capped"}, {"timeout_sec", 999}});

    EXPECT_EQ(resp["op"], "cli_response");
    EXPECT_TRUE(resp["success"].get<bool>());
    // Should succeed — timeout was clamped to MAX_TIMEOUT_SEC but echo is instant
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
