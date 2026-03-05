#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nlohmann/json.hpp>
#include <future>
#include <chrono>

#include "cpp_rosbridge_core/protocol.hpp"

class ActionCapabilityTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    void SetUp() override {
        node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("action_test_node");
        protocol_ = std::make_unique<cpp_rosbridge_core::Protocol>(node_.get());
    }

    void TearDown() override {
        protocol_.reset();
        node_.reset();
    }

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::unique_ptr<cpp_rosbridge_core::Protocol> protocol_;
};

TEST_F(ActionCapabilityTest, SendActionGoalMissingFields) {
    std::string request = R"({"op":"send_action_goal"})";
    std::promise<std::string> promise;
    auto future = promise.get_future();

    protocol_->handle_message(request, [&](const std::string& response) {
        promise.set_value(response);
    });

    ASSERT_EQ(future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
    auto resp = nlohmann::json::parse(future.get());
    EXPECT_TRUE(resp.contains("error"));
}

TEST_F(ActionCapabilityTest, SendActionGoalUnknownType) {
    std::string request = R"({
        "op": "send_action_goal",
        "action": "/nonexistent",
        "action_type": "nonexistent_pkg/action/Fake"
    })";
    std::promise<std::string> promise;
    auto future = promise.get_future();

    protocol_->handle_message(request, [&](const std::string& response) {
        promise.set_value(response);
    });

    ASSERT_EQ(future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
    auto resp = nlohmann::json::parse(future.get());
    EXPECT_TRUE(resp.contains("error"));
}

TEST_F(ActionCapabilityTest, CancelActionMissingFields) {
    std::string request = R"({"op":"cancel_action_goal"})";
    std::promise<std::string> promise;
    auto future = promise.get_future();

    protocol_->handle_message(request, [&](const std::string& response) {
        promise.set_value(response);
    });

    ASSERT_EQ(future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
    auto resp = nlohmann::json::parse(future.get());
    EXPECT_TRUE(resp.contains("error"));
}

TEST_F(ActionCapabilityTest, CancelActionNoActiveBridge) {
    std::string request = R"({
        "op": "cancel_action_goal",
        "action": "/nonexistent",
        "action_type": "nonexistent_pkg/action/Fake",
        "session_id": 42
    })";
    std::promise<std::string> promise;
    auto future = promise.get_future();

    protocol_->handle_message(request, [&](const std::string& response) {
        promise.set_value(response);
    });

    ASSERT_EQ(future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
    auto resp = nlohmann::json::parse(future.get());
    EXPECT_TRUE(resp.contains("error"));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
