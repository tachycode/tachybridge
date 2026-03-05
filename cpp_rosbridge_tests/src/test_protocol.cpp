#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nlohmann/json.hpp>
#include <future>
#include <chrono>

#include "cpp_rosbridge_core/protocol.hpp"

class ProtocolTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    void SetUp() override {
        node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
        protocol_ = std::make_unique<cpp_rosbridge_core::Protocol>(node_.get());
    }

    void TearDown() override {
        node_.reset();
        protocol_.reset();
    }

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::unique_ptr<cpp_rosbridge_core::Protocol> protocol_;
};

TEST_F(ProtocolTest, PingOpReturnsPong) {
    std::string request = R"({"op":"ping"})";
    std::promise<std::string> promise;
    auto future = promise.get_future();

    auto sender = [&](const std::string& response) {
        promise.set_value(response);
    };

    protocol_->handle_message(request, sender);

    ASSERT_EQ(future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
    
    nlohmann::json response_json = nlohmann::json::parse(future.get());
    EXPECT_EQ(response_json["op"], "pong");
}

TEST_F(ProtocolTest, PingWithIdReturnsPongWithId) {
    std::string request = R"({"op":"ping", "id":"test_id_123"})";
    std::promise<std::string> promise;
    auto future = promise.get_future();

    auto sender = [&](const std::string& response) {
        promise.set_value(response);
    };

    protocol_->handle_message(request, sender);

    ASSERT_EQ(future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
    
    nlohmann::json response_json = nlohmann::json::parse(future.get());
    EXPECT_EQ(response_json["op"], "pong");
    EXPECT_EQ(response_json["id"], "test_id_123");
}

TEST_F(ProtocolTest, InvalidJsonReturnsNothing) {
    std::string request = R"({"op":"ping")"; // Invalid JSON
    std::promise<std::string> promise;
    auto future = promise.get_future();

    bool sender_called = false;
    auto sender = [&](const std::string&) {
        sender_called = true;
    };

    protocol_->handle_message(request, sender);

    // We expect the sender to NOT be called, so the future should time out.
    ASSERT_EQ(future.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);
    ASSERT_FALSE(sender_called);
}

TEST_F(ProtocolTest, MissingOpReturnsNothing) {
    std::string request = R"({"id":"test_id_123"})"; // Missing "op"
    std::promise<std::string> promise;
    auto future = promise.get_future();

    bool sender_called = false;
    auto sender = [&](const std::string&) {
        sender_called = true;
    };

    protocol_->handle_message(request, sender);
    
    ASSERT_EQ(future.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);
    ASSERT_FALSE(sender_called);
}

TEST_F(ProtocolTest, UnknownOpReturnsNothing) {
    std::string request = R"({"op":"this_op_does_not_exist"})";
    std::promise<std::string> promise;
    auto future = promise.get_future();

    bool sender_called = false;
    auto sender = [&](const std::string&) {
        sender_called = true;
    };

    protocol_->handle_message(request, sender);
    
    ASSERT_EQ(future.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);
    ASSERT_FALSE(sender_called);
}

TEST_F(ProtocolTest, CborPingReturnsCborPong) {
    nlohmann::json request = {{"op", "ping"}, {"id", "cbor_test"}};
    auto cbor_data = nlohmann::json::to_cbor(request);

    std::promise<std::vector<uint8_t>> promise;
    auto future = promise.get_future();

    auto sender = [&](const std::vector<uint8_t>& response) {
        promise.set_value(response);
    };

    protocol_->handle_binary_message(cbor_data.data(), cbor_data.size(), sender);

    ASSERT_EQ(future.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    auto response_json = nlohmann::json::from_cbor(future.get());
    EXPECT_EQ(response_json["op"], "pong");
    EXPECT_EQ(response_json["id"], "cbor_test");
}

TEST_F(ProtocolTest, InvalidCborReturnsNothing) {
    std::vector<uint8_t> bad_cbor = {0xFF, 0xFF, 0xFF};

    bool sender_called = false;
    auto sender = [&](const std::vector<uint8_t>&) {
        sender_called = true;
    };

    protocol_->handle_binary_message(bad_cbor.data(), bad_cbor.size(), sender);
    ASSERT_FALSE(sender_called);
}

TEST_F(ProtocolTest, PublisherTypeTracking) {
    auto pub = node_->create_generic_publisher("test_topic", "std_msgs/msg/String", rclcpp::QoS(10));
    protocol_->add_publisher("test_topic", "std_msgs/msg/String", pub);

    EXPECT_NE(protocol_->get_publisher("test_topic"), nullptr);
    EXPECT_EQ(protocol_->get_publisher_type("test_topic"), "std_msgs/msg/String");
    EXPECT_EQ(protocol_->get_publisher("nonexistent"), nullptr);
    EXPECT_EQ(protocol_->get_publisher_type("nonexistent"), "");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
