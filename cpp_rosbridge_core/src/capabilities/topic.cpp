#include "cpp_rosbridge_core/capabilities/topic.hpp"
#include "cpp_rosbridge_core/introspection.hpp"
#include <chrono>
#include <deque>
#include <mutex>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <unordered_map>
#include <unordered_set>

namespace cpp_rosbridge_core {
namespace capabilities {

namespace {

constexpr double kHzWindowSec = 5.0;
constexpr double kHzPublishIntervalSec = 1.0;
constexpr const char* kPublisherHzStatusTopic = "/simdd/publisher_hz_status";

struct TopicHzCounter {
    std::deque<std::chrono::steady_clock::time_point> samples;
    std::chrono::steady_clock::time_point last_publish{};
};

std::mutex g_hz_mutex;
std::unordered_map<std::string, TopicHzCounter> g_hz_counters;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_hz_publisher;

bool is_saved_topic(const std::string& topic_name) {
    static const std::unordered_set<std::string> saved_topics = {
        "/simdd/left_wrist_cam/compressed",
        "/simdd/right_wrist_cam/compressed",
        "/simdd/head_cam/compressed",
        "/simdd/pose_head",
        "/simdd/pose_left",
        "/simdd/pose_right",
        "/simdd/trigger_left",
        "/simdd/trigger_right",
    };
    return saved_topics.find(topic_name) != saved_topics.end();
}

double calculate_hz(const std::deque<std::chrono::steady_clock::time_point>& samples) {
    if (samples.size() < 2) {
        return 0.0;
    }
    const std::chrono::duration<double> elapsed = samples.back() - samples.front();
    if (elapsed.count() <= 0.0) {
        return 0.0;
    }
    return static_cast<double>(samples.size() - 1) / elapsed.count();
}

void report_publisher_hz(
    rclcpp_lifecycle::LifecycleNode* node,
    const std::string& topic_name) {
    if (!is_saved_topic(topic_name)) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    std_msgs::msg::String msg;
    {
        std::lock_guard<std::mutex> lock(g_hz_mutex);
        if (!g_hz_publisher) {
            g_hz_publisher = node->create_publisher<std_msgs::msg::String>(
                kPublisherHzStatusTopic,
                rclcpp::QoS(10));
        }

        auto& counter = g_hz_counters[topic_name];
        counter.samples.push_back(now);
        const auto cutoff = now - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(kHzWindowSec));
        while (!counter.samples.empty() && counter.samples.front() < cutoff) {
            counter.samples.pop_front();
        }

        if (
            counter.last_publish.time_since_epoch().count() != 0
            && std::chrono::duration<double>(now - counter.last_publish).count()
                < kHzPublishIntervalSec) {
            return;
        }
        counter.last_publish = now;

        nlohmann::json payload = {
            {"topic_name", topic_name},
            {"hz", calculate_hz(counter.samples)},
            {"source", "cpp_rosbridge_server"},
            {"stamp", node->now().seconds()},
        };
        msg.data = payload.dump();
    }
    g_hz_publisher->publish(msg);
}

}  // namespace

// --- Ping ---

void Ping::handle_message(const nlohmann::json& message, std::function<void(const nlohmann::json&)> sender, bool /*cbor_mode*/) {
    nlohmann::json response;
    response["op"] = "pong";

    if (message.contains("id")) {
        response["id"] = message["id"];
    }

    sender(response);
}

// --- Advertise ---

void Advertise::handle_message(const nlohmann::json& message, std::function<void(const nlohmann::json&)> sender, bool /*cbor_mode*/) {
    if (!message.contains("topic") || !message.contains("type")) {
        nlohmann::json err = {
            {"op", "status"},
            {"level", "error"},
            {"msg", "Missing 'topic' or 'type' field in advertise"}
        };
        sender(err);
        return;
    }

    std::string topic_name = message["topic"];
    std::string type_name = message["type"];

    if (protocol_->get_publisher(topic_name) != nullptr) {
        return;
    }

    rclcpp_lifecycle::LifecycleNode* node = protocol_->get_node();
    RCLCPP_INFO(node->get_logger(), "Advertising topic '%s' with type '%s'", topic_name.c_str(), type_name.c_str());

    try {
        auto publisher = node->create_generic_publisher(topic_name, type_name, rclcpp::QoS(10));
        protocol_->add_publisher(topic_name, type_name, publisher);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to create generic publisher for topic '%s': %s", topic_name.c_str(), e.what());
    }
}

// --- Publish ---

void Publish::handle_message(const nlohmann::json& message, std::function<void(const nlohmann::json&)> sender, bool /*cbor_mode*/) {
    if (!message.contains("topic") || !message.contains("msg")) {
        return;
    }

    std::string topic_name = message["topic"];
    auto publisher = protocol_->get_publisher(topic_name);

    if (publisher == nullptr) {
        RCLCPP_WARN(protocol_->get_node()->get_logger(),
            "Received publish for unadvertised topic '%s'", topic_name.c_str());
        return;
    }

    std::string type_name = protocol_->get_publisher_type(topic_name);
    if (type_name.empty()) {
        nlohmann::json response = {
            {"op", "status"},
            {"level", "error"},
            {"msg", "No type info for topic: " + topic_name}
        };
        sender(response);
        return;
    }

    try {
        auto serialized = RosMessageConverter::convert_json_to_ros_message(
            type_name, message["msg"]);
        auto* generic_pub = dynamic_cast<rclcpp::GenericPublisher*>(publisher.get());
        if (generic_pub) {
            generic_pub->publish(*serialized);
            report_publisher_hz(protocol_->get_node(), topic_name);
        } else {
            nlohmann::json response = {
                {"op", "status"},
                {"level", "error"},
                {"msg", "Publisher for '" + topic_name + "' is not a GenericPublisher"}
            };
            sender(response);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(protocol_->get_node()->get_logger(),
            "Failed to publish on '%s': %s", topic_name.c_str(), e.what());
        nlohmann::json response = {
            {"op", "status"},
            {"level", "error"},
            {"msg", std::string("Publish failed: ") + e.what()}
        };
        sender(response);
    }
}

// --- Subscribe ---

void Subscribe::handle_message(const nlohmann::json& message,
                                std::function<void(const nlohmann::json&)> sender,
                                bool /*cbor_mode*/) {
    if (!message.contains("topic") || !message.contains("type")) {
        nlohmann::json err = {
            {"op", "status"},
            {"level", "error"},
            {"msg", "Missing 'topic' or 'type' field in subscribe"}
        };
        sender(err);
        return;
    }

    std::string topic = message["topic"];
    std::string type = message["type"];
    uint64_t session_id = Protocol::current_session_id();

    protocol_->subscription_manager().subscribe(topic, type, session_id, sender);
}

// --- Unsubscribe ---

void Unsubscribe::handle_message(const nlohmann::json& message,
                                  std::function<void(const nlohmann::json&)> sender,
                                  bool /*cbor_mode*/) {
    if (!message.contains("topic")) {
        nlohmann::json err = {
            {"op", "status"},
            {"level", "error"},
            {"msg", "Missing 'topic' field in unsubscribe"}
        };
        sender(err);
        return;
    }

    std::string topic = message["topic"];
    uint64_t session_id = Protocol::current_session_id();

    protocol_->subscription_manager().unsubscribe(topic, session_id);
}

}  // namespace capabilities
}  // namespace cpp_rosbridge_core
