#include "cpp_rosbridge_core/capabilities/topic.hpp"
#include "cpp_rosbridge_core/introspection.hpp"
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace cpp_rosbridge_core {
namespace capabilities {

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
