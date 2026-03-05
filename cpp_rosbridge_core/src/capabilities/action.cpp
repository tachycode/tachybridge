#include "cpp_rosbridge_core/capabilities/action.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace cpp_rosbridge_core {
namespace capabilities {

// --- SendActionGoal ---

void SendActionGoal::handle_message(const nlohmann::json& message,
                                     std::function<void(const nlohmann::json&)> sender,
                                     bool cbor_mode) {
    std::string action_topic = message.value("action", message.value("serverName", ""));
    std::string action_type = message.value("action_type", message.value("actionName", ""));

    if (action_topic.empty() || action_type.empty()) {
        nlohmann::json err = {
            {"op", "action_result"},
            {"error", "Missing 'action'/'serverName' or 'action_type'/'actionName' field"}
        };
        sender(err);
        return;
    }
    nlohmann::json goal_data = message.value("goal", nlohmann::json::object());

    auto node = protocol_->get_node();
    auto& registry = protocol_->action_registry();

    if (!registry.has_bridge(action_type)) {
        RCLCPP_ERROR(node->get_logger(), "No bridge registered for action type '%s'",
                     action_type.c_str());
        nlohmann::json err = {
            {"op", "action_result"},
            {"action", action_topic},
            {"error", "Unknown action type: " + action_type}
        };
        sender(err);
        return;
    }

    auto* bridge = registry.get_bridge(node, action_type, action_topic, 10);
    if (!bridge) {
        nlohmann::json err = {
            {"op", "action_result"},
            {"action", action_topic},
            {"error", "Failed to create bridge for " + action_type}
        };
        sender(err);
        return;
    }

    action::ActionCallbacks callbacks;
    if (message.contains("session_id")) {
        const auto& sid = message["session_id"];
        if (sid.is_number()) {
            callbacks.session_id = sid.get<uint64_t>();
        } else if (sid.is_string()) {
            try { callbacks.session_id = std::stoull(sid.get<std::string>()); } catch (...) {}
        }
    }
    callbacks.send_text = [sender](const std::string& text) {
        try {
            sender(nlohmann::json::parse(text));
        } catch (...) {
            sender(nlohmann::json{{"op", "action_result"}, {"raw", text}});
        }
    };
    auto binary_fn = Protocol::current_binary_sender();
    callbacks.send_binary = [sender, binary_fn](const std::vector<uint8_t>& data) {
        if (binary_fn) {
            binary_fn(data);
        } else {
            try {
                sender(nlohmann::json::from_cbor(data));
            } catch (...) {}
        }
    };

    RCLCPP_INFO(node->get_logger(), "Sending action goal to '%s' (type: %s)",
                action_topic.c_str(), action_type.c_str());

    try {
        bridge->execute(goal_data, callbacks, cbor_mode);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Action execute failed for '%s': %s",
                     action_topic.c_str(), e.what());
        nlohmann::json err = {
            {"op", "action_result"},
            {"action", action_topic},
            {"error", std::string("Action execute failed: ") + e.what()}
        };
        sender(err);
    }
}

// --- CancelActionGoal ---

void CancelActionGoal::handle_message(const nlohmann::json& message,
                                       std::function<void(const nlohmann::json&)> sender,
                                       bool cbor_mode) {
    (void)cbor_mode;

    if (!message.contains("action") || !message.contains("action_type")) {
        nlohmann::json err = {
            {"op", "cancel_action_result"},
            {"error", "Missing 'action' or 'action_type' field"}
        };
        sender(err);
        return;
    }

    std::string action_topic = message["action"];
    std::string action_type = message["action_type"];
    uint64_t session_id = 0;
    if (message.contains("session_id")) {
        const auto& sid = message["session_id"];
        if (sid.is_number()) {
            session_id = sid.get<uint64_t>();
        } else if (sid.is_string()) {
            try { session_id = std::stoull(sid.get<std::string>()); } catch (...) {}
        }
    }

    auto node = protocol_->get_node();
    auto& registry = protocol_->action_registry();

    auto* bridge = registry.get_bridge(node, action_type, action_topic, 10);
    if (!bridge) {
        RCLCPP_WARN(node->get_logger(), "No bridge found for cancel: %s on %s",
                    action_type.c_str(), action_topic.c_str());
        nlohmann::json err = {
            {"op", "cancel_action_result"},
            {"action", action_topic},
            {"error", "No active bridge for " + action_type}
        };
        sender(err);
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Cancelling action on '%s' (session: %lu)",
                action_topic.c_str(), session_id);

    bridge->cancel(session_id);

    nlohmann::json ack = {
        {"op", "cancel_action_result"},
        {"action", action_topic},
        {"result", true}
    };
    sender(ack);
}

}  // namespace capabilities

// ── ActionBridgeRegistry ─────────────────────────────────────────────────────

namespace action {

void ActionBridgeRegistry::register_bridge(const std::string& action_name,
                                           BridgeFactory factory) {
    std::unique_lock lock(mutex_);
    factories_[action_name] = std::move(factory);
}

ActionBridgeBase* ActionBridgeRegistry::get_bridge(
    rclcpp_lifecycle::LifecycleNode* node, const std::string& action_name,
    const std::string& topic, int timeout_s) {
    std::string key = action_name + "|" + topic;

    {
        std::shared_lock lock(mutex_);
        auto it = bridges_.find(key);
        if (it != bridges_.end()) {
            return it->second.get();
        }
    }

    std::unique_lock lock(mutex_);
    auto it = bridges_.find(key);
    if (it != bridges_.end()) {
        return it->second.get();
    }

    auto fac_it = factories_.find(action_name);
    if (fac_it == factories_.end()) {
        return nullptr;
    }

    auto bridge = fac_it->second(node, topic, timeout_s);
    auto* ptr = bridge.get();
    bridges_[key] = std::move(bridge);
    return ptr;
}

bool ActionBridgeRegistry::has_bridge(const std::string& action_name) const {
    std::shared_lock lock(mutex_);
    return factories_.count(action_name) > 0;
}

std::vector<std::string> ActionBridgeRegistry::action_names() const {
    std::shared_lock lock(mutex_);
    std::vector<std::string> names;
    names.reserve(factories_.size());
    for (const auto& [name, _] : factories_) {
        names.push_back(name);
    }
    return names;
}

}  // namespace action
}  // namespace cpp_rosbridge_core
