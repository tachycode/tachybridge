#pragma once

#include "cpp_rosbridge_core/protocol.hpp"
#include "cpp_rosbridge_core/introspection.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <future>

#include <rclcpp_action/rclcpp_action.hpp>

namespace cpp_rosbridge_core {

// ── Capability classes ───────────────────────────────────────────────────────

namespace capabilities {

class SendActionGoal : public Capability {
public:
    using Capability::Capability;
    void handle_message(const nlohmann::json& message,
                        std::function<void(const nlohmann::json&)> sender,
                        bool cbor_mode = false) override;
};

class CancelActionGoal : public Capability {
public:
    using Capability::Capability;
    void handle_message(const nlohmann::json& message,
                        std::function<void(const nlohmann::json&)> sender,
                        bool cbor_mode = false) override;
};

}  // namespace capabilities

// ── Generic action bridge ────────────────────────────────────────────────────

namespace action {

template <typename ActionT>
class GenericBridge : public ActionBridgeBase {
    using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

public:
    GenericBridge(rclcpp_lifecycle::LifecycleNode* node, const std::string& topic,
                  int timeout_s)
        : node_(node), topic_(topic), timeout_s_(timeout_s) {
        client_ = rclcpp_action::create_client<ActionT>(
            node_->get_node_base_interface(),
            node_->get_node_graph_interface(),
            node_->get_node_logging_interface(),
            node_->get_node_waitables_interface(),
            topic);

        goal_members_ = introspection::get_members<typename ActionT::Goal>();
        feedback_members_ = introspection::get_members<typename ActionT::Feedback>();
        result_members_ = introspection::get_members<typename ActionT::Result>();

        feedback_has_images_ =
            introspection::has_compressed_image_fields(feedback_members_);
        result_has_images_ =
            introspection::has_compressed_image_fields(result_members_);

        std::string name = goal_members_->message_name_;
        auto pos = name.rfind("_Goal");
        action_label_ = (pos != std::string::npos) ? name.substr(0, pos) : name;

        RCLCPP_INFO(node_->get_logger(),
                    "GenericBridge<%s> created for '%s'%s",
                    action_label_.c_str(), topic.c_str(),
                    feedback_has_images_ ? " (binary image streaming)" : "");
    }

    void execute(const nlohmann::json& goal_data,
                 const ActionCallbacks& callbacks,
                 bool cbor_mode) override {
        const std::string goal_service = topic_ + "/_action/send_goal";
        std::size_t goal_service_servers = 0;
        const auto service_map = node_->get_service_names_and_types();
        for (const auto& [service_name, _] : service_map) {
            if (service_name == goal_service) {
                ++goal_service_servers;
            }
        }
        if (goal_service_servers > 1) {
            RCLCPP_WARN(
                node_->get_logger(),
                "Detected %zu servers for action goal service '%s'. "
                "Multiple action servers on the same name can trigger "
                "'unknown goal response, ignoring...' in rclcpp_action.",
                goal_service_servers, goal_service.c_str());
        }

        if (!client_->wait_for_action_server(std::chrono::seconds(timeout_s_))) {
            nlohmann::json err = {
                {"type", "error"},
                {"message", action_label_ + " action server not available"}
            };
            callbacks.send_text(err.dump());
            return;
        }

        typename ActionT::Goal goal_msg{};
        introspection::json_to_msg(goal_data, &goal_msg, goal_members_);

        RCLCPP_INFO(node_->get_logger(), "Sending %s goal to %s",
                    action_label_.c_str(), topic_.c_str());

        nlohmann::json ack = {
            {"type", "request"},
            {"message", "Sending goal to " + action_label_},
            {"goal", goal_data}
        };
        callbacks.send_text(ack.dump());

        auto send_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
        auto session_id = callbacks.session_id;
        const uint64_t request_id = next_request_id_.fetch_add(1, std::memory_order_relaxed);

        // Copy callbacks for use in lambdas
        auto cb = std::make_shared<ActionCallbacks>(callbacks);
        bool use_cbor = cbor_mode;

        send_options.feedback_callback =
            [this, cb, use_cbor](
                typename GoalHandle::SharedPtr,
                const std::shared_ptr<const typename ActionT::Feedback> fb) {
                try {
                    nlohmann::json wrapper;
                    wrapper["type"] = "feedback";
                    if (use_cbor) {
                        wrapper["feedback"] = introspection::msg_to_cbor_json(
                            fb.get(), feedback_members_);
                        auto cbor_data = nlohmann::json::to_cbor(wrapper);
                        cb->send_binary(cbor_data);
                    } else {
                        wrapper["feedback"] = introspection::msg_to_json(
                            fb.get(), feedback_members_);
                        cb->send_text(wrapper.dump());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(node_->get_logger(),
                                 "%s feedback error: %s",
                                 action_label_.c_str(), e.what());
                }
            };

        send_options.goal_response_callback =
            [this, cb, session_id, request_id](typename GoalHandle::SharedPtr goal_handle) {
                {
                    std::lock_guard lock(goals_mutex_);
                    pending_goal_requests_.erase(request_id);
                }
                if (!goal_handle) {
                    nlohmann::json err = {
                        {"type", "error"},
                        {"message", action_label_ + " goal was rejected"}
                    };
                    cb->send_text(err.dump());
                    return;
                }
                {
                    std::lock_guard lock(goals_mutex_);
                    session_goals_[session_id] = goal_handle;
                }
                RCLCPP_INFO(node_->get_logger(), "%s goal accepted",
                            action_label_.c_str());
            };

        send_options.result_callback =
            [this, cb, session_id, request_id, use_cbor](
                const typename GoalHandle::WrappedResult& result) {
                try {
                    {
                        std::lock_guard lock(goals_mutex_);
                        session_goals_.erase(session_id);
                        pending_goal_requests_.erase(request_id);
                    }

                    nlohmann::json msg;
                    msg["type"] = "result";
                    msg["status"] = static_cast<int>(result.code);

                    if (!result.result) {
                        msg["result"] = nullptr;
                    } else if (use_cbor) {
                        msg["result"] = introspection::msg_to_cbor_json(
                            result.result.get(), result_members_);
                        auto cbor_data = nlohmann::json::to_cbor(msg);
                        cb->send_binary(cbor_data);
                        RCLCPP_INFO(node_->get_logger(), "%s result sent",
                                    action_label_.c_str());
                        return;
                    } else {
                        msg["result"] = introspection::msg_to_json(
                            result.result.get(), result_members_);
                    }
                    cb->send_text(msg.dump());
                    RCLCPP_INFO(node_->get_logger(), "%s result sent",
                                action_label_.c_str());
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(node_->get_logger(),
                                 "%s result callback error: %s",
                                 action_label_.c_str(), e.what());
                    nlohmann::json err;
                    err["type"] = "error";
                    err["message"] = action_label_ + " result error: " + e.what();
                    cb->send_text(err.dump());
                }
            };

        auto goal_future = client_->async_send_goal(goal_msg, send_options);
        {
            std::lock_guard lock(goals_mutex_);
            pending_goal_requests_[request_id] = goal_future;
        }
    }

    void cancel(uint64_t session_id) override {
        std::lock_guard lock(goals_mutex_);
        auto it = session_goals_.find(session_id);
        if (it != session_goals_.end() && it->second) {
            client_->async_cancel_goal(it->second);
            session_goals_.erase(it);
        }
    }

private:
    rclcpp_lifecycle::LifecycleNode* node_;
    std::string topic_;
    typename rclcpp_action::Client<ActionT>::SharedPtr client_;
    int timeout_s_;
    std::string action_label_;

    const introspection::MessageMembers* goal_members_;
    const introspection::MessageMembers* feedback_members_;
    const introspection::MessageMembers* result_members_;
    bool feedback_has_images_ = false;
    bool result_has_images_ = false;

    std::mutex goals_mutex_;
    std::atomic<uint64_t> next_request_id_{1};
    std::unordered_map<uint64_t, std::shared_future<typename GoalHandle::SharedPtr>> pending_goal_requests_;
    std::unordered_map<uint64_t, typename GoalHandle::SharedPtr> session_goals_;
};

template <typename ActionT>
void register_generic(ActionBridgeRegistry& registry,
                      const std::string& action_name) {
    registry.register_bridge(
        action_name,
        [](rclcpp_lifecycle::LifecycleNode* node, const std::string& topic,
           int timeout_s) -> std::unique_ptr<ActionBridgeBase> {
            return std::make_unique<GenericBridge<ActionT>>(
                node, topic, timeout_s);
        });
}

}  // namespace action
}  // namespace cpp_rosbridge_core
