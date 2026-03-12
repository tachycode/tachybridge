#include "cpp_rosbridge_core/protocol.hpp"
#include "cpp_rosbridge_core/capabilities.hpp"
#include <boost/asio/post.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace cpp_rosbridge_core {

thread_local uint64_t Protocol::tl_session_id_ = 0;
thread_local Protocol::BinarySenderFn Protocol::tl_binary_sender_;

Protocol::~Protocol() = default;

Protocol::Protocol(rclcpp_lifecycle::LifecycleNode* node)
    : node_(node),
      sub_manager_(std::make_unique<SubscriptionManager>(node)) {
    // Register all available capabilities here
    register_capability("ping", std::make_unique<capabilities::Ping>(this));
    register_capability("subscribe", std::make_unique<capabilities::Subscribe>(this));
    register_capability("unsubscribe", std::make_unique<capabilities::Unsubscribe>(this));
    register_capability("advertise", std::make_unique<capabilities::Advertise>(this));
    register_capability("publish", std::make_unique<capabilities::Publish>(this));
    register_capability("call_service", std::make_unique<capabilities::CallService>(this));
    register_capability("send_action_goal", std::make_unique<capabilities::SendActionGoal>(this));
    register_capability("cancel_action_goal", std::make_unique<capabilities::CancelActionGoal>(this));
    register_capability("execute_cli", std::make_unique<capabilities::ExecuteCLI>(this));
    register_generated_actions(action_registry_);
}

void Protocol::register_capability(const std::string& op_code, std::unique_ptr<Capability> capability) {
    capabilities_[op_code] = std::move(capability);
}

/// If type_name is short format "package/Type", expand to "package/{sub_ns}/Type".
/// If already "package/sub_ns/Type", return as-is.
static std::string normalize_type(const std::string& type_name, const std::string& default_sub_ns) {
    if (type_name.empty()) return type_name;
    auto first = type_name.find('/');
    if (first == std::string::npos) return type_name;
    if (first != type_name.rfind('/')) return type_name;  // already has sub_ns
    return type_name.substr(0, first) + "/" + default_sub_ns + "/" + type_name.substr(first + 1);
}

static void normalize_message_types(nlohmann::json& msg, const std::string& op) {
    if (msg.contains("type") && msg["type"].is_string()) {
        std::string sub_ns;
        if (op == "subscribe" || op == "unsubscribe" || op == "advertise" || op == "publish")
            sub_ns = "msg";
        else if (op == "call_service")
            sub_ns = "srv";
        if (!sub_ns.empty())
            msg["type"] = normalize_type(msg["type"].get<std::string>(), sub_ns);
    }
    if (op == "send_action_goal" || op == "cancel_action_goal") {
        for (const char* key : {"action_type", "actionName"}) {
            if (msg.contains(key) && msg[key].is_string())
                msg[key] = normalize_type(msg[key].get<std::string>(), "action");
        }
    }
}

void Protocol::handle_message(const std::string& message_string, std::function<void(const std::string&)> sender) {
    nlohmann::json message_json;
    try {
        message_json = nlohmann::json::parse(message_string);
    } catch (const nlohmann::json::parse_error& e) {
        RCLCPP_ERROR(node_->get_logger(), "JSON Parse Error: %s", e.what());
        return;
    }

    if (!message_json.contains("op")) {
        RCLCPP_WARN(node_->get_logger(), "Received message without 'op' field.");
        return;
    }

    const std::string op = message_json["op"];
    normalize_message_types(message_json, op);

    auto it = capabilities_.find(op);

    if (it != capabilities_.end()) {
        auto json_sender = [sender](const nlohmann::json& json_msg){
            sender(json_msg.dump());
        };
        it->second->handle_message(message_json, json_sender);
    } else {
        RCLCPP_WARN(node_->get_logger(), "No capability handler registered for op: %s", op.c_str());
    }
}

void Protocol::handle_message(const std::string& message_string,
                               std::function<void(const std::string&)> sender,
                               uint64_t session_id,
                               BinarySenderFn binary_sender) {
    tl_session_id_ = session_id;
    tl_binary_sender_ = std::move(binary_sender);
    handle_message(message_string, std::move(sender));
    tl_binary_sender_ = nullptr;
    tl_session_id_ = 0;
}

uint64_t Protocol::current_session_id() {
    return tl_session_id_;
}

Protocol::BinarySenderFn Protocol::current_binary_sender() {
    return tl_binary_sender_;
}

void Protocol::handle_binary_message(const uint8_t* data, size_t size,
                                      std::function<void(const std::vector<uint8_t>&)> binary_sender) {
    nlohmann::json msg_json;
    try {
        msg_json = nlohmann::json::from_cbor(data, data + size);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "CBOR parse error: %s", e.what());
        return;
    }

    if (!msg_json.contains("op")) {
        RCLCPP_WARN(node_->get_logger(), "CBOR message without 'op' field.");
        return;
    }

    const std::string op = msg_json["op"];
    normalize_message_types(msg_json, op);

    auto it = capabilities_.find(op);

    if (it != capabilities_.end()) {
        auto cbor_sender = [binary_sender](const nlohmann::json& j) {
            binary_sender(nlohmann::json::to_cbor(j));
        };
        it->second->handle_message(msg_json, cbor_sender, true);
    } else {
        RCLCPP_WARN(node_->get_logger(), "No capability handler for CBOR op: %s", op.c_str());
    }
}

void Protocol::add_publisher(const std::string& topic, const std::string& type, rclcpp::PublisherBase::SharedPtr pub) {
    std::unique_lock lock(publishers_mutex_);
    publishers_[topic] = pub;
    publisher_types_[topic] = type;
}

std::string Protocol::get_publisher_type(const std::string& topic) {
    std::shared_lock lock(publishers_mutex_);
    auto it = publisher_types_.find(topic);
    return (it != publisher_types_.end()) ? it->second : "";
}

rclcpp::PublisherBase::SharedPtr Protocol::get_publisher(const std::string& topic_name) {
    std::shared_lock lock(publishers_mutex_);
    if (publishers_.count(topic_name)) {
        return publishers_[topic_name];
    }
    return nullptr;
}

rclcpp_lifecycle::LifecycleNode* Protocol::get_node() {
    return node_;
}

action::ActionBridgeRegistry& Protocol::action_registry() {
    return action_registry_;
}

void Protocol::set_work_pool(boost::asio::thread_pool* pool) {
    work_pool_ = pool;
}

void Protocol::post_work(std::function<void()> fn) {
    if (work_pool_) {
        boost::asio::post(*work_pool_, std::move(fn));
    } else {
        fn();
    }
}

SubscriptionManager& Protocol::subscription_manager() {
    return *sub_manager_;
}

// ============================================================================
// Binary protocol (image frame encoding)
// ============================================================================

std::vector<uint8_t> encode_image_frame(
    const std::string& camera_name,
    const uint8_t* data, size_t size) {
  std::vector<uint8_t> frame;
  frame.reserve(2 + camera_name.size() + size);

  uint16_t name_len = static_cast<uint16_t>(camera_name.size());
  frame.push_back(static_cast<uint8_t>((name_len >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(name_len & 0xFF));

  frame.insert(frame.end(), camera_name.begin(), camera_name.end());
  frame.insert(frame.end(), data, data + size);

  return frame;
}

std::vector<uint8_t> encode_image_frame(
    const std::string& camera_name,
    const std::vector<uint8_t>& jpeg_data) {
  return encode_image_frame(camera_name, jpeg_data.data(), jpeg_data.size());
}

// ============================================================================
// SubscriptionManager
// ============================================================================

/// Detect publisher QoS for a topic and return a compatible subscription QoS.
/// BEST_EFFORT subscriber can receive from both BEST_EFFORT and RELIABLE publishers,
/// so it's the safe default when no publishers exist yet.
static rclcpp::QoS detect_qos_for_subscription(
    rclcpp_lifecycle::LifecycleNode* node, const std::string& topic) {
    auto publishers = node->get_publishers_info_by_topic(topic);

    bool any_best_effort = false;
    bool any_volatile = false;
    bool all_transient_local = !publishers.empty();

    for (const auto& info : publishers) {
        auto reliability = info.qos_profile().reliability();
        auto durability = info.qos_profile().durability();

        if (reliability == rclcpp::ReliabilityPolicy::BestEffort) {
            any_best_effort = true;
        }
        if (durability == rclcpp::DurabilityPolicy::Volatile) {
            any_volatile = true;
            all_transient_local = false;
        }
    }

    auto qos = rclcpp::QoS(10);

    if (publishers.empty() || any_best_effort) {
        qos.best_effort();
    } else {
        qos.reliable();
    }

    if (publishers.empty() || any_volatile || !all_transient_local) {
        qos.durability_volatile();
    } else {
        qos.transient_local();
    }

    RCLCPP_DEBUG(node->get_logger(),
                "QoS for '%s': %s / %s (found %zu publisher(s))",
                topic.c_str(),
                (publishers.empty() || any_best_effort) ? "BEST_EFFORT" : "RELIABLE",
                (publishers.empty() || any_volatile || !all_transient_local) ? "VOLATILE" : "TRANSIENT_LOCAL",
                publishers.size());

    return qos;
}

SubscriptionManager::SubscriptionManager(rclcpp_lifecycle::LifecycleNode* node)
    : node_(node) {}

void SubscriptionManager::subscribe(const std::string& topic, const std::string& type,
                                     uint64_t session_id, SenderFn sender) {
    std::unique_lock lock(mutex_);

    auto it = subscriptions_.find(topic);
    if (it != subscriptions_.end()) {
        // Topic already subscribed — just add this session's sender
        it->second.senders[session_id] = std::move(sender);
        RCLCPP_DEBUG(node_->get_logger(),
                     "Session %lu joined existing subscription on '%s'",
                     session_id, topic.c_str());
        return;
    }

    // First subscriber for this topic — create the ROS subscription
    TopicEntry entry;
    entry.senders[session_id] = std::move(sender);

    RCLCPP_DEBUG(node_->get_logger(),
                 "Creating subscription on '%s' [%s]", topic.c_str(), type.c_str());

    try {
        auto qos = detect_qos_for_subscription(node_, topic);
        auto sub = node_->create_generic_subscription(
            topic, type, qos,
            [this, topic, type](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
                nlohmann::json msg_json;
                try {
                    msg_json = RosMessageConverter::convert_ros_message_to_json(type, *serialized_msg);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(node_->get_logger(),
                                 "Failed to convert message on '%s': %s", topic.c_str(), e.what());
                    return;
                }

                nlohmann::json pub_message = {
                    {"op", "publish"},
                    {"topic", topic},
                    {"msg", msg_json}
                };

                // Serialize once, share across all sessions
                auto serialized = std::make_shared<nlohmann::json>(std::move(pub_message));

                // Snapshot senders under lock, then fan-out without holding it
                std::vector<std::pair<uint64_t, SenderFn>> senders_snapshot;
                {
                    std::shared_lock lock(mutex_);
                    auto it = subscriptions_.find(topic);
                    if (it != subscriptions_.end()) {
                        senders_snapshot.reserve(it->second.senders.size());
                        for (auto& [sid, fn] : it->second.senders) {
                            senders_snapshot.emplace_back(sid, fn);
                        }
                    }
                }

                for (auto& [sid, send_fn] : senders_snapshot) {
                    try {
                        send_fn(*serialized);
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(node_->get_logger(),
                                    "Failed to send to session %lu on '%s': %s",
                                    sid, topic.c_str(), e.what());
                    }
                }
            });

        entry.subscription = sub;
        subscriptions_[topic] = std::move(entry);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to create subscription for '%s': %s", topic.c_str(), e.what());
    }
}

void SubscriptionManager::unsubscribe(const std::string& topic, uint64_t session_id) {
    std::unique_lock lock(mutex_);

    auto it = subscriptions_.find(topic);
    if (it == subscriptions_.end()) {
        return;
    }

    it->second.senders.erase(session_id);
    RCLCPP_DEBUG(node_->get_logger(),
                 "Session %lu unsubscribed from '%s' (%zu remaining)",
                 session_id, topic.c_str(), it->second.senders.size());

    if (it->second.senders.empty()) {
        RCLCPP_DEBUG(node_->get_logger(),
                     "No subscribers left on '%s', destroying subscription", topic.c_str());
        subscriptions_.erase(it);
    }
}

void SubscriptionManager::unsubscribe_all(uint64_t session_id) {
    std::unique_lock lock(mutex_);

    for (auto it = subscriptions_.begin(); it != subscriptions_.end();) {
        it->second.senders.erase(session_id);
        if (it->second.senders.empty()) {
            RCLCPP_DEBUG(node_->get_logger(),
                         "No subscribers left on '%s', destroying subscription",
                         it->first.c_str());
            it = subscriptions_.erase(it);
        } else {
            ++it;
        }
    }
}

}  // namespace cpp_rosbridge_core
