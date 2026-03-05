#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <boost/asio/thread_pool.hpp>

namespace cpp_rosbridge_core {

// ── Binary sub-protocol ─────────────────────────────────────────────────────

/// Encode a single camera image into the binary frame sub-protocol:
///   [2 bytes: name_len (uint16 BE)] [name_len bytes: camera_name] [JPEG data]
std::vector<uint8_t> encode_image_frame(
    const std::string& camera_name,
    const std::vector<uint8_t>& jpeg_data);

/// Overload accepting raw pointer + size (avoids copying into a vector)
std::vector<uint8_t> encode_image_frame(
    const std::string& camera_name,
    const uint8_t* data, size_t size);

// ── Capability base class ───────────────────────────────────────────────────

class Protocol;  // Forward declaration

/// Interface for all rosbridge capabilities (op handlers)
class Capability {
public:
    explicit Capability(Protocol* protocol) : protocol_(protocol) {}
    virtual ~Capability() = default;

    /// Each capability must implement this method to handle incoming messages.
    /// It can send one or more responses back via the sender function.
    virtual void handle_message(const nlohmann::json& message,
        std::function<void(const nlohmann::json&)> sender,
        bool cbor_mode = false) = 0;

protected:
    Protocol* protocol_;
};

// ── Action bridge framework ─────────────────────────────────────────────────

namespace action {

/// Callback interface for action bridge to communicate back to the protocol layer
struct ActionCallbacks {
    std::function<void(const std::string&)> send_text;
    std::function<void(const std::vector<uint8_t>&)> send_binary;
    uint64_t session_id = 0;
};

/// Base class for typed action bridges
class ActionBridgeBase {
public:
    virtual ~ActionBridgeBase() = default;

    virtual void execute(const nlohmann::json& goal_data,
                         const ActionCallbacks& callbacks,
                         bool cbor_mode = false) = 0;

    virtual void cancel(uint64_t session_id) = 0;
};

/// Factory function type for creating bridges
using BridgeFactory = std::function<std::unique_ptr<ActionBridgeBase>(
    rclcpp_lifecycle::LifecycleNode* node, const std::string& topic,
    int timeout_s)>;

/// Thread-safe registry mapping "package/action/Type" -> typed bridge factory
class ActionBridgeRegistry {
public:
    ActionBridgeRegistry() = default;

    void register_bridge(const std::string& action_name, BridgeFactory factory);

    ActionBridgeBase* get_bridge(rclcpp_lifecycle::LifecycleNode* node,
                                 const std::string& action_name,
                                 const std::string& topic, int timeout_s);

    bool has_bridge(const std::string& action_name) const;

    std::vector<std::string> action_names() const;

private:
    mutable std::shared_mutex mutex_;
    std::unordered_map<std::string, BridgeFactory> factories_;
    std::unordered_map<std::string, std::unique_ptr<ActionBridgeBase>> bridges_;
};

}  // namespace action

/// Register all action bridges (auto-discovered at build time)
void register_generated_actions(action::ActionBridgeRegistry& registry);

// ── Subscription manager ────────────────────────────────────────────────────

class SubscriptionManager {
public:
    using SenderFn = std::function<void(const nlohmann::json&)>;

    explicit SubscriptionManager(rclcpp_lifecycle::LifecycleNode* node);

    void subscribe(const std::string& topic, const std::string& type,
                   uint64_t session_id, SenderFn sender);

    void unsubscribe(const std::string& topic, uint64_t session_id);

    void unsubscribe_all(uint64_t session_id);

private:
    struct TopicEntry {
        rclcpp::SubscriptionBase::SharedPtr subscription;
        std::unordered_map<uint64_t, SenderFn> senders;  // session_id -> sender
    };

    rclcpp_lifecycle::LifecycleNode* node_;
    std::unordered_map<std::string, TopicEntry> subscriptions_;
    mutable std::shared_mutex mutex_;
};

// ── Protocol router ─────────────────────────────────────────────────────────

class Protocol {
public:
    using BinarySenderFn = std::function<void(const std::vector<uint8_t>&)>;

    explicit Protocol(rclcpp_lifecycle::LifecycleNode* node);
    ~Protocol();

    /// Handles an incoming JSON text message, routes it to the correct capability
    void handle_message(const std::string& message_string, std::function<void(const std::string&)> sender);

    /// Overload with per-call session context (session_id + binary_sender)
    void handle_message(const std::string& message_string,
                        std::function<void(const std::string&)> sender,
                        uint64_t session_id,
                        BinarySenderFn binary_sender);

    /// Handles an incoming CBOR binary message, routes it to the correct capability
    void handle_binary_message(const uint8_t* data, size_t size,
                               std::function<void(const std::vector<uint8_t>&)> binary_sender);

    /// Thread-local session context accessors (valid only during handle_message call)
    static uint64_t current_session_id();
    static BinarySenderFn current_binary_sender();

    // Publisher management
    void add_publisher(const std::string& topic, const std::string& type, rclcpp::PublisherBase::SharedPtr pub);
    rclcpp::PublisherBase::SharedPtr get_publisher(const std::string& topic_name);
    std::string get_publisher_type(const std::string& topic);

    // Action bridge registry
    action::ActionBridgeRegistry& action_registry();

    // Thread pool for offloading work
    void set_work_pool(boost::asio::thread_pool* pool);
    void post_work(std::function<void()> fn);

    rclcpp_lifecycle::LifecycleNode* get_node();

    // Subscription manager for multi-client subscribe/unsubscribe
    SubscriptionManager& subscription_manager();

    void register_capability(const std::string& op_code, std::unique_ptr<Capability> capability);

private:

    rclcpp_lifecycle::LifecycleNode* node_;
    std::unordered_map<std::string, std::unique_ptr<Capability>> capabilities_;
    std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr> publishers_;
    std::unordered_map<std::string, std::string> publisher_types_;
    std::shared_mutex publishers_mutex_;
    action::ActionBridgeRegistry action_registry_;
    std::unique_ptr<SubscriptionManager> sub_manager_;
    boost::asio::thread_pool* work_pool_ = nullptr;

    static thread_local uint64_t tl_session_id_;
    static thread_local BinarySenderFn tl_binary_sender_;
};

}  // namespace cpp_rosbridge_core
