#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <msgpack.hpp>

#include "cpp_zmq_server/habilis_types.hpp"
#include "cpp_zmq_server/habilis_bridge.hpp"

namespace cpp_zmq_server {

// Pending request for REQUEST/RESPONSE correlation
struct PendingRequest {
    std::string response_name;
    std::chrono::steady_clock::time_point created_at;
};

// Dispatches habilis protocol messages to/from ROS 2 topics.
// Thread safety: dispatch() is called from the work pool thread.
//                ROS callbacks run on the node's executor thread.
//                send_to_ai() is thread-safe (delegates to HabilisBridge::send).
class HabilisDispatcher {
public:
    static constexpr size_t kMaxPendingRequests = 64;
    static constexpr std::chrono::seconds kPendingRequestTimeout{30};

    HabilisDispatcher(
        rclcpp_lifecycle::LifecycleNode::SharedPtr node,
        std::shared_ptr<HabilisBridge> bridge);

    ~HabilisDispatcher() = default;

    // Called from work pool thread to handle an incoming message from AI manager.
    void dispatch(HabilisMsgType type, msgpack::object_handle body);

    // Configure ROS publishers/subscribers based on node parameters.
    // Must be called during on_configure().
    void configure();

    // Activate observation timer. Called during on_activate().
    void activate();

    // Deactivate and clean up. Called during on_deactivate().
    void deactivate();

    // Start/stop observation assembly (controlled by START/FINISH_INFERENCE)
    void start_observation_stream();
    void stop_observation_stream();

private:
    // Message handlers (AI -> ROS)
    void handle_heartbeat(const msgpack::object& body);
    void handle_tick(const msgpack::object& body);
    void handle_start_inference(const msgpack::object& body);
    void handle_finish_inference(const msgpack::object& body);
    void handle_start_training(const msgpack::object& body);
    void handle_finish_training(const msgpack::object& body);
    void handle_start_timer(const msgpack::object& body);
    void handle_finish_timer(const msgpack::object& body);
    void handle_robot_type(const msgpack::object& body);
    void handle_error(const msgpack::object& body);
    void handle_action(const msgpack::object& body);
    void handle_request(const msgpack::object& body);
    void handle_response(const msgpack::object& body);
    void handle_training_status(const msgpack::object& body);
    void handle_inference_status(const msgpack::object& body);

    // OBSERVATION assembly (ROS -> AI)
    void observation_timer_callback();
    void assemble_and_send_observation();

    // Send a msgpack payload to AI manager
    void send_to_ai(HabilisMsgType type, const msgpack::sbuffer& buf);

    // REQUEST/RESPONSE correlation
    void evict_expired_requests();

    // Node and bridge references
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::shared_ptr<HabilisBridge> bridge_;

    // Inference state
    std::atomic<bool> inference_active_{false};

    // --- ROS Publishers (AI -> ROS) ---
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr tick_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr inference_start_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr inference_finish_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr training_start_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr training_finish_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr timer_start_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr timer_finish_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr robot_type_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr error_pub_;
    rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr action_pub_;
    // request_pub_ removed — REQUEST forwarded to AI Manager via ZMQ

    // --- ROS Subscribers (ROS -> AI for OBSERVATION) ---
    std::vector<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr> camera_subs_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> joint_subs_;

    // OBSERVATION cache (mutex-guarded per-topic latest values)
    struct CameraCache {
        std::mutex mutex;
        sensor_msgs::msg::CompressedImage::SharedPtr latest;
    };
    struct JointCache {
        std::mutex mutex;
        sensor_msgs::msg::JointState::SharedPtr latest;
    };
    std::vector<std::shared_ptr<CameraCache>> camera_caches_;
    std::vector<std::shared_ptr<JointCache>> joint_caches_;

    // OBSERVATION parameters
    std::vector<std::string> camera_topics_;
    std::vector<std::string> camera_names_;
    std::vector<std::string> joint_state_topics_;
    std::vector<std::string> joint_order_;
    std::vector<std::string> action_order_;
    std::string action_topic_;
    double observation_fps_{30.0};

    // OBSERVATION timer
    rclcpp::TimerBase::SharedPtr observation_timer_;

    // REQUEST/RESPONSE correlation
    std::unordered_map<std::string, PendingRequest> pending_requests_;
    std::mutex pending_mutex_;
};

}  // namespace cpp_zmq_server
