#include <memory>
#include <string>
#include <thread>
#include <functional>

#include <nlohmann/json.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <boost/asio/thread_pool.hpp>

#include "cpp_zmq_server/zmq_server.hpp"
#include "cpp_rosbridge_core/protocol.hpp"

using json = nlohmann::json;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// ROS2 lifecycle node wrapping ZmqServer + Protocol.
///
/// Architecture note: Unlike the WebSocket server which calls
/// Protocol::handle_message() directly on I/O threads, the ZMQ server
/// dispatches handle_message() via post_work() to the thread pool.
/// This is an intentional deviation: the ZMQ poll thread must remain
/// unblocked for recv/send operations, and ZMQ sockets are not thread-safe
/// so we cannot call handle_message (which may trigger subscription callbacks
/// that enqueue replies) on arbitrary threads without the post_work dispatch.
class ZmqBridgeNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit ZmqBridgeNode(const rclcpp::NodeOptions& options)
        : rclcpp_lifecycle::LifecycleNode("zmq_bridge_node", options) {}

    CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(get_logger(), "Configuring ZmqBridgeNode");

        this->declare_parameter<int>("port", 5555);
        this->declare_parameter<std::string>("address", "127.0.0.1");
        this->declare_parameter<int>("work_pool_threads", 4);
        this->declare_parameter<int>("heartbeat_timeout_s", 30);

        port_ = static_cast<uint16_t>(this->get_parameter("port").as_int());
        address_ = this->get_parameter("address").as_string();
        int work_pool_size = this->get_parameter("work_pool_threads").as_int();
        heartbeat_timeout_s_ = this->get_parameter("heartbeat_timeout_s").as_int();

        work_pool_ = std::make_unique<boost::asio::thread_pool>(work_pool_size);

        zmq_server_ = std::make_unique<cpp_zmq_server::ZmqServer>();
        protocol_ = std::make_unique<cpp_rosbridge_core::Protocol>(this);
        protocol_->set_work_pool(work_pool_.get());

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(get_logger(), "Activating ZmqBridgeNode on %s:%u",
                    address_.c_str(), port_);

        // Wire disconnect callback: clean up subscriptions when client times out
        zmq_server_->set_on_disconnect([this](uint64_t session_id) {
            RCLCPP_DEBUG(get_logger(), "Session %lu disconnected, cleaning up", session_id);
            protocol_->subscription_manager().unsubscribe_all(session_id);
        });

        zmq_thread_ = std::thread([this]() {
            zmq_server_->run(
                address_, port_,
                [this](uint64_t session_id,
                       const std::string& message,
                       std::function<void(const std::string&)> reply_fn) {
                    handle_zmq_message(session_id, message, std::move(reply_fn));
                },
                std::chrono::seconds{heartbeat_timeout_s_}
            );
        });

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(get_logger(), "Deactivating ZmqBridgeNode");
        shutdown_resources();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(get_logger(), "Cleaning up ZmqBridgeNode");
        zmq_server_.reset();
        protocol_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(get_logger(), "Shutting down ZmqBridgeNode");
        shutdown_resources();
        return CallbackReturn::SUCCESS;
    }

private:
    void handle_zmq_message(uint64_t session_id,
                            const std::string& message,
                            std::function<void(const std::string&)> reply_fn) {
        RCLCPP_DEBUG(get_logger(), "Received ZMQ message (%zu bytes) from session %lu",
                     message.size(), session_id);

        // Dispatch to work pool to avoid blocking the ZMQ poll thread.
        // The reply_fn captures a weak_ptr<ZmqSession> internally, so replies
        // are safely enqueued even from work pool threads.
        // binary_sender is nullptr: CBOR/binary sub-protocol not supported in ZMQ bridge.
        auto captured_reply = std::move(reply_fn);
        auto captured_msg = message;
        protocol_->post_work(
            [this, captured_msg = std::move(captured_msg),
             captured_reply = std::move(captured_reply),
             session_id]() {
                // Build text sender: Protocol sends JSON string → we forward to reply_fn
                auto text_sender = [captured_reply](const std::string& reply) {
                    captured_reply(reply);
                };

                protocol_->handle_message(
                    captured_msg,
                    text_sender,
                    session_id,
                    nullptr  // No binary sender — CBOR not supported
                );
            }
        );
    }

    void shutdown_resources() {
        if (zmq_server_) {
            zmq_server_->stop();
        }
        if (work_pool_) {
            work_pool_->join();
        }
        if (zmq_thread_.joinable()) {
            zmq_thread_.join();
        }
    }

    uint16_t port_;
    std::string address_;
    int heartbeat_timeout_s_;
    std::unique_ptr<cpp_zmq_server::ZmqServer> zmq_server_;
    std::unique_ptr<cpp_rosbridge_core::Protocol> protocol_;
    std::thread zmq_thread_;
    std::unique_ptr<boost::asio::thread_pool> work_pool_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<ZmqBridgeNode>(rclcpp::NodeOptions());

    // Auto-transition: unconfigured → configured → active
    node->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
