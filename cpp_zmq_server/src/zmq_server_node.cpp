#include <memory>
#include <string>
#include <thread>
#include <functional>

#include <nlohmann/json.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <boost/asio/thread_pool.hpp>
#include <boost/asio/post.hpp>

#include "cpp_zmq_server/zmq_server.hpp"
#include "cpp_zmq_server/habilis_bridge.hpp"
#include "cpp_zmq_server/habilis_dispatcher.hpp"
#include "cpp_rosbridge_core/protocol.hpp"

using json = nlohmann::json;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// ROS2 lifecycle node wrapping ZmqServer (rosbridge v2) + HabilisBridge (habilis protocol).
///
/// Both protocols share a single zmq::context_t and boost::asio::thread_pool.
/// Each has its own poll thread for different socket patterns and poll intervals.
///
/// Shutdown order (prevents race conditions):
///   1. Stop both bridges (sets running_ = false, unblocks polls)
///   2. Join poll threads (loops have exited)
///   3. Join work pool (drain in-flight tasks — safe because bridges still alive,
///      send() no-ops via running_ flag)
///   4. Destroy bridges (sockets closed)
class ZmqBridgeNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit ZmqBridgeNode(const rclcpp::NodeOptions& options)
        : rclcpp_lifecycle::LifecycleNode("zmq_bridge_node", options)
        , zmq_ctx_(1) {}

    CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(get_logger(), "Configuring ZmqBridgeNode");

        // rosbridge v2 parameters
        this->declare_parameter<int>("port", 5555);
        this->declare_parameter<std::string>("address", "127.0.0.1");
        this->declare_parameter<int>("work_pool_threads", 4);
        this->declare_parameter<int>("heartbeat_timeout_s", 30);

        // habilis parameters
        this->declare_parameter<bool>("habilis_enabled", true);
        this->declare_parameter<int>("habilis_pub_port", 4270);
        this->declare_parameter<int>("habilis_sub_port", 4271);
        this->declare_parameter<std::string>("habilis_sub_address", "127.0.0.1");

        port_ = static_cast<uint16_t>(this->get_parameter("port").as_int());
        address_ = this->get_parameter("address").as_string();
        int work_pool_size = this->get_parameter("work_pool_threads").as_int();
        heartbeat_timeout_s_ = this->get_parameter("heartbeat_timeout_s").as_int();
        habilis_enabled_ = this->get_parameter("habilis_enabled").as_bool();

        work_pool_ = std::make_unique<boost::asio::thread_pool>(work_pool_size);

        // rosbridge v2 (ROUTER)
        zmq_server_ = std::make_unique<cpp_zmq_server::ZmqServer>(zmq_ctx_);
        protocol_ = std::make_unique<cpp_rosbridge_core::Protocol>(this);
        protocol_->set_work_pool(work_pool_.get());

        // habilis (PUB/SUB)
        if (habilis_enabled_) {
            habilis_bridge_ = std::make_shared<cpp_zmq_server::HabilisBridge>(zmq_ctx_);
            habilis_dispatcher_ = std::make_unique<cpp_zmq_server::HabilisDispatcher>(
                this->shared_from_this(), habilis_bridge_);
            habilis_dispatcher_->configure();
            RCLCPP_INFO(get_logger(), "Habilis protocol enabled");
        } else {
            RCLCPP_INFO(get_logger(), "Habilis protocol disabled");
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(get_logger(), "Activating ZmqBridgeNode on %s:%u",
                    address_.c_str(), port_);

        // Wire disconnect callback for rosbridge v2
        zmq_server_->set_on_disconnect([this](uint64_t session_id) {
            RCLCPP_DEBUG(get_logger(), "Session %lu disconnected, cleaning up", session_id);
            protocol_->subscription_manager().unsubscribe_all(session_id);
        });

        // Start rosbridge v2 ROUTER thread
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

        // Start habilis PUB/SUB thread
        if (habilis_enabled_ && habilis_bridge_) {
            uint16_t hab_pub_port = static_cast<uint16_t>(
                this->get_parameter("habilis_pub_port").as_int());
            uint16_t hab_sub_port = static_cast<uint16_t>(
                this->get_parameter("habilis_sub_port").as_int());
            std::string hab_sub_addr =
                this->get_parameter("habilis_sub_address").as_string();

            auto bridge_ptr = habilis_bridge_;
            auto dispatcher_ptr = habilis_dispatcher_.get();
            auto* pool = work_pool_.get();

            habilis_thread_ = std::thread([bridge_ptr, dispatcher_ptr, pool,
                                           hab_sub_addr, hab_sub_port, hab_pub_port]() {
                bridge_ptr->run(
                    hab_sub_addr, hab_sub_port, hab_pub_port,
                    [dispatcher_ptr, pool](cpp_zmq_server::HabilisMsgType type,
                                           msgpack::object_handle body) {
                        // Dispatch to work pool to avoid blocking the habilis poll thread
                        auto shared_body = std::make_shared<msgpack::object_handle>(
                            std::move(body));
                        boost::asio::post(*pool,
                            [dispatcher_ptr, type, shared_body]() {
                                dispatcher_ptr->dispatch(type, std::move(*shared_body));
                            });
                    });
            });

            habilis_dispatcher_->activate();
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(get_logger(), "Deactivating ZmqBridgeNode");
        shutdown_resources();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(get_logger(), "Cleaning up ZmqBridgeNode");
        if (habilis_dispatcher_) {
            habilis_dispatcher_.reset();
        }
        habilis_bridge_.reset();
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

        auto captured_reply = std::move(reply_fn);
        auto captured_msg = message;
        protocol_->post_work(
            [this, captured_msg = std::move(captured_msg),
             captured_reply = std::move(captured_reply),
             session_id]() {
                auto text_sender = [captured_reply](const std::string& reply) {
                    captured_reply(reply);
                };

                protocol_->handle_message(
                    captured_msg,
                    text_sender,
                    session_id,
                    nullptr  // No binary sender
                );
            }
        );
    }

    /// Shutdown order (prevents race conditions):
    ///   1. Stop bridges (sets running_ = false, unblocks polls)
    ///   2. Shutdown ZMQ context (unblocks any blocking socket ops)
    ///   3. Join poll threads
    ///   4. Join work pool (drain in-flight tasks)
    ///   5. Deactivate dispatcher (cleanup ROS subs/pubs)
    void shutdown_resources() {
        // 1. Stop bridges
        if (zmq_server_) {
            zmq_server_->stop();
        }
        if (habilis_bridge_) {
            habilis_bridge_->stop();
        }

        // 2. Shutdown shared context to unblock any zmq_poll/recv/send
        zmq_ctx_.shutdown();

        // 3. Join poll threads
        if (zmq_thread_.joinable()) {
            zmq_thread_.join();
        }
        if (habilis_thread_.joinable()) {
            habilis_thread_.join();
        }

        // 4. Join work pool (in-flight tasks call send() which no-ops via running_ flag)
        if (work_pool_) {
            work_pool_->join();
        }

        // 5. Deactivate dispatcher (cleanup ROS resources)
        if (habilis_dispatcher_) {
            habilis_dispatcher_->deactivate();
        }
    }

    // Shared ZMQ context for both protocols
    zmq::context_t zmq_ctx_;

    // rosbridge v2 (ROUTER)
    uint16_t port_;
    std::string address_;
    int heartbeat_timeout_s_;
    std::unique_ptr<cpp_zmq_server::ZmqServer> zmq_server_;
    std::unique_ptr<cpp_rosbridge_core::Protocol> protocol_;
    std::thread zmq_thread_;

    // habilis (PUB/SUB)
    bool habilis_enabled_{true};
    std::shared_ptr<cpp_zmq_server::HabilisBridge> habilis_bridge_;
    std::unique_ptr<cpp_zmq_server::HabilisDispatcher> habilis_dispatcher_;
    std::thread habilis_thread_;

    // Shared work pool
    std::unique_ptr<boost::asio::thread_pool> work_pool_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<ZmqBridgeNode>(rclcpp::NodeOptions());

    // Auto-transition: unconfigured -> configured -> active
    node->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
