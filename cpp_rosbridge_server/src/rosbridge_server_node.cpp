#include <nlohmann/json.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <boost/asio/thread_pool.hpp>
#include <memory>
#include <thread>
#include <functional>
#include <vector>
#include "cpp_rosbridge_server/websocket_server.hpp"
#include "cpp_rosbridge_core/protocol.hpp"

using json = nlohmann::json;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RosbridgeServerNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit RosbridgeServerNode(const rclcpp::NodeOptions& options)
        : rclcpp_lifecycle::LifecycleNode("rosbridge_server_node", options) {}

    CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(get_logger(), "Configuring RosbridgeServerNode");

        this->declare_parameter<int>("port", 9091);
        this->declare_parameter<std::string>("address", "127.0.0.1");
        this->declare_parameter<int>("num_threads", 4);
        this->declare_parameter<int>("work_pool_threads", 4);

        port_ = static_cast<uint16_t>(this->get_parameter("port").as_int());
        address_ = this->get_parameter("address").as_string();
        num_threads_ = static_cast<int>(this->get_parameter("num_threads").as_int());
        int work_pool_size = this->get_parameter("work_pool_threads").as_int();

        work_pool_ = std::make_unique<boost::asio::thread_pool>(work_pool_size);

        ws_server_ = std::make_unique<WebsocketServer>();
        protocol_ = std::make_unique<cpp_rosbridge_core::Protocol>(this);
        protocol_->set_work_pool(work_pool_.get());

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(get_logger(), "Activating RosbridgeServerNode on %s:%u (threads: %d)",
                    address_.c_str(), port_, num_threads_);

        ws_thread_ = std::thread([this]() {
            ws_server_->run(address_, port_, num_threads_,
                [this](std::shared_ptr<Session> session,
                       const std::string& message,
                       bool is_binary) {
                    handle_ws_message(session, message, is_binary);
                }
            );
        });

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(get_logger(), "Deactivating RosbridgeServerNode");
        shutdown_resources();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(get_logger(), "Cleaning up RosbridgeServerNode");
        ws_server_.reset();
        protocol_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(get_logger(), "Shutting down RosbridgeServerNode");
        shutdown_resources();
        return CallbackReturn::SUCCESS;
    }

private:
    void handle_ws_message(std::shared_ptr<Session> session,
                           const std::string& message,
                           bool is_binary) {
        // Wire disconnect callback on first message from this session
        if (!session->id()) { return; }
        session->set_on_disconnect([this](uint64_t session_id) {
            RCLCPP_DEBUG(get_logger(), "Session %lu disconnected, cleaning up", session_id);
            protocol_->subscription_manager().unsubscribe_all(session_id);
        });

        if (is_binary) {
            handle_binary(session, message);
        } else {
            handle_text(session, message);
        }
    }

    void handle_binary(std::shared_ptr<Session> session, const std::string& message) {
        const auto* data = reinterpret_cast<const uint8_t*>(message.data());
        size_t size = message.size();
        protocol_->handle_binary_message(
            data, size,
            [session](const std::vector<uint8_t>& reply) {
                session->send_binary(reply);
            });
    }

    void handle_text(std::shared_ptr<Session> session, const std::string& message) {
        RCLCPP_DEBUG(get_logger(), "Received text message (%zu bytes)", message.size());
        protocol_->handle_message(
            message,
            [session](const std::string& reply) {
                session->send_text(reply);
            },
            session->id(),
            [session](const std::vector<uint8_t>& data) {
                session->send_binary(data);
            });
    }

    void shutdown_resources() {
        if (ws_server_) {
            ws_server_->stop();
        }
        if (work_pool_) {
            work_pool_->join();
        }
        if (ws_thread_.joinable()) {
            ws_thread_.join();
        }
    }

    uint16_t port_;
    std::string address_;
    int num_threads_;
    std::unique_ptr<WebsocketServer> ws_server_;
    std::unique_ptr<cpp_rosbridge_core::Protocol> protocol_;
    std::thread ws_thread_;
    std::unique_ptr<boost::asio::thread_pool> work_pool_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<RosbridgeServerNode>(rclcpp::NodeOptions());

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
