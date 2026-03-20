#include "cpp_zmq_server/habilis_bridge.hpp"

#include <algorithm>

namespace cpp_zmq_server {

HabilisBridge::HabilisBridge(zmq::context_t& ctx)
    : ctx_(ctx) {}

HabilisBridge::~HabilisBridge() {
    stop();
}

void HabilisBridge::run(const std::string& sub_address, uint16_t sub_port,
                        uint16_t pub_port, MessageCallback on_message) {
    on_message_ = std::move(on_message);

    std::string sub_endpoint = "tcp://" + sub_address + ":" + std::to_string(sub_port);
    std::string pub_endpoint = "tcp://*:" + std::to_string(pub_port);

    running_.store(true);
    poll_loop(sub_endpoint, pub_endpoint);
}

void HabilisBridge::stop() {
    running_.store(false);
}

bool HabilisBridge::send(HabilisMsgType type, const msgpack::sbuffer& body) {
    if (!running_.load()) {
        return false;
    }

    auto frame = serialize_habilis_frame(type, body);
    return send_raw(frame);
}

bool HabilisBridge::send_raw(const std::vector<uint8_t>& frame) {
    if (!running_.load()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(pub_mutex_);

    // Backpressure: drop oldest if queue full
    if (pub_queue_.size() >= kMaxPubQueueSize) {
        pub_queue_.pop_front();
        RCLCPP_DEBUG(rclcpp::get_logger("habilis_bridge"),
                     "PUB queue full, dropping oldest message");
    }

    pub_queue_.push_back(frame);
    return true;
}

void HabilisBridge::poll_loop(const std::string& sub_endpoint,
                              const std::string& pub_endpoint) {
    zmq::socket_t sub_socket(ctx_, zmq::socket_type::sub);
    zmq::socket_t pub_socket(ctx_, zmq::socket_type::pub);

    // SUB socket: connect to AI manager's PUB
    sub_socket.set(zmq::sockopt::subscribe, "");  // Subscribe to all
    sub_socket.set(zmq::sockopt::rcvhwm, 100);
    sub_socket.set(zmq::sockopt::linger, 0);
    sub_socket.connect(sub_endpoint);

    // PUB socket: bind for AI manager's SUB to connect
    pub_socket.set(zmq::sockopt::sndhwm, 100);
    pub_socket.set(zmq::sockopt::linger, 0);
    pub_socket.bind(pub_endpoint);

    RCLCPP_INFO(rclcpp::get_logger("habilis_bridge"),
                "HabilisBridge started: SUB connected to %s, PUB bound at %s",
                sub_endpoint.c_str(), pub_endpoint.c_str());

    zmq::pollitem_t items[] = {
        {static_cast<void*>(sub_socket), 0, ZMQ_POLLIN, 0},
    };

    while (running_.load()) {
        try {
            zmq::poll(items, 1, std::chrono::milliseconds{kPollTimeoutMs});
        } catch (const zmq::error_t& e) {
            if (e.num() == ETERM || !running_.load()) {
                break;  // Context terminated or stop() called
            }
            RCLCPP_WARN(rclcpp::get_logger("habilis_bridge"),
                        "Poll error: %s", e.what());
            continue;
        }

        // Handle incoming messages from AI manager
        if (items[0].revents & ZMQ_POLLIN) {
            handle_incoming(sub_socket);
        }

        // Drain PUB write queue
        drain_pub_queue(pub_socket);
    }

    // Drain remaining PUB messages before closing
    drain_pub_queue(pub_socket);

    sub_socket.close();
    pub_socket.close();

    RCLCPP_INFO(rclcpp::get_logger("habilis_bridge"),
                "HabilisBridge poll loop stopped");
}

void HabilisBridge::handle_incoming(zmq::socket_t& sub_socket) {
    zmq::message_t msg;

    while (true) {
        auto result = sub_socket.recv(msg, zmq::recv_flags::dontwait);
        if (!result.has_value()) {
            break;  // No more messages
        }

        auto* data = static_cast<const uint8_t*>(msg.data());
        size_t len = msg.size();

        HabilisMessage parsed;
        if (!parse_habilis_frame(data, len, parsed)) {
            RCLCPP_WARN(rclcpp::get_logger("habilis_bridge"),
                        "Malformed habilis frame (%zu bytes), dropping", len);
            continue;
        }

        RCLCPP_DEBUG(rclcpp::get_logger("habilis_bridge"),
                     "Received %s (%zu bytes)",
                     habilis_msg_type_name(parsed.type), len);

        if (on_message_) {
            on_message_(parsed.type, std::move(parsed.body));
        }
    }
}

void HabilisBridge::drain_pub_queue(zmq::socket_t& pub_socket) {
    // Lock-and-swap to minimize lock scope
    std::deque<std::vector<uint8_t>> local_queue;
    {
        std::lock_guard<std::mutex> lock(pub_mutex_);
        if (pub_queue_.empty()) return;
        local_queue.swap(pub_queue_);
    }

    size_t sent = 0;
    for (auto& frame : local_queue) {
        if (sent >= kMaxDrainPerIteration) {
            // Put remaining back into queue
            std::lock_guard<std::mutex> lock(pub_mutex_);
            while (!local_queue.empty()) {
                // Respect queue size limit
                if (pub_queue_.size() >= kMaxPubQueueSize) break;
                pub_queue_.push_front(std::move(local_queue.back()));
                local_queue.pop_back();
            }
            break;
        }

        try {
            zmq::message_t zmq_msg(frame.data(), frame.size());
            pub_socket.send(zmq_msg, zmq::send_flags::dontwait);
            ++sent;
        } catch (const zmq::error_t& e) {
            if (e.num() == ETERM) break;
            RCLCPP_DEBUG(rclcpp::get_logger("habilis_bridge"),
                         "PUB send failed: %s", e.what());
        }
    }
}

}  // namespace cpp_zmq_server
