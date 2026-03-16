#include "cpp_zmq_server/zmq_server.hpp"

#include <algorithm>
#include <rclcpp/logging.hpp>

namespace cpp_zmq_server {

// ── ZmqSession ──────────────────────────────────────────────────────────────

ZmqSession::ZmqSession(std::string identity_bytes, uint64_t session_id)
    : identity_bytes_(std::move(identity_bytes)),
      session_id_(session_id),
      last_active_(std::chrono::steady_clock::now()) {}

bool ZmqSession::enqueue_reply(std::string message) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (write_queue_.size() >= kMaxWriteQueueSize) {
        // Backpressure: drop oldest message for slow clients
        write_queue_.pop_front();
        RCLCPP_DEBUG(rclcpp::get_logger("zmq_server"),
                     "Session %lu: write queue full, dropped oldest message", session_id_);
    }
    write_queue_.push_back(std::move(message));
    return true;
}

size_t ZmqSession::drain_replies(zmq::socket_t& router) {
    // Lock-and-swap pattern: minimize lock scope, no I/O under lock
    std::deque<std::string> local_queue;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        local_queue.swap(write_queue_);
    }

    // Bounded drain: max kMaxDrainPerIteration messages per poll iteration
    size_t sent = 0;
    for (auto it = local_queue.begin();
         it != local_queue.end() && sent < kMaxDrainPerIteration;
         ++it, ++sent) {
        try {
            // ROUTER framing: [identity | empty | payload]
            zmq::message_t id_frame(identity_bytes_.data(), identity_bytes_.size());
            zmq::message_t empty_frame(0);
            zmq::message_t payload_frame(it->data(), it->size());

            router.send(id_frame, zmq::send_flags::sndmore);
            router.send(empty_frame, zmq::send_flags::sndmore);
            router.send(payload_frame, zmq::send_flags::none);
        } catch (const zmq::error_t& e) {
            RCLCPP_WARN(rclcpp::get_logger("zmq_server"),
                        "Session %lu: send failed: %s", session_id_, e.what());
            break;
        }
    }

    // If we hit the drain limit, put remaining messages back
    if (sent < local_queue.size()) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        // Prepend unsent messages back to front of queue
        for (auto rit = local_queue.rbegin();
             rit != local_queue.rend() - static_cast<long>(sent);
             ++rit) {
            write_queue_.push_front(std::move(*rit));
        }
    }

    return sent;
}

void ZmqSession::touch() {
    last_active_ = std::chrono::steady_clock::now();
}

bool ZmqSession::is_expired(std::chrono::seconds timeout) const {
    auto elapsed = std::chrono::steady_clock::now() - last_active_;
    return elapsed > timeout;
}

// ── ZmqServer ───────────────────────────────────────────────────────────────

std::atomic<uint64_t> ZmqServer::next_session_id_{1};

ZmqServer::~ZmqServer() {
    stop();
}

void ZmqServer::stop() {
    running_ = false;
    // Context shutdown will unblock zmq_poll
    ctx_.shutdown();
}

void ZmqServer::run(const std::string& address, uint16_t port,
                     MessageCallback on_message,
                     std::chrono::seconds heartbeat_timeout) {
    on_message_ = std::move(on_message);
    heartbeat_timeout_ = heartbeat_timeout;
    running_ = true;

    std::string endpoint = "tcp://" + address + ":" + std::to_string(port);
    poll_loop(endpoint);
}

void ZmqServer::poll_loop(const std::string& endpoint) {
    zmq::socket_t router(ctx_, zmq::socket_type::router);

    // Safety: limit incoming message size to prevent OOM
    router.set(zmq::sockopt::maxmsgsize, kMaxMessageSize);
    // Clean shutdown: don't block on pending messages
    router.set(zmq::sockopt::linger, 0);
    // Enable ROUTER mandatory mode: fail on send to unknown identity
    router.set(zmq::sockopt::router_mandatory, 1);

    try {
        router.bind(endpoint);
    } catch (const zmq::error_t& e) {
        RCLCPP_ERROR(rclcpp::get_logger("zmq_server"),
                     "Failed to bind on %s: %s", endpoint.c_str(), e.what());
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("zmq_server"),
                "ZMQ ROUTER server listening on %s", endpoint.c_str());

    zmq::pollitem_t items[] = {{router.handle(), 0, ZMQ_POLLIN, 0}};
    constexpr int kPollTimeoutMs = 100;

    while (running_) {
        try {
            zmq::poll(items, 1, std::chrono::milliseconds{kPollTimeoutMs});
        } catch (const zmq::error_t& e) {
            if (e.num() == ETERM) {
                break;  // Context terminated, clean exit
            }
            RCLCPP_WARN(rclcpp::get_logger("zmq_server"), "poll error: %s", e.what());
            continue;
        }

        // Receive all pending messages
        if (items[0].revents & ZMQ_POLLIN) {
            handle_incoming(router);
        }

        // Drain all session write queues back through ROUTER
        drain_all_sessions(router);

        // Periodic reap of expired sessions
        if (++reap_counter_ >= kReapIntervalIterations) {
            reap_counter_ = 0;
            reap_expired_sessions();
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("zmq_server"), "ZMQ server stopped.");
}

void ZmqServer::handle_incoming(zmq::socket_t& router) {
    // Drain all available messages in a non-blocking loop
    while (true) {
        zmq::message_t identity_frame;
        auto id_result = router.recv(identity_frame, zmq::recv_flags::dontwait);
        if (!id_result.has_value()) {
            break;  // No more messages
        }

        // Expect empty delimiter frame
        zmq::message_t delimiter_frame;
        auto delim_result = router.recv(delimiter_frame, zmq::recv_flags::dontwait);
        if (!delim_result.has_value()) {
            RCLCPP_WARN(rclcpp::get_logger("zmq_server"),
                        "Malformed ZMQ frame: missing delimiter after identity");
            continue;
        }

        // Expect payload frame
        zmq::message_t payload_frame;
        auto payload_result = router.recv(payload_frame, zmq::recv_flags::dontwait);
        if (!payload_result.has_value() || payload_frame.size() == 0) {
            RCLCPP_WARN(rclcpp::get_logger("zmq_server"),
                        "Malformed ZMQ frame: missing or empty payload");
            continue;
        }

        std::string identity_bytes(
            static_cast<const char*>(identity_frame.data()), identity_frame.size());
        std::string payload(
            static_cast<const char*>(payload_frame.data()), payload_frame.size());

        RCLCPP_DEBUG(rclcpp::get_logger("zmq_server"),
                     "Received message (%zu bytes) from identity (%zu bytes)",
                     payload.size(), identity_bytes.size());

        uint64_t session_id = get_or_create_session(identity_bytes);
        auto session = sessions_[identity_bytes];
        session->touch();

        // Build reply function that enqueues into this session's write queue.
        // Use weak_ptr to avoid preventing session cleanup.
        std::weak_ptr<ZmqSession> weak_session = session;
        auto reply_fn = [weak_session](const std::string& reply) {
            if (auto s = weak_session.lock()) {
                s->enqueue_reply(reply);
            }
        };

        // Dispatch to the callback (which should post_work to avoid blocking poll thread)
        if (on_message_) {
            on_message_(session_id, payload, reply_fn);
        }
    }
}

void ZmqServer::drain_all_sessions(zmq::socket_t& router) {
    for (auto& [identity, session] : sessions_) {
        session->drain_replies(router);
    }
}

void ZmqServer::reap_expired_sessions() {
    auto it = sessions_.begin();
    while (it != sessions_.end()) {
        if (it->second->is_expired(heartbeat_timeout_)) {
            uint64_t sid = it->second->id();
            RCLCPP_INFO(rclcpp::get_logger("zmq_server"),
                        "Session %lu expired (heartbeat timeout), cleaning up", sid);
            if (on_disconnect_) {
                on_disconnect_(sid);
            }
            it = sessions_.erase(it);
        } else {
            ++it;
        }
    }
}

uint64_t ZmqServer::get_or_create_session(const std::string& identity_bytes) {
    auto it = sessions_.find(identity_bytes);
    if (it != sessions_.end()) {
        return it->second->id();
    }

    uint64_t sid = next_session_id_.fetch_add(1);
    sessions_[identity_bytes] = std::make_shared<ZmqSession>(identity_bytes, sid);

    RCLCPP_INFO(rclcpp::get_logger("zmq_server"),
                "New ZMQ session %lu created", sid);
    return sid;
}

}  // namespace cpp_zmq_server
