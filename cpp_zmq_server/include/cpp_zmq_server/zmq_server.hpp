#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <zmq.hpp>
#include <rclcpp/logging.hpp>

namespace cpp_zmq_server {

// Per-client session tracking ZMQ identity and write queue
class ZmqSession {
public:
    static constexpr size_t kMaxWriteQueueSize = 1024;

    ZmqSession(std::string identity_bytes, uint64_t session_id);

    uint64_t id() const { return session_id_; }
    const std::string& identity() const { return identity_bytes_; }

    // Queue a reply for this client (thread-safe, called from work_pool).
    // Returns false if queue is full (backpressure: oldest message dropped).
    bool enqueue_reply(std::string message);

    // Drain queued replies into the ROUTER socket (poll thread ONLY).
    // Uses lock-and-swap to minimize lock scope.
    // Returns number of messages sent. Bounded to kMaxDrainPerIteration.
    size_t drain_replies(zmq::socket_t& router);

    // Heartbeat tracking
    void touch();
    bool is_expired(std::chrono::seconds timeout) const;

    static constexpr size_t kMaxDrainPerIteration = 16;

private:
    std::string identity_bytes_;
    uint64_t session_id_;
    std::deque<std::string> write_queue_;
    mutable std::mutex queue_mutex_;
    std::chrono::steady_clock::time_point last_active_;
};

// ZMQ ROUTER server that dispatches messages to a callback
class ZmqServer {
public:
    // Callback signature: (session_id, message_payload, reply_fn)
    // reply_fn enqueues a response back to the client via ROUTER
    using MessageCallback = std::function<void(
        uint64_t session_id,
        const std::string& message,
        std::function<void(const std::string&)> reply_fn)>;

    using DisconnectCallback = std::function<void(uint64_t session_id)>;

    // Accepts shared context (owned externally, e.g., by ZmqBridgeNode)
    explicit ZmqServer(zmq::context_t& ctx);
    ~ZmqServer();

    // Non-copyable
    ZmqServer(const ZmqServer&) = delete;
    ZmqServer& operator=(const ZmqServer&) = delete;

    void set_on_disconnect(DisconnectCallback cb) { on_disconnect_ = std::move(cb); }

    // Blocking: runs the poll loop on the calling thread.
    // Call stop() from another thread to terminate.
    void run(const std::string& address, uint16_t port,
             MessageCallback on_message,
             std::chrono::seconds heartbeat_timeout = std::chrono::seconds{30});

    void stop();

    // Max incoming message size (bytes). Default 64MB.
    static constexpr int64_t kMaxMessageSize = 64 * 1024 * 1024;

private:
    void poll_loop(const std::string& endpoint);
    void handle_incoming(zmq::socket_t& router);
    void drain_all_sessions(zmq::socket_t& router);
    void reap_expired_sessions();
    uint64_t get_or_create_session(const std::string& identity_bytes);

    zmq::context_t& ctx_;
    std::atomic<bool> running_{false};
    MessageCallback on_message_;
    DisconnectCallback on_disconnect_;
    std::chrono::seconds heartbeat_timeout_{30};

    std::unordered_map<std::string, std::shared_ptr<ZmqSession>> sessions_;
    // sessions_ only accessed from poll thread — no mutex needed

    static std::atomic<uint64_t> next_session_id_;

    // Reap timer: check every N poll iterations to avoid per-iteration overhead
    static constexpr int kReapIntervalIterations = 100;  // ~10s at 100ms poll
    int reap_counter_ = 0;
};

}  // namespace cpp_zmq_server
