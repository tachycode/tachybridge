#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <zmq.hpp>
#include <rclcpp/logging.hpp>

#include "cpp_zmq_server/habilis_types.hpp"

namespace cpp_zmq_server {

// PUB/SUB bridge for the habilis binary protocol.
// Manages a SUB socket (receives from AI manager) and a PUB socket (sends to
// AI manager). Runs its own poll loop on a dedicated thread.
//
// Thread model:
//   - poll_loop runs on a dedicated thread (started externally)
//   - send() is thread-safe (called from work pool threads)
//   - MessageCallback is invoked on the poll thread (caller should post to
//     work pool for heavy processing)
class HabilisBridge {
public:
    using MessageCallback = std::function<void(HabilisMsgType, msgpack::object_handle)>;

    static constexpr size_t kMaxPubQueueSize = 1024;
    static constexpr size_t kMaxDrainPerIteration = 32;
    static constexpr int kPollTimeoutMs = 5;  // 5ms for 30Hz data

    // Takes a shared ZMQ context (owned by ZmqBridgeNode).
    explicit HabilisBridge(zmq::context_t& ctx);
    ~HabilisBridge();

    // Non-copyable
    HabilisBridge(const HabilisBridge&) = delete;
    HabilisBridge& operator=(const HabilisBridge&) = delete;

    // Blocking: runs the poll loop on the calling thread.
    // sub_address: AI manager's PUB address to connect to (e.g., "127.0.0.1")
    // sub_port: AI manager's PUB port (default 4271)
    // pub_port: port to bind PUB socket on (default 4270)
    void run(const std::string& sub_address, uint16_t sub_port,
             uint16_t pub_port, MessageCallback on_message);

    // Signal the poll loop to stop. Thread-safe.
    void stop();

    // Enqueue a message to send via PUB socket. Thread-safe.
    // Returns false if queue is full (oldest message dropped for backpressure).
    // No-ops if bridge is stopped (running_ == false).
    bool send(HabilisMsgType type, const msgpack::sbuffer& body);

    // Overload for pre-serialized frame data
    bool send_raw(const std::vector<uint8_t>& frame);

    bool is_running() const { return running_.load(); }

private:
    void poll_loop(const std::string& sub_endpoint, const std::string& pub_endpoint);
    void handle_incoming(zmq::socket_t& sub_socket);
    void drain_pub_queue(zmq::socket_t& pub_socket);

    zmq::context_t& ctx_;
    std::atomic<bool> running_{false};
    MessageCallback on_message_;

    // PUB write queue (thread-safe, bounded)
    std::deque<std::vector<uint8_t>> pub_queue_;
    mutable std::mutex pub_mutex_;
};

}  // namespace cpp_zmq_server
