#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/asio.hpp>
#include <boost/beast.hpp>

#include "cpp_zenoh_gateway/stream_lane.hpp"
#include "cpp_zenoh_gateway/wire_format.hpp"

namespace zenoh_gateway {

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

// A single WebSocket transport (either control or image)
class WsTransport : public std::enable_shared_from_this<WsTransport> {
public:
    explicit WsTransport(tcp::socket&& socket);

    void run(const std::string& expected_path);
    void send_text(std::string message);
    void send_binary(std::vector<uint8_t> data);
    void close();

    const std::string& path() const { return path_; }

    using OnMessage = std::function<void(const std::string&, bool is_binary)>;
    using OnClose = std::function<void()>;
    using OnPath = std::function<void(const std::string& path)>;
    void set_on_message(OnMessage cb);
    void set_on_close(OnClose cb);
    void set_on_path(OnPath cb);

private:
    struct QueueItem {
        bool is_binary;
        std::string text;
        std::vector<uint8_t> binary;
    };

    static constexpr size_t kMaxControlQueueSize = 256;

    void on_http_read(beast::error_code ec, std::size_t bytes);
    void on_accept(beast::error_code ec);
    void do_read();
    void on_read(beast::error_code ec, std::size_t bytes);
    void do_write();
    void on_write(beast::error_code ec, std::size_t bytes);

    websocket::stream<beast::tcp_stream> ws_;
    beast::flat_buffer buffer_;
    boost::beast::http::request<boost::beast::http::empty_body> upgrade_req_;
    std::deque<QueueItem> write_queue_;
    bool writing_ = false;
    OnMessage on_message_;
    OnClose on_close_;
    OnPath on_path_;
    std::string path_;
};

// Client session — owns two transports + StreamLanes per topic
class GatewaySession : public std::enable_shared_from_this<GatewaySession> {
public:
    explicit GatewaySession(uint64_t id);

    uint64_t id() const { return id_; }

    void set_control_transport(std::shared_ptr<WsTransport> t);
    void set_image_transport(std::shared_ptr<WsTransport> t);

    void send_control(const std::string& json);

    std::shared_ptr<StreamLane> get_or_create_lane(
        uint8_t topic_id, BackpressurePolicy policy,
        size_t max_depth, size_t max_bytes);

    // Drain all lanes round-robin and send to appropriate transport
    void drain_lanes();

    bool is_alive() const;
    void close();

    using OnControl = std::function<void(uint64_t session_id, const std::string& message)>;
    void set_on_control(OnControl cb);

    using OnDisconnect = std::function<void(uint64_t session_id)>;
    void set_on_disconnect(OnDisconnect cb);

private:
    uint64_t id_;
    std::shared_ptr<WsTransport> control_;
    std::shared_ptr<WsTransport> image_;

    std::unordered_map<uint8_t, std::shared_ptr<StreamLane>> lanes_;
    std::mutex lanes_mutex_;

    std::atomic<bool> alive_{true};
    std::once_flag disconnect_once_;
    OnControl on_control_;
    OnDisconnect on_disconnect_;

    uint32_t drain_cycle_ = 0;
    static constexpr uint32_t kQualityCheckInterval = 200;  // ~1s at 5ms drain
};

}  // namespace zenoh_gateway
