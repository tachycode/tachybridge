#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <nlohmann/json.hpp>
#include <zenoh.hxx>

#include "cpp_zenoh_gateway/gateway_session.hpp"
#include "cpp_zenoh_gateway/room.hpp"
#include "cpp_zenoh_gateway/wire_format.hpp"

namespace zenoh_gateway {

namespace net = boost::asio;
namespace beast = boost::beast;
using tcp = boost::asio::ip::tcp;

struct ServerConfig {
    std::string address = "0.0.0.0";
    uint16_t port = 9090;
    int io_threads = 4;
    int drain_interval_ms = 5;
    std::vector<TopicConfig> topics;
    std::string zenoh_config_path;
    std::string reader_executable = "mcap_reader_node";
};

class GatewayServer {
public:
    explicit GatewayServer(const ServerConfig& config);
    ~GatewayServer();

    void run();
    void stop();

private:
    void start_accept();
    void on_accept(beast::error_code ec, tcp::socket socket);
    void identify_transport(tcp::socket socket);

    void handle_control(uint64_t session_id, const std::string& message);
    void handle_subscribe(uint64_t session_id, const nlohmann::json& msg);
    void handle_unsubscribe(uint64_t session_id, const nlohmann::json& msg);

    void setup_zenoh();
    void setup_room_zenoh(std::shared_ptr<Room> room);

    void on_session_disconnect(uint64_t session_id);
    void drain_tick();

    static bool parse_path(const std::string& path,
                           std::string& room_id, bool& is_image);
    std::shared_ptr<Room> get_or_create_room(const std::string& room_id);
    void check_room_cleanup(const std::string& room_id);

    ServerConfig config_;

    net::io_context ioc_;
    tcp::acceptor acceptor_;
    net::steady_timer drain_timer_;

    std::unique_ptr<zenoh::Session> zenoh_session_;
    std::vector<zenoh::Subscriber<void>> zenoh_subs_;

    std::unordered_map<uint64_t, std::shared_ptr<GatewaySession>> sessions_;
    std::shared_mutex sessions_mutex_;
    std::atomic<uint64_t> next_session_id_{1};

    // Session-to-room mapping (protected by sessions_mutex_)
    std::unordered_map<uint64_t, std::string> session_rooms_;

    // Room management
    std::unordered_map<std::string, std::shared_ptr<Room>> rooms_;
    std::shared_mutex rooms_mutex_;

    // Topic template for creating rooms
    std::vector<TopicConfig> topic_template_;

    std::chrono::steady_clock::time_point start_time_;
};

}  // namespace zenoh_gateway
