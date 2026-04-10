#include "cpp_zenoh_gateway/gateway_server.hpp"

#include <chrono>
#include <iostream>

namespace zenoh_gateway {

GatewayServer::GatewayServer(const ServerConfig& config)
    : config_(config)
    , ioc_(config.io_threads)
    , acceptor_(ioc_)
    , drain_timer_(ioc_)
{
    for (const auto& tc : config_.topics) {
        topic_by_name_[tc.display_name] = tc;
        topic_by_id_[tc.topic_id] = tc;
        hubs_[tc.topic_id] = std::make_shared<FanoutHub>();
    }
    start_time_ = std::chrono::steady_clock::now();
}

GatewayServer::~GatewayServer() { stop(); }

void GatewayServer::run() {
    setup_zenoh();

    auto endpoint = tcp::endpoint(
        net::ip::make_address(config_.address), config_.port);
    acceptor_.open(endpoint.protocol());
    acceptor_.set_option(net::socket_base::reuse_address(true));
    acceptor_.bind(endpoint);
    acceptor_.listen(net::socket_base::max_listen_connections);

    std::cout << "[gateway] Listening on " << config_.address
              << ":" << config_.port << std::endl;

    start_accept();
    drain_tick();

    std::vector<std::thread> threads;
    for (int i = 1; i < config_.io_threads; ++i) {
        threads.emplace_back([this]() { ioc_.run(); });
    }
    ioc_.run();

    for (auto& t : threads) {
        if (t.joinable()) t.join();
    }
}

void GatewayServer::stop() {
    zenoh_subs_.clear();       // stop Zenoh callbacks first
    zenoh_session_.reset();    // then destroy session
    ioc_.stop();               // then stop Asio
}

void GatewayServer::start_accept() {
    acceptor_.async_accept(
        [this](beast::error_code ec, tcp::socket socket) {
            on_accept(ec, std::move(socket));
        });
}

void GatewayServer::on_accept(beast::error_code ec, tcp::socket socket) {
    if (ec) {
        std::cerr << "[gateway] Accept error: " << ec.message() << std::endl;
    } else {
        identify_transport(std::move(socket));
    }
    start_accept();
}

void GatewayServer::identify_transport(tcp::socket socket) {
    auto transport = std::make_shared<WsTransport>(std::move(socket));

    uint64_t sid = next_session_id_.fetch_add(1);
    auto session = std::make_shared<GatewaySession>(sid);

    session->set_on_control([this](uint64_t id, const std::string& msg) {
        handle_control(id, msg);
    });
    session->set_on_disconnect([this](uint64_t id) {
        on_session_disconnect(id);
    });
    session->set_control_transport(transport);

    {
        std::unique_lock lock(sessions_mutex_);
        sessions_[sid] = session;
    }

    transport->run("/");

    nlohmann::json welcome = {
        {"op", "welcome"},
        {"session_id", sid},
        {"topics", nlohmann::json::array()}
    };
    for (const auto& tc : config_.topics) {
        welcome["topics"].push_back({
            {"name", tc.display_name},
            {"topic_id", tc.topic_id},
            {"type", tc.stream_type == StreamType::IMAGE ? "image" : "pose"}
        });
    }
    session->send_control(welcome.dump());
}

void GatewayServer::handle_control(uint64_t session_id, const std::string& message) {
    try {
        auto msg = nlohmann::json::parse(message);
        auto op = msg.value("op", "");

        if (op == "subscribe") {
            handle_subscribe(session_id, msg);
        } else if (op == "unsubscribe") {
            handle_unsubscribe(session_id, msg);
        }
    } catch (const std::exception& e) {
        std::cerr << "[gateway] Control parse error: " << e.what() << std::endl;
    }
}

void GatewayServer::handle_subscribe(uint64_t session_id, const nlohmann::json& msg) {
    auto topic_name = msg.value("topic", "");
    auto it = topic_by_name_.find(topic_name);
    if (it == topic_by_name_.end()) return;

    const auto& tc = it->second;

    std::shared_ptr<GatewaySession> session;
    {
        std::shared_lock lock(sessions_mutex_);
        auto sit = sessions_.find(session_id);
        if (sit == sessions_.end()) return;
        session = sit->second;
    }

    BackpressurePolicy policy;
    size_t depth, bytes;
    if (tc.stream_type == StreamType::IMAGE) {
        policy = BackpressurePolicy::LATEST_ONLY;
        depth = 2;
        bytes = 16 * 1024 * 1024;
    } else {
        policy = BackpressurePolicy::DROP_OLDEST;
        depth = 128;
        bytes = 2 * 1024 * 1024;
    }

    auto lane = session->get_or_create_lane(tc.topic_id, policy, depth, bytes);

    auto hub_it = hubs_.find(tc.topic_id);
    if (hub_it != hubs_.end()) {
        hub_it->second->add_client(session_id, lane);
    }

    if (tc.stream_type == StreamType::POSE) {
        auto schema = encode_schema_message(
            tc.display_name, tc.topic_id, "f64[7]",
            {"px", "py", "pz", "qx", "qy", "qz", "qw"});
        session->send_control(schema.dump());
    }
}

void GatewayServer::handle_unsubscribe(uint64_t session_id, const nlohmann::json& msg) {
    auto topic_name = msg.value("topic", "");
    auto it = topic_by_name_.find(topic_name);
    if (it == topic_by_name_.end()) return;

    auto hub_it = hubs_.find(it->second.topic_id);
    if (hub_it != hubs_.end()) {
        hub_it->second->remove_client(session_id);
    }
}

void GatewayServer::setup_zenoh() {
    auto config = zenoh::Config::create_default();
    zenoh_session_ = std::make_unique<zenoh::Session>(
        zenoh::Session::open(std::move(config)));

    for (const auto& tc : config_.topics) {
        auto sub = zenoh_session_->declare_subscriber(
            zenoh::KeyExpr(tc.zenoh_key_expr),
            [this, tc](zenoh::Sample& sample) {
                on_zenoh_sample(tc, sample);
            },
            []() {}  // on_drop
        );
        zenoh_subs_.push_back(std::move(sub));
    }

    std::cout << "[gateway] Zenoh session opened, "
              << zenoh_subs_.size() << " subscribers" << std::endl;
}

void GatewayServer::on_zenoh_sample(
    const TopicConfig& topic, zenoh::Sample& sample)
{
    const auto& payload = sample.get_payload();
    auto bytes_vec = payload.as_vector();

    auto now = std::chrono::steady_clock::now();
    uint32_t ts_offset = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - start_time_).count());

    auto wire = encode_data_frame(
        topic.stream_type, topic.topic_id, ts_offset,
        bytes_vec.data(), bytes_vec.size());

    auto shared_bytes = std::make_shared<const std::vector<uint8_t>>(
        std::move(wire));

    SharedFrame frame{
        topic.stream_type, topic.topic_id, ts_offset, shared_bytes};

    auto hub_it = hubs_.find(topic.topic_id);
    if (hub_it != hubs_.end()) {
        hub_it->second->distribute(frame);
    }
}

void GatewayServer::on_session_disconnect(uint64_t session_id) {
    for (auto& [_, hub] : hubs_) {
        hub->remove_client(session_id);
    }
    size_t remaining;
    {
        std::unique_lock lock(sessions_mutex_);
        sessions_.erase(session_id);
        remaining = sessions_.size();
    }
    std::cout << "[gateway] Session " << session_id
              << " disconnected. Active: " << remaining << std::endl;
}

void GatewayServer::drain_tick() {
    drain_timer_.expires_after(
        std::chrono::milliseconds(config_.drain_interval_ms));
    drain_timer_.async_wait([this](beast::error_code ec) {
        if (ec) return;

        std::vector<std::shared_ptr<GatewaySession>> snapshot;
        {
            std::shared_lock lock(sessions_mutex_);
            snapshot.reserve(sessions_.size());
            for (auto& [_, s] : sessions_) {
                snapshot.push_back(s);
            }
        }

        for (auto& session : snapshot) {
            session->drain_lanes();
        }

        drain_tick();
    });
}

}  // namespace zenoh_gateway
