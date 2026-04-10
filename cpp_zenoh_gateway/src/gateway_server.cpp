#include "cpp_zenoh_gateway/gateway_server.hpp"

#include <chrono>
#include <iostream>

namespace zenoh_gateway {

GatewayServer::GatewayServer(const ServerConfig& config)
    : config_(config)
    , ioc_(config.io_threads)
    , acceptor_(ioc_)
    , drain_timer_(ioc_)
    , topic_template_(config.topics)
{
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

// --- Path parsing ---

bool GatewayServer::parse_path(const std::string& path,
                                std::string& room_id, bool& is_image)
{
    // Expected: /room/{id} or /room/{id}/image
    if (path.size() < 7 || path.substr(0, 6) != "/room/") return false;
    auto rest = path.substr(6);
    auto slash_pos = rest.find('/');
    if (slash_pos == std::string::npos) {
        room_id = rest;
        is_image = false;
    } else {
        room_id = rest.substr(0, slash_pos);
        is_image = (rest.substr(slash_pos) == "/image");
        if (!is_image) return false;
    }
    return !room_id.empty();
}

// --- Connection routing ---

void GatewayServer::identify_transport(tcp::socket socket) {
    auto transport = std::make_shared<WsTransport>(std::move(socket));

    transport->set_on_path([this, transport](const std::string& path) {
        std::string room_id;
        bool is_image = false;

        if (!parse_path(path, room_id, is_image)) {
            transport->close();
            return;
        }

        auto room = get_or_create_room(room_id);

        if (!is_image) {
            // Control transport -- new session
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
                session_rooms_[sid] = room_id;
            }

            room->add_client(sid);

            nlohmann::json welcome = {
                {"op", "welcome"},
                {"session_id", sid},
                {"room_id", room_id},
                {"topics", nlohmann::json::array()}
            };
            for (const auto& tc : room->topics()) {
                welcome["topics"].push_back({
                    {"name", tc.display_name},
                    {"topic_id", tc.topic_id},
                    {"type", tc.stream_type == StreamType::IMAGE ? "image" : "pose"}
                });
            }
            session->send_control(welcome.dump());
        } else {
            // Image transport -- find existing session in this room
            std::shared_ptr<GatewaySession> target;
            {
                std::shared_lock lock(sessions_mutex_);
                for (auto& [sid, rid] : session_rooms_) {
                    if (rid == room_id) {
                        auto it = sessions_.find(sid);
                        if (it != sessions_.end() && it->second->is_alive()) {
                            target = it->second;
                            break;
                        }
                    }
                }
            }
            if (target) {
                target->set_image_transport(transport);
            } else {
                transport->close();
            }
        }
    });

    transport->run("/");
}

// --- Control message handling ---

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

    std::string room_id;
    std::shared_ptr<GatewaySession> session;
    {
        std::shared_lock lock(sessions_mutex_);
        auto rid_it = session_rooms_.find(session_id);
        if (rid_it == session_rooms_.end()) return;
        room_id = rid_it->second;
        auto sit = sessions_.find(session_id);
        if (sit == sessions_.end()) return;
        session = sit->second;
    }

    std::shared_ptr<Room> room;
    {
        std::shared_lock lock(rooms_mutex_);
        auto rit = rooms_.find(room_id);
        if (rit == rooms_.end()) return;
        room = rit->second;
    }

    uint8_t topic_id = 255;
    StreamType stream_type = StreamType::POSE;
    for (const auto& tc : room->topics()) {
        if (tc.display_name == topic_name) {
            topic_id = tc.topic_id;
            stream_type = tc.stream_type;
            break;
        }
    }
    if (topic_id == 255) return;

    auto lane = room->subscribe_client(session_id, topic_id);
    session->get_or_create_lane(topic_id,
        stream_type == StreamType::IMAGE
            ? BackpressurePolicy::LATEST_ONLY : BackpressurePolicy::DROP_OLDEST,
        stream_type == StreamType::IMAGE ? 2 : 128,
        stream_type == StreamType::IMAGE ? 16*1024*1024 : 2*1024*1024);

    if (stream_type == StreamType::POSE) {
        auto schema = encode_schema_message(
            topic_name, topic_id, "f64[7]",
            {"px", "py", "pz", "qx", "qy", "qz", "qw"});
        session->send_control(schema.dump());
    }

    // Late-join: push latest frame
    auto latest = room->latest_frame(topic_id);
    if (latest.has_value() && lane) {
        lane->push(*latest);
    }
}

void GatewayServer::handle_unsubscribe(uint64_t session_id, const nlohmann::json& msg) {
    auto topic_name = msg.value("topic", "");

    std::string room_id;
    {
        std::shared_lock lock(sessions_mutex_);
        auto rid_it = session_rooms_.find(session_id);
        if (rid_it == session_rooms_.end()) return;
        room_id = rid_it->second;
    }

    std::shared_ptr<Room> room;
    {
        std::shared_lock lock(rooms_mutex_);
        auto rit = rooms_.find(room_id);
        if (rit == rooms_.end()) return;
        room = rit->second;
    }

    // Find topic_id and remove from hub
    for (const auto& tc : room->topics()) {
        if (tc.display_name == topic_name) {
            // Per-topic unsubscribe: Room doesn't expose per-topic client removal yet.
            // Full cleanup happens on disconnect via remove_client().
            break;
        }
    }
}

// --- Zenoh setup ---

void GatewayServer::setup_zenoh() {
    auto config = zenoh::Config::create_default();
    zenoh_session_ = std::make_unique<zenoh::Session>(
        zenoh::Session::open(std::move(config)));
    std::cout << "[gateway] Zenoh session opened" << std::endl;
}

void GatewayServer::setup_room_zenoh(std::shared_ptr<Room> room) {
    for (const auto& tc : room->topics()) {
        auto sub = zenoh_session_->declare_subscriber(
            zenoh::KeyExpr(tc.zenoh_key_expr),
            [this, room, tc](zenoh::Sample& sample) {
                const auto& payload = sample.get_payload();
                auto bytes_vec = payload.as_vector();

                auto now = std::chrono::steady_clock::now();
                uint32_t ts_offset = static_cast<uint32_t>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - start_time_).count());

                auto wire = encode_data_frame(
                    tc.stream_type, tc.topic_id, ts_offset,
                    bytes_vec.data(), bytes_vec.size());
                auto shared_bytes = std::make_shared<const std::vector<uint8_t>>(
                    std::move(wire));

                SharedFrame frame{tc.stream_type, tc.topic_id, ts_offset, shared_bytes};
                room->distribute(tc.topic_id, frame);
            },
            []() {}
        );
        zenoh_subs_.push_back(std::move(sub));
    }
}

// --- Room management ---

std::shared_ptr<Room> GatewayServer::get_or_create_room(const std::string& room_id) {
    {
        std::shared_lock lock(rooms_mutex_);
        auto it = rooms_.find(room_id);
        if (it != rooms_.end()) return it->second;
    }

    std::vector<TopicConfig> room_topics;
    for (const auto& tmpl : topic_template_) {
        TopicConfig tc = tmpl;
        // Replace room prefix in zenoh key expr
        // Template has "room/01/cam0" -> replace "01" with actual room_id
        auto first_slash = tc.zenoh_key_expr.find('/');
        if (first_slash != std::string::npos) {
            auto second_slash = tc.zenoh_key_expr.find('/', first_slash + 1);
            if (second_slash != std::string::npos) {
                auto suffix = tc.zenoh_key_expr.substr(second_slash);
                tc.zenoh_key_expr = "room/" + room_id + suffix;
            }
        }
        room_topics.push_back(tc);
    }

    auto room = std::make_shared<Room>(room_id, room_topics);
    {
        std::unique_lock lock(rooms_mutex_);
        // Double-check after acquiring exclusive lock
        auto it = rooms_.find(room_id);
        if (it != rooms_.end()) return it->second;
        rooms_[room_id] = room;
    }

    setup_room_zenoh(room);
    std::cout << "[gateway] Room " << room_id << " created" << std::endl;
    return room;
}

void GatewayServer::check_room_cleanup(const std::string& room_id) {
    std::unique_lock lock(rooms_mutex_);
    auto it = rooms_.find(room_id);
    if (it != rooms_.end() && it->second->is_empty()) {
        rooms_.erase(it);
        std::cout << "[gateway] Room " << room_id << " destroyed (empty)" << std::endl;
    }
}

// --- Session lifecycle ---

void GatewayServer::on_session_disconnect(uint64_t session_id) {
    std::string room_id;
    {
        std::unique_lock lock(sessions_mutex_);
        auto rid_it = session_rooms_.find(session_id);
        if (rid_it != session_rooms_.end()) {
            room_id = rid_it->second;
            session_rooms_.erase(rid_it);
        }
        sessions_.erase(session_id);
    }

    if (!room_id.empty()) {
        std::shared_ptr<Room> room;
        {
            std::shared_lock lock(rooms_mutex_);
            auto it = rooms_.find(room_id);
            if (it != rooms_.end()) room = it->second;
        }
        if (room) {
            room->remove_client(session_id);
            if (room->is_empty()) {
                check_room_cleanup(room_id);
            }
        }
    }

    std::cout << "[gateway] Session " << session_id << " disconnected" << std::endl;
}

// --- Drain timer ---

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
