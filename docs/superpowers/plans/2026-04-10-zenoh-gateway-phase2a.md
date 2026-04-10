# Zenoh-WS Gateway Phase 2a — Dual WS, Room Routing, Seek Fix

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire dual WebSocket transport end-to-end with path-based routing, add room-aware topic isolation, fix MCAP backward seek, and add late-join initial state.

**Architecture:** Clients connect to `ws://host:9090/room/{id}` (control+pose) and `ws://host:9090/room/{id}/image` (image frames). The server reads the HTTP upgrade target path to determine transport type and room. Each room has its own set of FanoutHubs and Zenoh subscriptions. MCAP backward seek is fixed by re-opening the reader with a start-time filter.

**Tech Stack:** C++17, zenoh-cpp, Boost.Beast (HTTP upgrade + WebSocket), nlohmann_json

**Prerequisite:** Phase 1 complete — both packages build and all 7 tests pass in Docker.

---

## File Structure

```
cpp_zenoh_gateway/
├── include/cpp_zenoh_gateway/
│   ├── wire_format.hpp           # unchanged
│   ├── stream_lane.hpp           # unchanged
│   ├── fanout_hub.hpp            # unchanged
│   ├── gateway_session.hpp       # MODIFY: add room_id to session
│   ├── gateway_server.hpp        # MODIFY: add room manager, path routing
│   └── room.hpp                  # CREATE: Room struct (hubs + zenoh subs + state)
├── src/
│   ├── stream_lane.cpp           # unchanged
│   ├── fanout_hub.cpp            # unchanged
│   ├── gateway_session.cpp       # MODIFY: minor (room_id field)
│   ├── gateway_server.cpp        # MODIFY: HTTP upgrade parsing, room routing, late-join
│   ├── room.cpp                  # CREATE: Room implementation
│   └── main.cpp                  # MODIFY: config supports room templates
├── test/
│   ├── test_wire_format.cpp      # unchanged
│   ├── test_stream_lane.cpp      # unchanged
│   ├── test_fanout_hub.cpp       # unchanged
│   ├── test_room.cpp             # CREATE: room lifecycle tests
│   └── test_integration_smoke.cpp # unchanged

cpp_mcap_reader/
├── src/
│   └── mcap_publisher.cpp        # MODIFY: fix backward seek
├── test/
│   └── test_seek.cpp             # CREATE: seek behavior tests
```

---

## Task 1: HTTP Upgrade Path Detection in WsTransport

**Files:**
- Modify: `cpp_zenoh_gateway/include/cpp_zenoh_gateway/gateway_session.hpp`
- Modify: `cpp_zenoh_gateway/src/gateway_session.cpp`

Currently `WsTransport::run()` calls `ws_.async_accept()` directly, which skips HTTP layer processing. We need to read the HTTP upgrade request to extract the target path (e.g., `/room/01/image`).

- [ ] **Step 1: Add path accessor and HTTP-aware accept to WsTransport header**

In `gateway_session.hpp`, add to `WsTransport` public section:

```cpp
    const std::string& path() const { return path_; }
```

Add to private section:

```cpp
    std::string path_;
    boost::beast::http::request<boost::beast::http::empty_body> upgrade_req_;
    void on_http_read(beast::error_code ec, std::size_t bytes);
```

- [ ] **Step 2: Rewrite WsTransport::run() to read HTTP upgrade request first**

In `gateway_session.cpp`, replace the `run()` method:

```cpp
void WsTransport::run(const std::string& /*expected_path*/) {
    // Read the HTTP upgrade request to extract the target path
    boost::beast::http::async_read(ws_.next_layer(), buffer_, upgrade_req_,
        beast::bind_front_handler(&WsTransport::on_http_read, shared_from_this()));
}

void WsTransport::on_http_read(beast::error_code ec, std::size_t /*bytes*/) {
    if (ec) { if (on_close_) on_close_(); return; }

    // Extract path from HTTP request target
    path_ = std::string(upgrade_req_.target());

    ws_.set_option(websocket::stream_base::timeout::suggested(
        beast::role_type::server));
    ws_.set_option(websocket::stream_base::decorator(
        [](websocket::response_type& res) {
            res.set(boost::beast::http::field::server, "zenoh-ws-gateway");
        }));

    // Accept the WebSocket upgrade using the already-read HTTP request
    ws_.async_accept(upgrade_req_,
        beast::bind_front_handler(&WsTransport::on_accept, shared_from_this()));
}
```

- [ ] **Step 3: Add OnPath callback to WsTransport**

The server needs to know the path AFTER the HTTP request is read but BEFORE completing the accept. Add callback:

In header, add to public section:
```cpp
    using OnPath = std::function<void(const std::string& path)>;
    void set_on_path(OnPath cb);
```

In private section:
```cpp
    OnPath on_path_;
```

In `gateway_session.cpp`, add:
```cpp
void WsTransport::set_on_path(OnPath cb) { on_path_ = std::move(cb); }
```

In `on_http_read`, before `ws_.async_accept`, add:
```cpp
    if (on_path_) on_path_(path_);
```

- [ ] **Step 4: Verify build in Docker**

```bash
docker run --rm -v $(pwd):/ws/src/tachybridge tachybridge-build:latest bash -c "
source /opt/ros/jazzy/setup.bash && cd /ws &&
colcon build --packages-select cpp_zenoh_gateway --cmake-args -DCMAKE_PREFIX_PATH=/usr/local -DBUILD_TESTING=ON &&
colcon test --packages-select cpp_zenoh_gateway --event-handlers console_direct+"
```

Expected: Build succeeds, existing 4 test suites still pass (they don't use WsTransport directly).

- [ ] **Step 5: Commit**

```bash
git add cpp_zenoh_gateway/include/cpp_zenoh_gateway/gateway_session.hpp \
        cpp_zenoh_gateway/src/gateway_session.cpp
git commit -m "feat(zenoh-gateway): add HTTP upgrade path detection to WsTransport"
```

---

## Task 2: Room Struct

**Files:**
- Create: `cpp_zenoh_gateway/include/cpp_zenoh_gateway/room.hpp`
- Create: `cpp_zenoh_gateway/src/room.cpp`
- Create: `cpp_zenoh_gateway/test/test_room.cpp`
- Modify: `cpp_zenoh_gateway/CMakeLists.txt`

A Room owns its own FanoutHubs and Zenoh subscriptions. This replaces the global hubs_ map in GatewayServer.

- [ ] **Step 1: Write the failing test**

```cpp
// test/test_room.cpp
#include <gtest/gtest.h>
#include "cpp_zenoh_gateway/room.hpp"

using namespace zenoh_gateway;

TEST(Room, CreateWithTopics) {
    std::vector<TopicConfig> topics = {
        {"room/01/cam0", "/cam0", 0, StreamType::IMAGE},
        {"room/01/ee_pose0", "/ee_pose0", 3, StreamType::POSE},
    };
    Room room("01", topics);

    EXPECT_EQ(room.id(), "01");
    EXPECT_EQ(room.hub_count(), 2u);
    EXPECT_TRUE(room.has_hub(0));
    EXPECT_TRUE(room.has_hub(3));
    EXPECT_FALSE(room.has_hub(1));
}

TEST(Room, AddAndRemoveClient) {
    std::vector<TopicConfig> topics = {
        {"room/01/cam0", "/cam0", 0, StreamType::IMAGE},
    };
    Room room("01", topics);

    EXPECT_EQ(room.client_count(), 0u);
    room.add_client(1);
    EXPECT_EQ(room.client_count(), 1u);
    room.add_client(2);
    EXPECT_EQ(room.client_count(), 2u);
    room.remove_client(1);
    EXPECT_EQ(room.client_count(), 1u);
    room.remove_client(2);
    EXPECT_EQ(room.client_count(), 0u);
    EXPECT_TRUE(room.is_empty());
}

TEST(Room, SubscribeClientToTopic) {
    std::vector<TopicConfig> topics = {
        {"room/01/cam0", "/cam0", 0, StreamType::IMAGE},
    };
    Room room("01", topics);
    room.add_client(1);

    auto lane = room.subscribe_client(1, 0);
    ASSERT_NE(lane, nullptr);

    // Distribute a frame — should reach the lane
    auto bytes = std::make_shared<const std::vector<uint8_t>>(
        std::vector<uint8_t>{0xAA});
    SharedFrame frame{StreamType::IMAGE, 0, 100, bytes};
    room.distribute(0, frame);

    auto popped = lane->pop();
    ASSERT_TRUE(popped.has_value());
    EXPECT_EQ(popped->ts_offset_ms, 100u);
}

TEST(Room, UnsubscribeOnRemoveClient) {
    std::vector<TopicConfig> topics = {
        {"room/01/cam0", "/cam0", 0, StreamType::IMAGE},
    };
    Room room("01", topics);
    room.add_client(1);
    auto lane = room.subscribe_client(1, 0);
    room.remove_client(1);

    // Distribute after removal — lane should NOT receive
    auto bytes = std::make_shared<const std::vector<uint8_t>>(
        std::vector<uint8_t>{0xBB});
    room.distribute(0, SharedFrame{StreamType::IMAGE, 0, 200, bytes});
    EXPECT_FALSE(lane->pop().has_value());
}

TEST(Room, LatestFrameForLateJoin) {
    std::vector<TopicConfig> topics = {
        {"room/01/cam0", "/cam0", 0, StreamType::IMAGE},
    };
    Room room("01", topics);

    // Distribute before any client joins
    auto bytes = std::make_shared<const std::vector<uint8_t>>(
        std::vector<uint8_t>{0xCC});
    SharedFrame frame{StreamType::IMAGE, 0, 300, bytes};
    room.distribute(0, frame);

    // Late-join client should get the latest frame
    auto latest = room.latest_frame(0);
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(latest->ts_offset_ms, 300u);
}
```

- [ ] **Step 2: Run test to verify it fails**

- [ ] **Step 3: Write room.hpp**

```cpp
// include/cpp_zenoh_gateway/room.hpp
#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>

#include "cpp_zenoh_gateway/fanout_hub.hpp"
#include "cpp_zenoh_gateway/stream_lane.hpp"
#include "cpp_zenoh_gateway/wire_format.hpp"

namespace zenoh_gateway {

struct TopicConfig;  // forward decl from gateway_server.hpp — move to wire_format.hpp

struct TopicConfig {
    std::string zenoh_key_expr;
    std::string display_name;
    uint8_t topic_id;
    StreamType stream_type;
};

class Room {
public:
    Room(const std::string& id, const std::vector<TopicConfig>& topics);

    const std::string& id() const { return id_; }

    // Client management
    void add_client(uint64_t session_id);
    void remove_client(uint64_t session_id);
    size_t client_count() const;
    bool is_empty() const;

    // Topic subscription (creates lane, registers in hub)
    std::shared_ptr<StreamLane> subscribe_client(uint64_t session_id, uint8_t topic_id);

    // Data distribution
    void distribute(uint8_t topic_id, const SharedFrame& frame);

    // Late-join: get the most recent frame for a topic
    std::optional<SharedFrame> latest_frame(uint8_t topic_id) const;

    // Hub access
    bool has_hub(uint8_t topic_id) const;
    size_t hub_count() const;

    // Topic config access
    const std::vector<TopicConfig>& topics() const { return topics_; }

private:
    std::string id_;
    std::vector<TopicConfig> topics_;

    // Per-topic hubs
    std::unordered_map<uint8_t, std::shared_ptr<FanoutHub>> hubs_;

    // Per-topic latest frame (for late-join)
    mutable std::mutex latest_mutex_;
    std::unordered_map<uint8_t, SharedFrame> latest_frames_;

    // Connected clients
    mutable std::mutex clients_mutex_;
    std::set<uint64_t> clients_;
};

}  // namespace zenoh_gateway
```

- [ ] **Step 4: Write room.cpp**

```cpp
// src/room.cpp
#include "cpp_zenoh_gateway/room.hpp"

namespace zenoh_gateway {

Room::Room(const std::string& id, const std::vector<TopicConfig>& topics)
    : id_(id), topics_(topics)
{
    for (const auto& tc : topics_) {
        hubs_[tc.topic_id] = std::make_shared<FanoutHub>();
    }
}

void Room::add_client(uint64_t session_id) {
    std::lock_guard lock(clients_mutex_);
    clients_.insert(session_id);
}

void Room::remove_client(uint64_t session_id) {
    {
        std::lock_guard lock(clients_mutex_);
        clients_.erase(session_id);
    }
    // Unsubscribe from all hubs
    for (auto& [_, hub] : hubs_) {
        hub->remove_client(session_id);
    }
}

size_t Room::client_count() const {
    std::lock_guard lock(clients_mutex_);
    return clients_.size();
}

bool Room::is_empty() const {
    std::lock_guard lock(clients_mutex_);
    return clients_.empty();
}

std::shared_ptr<StreamLane> Room::subscribe_client(
    uint64_t session_id, uint8_t topic_id)
{
    auto hub_it = hubs_.find(topic_id);
    if (hub_it == hubs_.end()) return nullptr;

    // Determine policy based on topic config
    BackpressurePolicy policy = BackpressurePolicy::DROP_OLDEST;
    size_t depth = 128;
    size_t bytes = 2 * 1024 * 1024;

    for (const auto& tc : topics_) {
        if (tc.topic_id == topic_id && tc.stream_type == StreamType::IMAGE) {
            policy = BackpressurePolicy::LATEST_ONLY;
            depth = 2;
            bytes = 16 * 1024 * 1024;
            break;
        }
    }

    auto lane = std::make_shared<StreamLane>(policy, depth, bytes);
    hub_it->second->add_client(session_id, lane);
    return lane;
}

void Room::distribute(uint8_t topic_id, const SharedFrame& frame) {
    // Store latest for late-join
    {
        std::lock_guard lock(latest_mutex_);
        latest_frames_[topic_id] = frame;
    }

    auto hub_it = hubs_.find(topic_id);
    if (hub_it != hubs_.end()) {
        hub_it->second->distribute(frame);
    }
}

std::optional<SharedFrame> Room::latest_frame(uint8_t topic_id) const {
    std::lock_guard lock(latest_mutex_);
    auto it = latest_frames_.find(topic_id);
    if (it == latest_frames_.end()) return std::nullopt;
    return it->second;
}

bool Room::has_hub(uint8_t topic_id) const {
    return hubs_.count(topic_id) > 0;
}

size_t Room::hub_count() const {
    return hubs_.size();
}

}  // namespace zenoh_gateway
```

- [ ] **Step 5: Add room.cpp to CMakeLists.txt**

In `cpp_zenoh_gateway/CMakeLists.txt`, add `src/room.cpp` to the library sources:

```cmake
add_library(zenoh_gateway_lib
  src/stream_lane.cpp
  src/fanout_hub.cpp
  src/gateway_session.cpp
  src/gateway_server.cpp
  src/room.cpp
)
```

And add test:
```cmake
  ament_add_gtest(test_room test/test_room.cpp)
  target_link_libraries(test_room zenoh_gateway_lib)
```

- [ ] **Step 6: Build and test in Docker**

Expected: 5 test suites pass including new test_room (5 tests).

- [ ] **Step 7: Commit**

```bash
git add cpp_zenoh_gateway/include/cpp_zenoh_gateway/room.hpp \
        cpp_zenoh_gateway/src/room.cpp \
        cpp_zenoh_gateway/test/test_room.cpp \
        cpp_zenoh_gateway/CMakeLists.txt
git commit -m "feat(zenoh-gateway): add Room struct with per-topic hubs and late-join"
```

---

## Task 3: Path-Based Session Routing in GatewayServer

**Files:**
- Modify: `cpp_zenoh_gateway/include/cpp_zenoh_gateway/gateway_server.hpp`
- Modify: `cpp_zenoh_gateway/src/gateway_server.cpp`

Replace the global `hubs_` map with per-room management. Route incoming connections by path:
- `/room/{id}` → control+pose transport
- `/room/{id}/image` → image transport

- [ ] **Step 1: Update gateway_server.hpp**

Replace global hubs with room management:

```cpp
// Remove these members:
//   std::unordered_map<uint8_t, std::shared_ptr<FanoutHub>> hubs_;
//   std::unordered_map<std::string, TopicConfig> topic_by_name_;
//   std::unordered_map<uint8_t, TopicConfig> topic_by_id_;

// Add these:
#include "cpp_zenoh_gateway/room.hpp"

// In private section:
    // Room management
    std::unordered_map<std::string, std::shared_ptr<Room>> rooms_;
    std::shared_mutex rooms_mutex_;

    // Pending image connections waiting to be paired with a session
    // Key: room_id, Value: list of (session_id, transport)
    std::unordered_map<uint64_t, std::string> session_rooms_;  // session_id → room_id

    // Topic template (used when creating rooms)
    std::vector<TopicConfig> topic_template_;

    // Parse path into (room_id, is_image)
    static bool parse_path(const std::string& path,
                           std::string& room_id, bool& is_image);

    // Room operations
    std::shared_ptr<Room> get_or_create_room(const std::string& room_id);
    void setup_room_zenoh(std::shared_ptr<Room> room);
    void check_room_cleanup(const std::string& room_id);
```

- [ ] **Step 2: Implement parse_path**

```cpp
bool GatewayServer::parse_path(const std::string& path,
                                std::string& room_id, bool& is_image)
{
    // Expected paths:
    //   /room/{id}        → control+pose
    //   /room/{id}/image  → image transport
    if (path.size() < 7 || path.substr(0, 6) != "/room/") return false;

    auto rest = path.substr(6);  // "{id}" or "{id}/image"
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
```

- [ ] **Step 3: Rewrite identify_transport with path routing**

```cpp
void GatewayServer::identify_transport(tcp::socket socket) {
    auto transport = std::make_shared<WsTransport>(std::move(socket));

    transport->set_on_path([this, transport](const std::string& path) {
        std::string room_id;
        bool is_image = false;

        if (!parse_path(path, room_id, is_image)) {
            // Invalid path — close
            transport->close();
            return;
        }

        auto room = get_or_create_room(room_id);

        if (!is_image) {
            // Control transport — new session
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

            // Send welcome with room info
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
            // Image transport — find existing session for this room
            // Client must send session_id in query param or first message
            // For simplicity: attach to most recent session in this room
            std::shared_ptr<GatewaySession> target;
            {
                std::shared_lock lock(sessions_mutex_);
                for (auto& [sid, rid] : session_rooms_) {
                    if (rid == room_id) {
                        auto it = sessions_.find(sid);
                        if (it != sessions_.end() && it->second->is_alive()) {
                            target = it->second;
                            // Prefer session without image transport
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

    transport->run("/");  // path arg ignored now, parsed from HTTP
}
```

- [ ] **Step 4: Implement get_or_create_room and setup_room_zenoh**

```cpp
std::shared_ptr<Room> GatewayServer::get_or_create_room(const std::string& room_id) {
    {
        std::shared_lock lock(rooms_mutex_);
        auto it = rooms_.find(room_id);
        if (it != rooms_.end()) return it->second;
    }

    // Create room with topic template, replacing room prefix
    std::vector<TopicConfig> room_topics;
    for (const auto& tmpl : topic_template_) {
        TopicConfig tc = tmpl;
        // Replace "room/01" with "room/{room_id}" in zenoh key expr
        auto pos = tc.zenoh_key_expr.find('/');
        if (pos != std::string::npos) {
            auto suffix = tc.zenoh_key_expr.substr(
                tc.zenoh_key_expr.find('/', pos + 1));
            tc.zenoh_key_expr = "room/" + room_id + suffix;
        }
        room_topics.push_back(tc);
    }

    auto room = std::make_shared<Room>(room_id, room_topics);

    {
        std::unique_lock lock(rooms_mutex_);
        rooms_[room_id] = room;
    }

    setup_room_zenoh(room);

    std::cout << "[gateway] Room " << room_id << " created" << std::endl;
    return room;
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
            []() {}  // on_drop
        );
        zenoh_subs_.push_back(std::move(sub));
    }
}
```

- [ ] **Step 5: Update handle_subscribe to use rooms**

```cpp
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

    // Find topic_id by display_name
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

    // Send schema for pose topics
    if (stream_type == StreamType::POSE) {
        auto schema = encode_schema_message(
            topic_name, topic_id, "f64[7]",
            {"px", "py", "pz", "qx", "qy", "qz", "qw"});
        session->send_control(schema.dump());
    }

    // Late-join: send latest frame if available
    auto latest = room->latest_frame(topic_id);
    if (latest.has_value()) {
        auto wire_bytes = latest->bytes;
        std::vector<uint8_t> ref(wire_bytes->begin(), wire_bytes->end());
        if (stream_type == StreamType::IMAGE) {
            // Will be sent on next drain or push directly
            lane->push(*latest);
        } else {
            lane->push(*latest);
        }
    }
}
```

- [ ] **Step 6: Update on_session_disconnect to use rooms**

```cpp
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

void GatewayServer::check_room_cleanup(const std::string& room_id) {
    std::unique_lock lock(rooms_mutex_);
    auto it = rooms_.find(room_id);
    if (it != rooms_.end() && it->second->is_empty()) {
        rooms_.erase(it);
        std::cout << "[gateway] Room " << room_id << " destroyed (empty)" << std::endl;
    }
}
```

- [ ] **Step 7: Update constructor and setup_zenoh**

In constructor, store topics as template instead of creating global hubs:
```cpp
GatewayServer::GatewayServer(const ServerConfig& config)
    : config_(config)
    , ioc_(config.io_threads)
    , acceptor_(ioc_)
    , drain_timer_(ioc_)
    , topic_template_(config.topics)
{
    start_time_ = std::chrono::steady_clock::now();
}
```

`setup_zenoh()` just opens the session, no global subscribers:
```cpp
void GatewayServer::setup_zenoh() {
    auto config = zenoh::Config::create_default();
    zenoh_session_ = std::make_unique<zenoh::Session>(
        zenoh::Session::open(std::move(config)));
    std::cout << "[gateway] Zenoh session opened" << std::endl;
}
```

- [ ] **Step 8: Build and test in Docker**

Expected: Build succeeds. Existing tests pass (test_room is the key new one).

- [ ] **Step 9: Commit**

```bash
git commit -m "feat(zenoh-gateway): add room-based routing with dual WS and late-join"
```

---

## Task 4: Fix MCAP Backward Seek

**Files:**
- Modify: `cpp_mcap_reader/src/mcap_publisher.cpp`
- Create: `cpp_mcap_reader/test/test_seek.cpp`
- Modify: `cpp_mcap_reader/CMakeLists.txt`

The current seek implementation only works forward (skips messages via `continue`). Backward seek requires re-reading the MCAP file from the target position.

- [ ] **Step 1: Write the failing test**

```cpp
// test/test_seek.cpp
#include <gtest/gtest.h>
#include "cpp_mcap_reader/mcap_publisher.hpp"

// This test verifies the seek logic by testing the internal method
// Since publish_loop is blocking, we test the reader restart approach

#define MCAP_IMPLEMENTATION
#include <mcap/writer.hpp>

#include <filesystem>
#include <fstream>

using namespace mcap_reader;

static std::string create_test_mcap(const std::string& path, int num_messages) {
    mcap::McapWriter writer;
    auto opts = mcap::McapWriterOptions("test");
    writer.open(path, opts);

    mcap::Schema schema("test_schema", "raw", "");
    writer.addSchema(schema);

    mcap::Channel channel("test/topic", "raw", schema.id);
    writer.addChannel(channel);

    for (int i = 0; i < num_messages; ++i) {
        mcap::Message msg;
        msg.channelId = channel.id;
        msg.logTime = (i + 1) * 1000000000ULL;  // 1s, 2s, 3s, ...
        msg.publishTime = msg.logTime;
        uint8_t data = static_cast<uint8_t>(i);
        msg.data = &data;
        msg.dataSize = 1;
        writer.write(msg);
    }

    writer.close();
    return path;
}

TEST(McapSeek, ReadMessagesInTimeRange) {
    auto path = create_test_mcap("/tmp/test_seek.mcap", 10);

    mcap::McapReader reader;
    ASSERT_TRUE(reader.open(path).ok());

    // Read from timestamp 5s onwards
    mcap::ReadMessageOptions opts;
    opts.startTime = 5000000000ULL;  // 5s
    opts.endTime = std::numeric_limits<uint64_t>::max();

    auto view = reader.readMessages([](const auto&) {}, opts);
    int count = 0;
    uint64_t first_ts = 0;
    for (auto it = view.begin(); it != view.end(); ++it) {
        if (count == 0) first_ts = it->message.logTime;
        count++;
    }

    EXPECT_GE(first_ts, 5000000000ULL);
    EXPECT_EQ(count, 6);  // messages at 5s,6s,7s,8s,9s,10s

    reader.close();
    std::filesystem::remove(path);
}
```

- [ ] **Step 2: Run test to verify it fails (or passes to validate mcap API)**

- [ ] **Step 3: Refactor publish_loop to support backward seek**

Replace the seek logic in `mcap_publisher.cpp`. The key change: when seek is requested, close and reopen the reader with `ReadMessageOptions.startTime`.

```cpp
void McapPublisher::publish_loop() {
    while (!stopped_) {
        mcap::McapReader reader;
        auto status = reader.open(config_.mcap_path);
        if (!status.ok()) {
            std::cerr << "[mcap-reader] Failed to open: "
                      << config_.mcap_path << " - " << status.message << std::endl;
            return;
        }

        std::unordered_map<std::string, TopicMapping> topic_map;
        for (const auto& m : config_.mappings) {
            topic_map[m.mcap_topic] = m;
        }

        auto stats = reader.statistics();
        if (stats.has_value()) {
            duration_ns_ = stats->messageEndTime - stats->messageStartTime;
        }

        // Read with optional start time for seek support
        mcap::ReadMessageOptions read_opts;
        uint64_t seek_start = seek_target_.load();
        if (seek_requested_.exchange(false)) {
            read_opts.startTime = seek_start;
        }
        read_opts.endTime = std::numeric_limits<uint64_t>::max();

        auto view = reader.readMessages([](const auto&) {}, read_opts);
        uint64_t first_ts = 0;
        auto wall_start = std::chrono::steady_clock::now();
        bool first_message = true;

        for (auto it = view.begin(); it != view.end(); ++it) {
            if (stopped_) break;

            // Pause loop
            while (!playing_ && !stopped_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                // Check for seek during pause
                if (seek_requested_) break;
            }
            if (stopped_) break;

            // Seek requested — break out and restart reader
            if (seek_requested_) break;

            const auto& msg = it->message;
            const auto& channel = *it->channel;

            // Timing
            if (first_message) {
                first_ts = msg.logTime;
                wall_start = std::chrono::steady_clock::now();
                first_message = false;
            }

            uint64_t msg_offset_ns = msg.logTime - first_ts;
            double speed = speed_.load();
            if (speed > 0.0) {
                auto expected_wall = wall_start + std::chrono::nanoseconds(
                    static_cast<uint64_t>(msg_offset_ns / speed));
                auto now = std::chrono::steady_clock::now();
                if (expected_wall > now) {
                    std::this_thread::sleep_until(expected_wall);
                }
            }

            current_ts_ = msg.logTime;

            auto mit = topic_map.find(channel.topic);
            if (mit == topic_map.end()) continue;

            const auto& mapping = mit->second;
            const uint8_t* data = reinterpret_cast<const uint8_t*>(msg.data);
            size_t size = msg.dataSize;

            if (mapping.message_type == "sensor_msgs/msg/CompressedImage") {
                publish_image(mapping, data, size, msg.logTime);
            } else if (mapping.message_type == "geometry_msgs/msg/Pose") {
                publish_pose(mapping, data, size, msg.logTime);
            }
        }

        reader.close();

        // If we broke out due to seek, loop back and reopen with new start time
        // If we reached end naturally and no seek, we're done
        if (!seek_requested_ && !stopped_) {
            break;  // Normal end of playback
        }
    }

    std::cout << "[mcap-reader] Playback finished" << std::endl;
}
```

- [ ] **Step 4: Add test to CMakeLists.txt**

```cmake
  ament_add_gtest(test_seek test/test_seek.cpp)
  target_link_libraries(test_seek mcap_reader_lib)
```

**Note:** test_seek.cpp includes `mcap/writer.hpp` with `MCAP_IMPLEMENTATION`. Since mcap_publisher.cpp also defines it, the test must link only against the library (which already has the implementation). Remove the `#define MCAP_IMPLEMENTATION` from test_seek.cpp and instead make sure `mcap/writer.hpp` can be used without it, OR compile test_seek.cpp separately without linking mcap_publisher.o.

Actually, simpler: make the test a standalone executable that doesn't link mcap_reader_lib:

```cmake
  ament_add_gtest(test_seek test/test_seek.cpp)
  target_link_libraries(test_seek Threads::Threads PkgConfig::LZ4 PkgConfig::ZSTD)
  target_include_directories(test_seek PRIVATE /usr/local/include)
```

- [ ] **Step 5: Build and test in Docker**

Expected: test_seek passes, test_cdr_extractor still passes.

- [ ] **Step 6: Commit**

```bash
git commit -m "fix(mcap-reader): support backward seek by reopening reader with startTime"
```

---

## Self-Review Checklist

1. **Spec coverage:**
   - [x] Dual WS end-to-end → Task 1 (HTTP path detection) + Task 3 (path routing)
   - [x] Room-based isolation → Task 2 (Room struct) + Task 3 (room routing)
   - [x] Late-join initial state → Task 2 (latest_frame) + Task 3 (handle_subscribe)
   - [x] Backward seek → Task 4 (reader restart)
   - [x] Room cleanup on empty → Task 3 (check_room_cleanup)

2. **Placeholder scan:** No TBD/TODO found.

3. **Type consistency:**
   - `TopicConfig` — moved to room.hpp, used in Room and GatewayServer ✓
   - `SharedFrame` — consistent across all files ✓
   - `StreamType::IMAGE / POSE` — consistent ✓
   - `parse_path` → returns `(room_id, is_image)` — used in `identify_transport` ✓
   - `Room::subscribe_client` returns `shared_ptr<StreamLane>` — matches usage ✓

**Note:** TopicConfig is currently defined in gateway_server.hpp. Task 2 redefines it in room.hpp. During implementation, the definition in gateway_server.hpp should be removed and replaced with `#include "room.hpp"`. The implementer should handle this.
