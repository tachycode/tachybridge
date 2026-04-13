#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <sys/types.h>
#include <unordered_map>
#include <vector>

#include "cpp_zenoh_gateway/fanout_hub.hpp"
#include "cpp_zenoh_gateway/stream_lane.hpp"
#include "cpp_zenoh_gateway/wire_format.hpp"

namespace zenoh_gateway {

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

    void add_client(uint64_t session_id);
    void remove_client(uint64_t session_id);
    size_t client_count() const;
    bool is_empty() const;

    std::shared_ptr<StreamLane> subscribe_client(uint64_t session_id, uint8_t topic_id);
    void distribute(uint8_t topic_id, const SharedFrame& frame);
    std::optional<SharedFrame> latest_frame(uint8_t topic_id) const;

    bool has_hub(uint8_t topic_id) const;
    size_t hub_count() const;
    const std::vector<TopicConfig>& topics() const { return topics_; }

    // Process management
    void spawn_reader(const std::string& mcap_path, const std::string& reader_executable);
    void kill_reader();
    bool has_reader() const { return reader_pid_ > 0; }
    pid_t reader_pid() const { return reader_pid_; }

private:
    std::string id_;
    std::vector<TopicConfig> topics_;
    std::unordered_map<uint8_t, std::shared_ptr<FanoutHub>> hubs_;

    mutable std::mutex latest_mutex_;
    std::unordered_map<uint8_t, SharedFrame> latest_frames_;

    mutable std::mutex clients_mutex_;
    std::set<uint64_t> clients_;

    pid_t reader_pid_ = -1;
};

}  // namespace zenoh_gateway
