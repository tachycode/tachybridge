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
