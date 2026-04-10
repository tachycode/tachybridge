#include "cpp_zenoh_gateway/fanout_hub.hpp"
#include <vector>

namespace zenoh_gateway {

void FanoutHub::add_client(uint64_t session_id, std::shared_ptr<StreamLane> lane) {
    std::unique_lock lock(mutex_);
    lanes_[session_id] = std::move(lane);
}

void FanoutHub::remove_client(uint64_t session_id) {
    std::unique_lock lock(mutex_);
    lanes_.erase(session_id);
}

void FanoutHub::distribute(const SharedFrame& frame) {
    // Snapshot lanes under lock, then push without lock
    std::vector<std::shared_ptr<StreamLane>> snapshot;
    {
        std::shared_lock lock(mutex_);
        snapshot.reserve(lanes_.size());
        for (auto& [_, lane] : lanes_) {
            snapshot.push_back(lane);
        }
    }
    for (auto& lane : snapshot) {
        lane->push(frame);
    }
}

size_t FanoutHub::client_count() const {
    std::shared_lock lock(mutex_);
    return lanes_.size();
}

}  // namespace zenoh_gateway
