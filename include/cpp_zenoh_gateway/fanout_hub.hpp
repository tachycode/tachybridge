#pragma once

#include <cstdint>
#include <memory>
#include <shared_mutex>
#include <unordered_map>

#include "cpp_zenoh_gateway/stream_lane.hpp"
#include "cpp_zenoh_gateway/wire_format.hpp"

namespace zenoh_gateway {

class FanoutHub {
public:
    void add_client(uint64_t session_id, std::shared_ptr<StreamLane> lane);
    void remove_client(uint64_t session_id);
    void distribute(const SharedFrame& frame);
    size_t client_count() const;

private:
    std::unordered_map<uint64_t, std::shared_ptr<StreamLane>> lanes_;
    mutable std::shared_mutex mutex_;
};

}  // namespace zenoh_gateway
