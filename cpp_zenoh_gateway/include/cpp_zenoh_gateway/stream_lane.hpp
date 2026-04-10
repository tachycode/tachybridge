#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>

#include "cpp_zenoh_gateway/wire_format.hpp"

namespace zenoh_gateway {

enum class BackpressurePolicy {
    LATEST_ONLY,   // Keep only the newest frame (depth 1)
    DROP_OLDEST    // Drop oldest when full
};

class StreamLane {
public:
    StreamLane(BackpressurePolicy policy, size_t max_depth, size_t max_bytes);

    void push(SharedFrame frame);
    std::optional<SharedFrame> pop();

    size_t depth() const;
    size_t byte_usage() const;
    bool empty() const;

    // Drop statistics (reset after read)
    size_t drops_since_last_check();
    size_t total_drops() const;

private:
    void evict_locked();

    BackpressurePolicy policy_;
    size_t max_depth_;
    size_t max_bytes_;
    size_t current_bytes_ = 0;

    std::deque<SharedFrame> queue_;
    mutable std::mutex mutex_;

    size_t drops_ = 0;
    size_t total_drops_ = 0;
};

}  // namespace zenoh_gateway
