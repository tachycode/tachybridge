#include "cpp_zenoh_gateway/stream_lane.hpp"

namespace zenoh_gateway {

StreamLane::StreamLane(BackpressurePolicy policy, size_t max_depth, size_t max_bytes)
    : policy_(policy), max_depth_(max_depth), max_bytes_(max_bytes) {}

void StreamLane::push(SharedFrame frame) {
    std::lock_guard<std::mutex> lock(mutex_);

    size_t frame_bytes = frame.bytes ? frame.bytes->size() : 0;

    if (policy_ == BackpressurePolicy::LATEST_ONLY) {
        current_bytes_ = 0;
        queue_.clear();
        queue_.push_back(std::move(frame));
        current_bytes_ = frame_bytes;
        return;
    }

    // DROP_OLDEST
    queue_.push_back(std::move(frame));
    current_bytes_ += frame_bytes;
    evict_locked();
}

std::optional<SharedFrame> StreamLane::pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) return std::nullopt;

    auto frame = std::move(queue_.front());
    size_t frame_bytes = frame.bytes ? frame.bytes->size() : 0;
    queue_.pop_front();
    current_bytes_ -= frame_bytes;
    return frame;
}

size_t StreamLane::depth() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
}

size_t StreamLane::byte_usage() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_bytes_;
}

bool StreamLane::empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
}

void StreamLane::evict_locked() {
    while (queue_.size() > max_depth_ || current_bytes_ > max_bytes_) {
        if (queue_.empty()) break;
        size_t front_bytes = queue_.front().bytes ? queue_.front().bytes->size() : 0;
        queue_.pop_front();
        current_bytes_ -= front_bytes;
    }
}

}  // namespace zenoh_gateway
