#define MCAP_IMPLEMENTATION
#include <mcap/reader.hpp>

#include "cpp_mcap_reader/mcap_publisher.hpp"
#include "cpp_mcap_reader/cdr_extractor.hpp"

#include <chrono>
#include <iostream>
#include <thread>

namespace mcap_reader {

McapPublisher::McapPublisher(const ReaderConfig& config,
                             std::shared_ptr<zenoh::Session> session)
    : config_(config), session_(std::move(session)), speed_(config.playback_speed)
{
    for (const auto& m : config_.mappings) {
        auto pub = session_->declare_publisher(
            zenoh::KeyExpr(m.zenoh_key_expr));
        publishers_.emplace(m.zenoh_key_expr, std::move(pub));
    }
}

void McapPublisher::play() {
    playing_ = true;
    stopped_ = false;
    publish_loop();
}

void McapPublisher::pause() { playing_ = false; }
void McapPublisher::resume() { playing_ = true; }
void McapPublisher::stop() { stopped_ = true; playing_ = false; }

void McapPublisher::seek(uint64_t timestamp_ns) {
    seek_target_ = timestamp_ns;
    seek_requested_ = true;
}

void McapPublisher::set_speed(double speed) {
    speed_ = speed > 0.0 ? speed : 1.0;
}

bool McapPublisher::is_playing() const { return playing_; }
uint64_t McapPublisher::current_timestamp() const { return current_ts_; }
uint64_t McapPublisher::duration_ns() const { return duration_ns_; }

void McapPublisher::publish_loop() {
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

    auto view = reader.readMessages();
    uint64_t first_ts = 0;
    auto wall_start = std::chrono::steady_clock::now();
    bool first_message = true;

    for (auto it = view.begin(); it != view.end(); ++it) {
        if (stopped_) break;

        while (!playing_ && !stopped_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (stopped_) break;

        if (seek_requested_) {
            seek_requested_ = false;
            uint64_t target = seek_target_.load();
            if (it->message.logTime < target) continue;
            first_message = true;
        }

        const auto& msg = it->message;
        const auto& channel = *it->channel;

        if (first_message) {
            first_ts = msg.logTime;
            wall_start = std::chrono::steady_clock::now();
            first_message = false;
        }

        uint64_t msg_offset_ns = msg.logTime - first_ts;
        double speed = speed_.load();
        auto expected_wall = wall_start + std::chrono::nanoseconds(
            static_cast<uint64_t>(msg_offset_ns / speed));
        auto now = std::chrono::steady_clock::now();

        if (expected_wall > now) {
            std::this_thread::sleep_until(expected_wall);
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
    std::cout << "[mcap-reader] Playback finished" << std::endl;
}

void McapPublisher::publish_image(
    const TopicMapping& mapping, const uint8_t* cdr, size_t size,
    uint64_t /*log_time_ns*/)
{
    ImageExtractResult img;
    if (!extract_compressed_image(cdr, size, img)) return;

    auto it = publishers_.find(mapping.zenoh_key_expr);
    if (it == publishers_.end()) return;

    it->second.put(zenoh::Bytes(img.data_ptr, img.data_size));
}

void McapPublisher::publish_pose(
    const TopicMapping& mapping, const uint8_t* cdr, size_t size,
    uint64_t /*log_time_ns*/)
{
    PoseExtractResult pose;
    if (!extract_pose(cdr, size, pose)) return;

    auto it = publishers_.find(mapping.zenoh_key_expr);
    if (it == publishers_.end()) return;

    it->second.put(zenoh::Bytes(reinterpret_cast<const uint8_t*>(pose.values), 56));
}

}  // namespace mcap_reader
