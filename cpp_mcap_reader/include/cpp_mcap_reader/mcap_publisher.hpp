#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <zenoh.hxx>

namespace mcap_reader {

struct TopicMapping {
    std::string mcap_topic;
    std::string zenoh_key_expr;
    std::string message_type;
};

struct ReaderConfig {
    std::string mcap_path;
    std::string room_id = "01";
    std::vector<TopicMapping> mappings;
    double playback_speed = 1.0;
};

class McapPublisher {
public:
    explicit McapPublisher(const ReaderConfig& config,
                           std::shared_ptr<zenoh::Session> session);

    void play();
    void pause();
    void resume();
    void seek(uint64_t timestamp_ns);
    void set_speed(double speed);
    void stop();

    bool is_playing() const;
    uint64_t current_timestamp() const;
    uint64_t duration_ns() const;

private:
    void publish_loop();
    void publish_image(const TopicMapping& mapping,
                       const uint8_t* cdr, size_t size,
                       uint64_t log_time_ns);
    void publish_pose(const TopicMapping& mapping,
                      const uint8_t* cdr, size_t size,
                      uint64_t log_time_ns);

    ReaderConfig config_;
    std::shared_ptr<zenoh::Session> session_;
    std::unordered_map<std::string, zenoh::Publisher> publishers_;

    std::atomic<bool> playing_{false};
    std::atomic<bool> stopped_{false};
    std::atomic<double> speed_{1.0};
    std::atomic<uint64_t> current_ts_{0};
    std::atomic<uint64_t> seek_target_{0};
    std::atomic<bool> seek_requested_{false};
    uint64_t duration_ns_ = 0;
};

}  // namespace mcap_reader
