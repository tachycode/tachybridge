#pragma once

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace zenoh_gateway {

enum class StreamType : uint8_t {
    IMAGE = 0x01,
    POSE  = 0x02
};

struct SharedFrame {
    StreamType stream_type;
    uint8_t topic_id;
    uint32_t ts_offset_ms;
    std::shared_ptr<const std::vector<uint8_t>> bytes;
};

struct DataFrameView {
    StreamType stream_type;
    uint8_t topic_id;
    uint32_t ts_offset_ms;
    const uint8_t* payload_data;
    size_t payload_size;
};

// Wire format: [1B stream_type][1B topic_id][4B ts_offset_ms BE][payload]
static constexpr size_t kDataFrameHeaderSize = 6;

inline std::vector<uint8_t> encode_data_frame(
    StreamType type, uint8_t topic_id, uint32_t ts_offset_ms,
    const uint8_t* payload, size_t payload_size)
{
    std::vector<uint8_t> frame(kDataFrameHeaderSize + payload_size);
    frame[0] = static_cast<uint8_t>(type);
    frame[1] = topic_id;
    frame[2] = static_cast<uint8_t>((ts_offset_ms >> 24) & 0xFF);
    frame[3] = static_cast<uint8_t>((ts_offset_ms >> 16) & 0xFF);
    frame[4] = static_cast<uint8_t>((ts_offset_ms >> 8) & 0xFF);
    frame[5] = static_cast<uint8_t>(ts_offset_ms & 0xFF);
    if (payload_size > 0) {
        std::memcpy(frame.data() + kDataFrameHeaderSize, payload, payload_size);
    }
    return frame;
}

inline bool parse_data_frame(const uint8_t* data, size_t size, DataFrameView& view) {
    if (size < kDataFrameHeaderSize) return false;
    view.stream_type = static_cast<StreamType>(data[0]);
    view.topic_id = data[1];
    view.ts_offset_ms = (static_cast<uint32_t>(data[2]) << 24)
                      | (static_cast<uint32_t>(data[3]) << 16)
                      | (static_cast<uint32_t>(data[4]) << 8)
                      | static_cast<uint32_t>(data[5]);
    view.payload_data = data + kDataFrameHeaderSize;
    view.payload_size = size - kDataFrameHeaderSize;
    return true;
}

inline nlohmann::json encode_schema_message(
    const std::string& topic, uint8_t topic_id,
    const std::string& format, const std::vector<std::string>& fields)
{
    return {
        {"op", "schema"},
        {"topic", topic},
        {"topic_id", topic_id},
        {"format", format},
        {"fields", fields}
    };
}

}  // namespace zenoh_gateway
