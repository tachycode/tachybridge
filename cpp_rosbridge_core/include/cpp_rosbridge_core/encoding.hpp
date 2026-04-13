#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace cpp_rosbridge_core {

enum class Encoding : uint8_t {
    JSON = 0,
    CBOR = 1,
    CDR  = 2
};

inline Encoding encoding_from_string(const std::string& s) {
    if (s == "cdr")  return Encoding::CDR;
    if (s == "cbor") return Encoding::CBOR;
    return Encoding::JSON;
}

struct CdrFrameView {
    std::string topic;
    uint64_t timestamp_ns = 0;
    const uint8_t* payload_data = nullptr;
    size_t payload_size = 0;
};

inline std::vector<uint8_t> encode_cdr_frame(
    const std::string& topic,
    uint64_t timestamp_ns,
    const uint8_t* cdr_data,
    size_t cdr_size)
{
    const uint32_t topic_len = static_cast<uint32_t>(topic.size());
    std::vector<uint8_t> frame;
    frame.reserve(1 + 4 + topic_len + 8 + cdr_size);

    frame.push_back(0x02);

    frame.push_back(static_cast<uint8_t>((topic_len >> 24) & 0xFF));
    frame.push_back(static_cast<uint8_t>((topic_len >> 16) & 0xFF));
    frame.push_back(static_cast<uint8_t>((topic_len >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(topic_len & 0xFF));

    frame.insert(frame.end(), topic.begin(), topic.end());

    for (int i = 56; i >= 0; i -= 8) {
        frame.push_back(static_cast<uint8_t>((timestamp_ns >> i) & 0xFF));
    }

    if (cdr_data && cdr_size > 0) {
        frame.insert(frame.end(), cdr_data, cdr_data + cdr_size);
    }

    return frame;
}

inline bool parse_cdr_frame(const uint8_t* data, size_t size, CdrFrameView& view) {
    if (size < 13) return false;
    if (data[0] != 0x02) return false;

    uint32_t topic_len = (static_cast<uint32_t>(data[1]) << 24) |
                         (static_cast<uint32_t>(data[2]) << 16) |
                         (static_cast<uint32_t>(data[3]) << 8)  |
                         static_cast<uint32_t>(data[4]);

    size_t header_size = 1 + 4 + topic_len + 8;
    if (size < header_size) return false;

    view.topic = std::string(reinterpret_cast<const char*>(data + 5), topic_len);

    size_t ts_offset = 5 + topic_len;
    view.timestamp_ns = 0;
    for (int i = 0; i < 8; ++i) {
        view.timestamp_ns = (view.timestamp_ns << 8) | data[ts_offset + i];
    }

    view.payload_data = (size > header_size) ? data + header_size : nullptr;
    view.payload_size = size - header_size;

    return true;
}

}  // namespace cpp_rosbridge_core
