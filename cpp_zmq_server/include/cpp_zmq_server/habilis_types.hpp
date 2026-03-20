#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <msgpack.hpp>

namespace cpp_zmq_server {

// Must match Python constants.py MsgType(Enum) exactly.
// Python auto() starts at 1.
enum class HabilisMsgType : uint32_t {
    START_INFERENCE  = 1,
    FINISH_INFERENCE = 2,
    START_TIMER      = 3,
    FINISH_TIMER     = 4,
    TICK             = 5,
    OBSERVATION      = 6,
    ACTION           = 7,
    ROBOT_TYPE       = 8,
    START_TRAINING   = 9,
    FINISH_TRAINING  = 10,
    REQUEST          = 11,
    RESPONSE         = 12,
    ERROR            = 13,
    TRAINING_STATUS  = 14,
    INFERENCE_STATUS = 15,
    HEARTBEAT        = 16,
};

inline const char* habilis_msg_type_name(HabilisMsgType t) {
    switch (t) {
        case HabilisMsgType::START_INFERENCE:  return "START_INFERENCE";
        case HabilisMsgType::FINISH_INFERENCE: return "FINISH_INFERENCE";
        case HabilisMsgType::START_TIMER:      return "START_TIMER";
        case HabilisMsgType::FINISH_TIMER:     return "FINISH_TIMER";
        case HabilisMsgType::TICK:             return "TICK";
        case HabilisMsgType::OBSERVATION:      return "OBSERVATION";
        case HabilisMsgType::ACTION:           return "ACTION";
        case HabilisMsgType::ROBOT_TYPE:       return "ROBOT_TYPE";
        case HabilisMsgType::START_TRAINING:   return "START_TRAINING";
        case HabilisMsgType::FINISH_TRAINING:  return "FINISH_TRAINING";
        case HabilisMsgType::REQUEST:          return "REQUEST";
        case HabilisMsgType::RESPONSE:         return "RESPONSE";
        case HabilisMsgType::ERROR:            return "ERROR";
        case HabilisMsgType::TRAINING_STATUS:  return "TRAINING_STATUS";
        case HabilisMsgType::INFERENCE_STATUS: return "INFERENCE_STATUS";
        case HabilisMsgType::HEARTBEAT:        return "HEARTBEAT";
        default: return "UNKNOWN";
    }
}

// Port constants matching Python constants.py PORTS dict
constexpr uint16_t kHabilisPubPort = 4270;   // comm2ai: bridge PUB -> AI SUB
constexpr uint16_t kHabilisSubPort = 4271;   // ai2comm: AI PUB -> bridge SUB

// Wire format: [4-byte BE header (MsgType value)] [msgpack body]
// Header format matches Python struct.pack("!I", msg_type.value)
constexpr size_t kHabilisHeaderSize = 4;

// Parsed habilis message
struct HabilisMessage {
    HabilisMsgType type;
    msgpack::object_handle body;
};

// Parse a raw ZMQ frame into header + msgpack body.
// Returns false on malformed input (too short, invalid msgpack).
inline bool parse_habilis_frame(const uint8_t* data, size_t len,
                                HabilisMessage& out) {
    if (len < kHabilisHeaderSize) {
        return false;
    }

    // Read 4-byte big-endian unsigned int
    uint32_t raw_type = (static_cast<uint32_t>(data[0]) << 24) |
                        (static_cast<uint32_t>(data[1]) << 16) |
                        (static_cast<uint32_t>(data[2]) << 8)  |
                        (static_cast<uint32_t>(data[3]));

    if (raw_type < 1 || raw_type > 16) {
        return false;
    }

    out.type = static_cast<HabilisMsgType>(raw_type);

    const uint8_t* body_ptr = data + kHabilisHeaderSize;
    size_t body_len = len - kHabilisHeaderSize;

    if (body_len == 0) {
        // Empty body is valid for some message types
        out.body = msgpack::object_handle();
        return true;
    }

    try {
        out.body = msgpack::unpack(
            reinterpret_cast<const char*>(body_ptr), body_len);
    } catch (const msgpack::unpack_error&) {
        return false;
    }

    return true;
}

// Serialize a habilis message into wire format: [4-byte BE header][msgpack body]
inline std::vector<uint8_t> serialize_habilis_frame(
    HabilisMsgType type, const msgpack::sbuffer& body) {
    uint32_t raw_type = static_cast<uint32_t>(type);

    std::vector<uint8_t> frame;
    frame.reserve(kHabilisHeaderSize + body.size());

    // Big-endian header
    frame.push_back(static_cast<uint8_t>((raw_type >> 24) & 0xFF));
    frame.push_back(static_cast<uint8_t>((raw_type >> 16) & 0xFF));
    frame.push_back(static_cast<uint8_t>((raw_type >> 8)  & 0xFF));
    frame.push_back(static_cast<uint8_t>((raw_type)       & 0xFF));

    // Append msgpack body
    frame.insert(frame.end(), body.data(), body.data() + body.size());

    return frame;
}

// Overload for serializing from raw bytes (pre-packed msgpack)
inline std::vector<uint8_t> serialize_habilis_frame(
    HabilisMsgType type, const uint8_t* body_data, size_t body_len) {
    uint32_t raw_type = static_cast<uint32_t>(type);

    std::vector<uint8_t> frame;
    frame.reserve(kHabilisHeaderSize + body_len);

    frame.push_back(static_cast<uint8_t>((raw_type >> 24) & 0xFF));
    frame.push_back(static_cast<uint8_t>((raw_type >> 16) & 0xFF));
    frame.push_back(static_cast<uint8_t>((raw_type >> 8)  & 0xFF));
    frame.push_back(static_cast<uint8_t>((raw_type)       & 0xFF));

    frame.insert(frame.end(), body_data, body_data + body_len);

    return frame;
}

// Numpy array descriptor (matches Python msg_utils.py pack_ndarray format)
struct NdarrayDesc {
    std::string dtype;
    std::vector<uint32_t> shape;
    std::vector<uint8_t> data;
};

// Unpack a msgpack object representing a numpy array {dtype, shape, data}
inline bool unpack_ndarray(const msgpack::object& obj, NdarrayDesc& out) {
    if (obj.type != msgpack::type::MAP) return false;

    try {
        auto map = obj.as<std::map<std::string, msgpack::object>>();

        auto it_dtype = map.find("dtype");
        auto it_shape = map.find("shape");
        auto it_data  = map.find("data");

        if (it_dtype == map.end() || it_shape == map.end() ||
            it_data == map.end()) {
            return false;
        }

        out.dtype = it_dtype->second.as<std::string>();

        auto shape_arr = it_shape->second.as<std::vector<uint32_t>>();
        out.shape = std::move(shape_arr);

        // data is binary (msgpack bin type)
        if (it_data->second.type == msgpack::type::BIN) {
            auto bin = it_data->second.via.bin;
            out.data.assign(
                reinterpret_cast<const uint8_t*>(bin.ptr),
                reinterpret_cast<const uint8_t*>(bin.ptr) + bin.size);
        } else {
            return false;
        }

        return true;
    } catch (...) {
        return false;
    }
}

// Pack a numpy array descriptor into msgpack
inline void pack_ndarray(msgpack::packer<msgpack::sbuffer>& packer,
                         const std::string& dtype,
                         const std::vector<uint32_t>& shape,
                         const uint8_t* data, size_t data_len) {
    packer.pack_map(3);

    packer.pack("dtype");
    packer.pack(dtype);

    packer.pack("shape");
    packer.pack_array(shape.size());
    for (auto s : shape) {
        packer.pack(s);
    }

    packer.pack("data");
    packer.pack_bin(data_len);
    packer.pack_bin_body(reinterpret_cast<const char*>(data), data_len);
}

}  // namespace cpp_zmq_server
