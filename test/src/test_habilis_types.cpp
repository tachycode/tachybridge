#include <gtest/gtest.h>
#include <cstring>
#include <vector>

#include "cpp_zmq_server/habilis_types.hpp"

using namespace cpp_zmq_server;

// ── parse/serialize round-trip ──────────────────────────────────────────────

TEST(HabilisTypesTest, SerializeParseRoundTrip) {
    // Pack a simple msgpack body: {"key": "value"}
    msgpack::sbuffer buf;
    msgpack::packer<msgpack::sbuffer> pk(buf);
    pk.pack_map(1);
    pk.pack("key");
    pk.pack("value");

    auto frame = serialize_habilis_frame(HabilisMsgType::HEARTBEAT, buf);

    HabilisMessage msg;
    ASSERT_TRUE(parse_habilis_frame(frame.data(), frame.size(), msg));
    EXPECT_EQ(msg.type, HabilisMsgType::HEARTBEAT);

    // Verify body content
    auto map = msg.body.get().as<std::map<std::string, std::string>>();
    EXPECT_EQ(map["key"], "value");
}

TEST(HabilisTypesTest, RoundTripAllMessageTypes) {
    // Verify round-trip for every valid MsgType (1-16)
    for (uint32_t i = 1; i <= 16; ++i) {
        auto type = static_cast<HabilisMsgType>(i);
        msgpack::sbuffer buf;
        msgpack::packer<msgpack::sbuffer> pk(buf);
        pk.pack(true);

        auto frame = serialize_habilis_frame(type, buf);
        HabilisMessage msg;
        ASSERT_TRUE(parse_habilis_frame(frame.data(), frame.size(), msg))
            << "Failed for type " << i;
        EXPECT_EQ(msg.type, type);
    }
}

TEST(HabilisTypesTest, SerializeRawBytesOverload) {
    // Test the raw bytes overload of serialize_habilis_frame
    std::vector<uint8_t> body = {0x93, 0x01, 0x02, 0x03};  // msgpack [1,2,3]
    auto frame = serialize_habilis_frame(HabilisMsgType::TICK, body.data(), body.size());

    HabilisMessage msg;
    ASSERT_TRUE(parse_habilis_frame(frame.data(), frame.size(), msg));
    EXPECT_EQ(msg.type, HabilisMsgType::TICK);

    auto arr = msg.body.get().as<std::vector<int>>();
    EXPECT_EQ(arr.size(), 3u);
    EXPECT_EQ(arr[0], 1);
    EXPECT_EQ(arr[1], 2);
    EXPECT_EQ(arr[2], 3);
}

// ── MsgType boundaries ─────────────────────────────────────────────────────

TEST(HabilisTypesTest, MsgTypeBoundaryValid) {
    // Type 1 (START_INFERENCE) — minimum valid
    msgpack::sbuffer buf;
    msgpack::packer<msgpack::sbuffer> pk(buf);
    pk.pack(42);

    auto frame = serialize_habilis_frame(HabilisMsgType::START_INFERENCE, buf);
    HabilisMessage msg;
    ASSERT_TRUE(parse_habilis_frame(frame.data(), frame.size(), msg));
    EXPECT_EQ(msg.type, HabilisMsgType::START_INFERENCE);

    // Type 16 (HEARTBEAT) — maximum valid
    auto frame2 = serialize_habilis_frame(HabilisMsgType::HEARTBEAT, buf);
    HabilisMessage msg2;
    ASSERT_TRUE(parse_habilis_frame(frame2.data(), frame2.size(), msg2));
    EXPECT_EQ(msg2.type, HabilisMsgType::HEARTBEAT);
}

TEST(HabilisTypesTest, MsgTypeBoundaryInvalidZero) {
    // Type 0 is invalid
    uint8_t frame[] = {0x00, 0x00, 0x00, 0x00, 0xc0};  // type=0, body=nil
    HabilisMessage msg;
    EXPECT_FALSE(parse_habilis_frame(frame, sizeof(frame), msg));
}

TEST(HabilisTypesTest, MsgTypeBoundaryInvalid17) {
    // Type 17 is out of range
    uint8_t frame[] = {0x00, 0x00, 0x00, 0x11, 0xc0};  // type=17, body=nil
    HabilisMessage msg;
    EXPECT_FALSE(parse_habilis_frame(frame, sizeof(frame), msg));
}

TEST(HabilisTypesTest, MsgTypeBoundaryInvalidLarge) {
    // Type 0xFFFFFFFF is invalid
    uint8_t frame[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xc0};
    HabilisMessage msg;
    EXPECT_FALSE(parse_habilis_frame(frame, sizeof(frame), msg));
}

// ── BigEndian header encoding ──────────────────────────────────────────────

TEST(HabilisTypesTest, BigEndianHeaderEncoding) {
    msgpack::sbuffer buf;
    msgpack::packer<msgpack::sbuffer> pk(buf);
    pk.pack(true);

    // HEARTBEAT = 16 = 0x00000010 in BE
    auto frame = serialize_habilis_frame(HabilisMsgType::HEARTBEAT, buf);
    EXPECT_EQ(frame[0], 0x00);
    EXPECT_EQ(frame[1], 0x00);
    EXPECT_EQ(frame[2], 0x00);
    EXPECT_EQ(frame[3], 0x10);

    // START_INFERENCE = 1 = 0x00000001 in BE
    auto frame2 = serialize_habilis_frame(HabilisMsgType::START_INFERENCE, buf);
    EXPECT_EQ(frame2[0], 0x00);
    EXPECT_EQ(frame2[1], 0x00);
    EXPECT_EQ(frame2[2], 0x00);
    EXPECT_EQ(frame2[3], 0x01);
}

// ── Malformed input ────────────────────────────────────────────────────────

TEST(HabilisTypesTest, MalformedTooShort) {
    uint8_t frame[] = {0x00, 0x01};  // Only 2 bytes, need at least 4
    HabilisMessage msg;
    EXPECT_FALSE(parse_habilis_frame(frame, sizeof(frame), msg));
}

TEST(HabilisTypesTest, MalformedEmptyFrame) {
    HabilisMessage msg;
    EXPECT_FALSE(parse_habilis_frame(nullptr, 0, msg));
}

TEST(HabilisTypesTest, MalformedThreeBytes) {
    uint8_t frame[] = {0x00, 0x00, 0x01};
    HabilisMessage msg;
    EXPECT_FALSE(parse_habilis_frame(frame, sizeof(frame), msg));
}

TEST(HabilisTypesTest, MalformedInvalidMsgpack) {
    // Valid header (type=1), but invalid msgpack body
    uint8_t frame[] = {0x00, 0x00, 0x00, 0x01, 0xC1};  // 0xC1 is never-used msgpack byte
    HabilisMessage msg;
    EXPECT_FALSE(parse_habilis_frame(frame, sizeof(frame), msg));
}

TEST(HabilisTypesTest, EmptyBodyIsValid) {
    // Header only with no body — valid for some message types
    uint8_t frame[] = {0x00, 0x00, 0x00, 0x05};  // TICK, no body
    HabilisMessage msg;
    EXPECT_TRUE(parse_habilis_frame(frame, sizeof(frame), msg));
    EXPECT_EQ(msg.type, HabilisMsgType::TICK);
}

// ── NdarrayDesc pack/unpack round-trip ──────────────────────────────────────

TEST(HabilisTypesTest, NdarrayRoundTrip) {
    // Pack a float32 array [1.0, 2.0, 3.0]
    std::vector<float> values = {1.0f, 2.0f, 3.0f};
    std::vector<uint32_t> shape = {3};

    msgpack::sbuffer buf;
    msgpack::packer<msgpack::sbuffer> pk(buf);
    pack_ndarray(pk, "float32", shape,
                 reinterpret_cast<const uint8_t*>(values.data()),
                 values.size() * sizeof(float));

    // Unpack
    auto oh = msgpack::unpack(buf.data(), buf.size());
    NdarrayDesc desc;
    ASSERT_TRUE(unpack_ndarray(oh.get(), desc));

    EXPECT_EQ(desc.dtype, "float32");
    EXPECT_EQ(desc.shape.size(), 1u);
    EXPECT_EQ(desc.shape[0], 3u);
    EXPECT_EQ(desc.data.size(), 3 * sizeof(float));

    // Verify actual values
    const auto* unpacked = reinterpret_cast<const float*>(desc.data.data());
    EXPECT_FLOAT_EQ(unpacked[0], 1.0f);
    EXPECT_FLOAT_EQ(unpacked[1], 2.0f);
    EXPECT_FLOAT_EQ(unpacked[2], 3.0f);
}

TEST(HabilisTypesTest, NdarrayMultiDimensional) {
    // Pack a 2D float32 array with shape [2, 3]
    std::vector<float> values = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    std::vector<uint32_t> shape = {2, 3};

    msgpack::sbuffer buf;
    msgpack::packer<msgpack::sbuffer> pk(buf);
    pack_ndarray(pk, "float32", shape,
                 reinterpret_cast<const uint8_t*>(values.data()),
                 values.size() * sizeof(float));

    auto oh = msgpack::unpack(buf.data(), buf.size());
    NdarrayDesc desc;
    ASSERT_TRUE(unpack_ndarray(oh.get(), desc));

    EXPECT_EQ(desc.dtype, "float32");
    EXPECT_EQ(desc.shape.size(), 2u);
    EXPECT_EQ(desc.shape[0], 2u);
    EXPECT_EQ(desc.shape[1], 3u);
}

TEST(HabilisTypesTest, NdarrayUnpackMissingFields) {
    // Map with missing 'data' field
    msgpack::sbuffer buf;
    msgpack::packer<msgpack::sbuffer> pk(buf);
    pk.pack_map(2);
    pk.pack("dtype");
    pk.pack("float32");
    pk.pack("shape");
    pk.pack_array(1);
    pk.pack(3u);
    // Missing "data" field

    auto oh = msgpack::unpack(buf.data(), buf.size());
    NdarrayDesc desc;
    EXPECT_FALSE(unpack_ndarray(oh.get(), desc));
}

TEST(HabilisTypesTest, NdarrayUnpackNotAMap) {
    // Body is an array, not a map
    msgpack::sbuffer buf;
    msgpack::packer<msgpack::sbuffer> pk(buf);
    pk.pack_array(3);
    pk.pack(1);
    pk.pack(2);
    pk.pack(3);

    auto oh = msgpack::unpack(buf.data(), buf.size());
    NdarrayDesc desc;
    EXPECT_FALSE(unpack_ndarray(oh.get(), desc));
}

TEST(HabilisTypesTest, NdarrayUnpackDataNotBin) {
    // 'data' field is a string instead of bin
    msgpack::sbuffer buf;
    msgpack::packer<msgpack::sbuffer> pk(buf);
    pk.pack_map(3);
    pk.pack("dtype");
    pk.pack("float32");
    pk.pack("shape");
    pk.pack_array(1);
    pk.pack(1u);
    pk.pack("data");
    pk.pack("not-binary");  // string, not bin

    auto oh = msgpack::unpack(buf.data(), buf.size());
    NdarrayDesc desc;
    EXPECT_FALSE(unpack_ndarray(oh.get(), desc));
}

// ── MsgType name lookup ────────────────────────────────────────────────────

TEST(HabilisTypesTest, MsgTypeNames) {
    EXPECT_STREQ(habilis_msg_type_name(HabilisMsgType::START_INFERENCE), "START_INFERENCE");
    EXPECT_STREQ(habilis_msg_type_name(HabilisMsgType::HEARTBEAT), "HEARTBEAT");
    EXPECT_STREQ(habilis_msg_type_name(HabilisMsgType::ACTION), "ACTION");
    EXPECT_STREQ(habilis_msg_type_name(static_cast<HabilisMsgType>(99)), "UNKNOWN");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
