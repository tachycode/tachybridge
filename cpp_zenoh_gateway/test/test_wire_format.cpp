#include <gtest/gtest.h>
#include "cpp_zenoh_gateway/wire_format.hpp"

using namespace zenoh_gateway;

TEST(WireFormat, EncodeImageFrame) {
    std::vector<uint8_t> jpeg = {0xFF, 0xD8, 0xFF, 0xE0, 0x00};
    auto frame = encode_data_frame(StreamType::IMAGE, 1, 42000, jpeg.data(), jpeg.size());

    ASSERT_GE(frame.size(), 6 + jpeg.size());
    EXPECT_EQ(frame[0], static_cast<uint8_t>(StreamType::IMAGE));
    EXPECT_EQ(frame[1], 1);
    uint32_t ts = (frame[2] << 24) | (frame[3] << 16) | (frame[4] << 8) | frame[5];
    EXPECT_EQ(ts, 42000u);
    EXPECT_EQ(std::memcmp(frame.data() + 6, jpeg.data(), jpeg.size()), 0);
}

TEST(WireFormat, EncodePoseFrame) {
    double pose[7] = {1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0};
    auto frame = encode_data_frame(StreamType::POSE, 3, 100,
        reinterpret_cast<const uint8_t*>(pose), sizeof(pose));

    ASSERT_EQ(frame.size(), 6 + 56);
    EXPECT_EQ(frame[0], static_cast<uint8_t>(StreamType::POSE));
    EXPECT_EQ(frame[1], 3);

    double* parsed = reinterpret_cast<double*>(frame.data() + 6);
    EXPECT_DOUBLE_EQ(parsed[0], 1.0);
    EXPECT_DOUBLE_EQ(parsed[6], 1.0);
}

TEST(WireFormat, ParseDataFrame) {
    std::vector<uint8_t> payload = {0xAA, 0xBB};
    auto frame = encode_data_frame(StreamType::IMAGE, 5, 9999, payload.data(), payload.size());

    DataFrameView view;
    ASSERT_TRUE(parse_data_frame(frame.data(), frame.size(), view));
    EXPECT_EQ(view.stream_type, StreamType::IMAGE);
    EXPECT_EQ(view.topic_id, 5);
    EXPECT_EQ(view.ts_offset_ms, 9999u);
    EXPECT_EQ(view.payload_size, 2u);
    EXPECT_EQ(view.payload_data[0], 0xAA);
}

TEST(WireFormat, ParseTooShort) {
    uint8_t tiny[3] = {0x01, 0x00, 0x00};
    DataFrameView view;
    EXPECT_FALSE(parse_data_frame(tiny, 3, view));
}

TEST(SharedFrame, RefCountSharing) {
    auto payload = std::make_shared<const std::vector<uint8_t>>(
        std::vector<uint8_t>{1, 2, 3});
    SharedFrame f1{StreamType::IMAGE, 0, 100, payload};
    SharedFrame f2 = f1;

    EXPECT_EQ(payload.use_count(), 3);
    EXPECT_EQ(f1.bytes->size(), 3u);
    EXPECT_EQ(f2.bytes->size(), 3u);
    EXPECT_EQ(f1.bytes.get(), f2.bytes.get());
}

TEST(WireFormat, EncodeSchemaMessage) {
    auto json = encode_schema_message("/ee_pose_0", 3, "f64[7]",
        {"px", "py", "pz", "qx", "qy", "qz", "qw"});
    EXPECT_EQ(json["op"], "schema");
    EXPECT_EQ(json["topic"], "/ee_pose_0");
    EXPECT_EQ(json["topic_id"], 3);
    EXPECT_EQ(json["format"], "f64[7]");
    EXPECT_EQ(json["fields"].size(), 7u);
}
