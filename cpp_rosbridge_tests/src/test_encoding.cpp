#include <gtest/gtest.h>
#include "cpp_rosbridge_core/encoding.hpp"

using cpp_rosbridge_core::Encoding;
using cpp_rosbridge_core::encoding_from_string;
using cpp_rosbridge_core::encode_cdr_frame;
using cpp_rosbridge_core::parse_cdr_frame;
using cpp_rosbridge_core::CdrFrameView;

class EncodingTest : public ::testing::Test {};

TEST_F(EncodingTest, ParseJsonEncoding) {
    EXPECT_EQ(encoding_from_string("json"), Encoding::JSON);
}

TEST_F(EncodingTest, ParseCborEncoding) {
    EXPECT_EQ(encoding_from_string("cbor"), Encoding::CBOR);
}

TEST_F(EncodingTest, ParseCdrEncoding) {
    EXPECT_EQ(encoding_from_string("cdr"), Encoding::CDR);
}

TEST_F(EncodingTest, ParseEmptyDefaultsToJson) {
    EXPECT_EQ(encoding_from_string(""), Encoding::JSON);
}

TEST_F(EncodingTest, ParseUnknownDefaultsToJson) {
    EXPECT_EQ(encoding_from_string("protobuf"), Encoding::JSON);
}

TEST_F(EncodingTest, CdrFrameRoundTrip) {
    std::string topic = "/laser_scan";
    std::vector<uint8_t> payload = {0x00, 0x01, 0x00, 0x00, 0xDE, 0xAD};
    uint64_t timestamp_ns = 1234567890ULL;

    auto frame = encode_cdr_frame(topic, timestamp_ns, payload.data(), payload.size());

    CdrFrameView view;
    ASSERT_TRUE(parse_cdr_frame(frame.data(), frame.size(), view));
    EXPECT_EQ(view.topic, "/laser_scan");
    EXPECT_EQ(view.timestamp_ns, 1234567890ULL);
    EXPECT_EQ(view.payload_size, 6u);
    EXPECT_EQ(std::memcmp(view.payload_data, payload.data(), payload.size()), 0);
}

TEST_F(EncodingTest, CdrFrameTooShortFails) {
    std::vector<uint8_t> tiny = {0x02, 0x00};
    CdrFrameView view;
    EXPECT_FALSE(parse_cdr_frame(tiny.data(), tiny.size(), view));
}

TEST_F(EncodingTest, CdrFrameEmptyPayload) {
    std::string topic = "/empty";
    auto frame = encode_cdr_frame(topic, 0, nullptr, 0);

    CdrFrameView view;
    ASSERT_TRUE(parse_cdr_frame(frame.data(), frame.size(), view));
    EXPECT_EQ(view.topic, "/empty");
    EXPECT_EQ(view.payload_size, 0u);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
