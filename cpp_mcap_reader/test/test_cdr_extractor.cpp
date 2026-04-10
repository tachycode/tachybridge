#include <gtest/gtest.h>
#include "cpp_mcap_reader/cdr_extractor.hpp"
#include <cstring>

using namespace mcap_reader;

static std::vector<uint8_t> make_compressed_image_cdr(
    const std::string& format, const std::vector<uint8_t>& jpeg_data)
{
    std::vector<uint8_t> buf;

    // CDR encapsulation header (little-endian)
    buf.push_back(0x00); buf.push_back(0x01);
    buf.push_back(0x00); buf.push_back(0x00);

    // Header.stamp.sec
    uint32_t sec = 1000;
    buf.insert(buf.end(), reinterpret_cast<uint8_t*>(&sec),
               reinterpret_cast<uint8_t*>(&sec) + 4);
    // Header.stamp.nanosec
    uint32_t nsec = 500;
    buf.insert(buf.end(), reinterpret_cast<uint8_t*>(&nsec),
               reinterpret_cast<uint8_t*>(&nsec) + 4);
    // Header.frame_id
    std::string frame_id = "cam0";
    uint32_t fid_len = frame_id.size() + 1;
    buf.insert(buf.end(), reinterpret_cast<uint8_t*>(&fid_len),
               reinterpret_cast<uint8_t*>(&fid_len) + 4);
    buf.insert(buf.end(), frame_id.begin(), frame_id.end());
    buf.push_back(0x00);
    while (buf.size() % 4 != 0) buf.push_back(0x00);

    // format
    uint32_t fmt_len = format.size() + 1;
    buf.insert(buf.end(), reinterpret_cast<uint8_t*>(&fmt_len),
               reinterpret_cast<uint8_t*>(&fmt_len) + 4);
    buf.insert(buf.end(), format.begin(), format.end());
    buf.push_back(0x00);
    while (buf.size() % 4 != 0) buf.push_back(0x00);

    // data
    uint32_t data_len = jpeg_data.size();
    buf.insert(buf.end(), reinterpret_cast<uint8_t*>(&data_len),
               reinterpret_cast<uint8_t*>(&data_len) + 4);
    buf.insert(buf.end(), jpeg_data.begin(), jpeg_data.end());

    return buf;
}

static std::vector<uint8_t> make_pose_cdr(double px, double py, double pz,
                                           double qx, double qy, double qz, double qw)
{
    std::vector<uint8_t> buf;
    buf.push_back(0x00); buf.push_back(0x01);
    buf.push_back(0x00); buf.push_back(0x00);
    // padding for 8-byte alignment
    buf.push_back(0x00); buf.push_back(0x00);
    buf.push_back(0x00); buf.push_back(0x00);

    double vals[7] = {px, py, pz, qx, qy, qz, qw};
    auto* raw = reinterpret_cast<const uint8_t*>(vals);
    buf.insert(buf.end(), raw, raw + 56);
    return buf;
}

TEST(CdrExtractor, ExtractJpegFromCompressedImage) {
    std::vector<uint8_t> fake_jpeg = {0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10};
    auto cdr = make_compressed_image_cdr("jpeg", fake_jpeg);

    ImageExtractResult result;
    ASSERT_TRUE(extract_compressed_image(cdr.data(), cdr.size(), result));
    EXPECT_EQ(result.format, "jpeg");
    ASSERT_EQ(result.data_size, fake_jpeg.size());
    EXPECT_EQ(std::memcmp(result.data_ptr, fake_jpeg.data(), fake_jpeg.size()), 0);
}

TEST(CdrExtractor, ExtractPose) {
    auto cdr = make_pose_cdr(1.0, 2.0, 3.0, 0.0, 0.0, 0.707, 0.707);

    PoseExtractResult result;
    ASSERT_TRUE(extract_pose(cdr.data(), cdr.size(), result));
    EXPECT_DOUBLE_EQ(result.values[0], 1.0);
    EXPECT_DOUBLE_EQ(result.values[1], 2.0);
    EXPECT_DOUBLE_EQ(result.values[2], 3.0);
    EXPECT_DOUBLE_EQ(result.values[5], 0.707);
    EXPECT_DOUBLE_EQ(result.values[6], 0.707);
}

TEST(CdrExtractor, TooShortReturnsFailure) {
    uint8_t tiny[4] = {0, 1, 0, 0};
    ImageExtractResult img;
    EXPECT_FALSE(extract_compressed_image(tiny, 4, img));
    PoseExtractResult pose;
    EXPECT_FALSE(extract_pose(tiny, 4, pose));
}
