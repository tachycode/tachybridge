#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <cstdint>

class CborTransportTest : public ::testing::Test {};

TEST_F(CborTransportTest, JsonToCborRoundTrip) {
    nlohmann::json original = {
        {"op", "publish"},
        {"topic", "/test"},
        {"msg", {{"data", "hello"}}}
    };

    auto cbor = nlohmann::json::to_cbor(original);
    auto decoded = nlohmann::json::from_cbor(cbor);

    EXPECT_EQ(original, decoded);
}

TEST_F(CborTransportTest, BinaryDataPreserved) {
    std::vector<uint8_t> image_data = {0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A};

    nlohmann::json msg;
    msg["op"] = "publish";
    msg["data"] = nlohmann::json::binary(image_data);

    auto cbor = nlohmann::json::to_cbor(msg);
    auto decoded = nlohmann::json::from_cbor(cbor);

    EXPECT_TRUE(decoded["data"].is_binary());
    EXPECT_EQ(decoded["data"].get_binary(), nlohmann::json::binary_t(image_data));
}

TEST_F(CborTransportTest, NestedBinaryFields) {
    std::vector<uint8_t> bytes = {0x01, 0x02, 0x03};

    nlohmann::json msg;
    msg["image"]["data"] = nlohmann::json::binary(bytes);
    msg["image"]["format"] = "png";

    auto cbor = nlohmann::json::to_cbor(msg);
    auto decoded = nlohmann::json::from_cbor(cbor);

    EXPECT_TRUE(decoded["image"]["data"].is_binary());
    EXPECT_EQ(decoded["image"]["data"].get_binary(), nlohmann::json::binary_t(bytes));
    EXPECT_EQ(decoded["image"]["format"], "png");
}

TEST_F(CborTransportTest, EmptyBinaryField) {
    nlohmann::json msg;
    msg["data"] = nlohmann::json::binary({});

    auto cbor = nlohmann::json::to_cbor(msg);
    auto decoded = nlohmann::json::from_cbor(cbor);

    EXPECT_TRUE(decoded["data"].is_binary());
    EXPECT_TRUE(decoded["data"].get_binary().empty());
}

TEST_F(CborTransportTest, LargeBinaryPayload) {
    std::vector<uint8_t> large(1024 * 100);
    for (size_t i = 0; i < large.size(); ++i) {
        large[i] = static_cast<uint8_t>(i % 256);
    }

    nlohmann::json msg;
    msg["op"] = "publish";
    msg["payload"] = nlohmann::json::binary(large);

    auto cbor = nlohmann::json::to_cbor(msg);
    auto decoded = nlohmann::json::from_cbor(cbor);

    EXPECT_EQ(decoded["payload"].get_binary(), nlohmann::json::binary_t(large));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
