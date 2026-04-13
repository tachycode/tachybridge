#include <gtest/gtest.h>
#include "cpp_zenoh_gateway/room.hpp"

using namespace zenoh_gateway;

TEST(Room, CreateWithTopics) {
    std::vector<TopicConfig> topics = {
        {"room/01/cam0", "/cam0", 0, StreamType::IMAGE},
        {"room/01/ee_pose0", "/ee_pose0", 3, StreamType::POSE},
    };
    Room room("01", topics);

    EXPECT_EQ(room.id(), "01");
    EXPECT_EQ(room.hub_count(), 2u);
    EXPECT_TRUE(room.has_hub(0));
    EXPECT_TRUE(room.has_hub(3));
    EXPECT_FALSE(room.has_hub(1));
}

TEST(Room, AddAndRemoveClient) {
    std::vector<TopicConfig> topics = {
        {"room/01/cam0", "/cam0", 0, StreamType::IMAGE},
    };
    Room room("01", topics);

    EXPECT_EQ(room.client_count(), 0u);
    room.add_client(1);
    EXPECT_EQ(room.client_count(), 1u);
    room.add_client(2);
    EXPECT_EQ(room.client_count(), 2u);
    room.remove_client(1);
    EXPECT_EQ(room.client_count(), 1u);
    room.remove_client(2);
    EXPECT_EQ(room.client_count(), 0u);
    EXPECT_TRUE(room.is_empty());
}

TEST(Room, SubscribeClientToTopic) {
    std::vector<TopicConfig> topics = {
        {"room/01/cam0", "/cam0", 0, StreamType::IMAGE},
    };
    Room room("01", topics);
    room.add_client(1);

    auto lane = room.subscribe_client(1, 0);
    ASSERT_NE(lane, nullptr);

    auto bytes = std::make_shared<const std::vector<uint8_t>>(
        std::vector<uint8_t>{0xAA});
    SharedFrame frame{StreamType::IMAGE, 0, 100, bytes};
    room.distribute(0, frame);

    auto popped = lane->pop();
    ASSERT_TRUE(popped.has_value());
    EXPECT_EQ(popped->ts_offset_ms, 100u);
}

TEST(Room, UnsubscribeOnRemoveClient) {
    std::vector<TopicConfig> topics = {
        {"room/01/cam0", "/cam0", 0, StreamType::IMAGE},
    };
    Room room("01", topics);
    room.add_client(1);
    auto lane = room.subscribe_client(1, 0);
    room.remove_client(1);

    auto bytes = std::make_shared<const std::vector<uint8_t>>(
        std::vector<uint8_t>{0xBB});
    room.distribute(0, SharedFrame{StreamType::IMAGE, 0, 200, bytes});
    EXPECT_FALSE(lane->pop().has_value());
}

TEST(Room, LatestFrameForLateJoin) {
    std::vector<TopicConfig> topics = {
        {"room/01/cam0", "/cam0", 0, StreamType::IMAGE},
    };
    Room room("01", topics);

    auto bytes = std::make_shared<const std::vector<uint8_t>>(
        std::vector<uint8_t>{0xCC});
    SharedFrame frame{StreamType::IMAGE, 0, 300, bytes};
    room.distribute(0, frame);

    auto latest = room.latest_frame(0);
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(latest->ts_offset_ms, 300u);
}
