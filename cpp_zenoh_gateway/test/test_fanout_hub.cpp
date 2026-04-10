#include <gtest/gtest.h>
#include "cpp_zenoh_gateway/fanout_hub.hpp"

using namespace zenoh_gateway;

static SharedFrame make_test_frame(uint8_t topic_id, uint32_t ts) {
    auto bytes = std::make_shared<const std::vector<uint8_t>>(
        encode_data_frame(StreamType::IMAGE, topic_id, ts,
                          reinterpret_cast<const uint8_t*>("JPEG"), 4));
    return SharedFrame{StreamType::IMAGE, topic_id, ts, bytes};
}

TEST(FanoutHub, DistributeToMultipleClients) {
    FanoutHub hub;

    auto lane_a = std::make_shared<StreamLane>(
        BackpressurePolicy::LATEST_ONLY, 1, 1024*1024);
    auto lane_b = std::make_shared<StreamLane>(
        BackpressurePolicy::LATEST_ONLY, 1, 1024*1024);

    hub.add_client(1, lane_a);
    hub.add_client(2, lane_b);

    hub.distribute(make_test_frame(0, 100));

    auto fa = lane_a->pop();
    auto fb = lane_b->pop();
    ASSERT_TRUE(fa.has_value());
    ASSERT_TRUE(fb.has_value());
    EXPECT_EQ(fa->ts_offset_ms, 100u);
    EXPECT_EQ(fb->ts_offset_ms, 100u);
    EXPECT_EQ(fa->bytes.get(), fb->bytes.get());  // same pointer
}

TEST(FanoutHub, RemoveClient) {
    FanoutHub hub;
    auto lane = std::make_shared<StreamLane>(
        BackpressurePolicy::DROP_OLDEST, 64, 1024*1024);
    hub.add_client(1, lane);
    hub.remove_client(1);

    hub.distribute(make_test_frame(0, 200));
    EXPECT_TRUE(lane->empty());
}

TEST(FanoutHub, ClientCount) {
    FanoutHub hub;
    EXPECT_EQ(hub.client_count(), 0u);

    auto lane = std::make_shared<StreamLane>(
        BackpressurePolicy::LATEST_ONLY, 1, 1024*1024);
    hub.add_client(1, lane);
    EXPECT_EQ(hub.client_count(), 1u);

    hub.remove_client(1);
    EXPECT_EQ(hub.client_count(), 0u);
}

TEST(FanoutHub, SlowClientDoesNotBlockFast) {
    FanoutHub hub;

    auto slow_lane = std::make_shared<StreamLane>(
        BackpressurePolicy::DROP_OLDEST, 2, 100);
    auto fast_lane = std::make_shared<StreamLane>(
        BackpressurePolicy::DROP_OLDEST, 64, 1024*1024);

    hub.add_client(1, slow_lane);
    hub.add_client(2, fast_lane);

    for (uint32_t i = 0; i < 100; ++i) {
        hub.distribute(make_test_frame(0, i));
    }

    EXPECT_EQ(fast_lane->depth(), 64u);
    EXPECT_LE(slow_lane->depth(), 2u);
}
