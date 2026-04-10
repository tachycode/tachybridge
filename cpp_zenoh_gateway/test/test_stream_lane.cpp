#include <gtest/gtest.h>
#include <thread>
#include "cpp_zenoh_gateway/stream_lane.hpp"
#include "cpp_zenoh_gateway/wire_format.hpp"

using namespace zenoh_gateway;

static SharedFrame make_frame(uint8_t id, uint32_t ts) {
    auto bytes = std::make_shared<const std::vector<uint8_t>>(
        std::vector<uint8_t>{0xAA, 0xBB});
    return SharedFrame{StreamType::IMAGE, id, ts, bytes};
}

TEST(StreamLane, LatestOnlyKeepsNewest) {
    StreamLane lane(BackpressurePolicy::LATEST_ONLY, 1, 1024*1024);

    lane.push(make_frame(0, 100));
    lane.push(make_frame(0, 200));
    lane.push(make_frame(0, 300));

    EXPECT_EQ(lane.depth(), 1u);
    auto f = lane.pop();
    ASSERT_TRUE(f.has_value());
    EXPECT_EQ(f->ts_offset_ms, 300u);
    EXPECT_FALSE(lane.pop().has_value());
}

TEST(StreamLane, DropOldestMaintainsDepth) {
    StreamLane lane(BackpressurePolicy::DROP_OLDEST, 3, 1024*1024);

    lane.push(make_frame(0, 1));
    lane.push(make_frame(0, 2));
    lane.push(make_frame(0, 3));
    lane.push(make_frame(0, 4));

    EXPECT_EQ(lane.depth(), 3u);
    auto f = lane.pop();
    EXPECT_EQ(f->ts_offset_ms, 2u);
}

TEST(StreamLane, ByteBudgetEnforced) {
    StreamLane lane(BackpressurePolicy::DROP_OLDEST, 100, 5);

    lane.push(make_frame(0, 1));  // 2 bytes
    lane.push(make_frame(0, 2));  // 2 bytes = 4 total
    lane.push(make_frame(0, 3));  // 2 bytes = 6 > 5 → evict oldest

    EXPECT_LE(lane.byte_usage(), 5u);
}

TEST(StreamLane, EmptyPopReturnsNullopt) {
    StreamLane lane(BackpressurePolicy::DROP_OLDEST, 64, 1024*1024);
    EXPECT_FALSE(lane.pop().has_value());
    EXPECT_EQ(lane.depth(), 0u);
    EXPECT_TRUE(lane.empty());
}

TEST(StreamLane, TracksDropCount) {
    StreamLane lane(BackpressurePolicy::DROP_OLDEST, 2, 1024*1024);

    // Fill beyond capacity
    lane.push(make_frame(0, 1));
    lane.push(make_frame(0, 2));
    lane.push(make_frame(0, 3));  // drops frame 1
    lane.push(make_frame(0, 4));  // drops frame 2

    EXPECT_EQ(lane.total_drops(), 2u);
    EXPECT_EQ(lane.drops_since_last_check(), 2u);
    // Second call returns 0 (reset)
    EXPECT_EQ(lane.drops_since_last_check(), 0u);
}

TEST(StreamLane, LatestOnlyCountsDrops) {
    StreamLane lane(BackpressurePolicy::LATEST_ONLY, 1, 1024*1024);

    lane.push(make_frame(0, 1));
    lane.push(make_frame(0, 2));  // drops frame 1
    lane.push(make_frame(0, 3));  // drops frame 2

    EXPECT_EQ(lane.total_drops(), 2u);
}

TEST(StreamLane, ThreadSafety) {
    StreamLane lane(BackpressurePolicy::DROP_OLDEST, 64, 1024*1024);

    std::thread producer([&]() {
        for (uint32_t i = 0; i < 1000; ++i) {
            lane.push(make_frame(0, i));
        }
    });

    std::thread consumer([&]() {
        size_t consumed = 0;
        while (consumed < 500) {
            if (lane.pop().has_value()) consumed++;
        }
    });

    producer.join();
    consumer.join();
    EXPECT_LE(lane.depth(), 64u);
}
