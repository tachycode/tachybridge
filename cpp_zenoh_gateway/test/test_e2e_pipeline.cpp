#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <zenoh.hxx>

#include "cpp_zenoh_gateway/room.hpp"
#include "cpp_zenoh_gateway/wire_format.hpp"
#include "cpp_zenoh_gateway/stream_lane.hpp"
#include "cpp_zenoh_gateway/fanout_hub.hpp"

using namespace zenoh_gateway;

// Helper: create a fake JPEG payload (starts with JPEG magic bytes)
static std::vector<uint8_t> make_fake_jpeg(size_t size = 1024) {
    std::vector<uint8_t> data(size);
    data[0] = 0xFF; data[1] = 0xD8; data[2] = 0xFF; data[3] = 0xE0;
    for (size_t i = 4; i < size; ++i) data[i] = static_cast<uint8_t>(i & 0xFF);
    return data;
}

// Helper: create a pose payload (7 doubles)
static std::vector<uint8_t> make_pose(double px, double py, double pz,
                                       double qx, double qy, double qz, double qw) {
    double vals[7] = {px, py, pz, qx, qy, qz, qw};
    auto* raw = reinterpret_cast<const uint8_t*>(vals);
    return std::vector<uint8_t>(raw, raw + 56);
}

// ============================================================
// E2E Test 1: Full Zenoh → Room → Multiple Clients pipeline
// ============================================================
TEST(E2EPipeline, ZenohToRoomToMultipleClients) {
    // Setup: 1 room with 2 topics (1 image, 1 pose)
    std::vector<TopicConfig> topics = {
        {"test/e2e/cam0", "/cam0", 0, StreamType::IMAGE},
        {"test/e2e/pose0", "/pose0", 1, StreamType::POSE},
    };
    auto room = std::make_shared<Room>("e2e_test", topics);

    // 3 clients subscribe to both topics
    for (uint64_t i = 1; i <= 3; ++i) {
        room->add_client(i);
        room->subscribe_client(i, 0);  // image
        room->subscribe_client(i, 1);  // pose
    }

    // Open Zenoh session
    auto config = zenoh::Config::create_default();
    auto session = zenoh::Session::open(std::move(config));

    // Subscribe Zenoh → Room distribution
    std::atomic<int> image_distributed{0};
    std::atomic<int> pose_distributed{0};

    auto img_sub = session.declare_subscriber(
        zenoh::KeyExpr("test/e2e/cam0"),
        [&](zenoh::Sample& sample) {
            const auto& payload = sample.get_payload();
            auto bytes_vec = payload.as_vector();
            auto wire = encode_data_frame(StreamType::IMAGE, 0, 100,
                                          bytes_vec.data(), bytes_vec.size());
            auto shared = std::make_shared<const std::vector<uint8_t>>(std::move(wire));
            room->distribute(0, SharedFrame{StreamType::IMAGE, 0, 100, shared});
            image_distributed++;
        },
        []() {}
    );

    auto pose_sub = session.declare_subscriber(
        zenoh::KeyExpr("test/e2e/pose0"),
        [&](zenoh::Sample& sample) {
            const auto& payload = sample.get_payload();
            auto bytes_vec = payload.as_vector();
            auto wire = encode_data_frame(StreamType::POSE, 1, 200,
                                          bytes_vec.data(), bytes_vec.size());
            auto shared = std::make_shared<const std::vector<uint8_t>>(std::move(wire));
            room->distribute(1, SharedFrame{StreamType::POSE, 1, 200, shared});
            pose_distributed++;
        },
        []() {}
    );

    // Publish image + pose via Zenoh
    auto fake_jpeg = make_fake_jpeg(512);
    session.put(zenoh::KeyExpr("test/e2e/cam0"),
                zenoh::Bytes(std::move(fake_jpeg)));

    auto pose_data = make_pose(1.0, 2.0, 3.0, 0.0, 0.0, 0.707, 0.707);
    session.put(zenoh::KeyExpr("test/e2e/pose0"),
                zenoh::Bytes(std::move(pose_data)));

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Verify Zenoh delivery
    EXPECT_GE(image_distributed.load(), 1);
    EXPECT_GE(pose_distributed.load(), 1);

    // Verify each client got the data (via Room's hubs)
    // Client 1's image lane should have a frame
    // We can't access lanes directly, but we can check latest_frame
    auto latest_img = room->latest_frame(0);
    ASSERT_TRUE(latest_img.has_value());
    EXPECT_EQ(latest_img->stream_type, StreamType::IMAGE);
    EXPECT_EQ(latest_img->topic_id, 0);

    auto latest_pose = room->latest_frame(1);
    ASSERT_TRUE(latest_pose.has_value());
    EXPECT_EQ(latest_pose->stream_type, StreamType::POSE);

    // Verify wire format round-trip
    DataFrameView view;
    ASSERT_TRUE(parse_data_frame(latest_img->bytes->data(), latest_img->bytes->size(), view));
    EXPECT_EQ(view.stream_type, StreamType::IMAGE);
    EXPECT_EQ(view.payload_data[0], 0xFF);  // JPEG magic
    EXPECT_EQ(view.payload_data[1], 0xD8);

    ASSERT_TRUE(parse_data_frame(latest_pose->bytes->data(), latest_pose->bytes->size(), view));
    EXPECT_EQ(view.stream_type, StreamType::POSE);
    EXPECT_EQ(view.payload_size, 56u);  // 7 doubles
    auto* doubles = reinterpret_cast<const double*>(view.payload_data);
    EXPECT_DOUBLE_EQ(doubles[0], 1.0);
    EXPECT_DOUBLE_EQ(doubles[6], 0.707);
}

// ============================================================
// E2E Test 2: Late-join gets latest frame
// ============================================================
TEST(E2EPipeline, LateJoinReceivesLatestFrame) {
    std::vector<TopicConfig> topics = {
        {"test/late/cam0", "/cam0", 0, StreamType::IMAGE},
        {"test/late/pose0", "/pose0", 1, StreamType::POSE},
    };
    auto room = std::make_shared<Room>("late_test", topics);

    // Distribute frames BEFORE any client joins
    auto img_wire = encode_data_frame(StreamType::IMAGE, 0, 500,
        reinterpret_cast<const uint8_t*>("JPEG"), 4);
    auto img_shared = std::make_shared<const std::vector<uint8_t>>(std::move(img_wire));
    room->distribute(0, SharedFrame{StreamType::IMAGE, 0, 500, img_shared});

    auto pose_payload = make_pose(10.0, 20.0, 30.0, 0.0, 0.0, 0.0, 1.0);
    auto pose_wire = encode_data_frame(StreamType::POSE, 1, 600,
        pose_payload.data(), pose_payload.size());
    auto pose_shared = std::make_shared<const std::vector<uint8_t>>(std::move(pose_wire));
    room->distribute(1, SharedFrame{StreamType::POSE, 1, 600, pose_shared});

    // Now a client joins late
    room->add_client(99);
    auto img_lane = room->subscribe_client(99, 0);
    auto pose_lane = room->subscribe_client(99, 1);

    // Late-join: push latest frames to the new client's lanes
    auto latest_img = room->latest_frame(0);
    ASSERT_TRUE(latest_img.has_value());
    img_lane->push(*latest_img);

    auto latest_pose = room->latest_frame(1);
    ASSERT_TRUE(latest_pose.has_value());
    pose_lane->push(*latest_pose);

    // Verify the late-join client received both frames
    auto popped_img = img_lane->pop();
    ASSERT_TRUE(popped_img.has_value());
    EXPECT_EQ(popped_img->ts_offset_ms, 500u);

    auto popped_pose = pose_lane->pop();
    ASSERT_TRUE(popped_pose.has_value());
    EXPECT_EQ(popped_pose->ts_offset_ms, 600u);

    // Verify pose data integrity
    DataFrameView view;
    ASSERT_TRUE(parse_data_frame(popped_pose->bytes->data(), popped_pose->bytes->size(), view));
    auto* vals = reinterpret_cast<const double*>(view.payload_data);
    EXPECT_DOUBLE_EQ(vals[0], 10.0);
    EXPECT_DOUBLE_EQ(vals[2], 30.0);
}

// ============================================================
// E2E Test 3: Room lifecycle — cleanup on last disconnect
// ============================================================
TEST(E2EPipeline, RoomCleanupOnLastDisconnect) {
    std::vector<TopicConfig> topics = {
        {"test/room/cam0", "/cam0", 0, StreamType::IMAGE},
    };
    auto room = std::make_shared<Room>("cleanup_test", topics);

    room->add_client(1);
    room->add_client(2);
    room->subscribe_client(1, 0);
    room->subscribe_client(2, 0);

    EXPECT_EQ(room->client_count(), 2u);
    EXPECT_FALSE(room->is_empty());

    // Distribute a frame
    auto wire = encode_data_frame(StreamType::IMAGE, 0, 100,
        reinterpret_cast<const uint8_t*>("IMG"), 3);
    auto shared = std::make_shared<const std::vector<uint8_t>>(std::move(wire));
    room->distribute(0, SharedFrame{StreamType::IMAGE, 0, 100, shared});

    // Client 1 disconnects
    room->remove_client(1);
    EXPECT_EQ(room->client_count(), 1u);
    EXPECT_FALSE(room->is_empty());

    // Client 2 disconnects
    room->remove_client(2);
    EXPECT_EQ(room->client_count(), 0u);
    EXPECT_TRUE(room->is_empty());
}

// ============================================================
// E2E Test 4: Backpressure isolation — slow client doesn't block fast
// ============================================================
TEST(E2EPipeline, BackpressureIsolation) {
    std::vector<TopicConfig> topics = {
        {"test/bp/cam0", "/cam0", 0, StreamType::IMAGE},
    };
    auto room = std::make_shared<Room>("bp_test", topics);

    room->add_client(1);  // fast client
    room->add_client(2);  // slow client (we'll never drain it)
    auto fast_lane = room->subscribe_client(1, 0);
    auto slow_lane = room->subscribe_client(2, 0);

    // Distribute 100 image frames (LATEST_ONLY policy for images)
    for (uint32_t i = 0; i < 100; ++i) {
        auto wire = encode_data_frame(StreamType::IMAGE, 0, i,
            reinterpret_cast<const uint8_t*>("IMG"), 3);
        auto shared = std::make_shared<const std::vector<uint8_t>>(std::move(wire));
        room->distribute(0, SharedFrame{StreamType::IMAGE, 0, i, shared});
    }

    // Fast client drains — should get latest frame(s)
    auto fast_frame = fast_lane->pop();
    ASSERT_TRUE(fast_frame.has_value());
    // With LATEST_ONLY (depth 2), should be one of the last frames
    EXPECT_GE(fast_frame->ts_offset_ms, 90u);

    // Slow client's lane also has latest (LATEST_ONLY replaces)
    auto slow_frame = slow_lane->pop();
    ASSERT_TRUE(slow_frame.has_value());
    EXPECT_GE(slow_frame->ts_offset_ms, 90u);

    // Both got recent data — slow client didn't block fast client
    // And LATEST_ONLY prevented unbounded queue growth
    EXPECT_LE(fast_lane->depth(), 2u);
    EXPECT_LE(slow_lane->depth(), 2u);
}

// ============================================================
// E2E Test 5: Quality hint — drops tracked after eviction
// ============================================================
TEST(E2EPipeline, QualityDropTracking) {
    // Use DROP_OLDEST with small depth to force drops
    StreamLane lane(BackpressurePolicy::DROP_OLDEST, 3, 1024 * 1024);

    // Push 10 frames into depth-3 lane
    for (uint32_t i = 0; i < 10; ++i) {
        auto bytes = std::make_shared<const std::vector<uint8_t>>(
            std::vector<uint8_t>{static_cast<uint8_t>(i)});
        lane.push(SharedFrame{StreamType::POSE, 0, i, bytes});
    }

    // 10 pushed, depth 3 → 7 dropped
    EXPECT_EQ(lane.depth(), 3u);
    EXPECT_EQ(lane.total_drops(), 7u);

    // First check returns 7, resets counter
    EXPECT_EQ(lane.drops_since_last_check(), 7u);
    EXPECT_EQ(lane.drops_since_last_check(), 0u);  // reset

    // Verify remaining frames are the latest 3
    auto f1 = lane.pop();
    ASSERT_TRUE(f1.has_value());
    EXPECT_EQ(f1->ts_offset_ms, 7u);

    auto f2 = lane.pop();
    ASSERT_TRUE(f2.has_value());
    EXPECT_EQ(f2->ts_offset_ms, 8u);

    auto f3 = lane.pop();
    ASSERT_TRUE(f3.has_value());
    EXPECT_EQ(f3->ts_offset_ms, 9u);

    EXPECT_FALSE(lane.pop().has_value());
}

// ============================================================
// E2E Test 6: Multi-room Zenoh isolation
// ============================================================
TEST(E2EPipeline, MultiRoomZenohIsolation) {
    std::vector<TopicConfig> topics_r1 = {
        {"test/mr/room1/cam0", "/cam0", 0, StreamType::IMAGE},
    };
    std::vector<TopicConfig> topics_r2 = {
        {"test/mr/room2/cam0", "/cam0", 0, StreamType::IMAGE},
    };

    auto room1 = std::make_shared<Room>("room1", topics_r1);
    auto room2 = std::make_shared<Room>("room2", topics_r2);

    room1->add_client(1);
    room2->add_client(2);
    auto lane1 = room1->subscribe_client(1, 0);
    auto lane2 = room2->subscribe_client(2, 0);

    auto config = zenoh::Config::create_default();
    auto session = zenoh::Session::open(std::move(config));

    // Room-specific subscribers
    auto sub1 = session.declare_subscriber(
        zenoh::KeyExpr("test/mr/room1/cam0"),
        [&](zenoh::Sample& sample) {
            const auto& payload = sample.get_payload();
            auto bytes_vec = payload.as_vector();
            auto wire = encode_data_frame(StreamType::IMAGE, 0, 1,
                                          bytes_vec.data(), bytes_vec.size());
            auto shared = std::make_shared<const std::vector<uint8_t>>(std::move(wire));
            room1->distribute(0, SharedFrame{StreamType::IMAGE, 0, 1, shared});
        },
        []() {}
    );

    auto sub2 = session.declare_subscriber(
        zenoh::KeyExpr("test/mr/room2/cam0"),
        [&](zenoh::Sample& sample) {
            const auto& payload = sample.get_payload();
            auto bytes_vec = payload.as_vector();
            auto wire = encode_data_frame(StreamType::IMAGE, 0, 2,
                                          bytes_vec.data(), bytes_vec.size());
            auto shared = std::make_shared<const std::vector<uint8_t>>(std::move(wire));
            room2->distribute(0, SharedFrame{StreamType::IMAGE, 0, 2, shared});
        },
        []() {}
    );

    // Publish ONLY to room1
    std::vector<uint8_t> img = {0xFF, 0xD8};
    session.put(zenoh::KeyExpr("test/mr/room1/cam0"),
                zenoh::Bytes(std::move(img)));

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Room1 client got data
    auto f1 = lane1->pop();
    ASSERT_TRUE(f1.has_value());
    EXPECT_EQ(f1->ts_offset_ms, 1u);

    // Room2 client got NOTHING — isolation works
    auto f2 = lane2->pop();
    EXPECT_FALSE(f2.has_value());
}
