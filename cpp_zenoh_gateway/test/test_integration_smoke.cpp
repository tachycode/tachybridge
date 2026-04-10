#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <zenoh.hxx>

#include "cpp_zenoh_gateway/wire_format.hpp"
#include "cpp_zenoh_gateway/stream_lane.hpp"
#include "cpp_zenoh_gateway/fanout_hub.hpp"

using namespace zenoh_gateway;

TEST(IntegrationSmoke, ZenohToFanoutHub) {
    // Create Zenoh session (in-process, no router)
    auto config = zenoh::Config::create_default();
    auto session = zenoh::Session::open(std::move(config));

    auto hub = std::make_shared<FanoutHub>();
    auto lane = std::make_shared<StreamLane>(
        BackpressurePolicy::LATEST_ONLY, 2, 1024*1024);
    hub->add_client(1, lane);

    std::atomic<int> received{0};

    // Subscribe with Zenoh
    auto sub = session.declare_subscriber(
        zenoh::KeyExpr("test/cam0"),
        [&](const zenoh::Sample& sample) {
            auto payload = sample.get_payload().as_vector();
            auto wire = encode_data_frame(
                StreamType::IMAGE, 0, 100,
                payload.data(), payload.size());
            auto shared = std::make_shared<const std::vector<uint8_t>>(
                std::move(wire));
            hub->distribute(SharedFrame{StreamType::IMAGE, 0, 100, shared});
            received++;
        });

    // Publish fake JPEG
    std::vector<uint8_t> fake_jpeg = {0xFF, 0xD8, 0xFF, 0xE0};
    session.put(zenoh::KeyExpr("test/cam0"),
                zenoh::Bytes(fake_jpeg.data(), fake_jpeg.size()));

    // Wait for delivery
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    EXPECT_GE(received.load(), 1);

    auto frame = lane->pop();
    ASSERT_TRUE(frame.has_value());
    EXPECT_EQ(frame->stream_type, StreamType::IMAGE);
    EXPECT_EQ(frame->topic_id, 0);

    // Verify payload contains our JPEG bytes
    DataFrameView view;
    ASSERT_TRUE(parse_data_frame(
        frame->bytes->data(), frame->bytes->size(), view));
    EXPECT_EQ(view.payload_size, 4u);
    EXPECT_EQ(view.payload_data[0], 0xFF);
    EXPECT_EQ(view.payload_data[1], 0xD8);
}
