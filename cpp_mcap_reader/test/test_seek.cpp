// test/test_seek.cpp
#define MCAP_IMPLEMENTATION
#include <mcap/writer.hpp>
#include <mcap/reader.hpp>
#include <gtest/gtest.h>
#include <filesystem>
#include <limits>

TEST(McapSeek, ReadMessagesWithStartTime) {
    std::string path = "/tmp/test_seek.mcap";

    // Write test MCAP
    {
        mcap::McapWriter writer;
        auto opts = mcap::McapWriterOptions("test");
        (void)writer.open(path, opts);

        mcap::Schema schema("test_schema", "raw", "");
        writer.addSchema(schema);

        mcap::Channel channel("test/topic", "raw", schema.id);
        writer.addChannel(channel);

        for (int i = 0; i < 10; ++i) {
            mcap::Message msg;
            msg.channelId = channel.id;
            msg.logTime = (i + 1) * 1000000000ULL;  // 1s, 2s, ..., 10s
            msg.publishTime = msg.logTime;
            std::byte data = static_cast<std::byte>(i);
            msg.data = &data;
            msg.dataSize = 1;
            (void)writer.write(msg);
        }
        writer.close();
    }

    // Read from 5s onwards
    {
        mcap::McapReader reader;
        ASSERT_TRUE(reader.open(path).ok());

        mcap::ReadMessageOptions opts;
        opts.startTime = 5000000000ULL;
        opts.endTime = std::numeric_limits<uint64_t>::max();

        auto view = reader.readMessages([](const auto&) {}, opts);
        int count = 0;
        uint64_t first_ts = 0;
        for (auto it = view.begin(); it != view.end(); ++it) {
            if (count == 0) first_ts = it->message.logTime;
            count++;
        }

        EXPECT_GE(first_ts, 5000000000ULL);
        EXPECT_EQ(count, 6);  // 5s,6s,7s,8s,9s,10s

        reader.close();
    }

    std::filesystem::remove(path);
}
