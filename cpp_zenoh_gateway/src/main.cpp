#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "cpp_zenoh_gateway/gateway_server.hpp"

int main(int argc, char* argv[]) {
    zenoh_gateway::ServerConfig config;

    if (argc > 1) {
        std::ifstream f(argv[1]);
        if (f.is_open()) {
            auto j = nlohmann::json::parse(f);
            config.address = j.value("address", "0.0.0.0");
            config.port = j.value("port", 9090);
            config.io_threads = j.value("io_threads", 4);
            config.drain_interval_ms = j.value("drain_interval_ms", 5);
            for (auto& t : j["topics"]) {
                zenoh_gateway::TopicConfig tc;
                tc.zenoh_key_expr = t["zenoh_key_expr"];
                tc.display_name = t["display_name"];
                tc.topic_id = t["topic_id"];
                tc.stream_type = t["type"] == "image"
                    ? zenoh_gateway::StreamType::IMAGE
                    : zenoh_gateway::StreamType::POSE;
                config.topics.push_back(tc);
            }
        }
    } else {
        config.topics = {
            {"room/01/cam0",     "/cam0",     0, zenoh_gateway::StreamType::IMAGE},
            {"room/01/cam1",     "/cam1",     1, zenoh_gateway::StreamType::IMAGE},
            {"room/01/cam2",     "/cam2",     2, zenoh_gateway::StreamType::IMAGE},
            {"room/01/ee_pose0", "/ee_pose0", 3, zenoh_gateway::StreamType::POSE},
            {"room/01/ee_pose1", "/ee_pose1", 4, zenoh_gateway::StreamType::POSE},
            {"room/01/ee_pose2", "/ee_pose2", 5, zenoh_gateway::StreamType::POSE},
        };
    }

    std::cout << "[gateway] Starting with " << config.topics.size()
              << " topics on port " << config.port << std::endl;

    zenoh_gateway::GatewayServer server(config);
    server.run();

    return 0;
}
