#include <iostream>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <zenoh.hxx>

#include "cpp_mcap_reader/mcap_publisher.hpp"
#include "cpp_mcap_reader/playback_controller.hpp"

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: mcap_reader_node <config.json | mcap_file.mcap>"
                  << std::endl;
        return 1;
    }

    mcap_reader::ReaderConfig config;

    std::string arg1 = argv[1];
    if (arg1.size() > 5 && arg1.substr(arg1.size() - 5) == ".json") {
        std::ifstream f(arg1);
        auto j = nlohmann::json::parse(f);
        config.mcap_path = j["mcap_path"];
        config.room_id = j.value("room_id", "01");
        config.playback_speed = j.value("speed", 1.0);
        for (auto& m : j["mappings"]) {
            config.mappings.push_back({
                m["mcap_topic"],
                m["zenoh_key_expr"],
                m["message_type"]
            });
        }
    } else {
        config.mcap_path = arg1;
        config.room_id = argc > 2 ? argv[2] : "01";
        std::string prefix = "room/" + config.room_id;
        config.mappings = {
            {"/cam0/image_raw/compressed", prefix + "/cam0", "sensor_msgs/msg/CompressedImage"},
            {"/cam1/image_raw/compressed", prefix + "/cam1", "sensor_msgs/msg/CompressedImage"},
            {"/cam2/image_raw/compressed", prefix + "/cam2", "sensor_msgs/msg/CompressedImage"},
            {"/ee_pose_0", prefix + "/ee_pose0", "geometry_msgs/msg/Pose"},
            {"/ee_pose_1", prefix + "/ee_pose1", "geometry_msgs/msg/Pose"},
            {"/ee_pose_2", prefix + "/ee_pose2", "geometry_msgs/msg/Pose"},
        };
    }

    std::cout << "[mcap-reader] File: " << config.mcap_path << std::endl;
    std::cout << "[mcap-reader] Room: " << config.room_id << std::endl;
    std::cout << "[mcap-reader] Topics: " << config.mappings.size() << std::endl;

    auto zenoh_config = zenoh::Config::create_default();
    auto session = std::make_shared<zenoh::Session>(
        zenoh::Session::open(std::move(zenoh_config)));

    auto publisher = std::make_shared<mcap_reader::McapPublisher>(config, session);
    mcap_reader::PlaybackController controller(session, publisher, config.room_id);

    publisher->play();

    return 0;
}
