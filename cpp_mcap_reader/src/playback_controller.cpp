#include "cpp_mcap_reader/playback_controller.hpp"
#include <nlohmann/json.hpp>
#include <iostream>

namespace mcap_reader {

PlaybackController::PlaybackController(
    std::shared_ptr<zenoh::Session> session,
    std::shared_ptr<McapPublisher> publisher,
    const std::string& room_id)
    : publisher_(std::move(publisher))
    , queryable_(session->declare_queryable(
          zenoh::KeyExpr("room/" + room_id + "/control"),
          [this](const zenoh::Query& query) { handle_query(query); }))
{
}

void PlaybackController::handle_query(const zenoh::Query& query) {
    try {
        auto payload = query.get_payload();
        auto body = nlohmann::json::parse(payload.as_string());
        auto action = body.value("action", "");

        nlohmann::json response;

        if (action == "play") {
            publisher_->resume();
            response = {{"status", "playing"}};
        } else if (action == "pause") {
            publisher_->pause();
            response = {{"status", "paused"}};
        } else if (action == "seek") {
            uint64_t time_ns = body.value("time_ns", 0ULL);
            publisher_->seek(time_ns);
            response = {{"status", "seeking"}, {"time_ns", time_ns}};
        } else if (action == "set_speed") {
            double speed = body.value("speed", 1.0);
            publisher_->set_speed(speed);
            response = {{"status", "speed_set"}, {"speed", speed}};
        } else if (action == "status") {
            response = {
                {"status", publisher_->is_playing() ? "playing" : "paused"},
                {"current_ns", publisher_->current_timestamp()},
                {"duration_ns", publisher_->duration_ns()}
            };
        } else {
            response = {{"error", "unknown action: " + action}};
        }

        auto reply_str = response.dump();
        query.reply(query.get_keyexpr(), zenoh::Bytes(reply_str));

    } catch (const std::exception& e) {
        std::cerr << "[playback-ctrl] Error: " << e.what() << std::endl;
        auto err = nlohmann::json{{"error", e.what()}}.dump();
        query.reply(query.get_keyexpr(), zenoh::Bytes(err));
    }
}

}  // namespace mcap_reader
