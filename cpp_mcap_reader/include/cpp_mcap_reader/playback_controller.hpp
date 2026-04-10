#pragma once

#include <memory>
#include <string>
#include <zenoh.hxx>
#include "cpp_mcap_reader/mcap_publisher.hpp"

namespace mcap_reader {

class PlaybackController {
public:
    PlaybackController(std::shared_ptr<zenoh::Session> session,
                       std::shared_ptr<McapPublisher> publisher,
                       const std::string& room_id);

private:
    void handle_query(const zenoh::Query& query);

    std::shared_ptr<McapPublisher> publisher_;
    zenoh::Queryable queryable_;
};

}  // namespace mcap_reader
