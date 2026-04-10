#include "cpp_zenoh_gateway/gateway_session.hpp"
#include <nlohmann/json.hpp>

namespace zenoh_gateway {

// ---- WsTransport ----

WsTransport::WsTransport(tcp::socket&& socket)
    : ws_(std::move(socket)) {}

void WsTransport::run(const std::string& expected_path) {
    expected_path_ = expected_path;
    ws_.set_option(websocket::stream_base::timeout::suggested(
        beast::role_type::server));
    ws_.set_option(websocket::stream_base::decorator(
        [](websocket::response_type& res) {
            res.set(boost::beast::http::field::server, "zenoh-ws-gateway");
        }));
    ws_.async_accept(
        beast::bind_front_handler(&WsTransport::on_accept, shared_from_this()));
}

void WsTransport::set_on_message(OnMessage cb) { on_message_ = std::move(cb); }
void WsTransport::set_on_close(OnClose cb) { on_close_ = std::move(cb); }

void WsTransport::on_accept(beast::error_code ec) {
    if (ec) { if (on_close_) on_close_(); return; }
    do_read();
}

void WsTransport::do_read() {
    ws_.async_read(buffer_,
        beast::bind_front_handler(&WsTransport::on_read, shared_from_this()));
}

void WsTransport::on_read(beast::error_code ec, std::size_t /*bytes*/) {
    if (ec) { if (on_close_) on_close_(); return; }
    if (on_message_) {
        auto data = beast::buffers_to_string(buffer_.data());
        on_message_(data, ws_.got_binary());
    }
    buffer_.consume(buffer_.size());
    do_read();
}

void WsTransport::send_text(std::string message) {
    net::post(ws_.get_executor(),
        [self = shared_from_this(), msg = std::move(message)]() mutable {
            self->write_queue_.push_back(QueueItem{false, std::move(msg), {}});
            if (self->write_queue_.size() > kMaxControlQueueSize) {
                self->write_queue_.pop_front();
            }
            if (!self->writing_) self->do_write();
        });
}

void WsTransport::send_binary(std::vector<uint8_t> data) {
    net::post(ws_.get_executor(),
        [self = shared_from_this(), d = std::move(data)]() mutable {
            self->write_queue_.push_back(QueueItem{true, {}, std::move(d)});
            if (self->write_queue_.size() > kMaxControlQueueSize) {
                self->write_queue_.pop_front();
            }
            if (!self->writing_) self->do_write();
        });
}

void WsTransport::close() {
    net::post(ws_.get_executor(),
        [self = shared_from_this()]() {
            beast::error_code ec;
            self->ws_.close(websocket::close_code::normal, ec);
        });
}

void WsTransport::do_write() {
    if (write_queue_.empty()) { writing_ = false; return; }
    writing_ = true;
    auto& item = write_queue_.front();
    ws_.binary(item.is_binary);
    if (item.is_binary) {
        ws_.async_write(net::buffer(item.binary),
            beast::bind_front_handler(&WsTransport::on_write, shared_from_this()));
    } else {
        ws_.async_write(net::buffer(item.text),
            beast::bind_front_handler(&WsTransport::on_write, shared_from_this()));
    }
}

void WsTransport::on_write(beast::error_code ec, std::size_t /*bytes*/) {
    if (ec) { if (on_close_) on_close_(); return; }
    write_queue_.pop_front();
    do_write();
}

// ---- GatewaySession ----

GatewaySession::GatewaySession(uint64_t id) : id_(id) {}

void GatewaySession::set_control_transport(std::shared_ptr<WsTransport> t) {
    control_ = std::move(t);
    control_->set_on_message([this](const std::string& msg, bool /*is_binary*/) {
        if (on_control_) on_control_(id_, msg);
    });
    control_->set_on_close([this]() {
        alive_ = false;
        if (on_disconnect_) on_disconnect_(id_);
    });
}

void GatewaySession::set_image_transport(std::shared_ptr<WsTransport> t) {
    image_ = std::move(t);
    image_->set_on_close([this]() {
        alive_ = false;
        if (on_disconnect_) on_disconnect_(id_);
    });
}

void GatewaySession::send_control(const std::string& json) {
    if (control_) control_->send_text(json);
}

std::shared_ptr<StreamLane> GatewaySession::get_or_create_lane(
    uint8_t topic_id, BackpressurePolicy policy,
    size_t max_depth, size_t max_bytes)
{
    std::lock_guard lock(lanes_mutex_);
    auto it = lanes_.find(topic_id);
    if (it != lanes_.end()) return it->second;
    auto lane = std::make_shared<StreamLane>(policy, max_depth, max_bytes);
    lanes_[topic_id] = lane;
    return lane;
}

void GatewaySession::drain_lanes() {
    if (!alive_) return;

    std::vector<std::pair<uint8_t, std::shared_ptr<StreamLane>>> snapshot;
    {
        std::lock_guard lock(lanes_mutex_);
        snapshot.reserve(lanes_.size());
        for (auto& [id, lane] : lanes_) {
            snapshot.emplace_back(id, lane);
        }
    }

    for (auto& [topic_id, lane] : snapshot) {
        auto frame = lane->pop();
        if (!frame.has_value()) continue;

        if (frame->stream_type == StreamType::IMAGE && image_) {
            image_->send_binary(std::vector<uint8_t>(
                frame->bytes->begin(), frame->bytes->end()));
        } else if (control_) {
            control_->send_binary(std::vector<uint8_t>(
                frame->bytes->begin(), frame->bytes->end()));
        }
    }
}

bool GatewaySession::is_alive() const { return alive_; }

void GatewaySession::close() {
    alive_ = false;
    if (control_) control_->close();
    if (image_) image_->close();
}

void GatewaySession::set_on_control(OnControl cb) { on_control_ = std::move(cb); }
void GatewaySession::set_on_disconnect(OnDisconnect cb) { on_disconnect_ = std::move(cb); }

}  // namespace zenoh_gateway
