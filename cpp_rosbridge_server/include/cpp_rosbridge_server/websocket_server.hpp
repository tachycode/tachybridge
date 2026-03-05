#pragma once

#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <atomic>
#include <cstdlib>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include <rclcpp/logging.hpp>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

class Session;

using MessageCallback = std::function<void(
    std::shared_ptr<Session> session,
    const std::string& message,
    bool is_binary)>;

class Session : public std::enable_shared_from_this<Session> {
    websocket::stream<beast::tcp_stream> ws_;
    beast::flat_buffer buffer_;
    MessageCallback on_message_cb_;

    struct QueueItem {
        bool is_binary;
        std::string text;
        std::vector<uint8_t> binary;
    };
    std::deque<QueueItem> write_queue_;
    bool writing_ = false;

    static std::atomic<uint64_t> next_id_;
    uint64_t id_;

public:
    explicit Session(tcp::socket&& socket, MessageCallback on_message_cb)
        : ws_(std::move(socket)), on_message_cb_(on_message_cb),
          id_(next_id_.fetch_add(1)) {}

    uint64_t id() const { return id_; }

    void run() {
        net::dispatch(ws_.get_executor(),
            beast::bind_front_handler(&Session::on_run, shared_from_this()));
    }

    void send_text(std::string message) {
        net::post(ws_.get_executor(),
            [self = shared_from_this(), msg = std::move(message)]() {
                QueueItem item;
                item.is_binary = false;
                item.text = std::move(msg);
                self->write_queue_.push_back(std::move(item));
                if (!self->writing_) {
                    self->writing_ = true;
                    self->do_write();
                }
            });
    }

    void send_binary(std::vector<uint8_t> data) {
        net::post(ws_.get_executor(),
            [self = shared_from_this(), d = std::move(data)]() {
                QueueItem item;
                item.is_binary = true;
                item.binary = std::move(d);
                self->write_queue_.push_back(std::move(item));
                if (!self->writing_) {
                    self->writing_ = true;
                    self->do_write();
                }
            });
    }

    // Legacy: send text (for backward compat with old callback style)
    void send(std::string message) { send_text(std::move(message)); }

private:
    void on_run() {
        ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
        ws_.set_option(websocket::stream_base::decorator([](websocket::response_type& res) {
            res.set(http::field::server,
                std::string(BOOST_BEAST_VERSION_STRING) + " websocket-server-async");
        }));
        ws_.async_accept(
            beast::bind_front_handler(&Session::on_accept, shared_from_this()));
    }

    void on_accept(beast::error_code ec) {
        if (ec) {
            RCLCPP_ERROR(rclcpp::get_logger("websocket_server"), "accept: %s", ec.message().c_str());
            return;
        }
        do_read();
    }

    void do_read() {
        ws_.async_read(buffer_,
            beast::bind_front_handler(&Session::on_read, shared_from_this()));
    }

    void on_read(beast::error_code ec, std::size_t bytes_transferred) {
        boost::ignore_unused(bytes_transferred);
        if (ec == websocket::error::closed) return;
        if (ec) {
            RCLCPP_WARN(rclcpp::get_logger("websocket_server"), "read: %s", ec.message().c_str());
            return;
        }

        bool is_binary = ws_.got_binary();
        std::string msg = beast::buffers_to_string(buffer_.data());
        buffer_.consume(buffer_.size());

        if (on_message_cb_) {
            try {
                on_message_cb_(shared_from_this(), msg, is_binary);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("websocket_server"),
                             "Exception in message handler: %s", e.what());
            } catch (...) {
                RCLCPP_ERROR(rclcpp::get_logger("websocket_server"),
                             "Unknown exception in message handler");
            }
        }

        do_read();
    }

    void do_write() {
        if (write_queue_.empty()) {
            writing_ = false;
            return;
        }

        auto& front = write_queue_.front();
        ws_.binary(front.is_binary);

        if (front.is_binary) {
            ws_.async_write(net::buffer(front.binary),
                beast::bind_front_handler(&Session::on_write, shared_from_this()));
        } else {
            ws_.async_write(net::buffer(front.text),
                beast::bind_front_handler(&Session::on_write, shared_from_this()));
        }
    }

    void on_write(beast::error_code ec, std::size_t bytes_transferred) {
        boost::ignore_unused(bytes_transferred);
        if (ec) {
            RCLCPP_WARN(rclcpp::get_logger("websocket_server"), "write: %s", ec.message().c_str());
            writing_ = false;
            return;
        }

        write_queue_.pop_front();
        do_write();
    }
};

inline std::atomic<uint64_t> Session::next_id_{1};

class Listener : public std::enable_shared_from_this<Listener> {
    net::io_context& ioc_;
    tcp::acceptor acceptor_;
    MessageCallback on_message_cb_;
    bool ready_ = false;

public:
    Listener(net::io_context& ioc, tcp::endpoint endpoint, MessageCallback on_message_cb)
        : ioc_(ioc), acceptor_(ioc), on_message_cb_(on_message_cb) {
        beast::error_code ec;

        acceptor_.open(endpoint.protocol(), ec);
        if (ec) { RCLCPP_ERROR(rclcpp::get_logger("websocket_server"), "open: %s", ec.message().c_str()); return; }

        acceptor_.set_option(net::socket_base::reuse_address(true), ec);
        if (ec) { RCLCPP_ERROR(rclcpp::get_logger("websocket_server"), "set_option: %s", ec.message().c_str()); return; }

        acceptor_.bind(endpoint, ec);
        if (ec) { RCLCPP_ERROR(rclcpp::get_logger("websocket_server"), "bind: %s", ec.message().c_str()); return; }

        acceptor_.listen(net::socket_base::max_listen_connections, ec);
        if (ec) { RCLCPP_ERROR(rclcpp::get_logger("websocket_server"), "listen: %s", ec.message().c_str()); return; }

        ready_ = true;
    }

    bool is_ready() const { return ready_; }

    void run() {
        if (!ready_) {
            RCLCPP_ERROR(rclcpp::get_logger("websocket_server"),
                         "listener is not ready; skipping accept loop");
            return;
        }
        do_accept();
    }

private:
    void do_accept() {
        acceptor_.async_accept(net::make_strand(ioc_),
            beast::bind_front_handler(&Listener::on_accept, shared_from_this()));
    }

    void on_accept(beast::error_code ec, tcp::socket socket) {
        if (ec) {
            RCLCPP_ERROR(rclcpp::get_logger("websocket_server"), "accept: %s", ec.message().c_str());
            return;
        }
        std::make_shared<Session>(std::move(socket), on_message_cb_)->run();
        do_accept();
    }
};

class WebsocketServer {
    net::io_context ioc_;
    std::shared_ptr<Listener> listener_;
    std::vector<std::thread> threads_;

public:
    WebsocketServer() = default;

    void run(const std::string& address, uint16_t port, int num_threads, MessageCallback on_message_cb) {
        auto const addr = net::ip::make_address(address);

        ioc_.restart();

        listener_ = std::make_shared<Listener>(
            ioc_, tcp::endpoint{addr, port}, on_message_cb);

        if (!listener_->is_ready()) {
            RCLCPP_ERROR(rclcpp::get_logger("websocket_server"),
                        "WebSocket server failed to start on %s:%u", address.c_str(), port);
            return;
        }

        listener_->run();

        RCLCPP_INFO(rclcpp::get_logger("websocket_server"),
                    "WebSocket server listening on %s:%u", address.c_str(), port);

        threads_.clear();
        threads_.reserve(num_threads);
        for (auto i = num_threads - 1; i > 0; --i) {
            threads_.emplace_back([this] { ioc_.run(); });
        }
        ioc_.run();
    }

    void stop() {
        ioc_.stop();
        for (auto& t : threads_) {
            if (t.joinable()) {
                t.join();
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("websocket_server"), "WebSocket server stopped.");
    }
};
