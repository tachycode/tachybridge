#include "cpp_rosbridge_core/capabilities/service.hpp"
#include <rcl/client.h>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <thread>
#include <chrono>

namespace cpp_rosbridge_core {
namespace capabilities {

namespace {

nlohmann::json make_service_response(
    const std::string& service_name,
    bool result,
    const nlohmann::json& values,
    const std::string& error = "") {
    nlohmann::json response = {
        {"op", "service_response"},
        {"service", service_name},
        {"result", result},
        {"values", values}
    };
    if (!error.empty()) {
        response["error"] = error;
    }
    return response;
}

void attach_id(nlohmann::json& response, const nlohmann::json& message) {
    if (message.contains("id")) {
        response["id"] = message["id"];
    }
}

void attach_optional_id(nlohmann::json& response, const nlohmann::json& id) {
    if (!id.is_null()) {
        response["id"] = id;
    }
}

std::shared_ptr<rclcpp::GenericClient> create_generic_client_for_lifecycle(
    rclcpp_lifecycle::LifecycleNode* node,
    const std::string& service_name,
    const std::string& type_name) {
    rcl_client_options_t options = rcl_client_get_default_options();
    auto client = std::make_shared<rclcpp::GenericClient>(
        node->get_node_base_interface().get(),
        node->get_node_graph_interface(),
        service_name, type_name, options);
    node->get_node_services_interface()->add_client(
        std::dynamic_pointer_cast<rclcpp::ClientBase>(client), nullptr);
    return client;
}

}  // namespace

// --- CallService ---

void CallService::handle_message(const nlohmann::json& message, std::function<void(const nlohmann::json&)> sender, bool /*cbor_mode*/) {
    if (!message.contains("service") || !message.contains("type")) {
        return;
    }

    std::string service_name = message["service"];
    std::string type_name = message["type"];
    auto node = protocol_->get_node();

    RCLCPP_DEBUG(node->get_logger(), "Calling service '%s' of type '%s'",
        service_name.c_str(), type_name.c_str());

    // Offload the entire service call (including readiness polling) to the
    // work pool so it never blocks a WebSocket I/O thread.
    auto msg_copy = message;
    auto sender_copy = sender;
    protocol_->post_work(
        [this, msg_copy, sender_copy, service_name, type_name, node]() mutable {
        auto sender_fn = sender_copy;
        RequestContext ctx{service_name, type_name, node, msg_copy, sender_fn};

        auto client = get_or_create_client(ctx);
        if (!client) {
            return;
        }

        // Poll service_is_ready() on the work pool thread (not I/O thread).
        // wait_for_service() uses graph event waitsets internally, which conflict
        // with the executor's waitset when called from a non-executor thread.
        bool ready = client->service_is_ready();
        if (!ready) {
            constexpr int kMaxRetries = 50;           // 50 x 100ms = 5s max
            constexpr auto kPollInterval = std::chrono::milliseconds(100);
            for (int i = 0; i < kMaxRetries && !ready; ++i) {
                std::this_thread::sleep_for(kPollInterval);
                ready = client->service_is_ready();
            }
        }

        if (!ready) {
            RCLCPP_WARN(node->get_logger(),
                "Service '%s' not available (polled for 5s)", service_name.c_str());
            auto response = make_service_response(service_name, false,
                nlohmann::json::object(), "Service not available");
            attach_id(response, msg_copy);
            sender_fn(response);
            return;
        }

        send_request(client, ctx);
    });
}

std::shared_ptr<rclcpp::GenericClient> CallService::get_or_create_client(const RequestContext& ctx) {
    if (!clients_.count(ctx.service_name)) {
        try {
            auto client = create_generic_client_for_lifecycle(
                ctx.node, ctx.service_name, ctx.type_name);
            clients_[ctx.service_name] = client;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ctx.node->get_logger(), "Failed to create client for '%s': %s",
                ctx.service_name.c_str(), e.what());
            auto response = make_service_response(ctx.service_name, false, nlohmann::json::object());
            attach_id(response, ctx.message);
            ctx.sender(response);
            return nullptr;
        }
    }

    auto client = std::dynamic_pointer_cast<rclcpp::GenericClient>(clients_[ctx.service_name]);
    if (!client) {
        auto response = make_service_response(ctx.service_name, false, nlohmann::json::object());
        attach_id(response, ctx.message);
        ctx.sender(response);
        return nullptr;
    }
    return client;
}

std::shared_ptr<std::vector<uint8_t>> CallService::build_request_message(
    const std::string& request_type, const nlohmann::json& args) {
    const auto* members = introspection::get_members_by_type_name(request_type);
    if (!members) {
        throw std::runtime_error("Unknown request type: " + request_type);
    }
    auto buffer = std::make_shared<std::vector<uint8_t>>(members->size_of_);
    members->init_function(buffer->data(), rosidl_runtime_cpp::MessageInitialization::ZERO);
    introspection::json_to_msg(args, buffer->data(), members);
    return buffer;
}

void CallService::send_request(std::shared_ptr<rclcpp::GenericClient>& client, const RequestContext& ctx) {
    std::string request_type = ctx.type_name + "_Request";
    nlohmann::json args = ctx.message.value("args", nlohmann::json::object());

    try {
        auto msg_buffer = build_request_message(request_type, args);
        auto future_and_id = client->async_send_request(msg_buffer->data());
        auto fut_ptr = std::make_shared<decltype(future_and_id.future)>(
            std::move(future_and_id.future));

        auto id = ctx.message.contains("id") ? ctx.message["id"] : nlohmann::json();
        std::string resp_type = ctx.type_name + "_Response";
        std::string svc_name = ctx.service_name;
        auto sender_copy = ctx.sender;

        protocol_->post_work(
            [fut_ptr, msg_buffer, sender_copy, svc_name, resp_type, id]() {
            try {
                // Wait with timeout to avoid blocking the work pool thread forever
                auto status = fut_ptr->wait_for(std::chrono::seconds(30));
                if (status != std::future_status::ready) {
                    auto resp = make_service_response(svc_name, false,
                        nlohmann::json::object(), "Service call timed out (30s)");
                    attach_optional_id(resp, id);
                    sender_copy(resp);
                    return;
                }
                auto response_ptr = fut_ptr->get();
                const auto* resp_members = introspection::get_members_by_type_name(resp_type);
                if (!resp_members) {
                    throw std::runtime_error("Unknown response type: " + resp_type);
                }
                auto response_json = introspection::msg_to_json(response_ptr.get(), resp_members);
                auto resp = make_service_response(svc_name, true, response_json);
                attach_optional_id(resp, id);
                sender_copy(resp);
            } catch (const std::exception& e) {
                auto resp = make_service_response(svc_name, false, nlohmann::json::object(), e.what());
                attach_optional_id(resp, id);
                sender_copy(resp);
            }
        });
    } catch (const std::exception& e) {
        RCLCPP_ERROR(ctx.node->get_logger(), "Failed to call service '%s': %s",
            ctx.service_name.c_str(), e.what());
        auto response = make_service_response(ctx.service_name, false, nlohmann::json::object(), e.what());
        attach_id(response, ctx.message);
        ctx.sender(response);
    }
}

}  // namespace capabilities
}  // namespace cpp_rosbridge_core
