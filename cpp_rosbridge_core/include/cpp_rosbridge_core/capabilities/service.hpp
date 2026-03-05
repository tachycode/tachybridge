#pragma once

#include "cpp_rosbridge_core/protocol.hpp"
#include "cpp_rosbridge_core/introspection.hpp"

#include <map>
#include <memory>
#include <string>

#include <rclcpp/client.hpp>
#include <rclcpp/generic_client.hpp>

namespace cpp_rosbridge_core {
namespace capabilities {

class CallService : public Capability {
public:
    using Capability::Capability;
    void handle_message(const nlohmann::json& message,
                        std::function<void(const nlohmann::json&)> sender,
                        bool cbor_mode = false) override;

private:
    struct RequestContext {
        std::string service_name;
        std::string type_name;
        rclcpp_lifecycle::LifecycleNode* node;
        const nlohmann::json& message;
        std::function<void(const nlohmann::json&)>& sender;
    };

    std::shared_ptr<rclcpp::GenericClient> get_or_create_client(const RequestContext& ctx);
    void send_request(std::shared_ptr<rclcpp::GenericClient>& client, const RequestContext& ctx);

    std::shared_ptr<std::vector<uint8_t>> build_request_message(
        const std::string& request_type, const nlohmann::json& args);

    std::map<std::string, rclcpp::ClientBase::SharedPtr> clients_;
};

}  // namespace capabilities
}  // namespace cpp_rosbridge_core
