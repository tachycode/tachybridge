#pragma once
#include "cpp_rosbridge_core/protocol.hpp"

namespace cpp_rosbridge_core {
namespace capabilities {

class ExecuteCLI : public Capability {
public:
    using Capability::Capability;
    void handle_message(const nlohmann::json& message,
                        std::function<void(const nlohmann::json&)> sender,
                        bool cbor_mode = false) override;
private:
    static bool is_command_allowed(const std::string& command);

    struct CliResult {
        std::string output;
        int return_code;
        bool success;
        std::string error;
    };
    static CliResult run_command(const std::string& command, int timeout_sec);
};

}  // namespace capabilities
}  // namespace cpp_rosbridge_core
