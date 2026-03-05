#include "cpp_rosbridge_core/capabilities/cli.hpp"
#include "cpp_rosbridge_core/cli_constants.hpp"
#include <cstdio>
#include <array>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace cpp_rosbridge_core {
namespace capabilities {

namespace {

void attach_id(nlohmann::json& response, const nlohmann::json& message) {
    if (message.contains("id")) {
        response["id"] = message["id"];
    }
}

nlohmann::json make_cli_response(bool success, const std::string& output,
                                  int return_code, const std::string& error) {
    return {
        {"op", "cli_response"},
        {"success", success},
        {"output", output},
        {"return_code", return_code},
        {"error", error}
    };
}

}  // namespace

bool ExecuteCLI::is_command_allowed(const std::string& command) {
    // Check blocked patterns first (shell injection prevention)
    for (const auto& pattern : cli_constants::BLOCKED_PATTERNS) {
        if (command.find(pattern) != std::string::npos) {
            return false;
        }
    }

    // Check allowed prefixes
    for (const auto& prefix : cli_constants::ALLOWED_PREFIXES) {
        if (command.size() >= prefix.size() &&
            command.compare(0, prefix.size(), prefix) == 0) {
            return true;
        }
    }

    return false;
}

ExecuteCLI::CliResult ExecuteCLI::run_command(const std::string& command, int timeout_sec) {
    std::string full_cmd = "timeout " + std::to_string(timeout_sec) + " " + command + " 2>&1";

    FILE* pipe = popen(full_cmd.c_str(), "r");
    if (!pipe) {
        return {"", -1, false, "Failed to execute command"};
    }

    std::string output;
    std::array<char, 4096> buffer;
    while (auto bytes_read = fread(buffer.data(), 1, buffer.size(), pipe)) {
        output.append(buffer.data(), bytes_read);
    }

    int status = pclose(pipe);
    int rc = WIFEXITED(status) ? WEXITSTATUS(status) : -1;

    if (rc == 124) {
        return {output, rc, false, "Command timed out after " + std::to_string(timeout_sec) + "s"};
    }

    return {output, rc, true, ""};
}

void ExecuteCLI::handle_message(const nlohmann::json& message,
                                 std::function<void(const nlohmann::json&)> sender,
                                 bool /*cbor_mode*/) {
    auto node = protocol_->get_node();

    // Validate command field
    if (!message.contains("command") || !message["command"].is_string()) {
        auto response = make_cli_response(false, "", -1, "Missing 'command' field");
        attach_id(response, message);
        sender(response);
        return;
    }

    std::string command = message["command"].get<std::string>();
    if (command.empty()) {
        auto response = make_cli_response(false, "", -1, "Empty command");
        attach_id(response, message);
        sender(response);
        return;
    }

    // Parse timeout
    int timeout_sec = cli_constants::DEFAULT_TIMEOUT_SEC;
    if (message.contains("timeout_sec") && message["timeout_sec"].is_number()) {
        timeout_sec = message["timeout_sec"].get<int>();
        if (timeout_sec <= 0) timeout_sec = cli_constants::DEFAULT_TIMEOUT_SEC;
        if (timeout_sec > cli_constants::MAX_TIMEOUT_SEC) timeout_sec = cli_constants::MAX_TIMEOUT_SEC;
    }

    // Security check
    if (!is_command_allowed(command)) {
        RCLCPP_WARN(node->get_logger(), "CLI command not allowed: %s", command.c_str());
        auto response = make_cli_response(false, "", -1, "Command not allowed");
        attach_id(response, message);
        sender(response);
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Executing CLI: %s (timeout=%ds)", command.c_str(), timeout_sec);

    // Capture id for async context
    auto id = message.contains("id") ? message["id"] : nlohmann::json();

    protocol_->post_work([command, timeout_sec, sender, id]() {
        auto result = run_command(command, timeout_sec);

        auto response = make_cli_response(result.success, result.output,
                                           result.return_code, result.error);
        if (!id.is_null()) {
            response["id"] = id;
        }
        sender(response);
    });
}

}  // namespace capabilities
}  // namespace cpp_rosbridge_core
