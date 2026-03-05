#pragma once
#include <array>
#include <string_view>

namespace cpp_rosbridge_core {
namespace cli_constants {

// Allowed command prefixes (ros2 + echo only)
constexpr std::array ALLOWED_PREFIXES = {
    std::string_view{"ros2 "},
    std::string_view{"echo "},
};

// Blocked patterns (shell injection prevention)
constexpr std::array BLOCKED_PATTERNS = {
    std::string_view{";"},
    std::string_view{"&&"},
    std::string_view{"||"},
    std::string_view{"|"},
    std::string_view{">"},
    std::string_view{"<"},
    std::string_view{"`"},
    std::string_view{"$("},
};

constexpr int DEFAULT_TIMEOUT_SEC = 30;
constexpr int MAX_TIMEOUT_SEC = 60;

}  // namespace cli_constants
}  // namespace cpp_rosbridge_core
