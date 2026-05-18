#pragma once

#include <mutex>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace cpp_rosbridge_core {

class ResourceProxy {
public:
    struct Config {
        bool enabled = false;
        bool prefer_local_packages = true;
        std::string robot_ip;
        std::string local_base_uri = "/resources";
        std::string cache_root = "/tmp/tachybridge_resource_cache";
        std::string remote_uri_template = "http://{robot_ip}:9090/resources/{package}/{path}";
    };

    explicit ResourceProxy(const Config& config);
    explicit ResourceProxy(rclcpp_lifecycle::LifecycleNode* node);

    bool maybe_rewrite_message(const std::string& topic,
                               const std::string& type,
                               nlohmann::json& msg_json);

    std::optional<std::string> rewrite_robot_description(const std::string& xml) const;
    static std::optional<std::string> resolve_local_resource_path(
        const std::string& request_path,
        const std::string& cache_root = "/tmp/tachybridge_resource_cache");

private:
    bool should_handle_topic(const std::string& topic,
                             const std::string& type,
                             const nlohmann::json& msg_json) const;
    bool should_proxy_package(const std::string& package_name) const;
    bool ensure_cached(const std::string& package_name,
                       const std::string& relative_path) const;
    std::string build_remote_uri(const std::string& package_name,
                                 const std::string& relative_path) const;
    std::string build_local_uri(const std::string& package_name,
                                const std::string& relative_path) const;
    std::string build_cache_path(const std::string& package_name,
                                 const std::string& relative_path) const;
    static std::string replace_all(std::string text,
                                   const std::string& from,
                                   const std::string& to);
    static std::string url_encode(const std::string& value, bool preserve_slash);
    static bool is_safe_relative_path(const std::string& path);
    static bool package_exists_locally(const std::string& package_name);

    Config config_;
    mutable std::mutex download_mutex_;
};

}  // namespace cpp_rosbridge_core
