#include "cpp_rosbridge_core/resource_proxy.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <curl/curl.h>

#include <cctype>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <rclcpp/logging.hpp>

namespace cpp_rosbridge_core {

namespace fs = std::filesystem;

namespace {

size_t write_file_callback(void* contents, size_t size, size_t nmemb, void* userp) {
    auto* out = static_cast<std::ofstream*>(userp);
    const size_t bytes = size * nmemb;
    out->write(static_cast<const char*>(contents), static_cast<std::streamsize>(bytes));
    return bytes;
}

bool download_to_file(const std::string& uri, const fs::path& destination) {
    CURL* curl = curl_easy_init();
    if (curl == nullptr) {
        return false;
    }

    std::ofstream out(destination, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) {
        curl_easy_cleanup(curl);
        return false;
    }

    curl_easy_setopt(curl, CURLOPT_URL, uri.c_str());
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 3L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_file_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &out);

    const CURLcode result = curl_easy_perform(curl);
    long status_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status_code);
    curl_easy_cleanup(curl);
    out.close();

    return result == CURLE_OK && status_code >= 200 && status_code < 300;
}

}  // namespace

ResourceProxy::ResourceProxy(const Config& config)
    : config_(config) {}

ResourceProxy::ResourceProxy(rclcpp_lifecycle::LifecycleNode* node)
    : config_{} {
    if (node->has_parameter("resource_proxy.enabled")) {
        config_.enabled = node->get_parameter("resource_proxy.enabled").as_bool();
    }
    if (node->has_parameter("resource_proxy.prefer_local_packages")) {
        config_.prefer_local_packages =
            node->get_parameter("resource_proxy.prefer_local_packages").as_bool();
    }
    if (node->has_parameter("resource_proxy.robot_ip")) {
        config_.robot_ip = node->get_parameter("resource_proxy.robot_ip").as_string();
    }
    if (node->has_parameter("resource_proxy.local_base_uri")) {
        config_.local_base_uri = node->get_parameter("resource_proxy.local_base_uri").as_string();
    }
    if (node->has_parameter("resource_proxy.cache_root")) {
        config_.cache_root = node->get_parameter("resource_proxy.cache_root").as_string();
    }
    if (node->has_parameter("resource_proxy.remote_uri_template")) {
        config_.remote_uri_template =
            node->get_parameter("resource_proxy.remote_uri_template").as_string();
    }
}

bool ResourceProxy::maybe_rewrite_message(const std::string& topic,
                                          const std::string& type,
                                          nlohmann::json& msg_json) {
    if (!should_handle_topic(topic, type, msg_json)) {
        return false;
    }

    const auto rewritten = rewrite_robot_description(msg_json["data"].get<std::string>());
    if (!rewritten.has_value()) {
        return false;
    }

    msg_json["data"] = *rewritten;
    return true;
}

std::optional<std::string> ResourceProxy::rewrite_robot_description(const std::string& xml) const {
    if (!config_.enabled || config_.robot_ip.empty()) {
        return std::nullopt;
    }

    static const std::regex package_uri_regex(R"(package://([^/\s"'<>]+)/([^\s"'<>]+))");
    std::string rewritten = xml;
    bool changed = false;

    for (auto it = std::sregex_iterator(xml.begin(), xml.end(), package_uri_regex);
         it != std::sregex_iterator();
         ++it) {
        const std::string original = it->str(0);
        const std::string package_name = it->str(1);
        const std::string relative_path = it->str(2);

        if (!should_proxy_package(package_name)) {
            continue;
        }
        if (!ensure_cached(package_name, relative_path)) {
            continue;
        }

        rewritten = replace_all(std::move(rewritten), original, build_local_uri(package_name, relative_path));
        changed = true;
    }

    if (!changed) {
        return std::nullopt;
    }
    return rewritten;
}

std::optional<std::string> ResourceProxy::resolve_local_resource_path(
    const std::string& request_path,
    const std::string& cache_root) {
    static const std::string prefix = "/resources/";
    if (request_path.rfind(prefix, 0) != 0) {
        return std::nullopt;
    }

    const std::string suffix = request_path.substr(prefix.size());
    if (!is_safe_relative_path(suffix)) {
        return std::nullopt;
    }

    std::vector<std::string> parts;
    std::stringstream ss(suffix);
    std::string item;
    while (std::getline(ss, item, '/')) {
        if (!item.empty()) {
            parts.push_back(item);
        }
    }
    if (parts.size() < 2) {
        return std::nullopt;
    }

    fs::path cache_path(cache_root);
    for (const auto& part : parts) {
        cache_path /= part;
    }
    if (fs::exists(cache_path) && fs::is_regular_file(cache_path)) {
        return cache_path.string();
    }

    const std::string package_name = parts[0];
    try {
        fs::path package_root(ament_index_cpp::get_package_share_directory(package_name));
        for (size_t i = 1; i < parts.size(); ++i) {
            package_root /= parts[i];
        }
        if (fs::exists(package_root) && fs::is_regular_file(package_root)) {
            return package_root.string();
        }
    } catch (const std::exception&) {
    }

    return std::nullopt;
}

bool ResourceProxy::should_handle_topic(const std::string& topic,
                                        const std::string& type,
                                        const nlohmann::json& msg_json) const {
    if (!config_.enabled || config_.robot_ip.empty()) {
        return false;
    }
    if (type != "std_msgs/msg/String") {
        return false;
    }
    const std::string suffix = "/robot_description";
    const bool is_robot_description =
        topic == suffix ||
        (topic.size() > suffix.size() &&
         topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) == 0);
    if (!is_robot_description) {
        return false;
    }
    if (!msg_json.is_object() || !msg_json.contains("data") || !msg_json["data"].is_string()) {
        return false;
    }
    return msg_json["data"].get_ref<const std::string&>().find("package://") != std::string::npos;
}

bool ResourceProxy::should_proxy_package(const std::string& package_name) const {
    if (!is_safe_relative_path(package_name)) {
        return false;
    }
    if (!config_.prefer_local_packages) {
        return true;
    }
    return !package_exists_locally(package_name);
}

bool ResourceProxy::ensure_cached(const std::string& package_name,
                                  const std::string& relative_path) const {
    if (!is_safe_relative_path(relative_path)) {
        return false;
    }

    const fs::path cache_path = build_cache_path(package_name, relative_path);
    if (fs::exists(cache_path)) {
        return true;
    }

    std::lock_guard<std::mutex> lock(download_mutex_);
    if (fs::exists(cache_path)) {
        return true;
    }

    fs::create_directories(cache_path.parent_path());
    const fs::path tmp_path = cache_path.string() + ".tmp";
    const std::string remote_uri = build_remote_uri(package_name, relative_path);
    const bool ok = download_to_file(remote_uri, tmp_path);
    if (!ok) {
        std::error_code ec;
        fs::remove(tmp_path, ec);
        RCLCPP_WARN(rclcpp::get_logger("resource_proxy"),
                    "Failed to cache mesh from '%s'",
                    remote_uri.c_str());
        return false;
    }

    std::error_code ec;
    fs::rename(tmp_path, cache_path, ec);
    if (ec) {
        fs::remove(tmp_path, ec);
        return fs::exists(cache_path);
    }
    return true;
}

std::string ResourceProxy::build_remote_uri(const std::string& package_name,
                                            const std::string& relative_path) const {
    std::string uri = config_.remote_uri_template;
    uri = replace_all(std::move(uri), "{robot_ip}", url_encode(config_.robot_ip, false));
    uri = replace_all(std::move(uri), "{package}", url_encode(package_name, false));
    uri = replace_all(std::move(uri), "{path}", url_encode(relative_path, true));
    return uri;
}

std::string ResourceProxy::build_local_uri(const std::string& package_name,
                                           const std::string& relative_path) const {
    std::string base = config_.local_base_uri;
    if (!base.empty() && base.back() == '/') {
        base.pop_back();
    }
    return base + "/" + url_encode(config_.robot_ip, false) + "/" +
           url_encode(package_name, false) + "/" + url_encode(relative_path, true);
}

std::string ResourceProxy::build_cache_path(const std::string& package_name,
                                            const std::string& relative_path) const {
    fs::path root(config_.cache_root);
    root /= config_.robot_ip;
    root /= package_name;
    root /= fs::path(relative_path);
    return root.string();
}

std::string ResourceProxy::replace_all(std::string text,
                                       const std::string& from,
                                       const std::string& to) {
    if (from.empty()) {
        return text;
    }
    size_t pos = 0;
    while ((pos = text.find(from, pos)) != std::string::npos) {
        text.replace(pos, from.size(), to);
        pos += to.size();
    }
    return text;
}

std::string ResourceProxy::url_encode(const std::string& value, bool preserve_slash) {
    std::ostringstream encoded;
    encoded.fill('0');
    encoded << std::hex << std::uppercase;

    for (unsigned char c : value) {
        if (std::isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~' ||
            (preserve_slash && c == '/')) {
            encoded << c;
        } else {
            encoded << '%' << std::setw(2) << static_cast<int>(c);
        }
    }
    return encoded.str();
}

bool ResourceProxy::is_safe_relative_path(const std::string& path) {
    if (path.empty() || path.front() == '/' || path.find("..") != std::string::npos) {
        return false;
    }
    return path.find('\\') == std::string::npos;
}

bool ResourceProxy::package_exists_locally(const std::string& package_name) {
    try {
        const auto share_dir = ament_index_cpp::get_package_share_directory(package_name);
        return !share_dir.empty();
    } catch (const std::exception&) {
        return false;
    }
}

}  // namespace cpp_rosbridge_core
