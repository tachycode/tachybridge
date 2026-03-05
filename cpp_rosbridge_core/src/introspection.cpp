#include "cpp_rosbridge_core/introspection.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <dlfcn.h>
#include <mutex>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

namespace cpp_rosbridge_core {
namespace introspection {

namespace rti = rosidl_typesupport_introspection_cpp;
using json = nlohmann::json;

namespace {

// --- Base64 for uint8/octet arrays ---

static const char kBase64Chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string base64_encode(const uint8_t* data, size_t len) {
  std::string out;
  out.reserve(((len + 2) / 3) * 4);
  for (size_t i = 0; i < len; i += 3) {
    uint32_t n = static_cast<uint32_t>(data[i]) << 16;
    if (i + 1 < len) n |= static_cast<uint32_t>(data[i + 1]) << 8;
    if (i + 2 < len) n |= static_cast<uint32_t>(data[i + 2]);
    out.push_back(kBase64Chars[(n >> 18) & 0x3F]);
    out.push_back(kBase64Chars[(n >> 12) & 0x3F]);
    out.push_back(
        (i + 1 < len) ? kBase64Chars[(n >> 6) & 0x3F] : '=');
    out.push_back(
        (i + 2 < len) ? kBase64Chars[n & 0x3F] : '=');
  }
  return out;
}

std::vector<uint8_t> base64_decode(const std::string& encoded) {
  static const auto make_table = []() {
    std::array<uint8_t, 256> t{};
    t.fill(255);
    for (int i = 0; i < 64; ++i) {
      t[static_cast<unsigned char>(kBase64Chars[i])] =
          static_cast<uint8_t>(i);
    }
    return t;
  };
  static const auto table = make_table();

  std::vector<uint8_t> out;
  out.reserve(encoded.size() * 3 / 4);

  uint32_t buf = 0;
  int bits = 0;
  for (char c : encoded) {
    if (c == '=' || c == '\n' || c == '\r') continue;
    uint8_t val = table[static_cast<unsigned char>(c)];
    if (val == 255) continue;
    buf = (buf << 6) | val;
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out.push_back(static_cast<uint8_t>((buf >> bits) & 0xFF));
    }
  }
  return out;
}

// --- Scalar conversions ---

json scalar_to_json(const void* ptr, uint8_t type_id) {
  switch (type_id) {
    case rti::ROS_TYPE_BOOLEAN:
      return *static_cast<const bool*>(ptr);
    case rti::ROS_TYPE_OCTET:
    case rti::ROS_TYPE_UINT8:
      return *static_cast<const uint8_t*>(ptr);
    case rti::ROS_TYPE_CHAR:
    case rti::ROS_TYPE_INT8:
      return *static_cast<const int8_t*>(ptr);
    case rti::ROS_TYPE_UINT16:
      return *static_cast<const uint16_t*>(ptr);
    case rti::ROS_TYPE_INT16:
      return *static_cast<const int16_t*>(ptr);
    case rti::ROS_TYPE_UINT32:
      return *static_cast<const uint32_t*>(ptr);
    case rti::ROS_TYPE_INT32:
      return *static_cast<const int32_t*>(ptr);
    case rti::ROS_TYPE_UINT64:
      return *static_cast<const uint64_t*>(ptr);
    case rti::ROS_TYPE_INT64:
      return *static_cast<const int64_t*>(ptr);
    case rti::ROS_TYPE_FLOAT:
      return *static_cast<const float*>(ptr);
    case rti::ROS_TYPE_DOUBLE:
      return *static_cast<const double*>(ptr);
    case rti::ROS_TYPE_STRING:
      return *static_cast<const std::string*>(ptr);
    default:
      return nullptr;
  }
}

template <typename T>
T coerce_numeric(const json& j) {
  if (j.is_number()) {
    return j.get<T>();
  }
  if (j.is_string()) {
    const auto& s = j.get_ref<const std::string&>();
    if constexpr (std::is_floating_point_v<T>) {
      return static_cast<T>(std::stod(s));
    } else if constexpr (std::is_unsigned_v<T>) {
      return static_cast<T>(std::stoull(s));
    } else {
      return static_cast<T>(std::stoll(s));
    }
  }
  if (j.is_boolean()) {
    return static_cast<T>(j.get<bool>() ? 1 : 0);
  }
  throw json::type_error::create(302,
      "type must be number (or string-encoded number), but is " + std::string(j.type_name()), &j);
}

void json_to_scalar(const json& j, void* ptr, uint8_t type_id) {
  switch (type_id) {
    case rti::ROS_TYPE_BOOLEAN:
      if (j.is_boolean()) {
        *static_cast<bool*>(ptr) = j.get<bool>();
      } else if (j.is_string()) {
        const auto& s = j.get_ref<const std::string&>();
        *static_cast<bool*>(ptr) = (s == "true" || s == "1");
      } else if (j.is_number()) {
        *static_cast<bool*>(ptr) = j.get<int>() != 0;
      }
      break;
    case rti::ROS_TYPE_OCTET:
    case rti::ROS_TYPE_UINT8:
      *static_cast<uint8_t*>(ptr) = coerce_numeric<uint8_t>(j);
      break;
    case rti::ROS_TYPE_CHAR:
    case rti::ROS_TYPE_INT8:
      *static_cast<int8_t*>(ptr) = coerce_numeric<int8_t>(j);
      break;
    case rti::ROS_TYPE_UINT16:
      *static_cast<uint16_t*>(ptr) = coerce_numeric<uint16_t>(j);
      break;
    case rti::ROS_TYPE_INT16:
      *static_cast<int16_t*>(ptr) = coerce_numeric<int16_t>(j);
      break;
    case rti::ROS_TYPE_UINT32:
      *static_cast<uint32_t*>(ptr) = coerce_numeric<uint32_t>(j);
      break;
    case rti::ROS_TYPE_INT32:
      *static_cast<int32_t*>(ptr) = coerce_numeric<int32_t>(j);
      break;
    case rti::ROS_TYPE_UINT64:
      *static_cast<uint64_t*>(ptr) = coerce_numeric<uint64_t>(j);
      break;
    case rti::ROS_TYPE_INT64:
      *static_cast<int64_t*>(ptr) = coerce_numeric<int64_t>(j);
      break;
    case rti::ROS_TYPE_FLOAT:
      *static_cast<float*>(ptr) = coerce_numeric<float>(j);
      break;
    case rti::ROS_TYPE_DOUBLE:
      *static_cast<double*>(ptr) = coerce_numeric<double>(j);
      break;
    case rti::ROS_TYPE_STRING:
      if (j.is_string()) {
        *static_cast<std::string*>(ptr) = j.get<std::string>();
      } else {
        *static_cast<std::string*>(ptr) = j.dump();
      }
      break;
    default:
      break;
  }
}

bool is_byte_array(const rti::MessageMember& m) {
  return m.is_array_ &&
         (m.type_id_ == rti::ROS_TYPE_UINT8 ||
          m.type_id_ == rti::ROS_TYPE_OCTET);
}

size_t array_size(const rti::MessageMember& m, const void* field) {
  if (m.array_size_ > 0 && !m.is_upper_bound_) {
    return m.array_size_;
  }
  return m.size_function(field);
}

// --- JSON mode: byte arrays as base64 ---
json serialize_field(const rti::MessageMember& m, const void* field) {
  if (!m.is_array_) {
    if (m.type_id_ == rti::ROS_TYPE_MESSAGE) {
      const auto* sub =
          static_cast<const MessageMembers*>(m.members_->data);
      return msg_to_json(field, sub);
    }
    return scalar_to_json(field, m.type_id_);
  }

  size_t count = array_size(m, field);

  if (is_byte_array(m) && count > 0) {
    const auto* first = static_cast<const uint8_t*>(
        m.get_const_function(field, 0));
    return base64_encode(first, count);
  }

  if (m.type_id_ == rti::ROS_TYPE_MESSAGE) {
    const auto* sub =
        static_cast<const MessageMembers*>(m.members_->data);
    auto arr = json::array();
    for (size_t idx = 0; idx < count; ++idx) {
      const void* elem = m.get_const_function(field, idx);
      arr.push_back(msg_to_json(elem, sub));
    }
    return arr;
  }

  auto arr = json::array();
  for (size_t idx = 0; idx < count; ++idx) {
    const void* elem = m.get_const_function(field, idx);
    arr.push_back(scalar_to_json(elem, m.type_id_));
  }
  return arr;
}

// --- CBOR mode: byte arrays as json::binary ---
json serialize_field_cbor(const rti::MessageMember& m, const void* field) {
  if (!m.is_array_) {
    if (m.type_id_ == rti::ROS_TYPE_MESSAGE) {
      const auto* sub =
          static_cast<const MessageMembers*>(m.members_->data);
      return msg_to_cbor_json(field, sub);
    }
    return scalar_to_json(field, m.type_id_);
  }

  size_t count = array_size(m, field);

  if (is_byte_array(m) && count > 0) {
    const auto* first = static_cast<const uint8_t*>(
        m.get_const_function(field, 0));
    // CBOR mode: use binary type instead of base64
    return json::binary({first, first + count});
  }

  if (m.type_id_ == rti::ROS_TYPE_MESSAGE) {
    const auto* sub =
        static_cast<const MessageMembers*>(m.members_->data);
    auto arr = json::array();
    for (size_t idx = 0; idx < count; ++idx) {
      const void* elem = m.get_const_function(field, idx);
      arr.push_back(msg_to_cbor_json(elem, sub));
    }
    return arr;
  }

  auto arr = json::array();
  for (size_t idx = 0; idx < count; ++idx) {
    const void* elem = m.get_const_function(field, idx);
    arr.push_back(scalar_to_json(elem, m.type_id_));
  }
  return arr;
}

// --- Dynamic type loading cache ---
std::mutex type_cache_mutex;
std::unordered_map<std::string, const MessageMembers*> type_cache;

/// Parsed type name components from "package/sub_ns/Type"
struct ParsedTypeName {
  std::string package;    // e.g. "physical_ai_interfaces"
  std::string sub_ns;     // e.g. "msg", "srv", "action"
  std::string type_name;  // e.g. "JSON_Request"
};

/// Parse type name "package/sub_ns/Type" into components.
/// Returns empty strings on failure.
ParsedTypeName parse_type_name(const std::string& type_name) {
  // Expected formats:
  //   "std_msgs/msg/String"
  //   "physical_ai_interfaces/srv/JSON_Request"
  //   "physical_ai_interfaces/action/ConvertData_FeedbackMessage"
  auto first_slash = type_name.find('/');
  if (first_slash == std::string::npos) return {"", "", ""};
  auto last_slash = type_name.rfind('/');
  if (last_slash == first_slash) return {"", "", ""};

  std::string package = type_name.substr(0, first_slash);
  std::string sub_ns = type_name.substr(first_slash + 1, last_slash - first_slash - 1);
  std::string msg_type = type_name.substr(last_slash + 1);
  return {package, sub_ns, msg_type};
}

bool member_is_compressed_image(const rti::MessageMember& m) {
  if (m.type_id_ != rti::ROS_TYPE_MESSAGE || m.members_ == nullptr) {
    return false;
  }
  const auto* sub = static_cast<const MessageMembers*>(m.members_->data);
  return std::strcmp(sub->message_namespace_, "sensor_msgs::msg") == 0 &&
         std::strcmp(sub->message_name_, "CompressedImage") == 0;
}

/// Extract format + raw bytes from a CompressedImage via introspection.
void extract_compressed_image_data(
    const void* img_ptr, const MessageMembers* img_members,
    std::string& out_format, const uint8_t*& out_data, size_t& out_size) {
  out_format.clear();
  out_data = nullptr;
  out_size = 0;

  for (uint32_t j = 0; j < img_members->member_count_; ++j) {
    const auto& fm = img_members->members_[j];
    const auto* fld =
        static_cast<const uint8_t*>(img_ptr) + fm.offset_;

    if (std::strcmp(fm.name_, "format") == 0 &&
        fm.type_id_ == rti::ROS_TYPE_STRING && !fm.is_array_) {
      out_format = *static_cast<const std::string*>(
          static_cast<const void*>(fld));
    } else if (std::strcmp(fm.name_, "data") == 0 && fm.is_array_ &&
               (fm.type_id_ == rti::ROS_TYPE_UINT8 ||
                fm.type_id_ == rti::ROS_TYPE_OCTET)) {
      out_size = array_size(fm, fld);
      if (out_size > 0) {
        out_data = static_cast<const uint8_t*>(
            fm.get_const_function(fld, 0));
      }
    }
  }
}

/// Resolve names for image array entries using companion string[] fields.
std::vector<std::string> resolve_image_names(
    const void* msg, const MessageMembers* members,
    const std::string& image_field_name, size_t image_count) {
  const char* candidates[] = {"camera_names"};

  for (const char* candidate : candidates) {
    for (uint32_t i = 0; i < members->member_count_; ++i) {
      const auto& m = members->members_[i];
      if (!m.is_array_ || m.type_id_ != rti::ROS_TYPE_STRING) continue;
      if (std::strcmp(m.name_, candidate) != 0) continue;

      const auto* field =
          static_cast<const uint8_t*>(msg) + m.offset_;
      size_t count = array_size(m, field);
      if (count != image_count) continue;

      std::vector<std::string> names;
      names.reserve(count);
      for (size_t idx = 0; idx < count; ++idx) {
        names.push_back(*static_cast<const std::string*>(
            m.get_const_function(field, idx)));
      }
      return names;
    }
  }

  // Also try "{field}_names"
  std::string field_names_key = image_field_name + "_names";
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto& m = members->members_[i];
    if (!m.is_array_ || m.type_id_ != rti::ROS_TYPE_STRING) continue;
    if (field_names_key != m.name_) continue;

    const auto* field =
        static_cast<const uint8_t*>(msg) + m.offset_;
    size_t count = array_size(m, field);
    if (count != image_count) continue;

    std::vector<std::string> names;
    names.reserve(count);
    for (size_t idx = 0; idx < count; ++idx) {
      names.push_back(*static_cast<const std::string*>(
          m.get_const_function(field, idx)));
    }
    return names;
  }

  // Fallback: generate indexed names
  std::vector<std::string> names;
  names.reserve(image_count);
  for (size_t i = 0; i < image_count; ++i) {
    names.push_back("camera_" + std::to_string(i));
  }
  return names;
}

/// Extract CompressedImage field(s) into out_images and add metadata to JSON.
void extract_image_field(
    const rti::MessageMember& m, const void* field,
    const void* msg, const MessageMembers* parent_members,
    std::vector<introspection::ExtractedImage>& out_images, json& j) {
  const auto* sub =
      static_cast<const MessageMembers*>(m.members_->data);

  if (!m.is_array_) {
    std::string format;
    const uint8_t* data = nullptr;
    size_t data_size = 0;
    extract_compressed_image_data(field, sub, format, data, data_size);
    if (data && data_size > 0) {
      out_images.push_back({m.name_, format, data, data_size});
    }
    j[std::string(m.name_) + "_present"] = true;
    return;
  }

  size_t count = array_size(m, field);
  auto names = resolve_image_names(
      msg, parent_members, m.name_, count);

  for (size_t idx = 0; idx < count; ++idx) {
    const void* elem = m.get_const_function(field, idx);
    std::string format;
    const uint8_t* data = nullptr;
    size_t data_size = 0;
    extract_compressed_image_data(elem, sub, format, data, data_size);
    if (data && data_size > 0) {
      out_images.push_back({names[idx], format, data, data_size});
    }
  }
  j[std::string(m.name_) + "_count"] = static_cast<int>(count);
}

}  // namespace

// --- Public API ---

json msg_to_json(const void* msg, const MessageMembers* members) {
  json j = json::object();
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto& m = members->members_[i];
    const auto* field =
        static_cast<const uint8_t*>(msg) + m.offset_;
    j[m.name_] = serialize_field(m, field);
  }
  return j;
}

json msg_to_cbor_json(const void* msg, const MessageMembers* members) {
  json j = json::object();
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto& m = members->members_[i];
    const auto* field =
        static_cast<const uint8_t*>(msg) + m.offset_;
    j[m.name_] = serialize_field_cbor(m, field);
  }
  return j;
}

void json_to_msg(const json& j, void* msg,
                 const MessageMembers* members) {
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto& m = members->members_[i];

    if (!j.contains(m.name_)) continue;

    auto* field = static_cast<uint8_t*>(msg) + m.offset_;
    const auto& val = j[m.name_];

    if (!m.is_array_) {
      if (m.type_id_ == rti::ROS_TYPE_MESSAGE) {
        const auto* sub =
            static_cast<const MessageMembers*>(m.members_->data);
        json_to_msg(val, field, sub);
      } else {
        json_to_scalar(val, field, m.type_id_);
      }
      continue;
    }

    // Handle CBOR binary values for byte arrays
    if (is_byte_array(m) && val.is_binary()) {
      const auto& bytes = val.get_binary();
      m.resize_function(field, bytes.size());
      if (!bytes.empty()) {
        auto* dest = static_cast<uint8_t*>(m.get_function(field, 0));
        std::memcpy(dest, bytes.data(), bytes.size());
      }
    }
    // Handle base64-encoded strings for byte arrays (JSON mode)
    else if (is_byte_array(m) && val.is_string()) {
      auto bytes = base64_decode(val.get<std::string>());
      m.resize_function(field, bytes.size());
      for (size_t idx = 0; idx < bytes.size(); ++idx) {
        auto* elem =
            static_cast<uint8_t*>(m.get_function(field, idx));
        *elem = bytes[idx];
      }
    } else if (val.is_array()) {
      size_t count = val.size();
      if (m.array_size_ == 0 || m.is_upper_bound_) {
        m.resize_function(field, count);
      }

      if (m.type_id_ == rti::ROS_TYPE_MESSAGE) {
        const auto* sub =
            static_cast<const MessageMembers*>(m.members_->data);
        for (size_t idx = 0; idx < count; ++idx) {
          void* elem = m.get_function(field, idx);
          json_to_msg(val[idx], elem, sub);
        }
      } else {
        for (size_t idx = 0; idx < count; ++idx) {
          void* elem = m.get_function(field, idx);
          json_to_scalar(val[idx], elem, m.type_id_);
        }
      }
    }
  }
}

bool has_compressed_image_fields(const MessageMembers* members) {
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    if (member_is_compressed_image(members->members_[i])) {
      return true;
    }
  }
  return false;
}

json msg_to_json_extract_images(const void* msg,
                                const MessageMembers* members,
                                std::vector<ExtractedImage>& out_images) {
  json j = json::object();
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto& m = members->members_[i];
    const auto* field =
        static_cast<const uint8_t*>(msg) + m.offset_;

    if (member_is_compressed_image(m)) {
      extract_image_field(m, field, msg, members, out_images, j);
    } else {
      j[m.name_] = serialize_field(m, field);
    }
  }
  return j;
}

const MessageMembers* get_members_by_type_name(const std::string& type_name) {
  {
    std::lock_guard<std::mutex> lock(type_cache_mutex);
    auto it = type_cache.find(type_name);
    if (it != type_cache.end()) {
      return it->second;
    }
  }

  auto parsed = parse_type_name(type_name);
  if (parsed.package.empty() || parsed.type_name.empty()) {
    return nullptr;
  }

  // Build the shared library name for the introspection type support
  // Pattern: lib{package}__rosidl_typesupport_introspection_cpp.so
  std::string lib_name = "lib" + parsed.package +
      "__rosidl_typesupport_introspection_cpp.so";

  void* lib = dlopen(lib_name.c_str(), RTLD_LAZY | RTLD_GLOBAL);
  if (!lib) {
    RCLCPP_WARN(rclcpp::get_logger("introspection"),
                "Failed to load type support library '%s': %s",
                lib_name.c_str(), dlerror());
    return nullptr;
  }

  // Symbol pattern uses the actual sub-namespace (msg, srv, action):
  // rosidl_typesupport_introspection_cpp__get_message_type_support_handle__{package}__{sub_ns}__{Type}
  std::string symbol_name =
      "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" +
      parsed.package + "__" + parsed.sub_ns + "__" + parsed.type_name;

  using GetTsFunc = const rosidl_message_type_support_t* (*)();
  auto func = reinterpret_cast<GetTsFunc>(dlsym(lib, symbol_name.c_str()));
  if (!func) {
    RCLCPP_WARN(rclcpp::get_logger("introspection"),
                "Symbol '%s' not found: %s",
                symbol_name.c_str(), dlerror());
    return nullptr;
  }

  const auto* ts = func();
  const auto* members = static_cast<const MessageMembers*>(ts->data);

  {
    std::lock_guard<std::mutex> lock(type_cache_mutex);
    type_cache[type_name] = members;
  }

  return members;
}

}  // namespace introspection

// ============================================================================
// RosMessageConverter
// ============================================================================

namespace {

const rosidl_message_type_support_t* get_type_support(const std::string& type_name) {
    const auto* members = introspection::get_members_by_type_name(type_name);
    if (!members) {
        throw std::runtime_error("Failed to load type support for: " + type_name);
    }

    auto [package, msg_type] = [&]() -> std::pair<std::string, std::string> {
        auto first_slash = type_name.find('/');
        auto last_slash = type_name.rfind('/');
        if (first_slash == std::string::npos || last_slash == first_slash) {
            return {"", ""};
        }
        return {type_name.substr(0, first_slash), type_name.substr(last_slash + 1)};
    }();

    std::string lib_name = "lib" + package + "__rosidl_typesupport_cpp.so";
    void* lib = dlopen(lib_name.c_str(), RTLD_LAZY | RTLD_GLOBAL);
    if (!lib) {
        throw std::runtime_error("Failed to load typesupport library: " + lib_name);
    }

    std::string symbol = "rosidl_typesupport_cpp__get_message_type_support_handle__" +
                          package + "__msg__" + msg_type;
    using GetTsFunc = const rosidl_message_type_support_t* (*)();
    auto func = reinterpret_cast<GetTsFunc>(dlsym(lib, symbol.c_str()));
    if (!func) {
        throw std::runtime_error("Symbol not found: " + symbol);
    }
    return func();
}

/// RAII wrapper for deserialized ROS message buffer.
struct DeserializedMessage {
    std::vector<uint8_t> buffer;
    const introspection::MessageMembers* members;

    void* ptr() { return buffer.data(); }
    const void* ptr() const { return buffer.data(); }
    ~DeserializedMessage() { if (members) members->fini_function(buffer.data()); }
};

DeserializedMessage deserialize(const std::string& type,
                                const rclcpp::SerializedMessage& serialized_msg) {
    const auto* members = introspection::get_members_by_type_name(type);
    if (!members) {
        throw std::runtime_error("Unknown message type: " + type);
    }

    DeserializedMessage msg{{}, members};
    msg.buffer.resize(members->size_of_);
    members->init_function(msg.ptr(), rosidl_runtime_cpp::MessageInitialization::ZERO);

    auto ret = rmw_deserialize(&serialized_msg.get_rcl_serialized_message(),
                               get_type_support(type), msg.ptr());
    if (ret != RMW_RET_OK) {
        throw std::runtime_error("rmw_deserialize failed for type: " + type);
    }
    return msg;
}

}  // namespace

nlohmann::json RosMessageConverter::convert_ros_message_to_json(
    const std::string& type,
    const rclcpp::SerializedMessage& serialized_msg) {
    auto msg = deserialize(type, serialized_msg);
    return introspection::msg_to_json(msg.ptr(), msg.members);
}

nlohmann::json RosMessageConverter::convert_ros_message_to_cbor_json(
    const std::string& type,
    const rclcpp::SerializedMessage& serialized_msg) {
    auto msg = deserialize(type, serialized_msg);
    return introspection::msg_to_cbor_json(msg.ptr(), msg.members);
}

std::shared_ptr<rclcpp::SerializedMessage> RosMessageConverter::convert_json_to_ros_message(
    const std::string& type,
    const nlohmann::json& json_msg) {
    const auto* members = introspection::get_members_by_type_name(type);
    if (!members) {
        throw std::runtime_error("Unknown message type: " + type);
    }

    std::vector<uint8_t> msg_buffer(members->size_of_);
    void* msg_ptr = msg_buffer.data();
    members->init_function(msg_ptr, rosidl_runtime_cpp::MessageInitialization::ZERO);

    introspection::json_to_msg(json_msg, msg_ptr, members);

    auto serialized = std::make_shared<rclcpp::SerializedMessage>();
    auto ret = rmw_serialize(msg_ptr, get_type_support(type),
                             &serialized->get_rcl_serialized_message());
    members->fini_function(msg_ptr);

    if (ret != RMW_RET_OK) {
        throw std::runtime_error("rmw_serialize failed for type: " + type);
    }
    return serialized;
}

}  // namespace cpp_rosbridge_core
