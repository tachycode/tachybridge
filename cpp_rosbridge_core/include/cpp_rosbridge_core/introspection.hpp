#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp>

namespace cpp_rosbridge_core {

// ── Introspection ───────────────────────────────────────────────────────────

namespace introspection {

using MessageMembers =
    rosidl_typesupport_introspection_cpp::MessageMembers;

/// Get the introspection MessageMembers for a compile-time ROS2 message type
template <typename MsgT>
const MessageMembers* get_members() {
  const auto* ts =
      rosidl_typesupport_introspection_cpp::
          get_message_type_support_handle<MsgT>();
  return static_cast<const MessageMembers*>(ts->data);
}

/// Dynamically load MessageMembers by type name string (e.g. "std_msgs/msg/String")
/// Uses dlopen to load the type support library at runtime.
/// Returns nullptr if the type cannot be found.
const MessageMembers* get_members_by_type_name(const std::string& type_name);

/// Convert a ROS2 message to JSON using introspection (base64 for byte arrays)
nlohmann::json msg_to_json(const void* msg, const MessageMembers* members);

/// Convert a ROS2 message to CBOR-friendly JSON (byte arrays as json::binary)
nlohmann::json msg_to_cbor_json(const void* msg, const MessageMembers* members);

/// Fill a ROS2 message from JSON using introspection
/// Handles both base64 strings and json::binary values for byte arrays
void json_to_msg(const nlohmann::json& j, void* msg,
                 const MessageMembers* members);

/// Convenience: serialize a typed message to JSON
template <typename MsgT>
nlohmann::json serialize(const MsgT& msg) {
  return msg_to_json(&msg, get_members<MsgT>());
}

/// Convenience: deserialize JSON into a typed message
template <typename MsgT>
MsgT deserialize(const nlohmann::json& j) {
  MsgT msg{};
  json_to_msg(j, &msg, get_members<MsgT>());
  return msg;
}

/// Check if a message type contains CompressedImage fields (direct children)
bool has_compressed_image_fields(const MessageMembers* members);

/// Extracted image data (borrowed pointers, valid during callback scope)
struct ExtractedImage {
  std::string name;       // identifier for binary frame (camera name or generated)
  std::string format;     // "jpeg", "png", etc.
  const uint8_t* data;    // pointer to compressed image bytes
  size_t size;            // number of bytes
};

/// Serialize message to JSON, extracting CompressedImage fields as binary data.
/// Image fields are replaced with "{field}_count" metadata in JSON output.
/// Non-image fields are serialized normally.
nlohmann::json msg_to_json_extract_images(
    const void* msg, const MessageMembers* members,
    std::vector<ExtractedImage>& out_images);

}  // namespace introspection

// ── ROS message converter ───────────────────────────────────────────────────

class RosMessageConverter {
public:
    static nlohmann::json convert_ros_message_to_json(
        const std::string& type,
        const rclcpp::SerializedMessage& serialized_msg);

    static nlohmann::json convert_ros_message_to_cbor_json(
        const std::string& type,
        const rclcpp::SerializedMessage& serialized_msg);

    static std::shared_ptr<rclcpp::SerializedMessage> convert_json_to_ros_message(
        const std::string& type,
        const nlohmann::json& json_msg);
};

}  // namespace cpp_rosbridge_core
