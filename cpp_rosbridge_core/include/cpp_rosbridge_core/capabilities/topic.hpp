#pragma once

#include "cpp_rosbridge_core/protocol.hpp"

namespace cpp_rosbridge_core {
namespace capabilities {

class Ping : public Capability {
public:
    using Capability::Capability;
    void handle_message(const nlohmann::json& message,
                        std::function<void(const nlohmann::json&)> sender,
                        bool cbor_mode = false) override;
};

class Advertise : public Capability {
public:
    using Capability::Capability;
    void handle_message(const nlohmann::json& message,
                        std::function<void(const nlohmann::json&)> sender,
                        bool cbor_mode = false) override;
};

class Publish : public Capability {
public:
    using Capability::Capability;
    void handle_message(const nlohmann::json& message,
                        std::function<void(const nlohmann::json&)> sender,
                        bool cbor_mode = false) override;
};

class Subscribe : public Capability {
public:
    using Capability::Capability;
    void handle_message(const nlohmann::json& message,
                        std::function<void(const nlohmann::json&)> sender,
                        bool cbor_mode = false) override;
};

class Unsubscribe : public Capability {
public:
    using Capability::Capability;
    void handle_message(const nlohmann::json& message,
                        std::function<void(const nlohmann::json&)> sender,
                        bool cbor_mode = false) override;
};

}  // namespace capabilities
}  // namespace cpp_rosbridge_core
