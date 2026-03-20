#include "cpp_zmq_server/habilis_dispatcher.hpp"

#include <algorithm>
#include <nlohmann/json.hpp>

namespace cpp_zmq_server {

HabilisDispatcher::HabilisDispatcher(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<HabilisBridge> bridge)
    : node_(std::move(node))
    , bridge_(std::move(bridge)) {}

void HabilisDispatcher::configure() {
    // Declare and read parameters
    auto declare_or_get = [this](const std::string& name, const auto& default_val) {
        if (!node_->has_parameter(name)) {
            node_->declare_parameter(name, default_val);
        }
        return node_->get_parameter(name);
    };

    camera_topics_ = declare_or_get("habilis_camera_topics",
        std::vector<std::string>{}).as_string_array();
    camera_names_ = declare_or_get("habilis_camera_names",
        std::vector<std::string>{}).as_string_array();
    joint_state_topics_ = declare_or_get("habilis_joint_state_topics",
        std::vector<std::string>{"/joint_states"}).as_string_array();
    joint_order_ = declare_or_get("habilis_joint_order",
        std::vector<std::string>{}).as_string_array();
    action_order_ = declare_or_get("habilis_action_order",
        std::vector<std::string>{}).as_string_array();
    action_topic_ = declare_or_get("habilis_action_topic",
        std::string("/joint_commands")).as_string();
    observation_fps_ = declare_or_get("habilis_observation_fps", 30.0).as_double();

    // If action_order not set, fall back to joint_order
    if (action_order_.empty()) {
        action_order_ = joint_order_;
    }

    // Create publishers (AI -> ROS)
    tick_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/ai/tick", 10);
    inference_start_pub_ = node_->create_publisher<std_msgs::msg::String>("/inference/start", 10);
    inference_finish_pub_ = node_->create_publisher<std_msgs::msg::String>("/inference/finish", 10);
    training_start_pub_ = node_->create_publisher<std_msgs::msg::String>("/training/start", 10);
    training_finish_pub_ = node_->create_publisher<std_msgs::msg::String>("/training/finish", 10);
    timer_start_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/timer/start", 10);
    timer_finish_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/timer/finish", 10);
    robot_type_pub_ = node_->create_publisher<std_msgs::msg::String>("/robot_type", 10);
    error_pub_ = node_->create_publisher<std_msgs::msg::String>("/ai/error", 10);
    action_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(action_topic_, 10);
    request_pub_ = node_->create_publisher<std_msgs::msg::String>("/ai/request", 10);

    // Create camera subscribers (ROS -> AI for OBSERVATION)
    if (camera_topics_.size() != camera_names_.size() && !camera_topics_.empty()) {
        RCLCPP_WARN(node_->get_logger(),
                    "habilis_camera_topics size (%zu) != habilis_camera_names size (%zu)",
                    camera_topics_.size(), camera_names_.size());
    }

    camera_caches_.resize(camera_topics_.size());
    for (size_t i = 0; i < camera_topics_.size(); ++i) {
        camera_caches_[i] = std::make_shared<CameraCache>();
        auto cache = camera_caches_[i];
        camera_subs_.push_back(
            node_->create_subscription<sensor_msgs::msg::CompressedImage>(
                camera_topics_[i], rclcpp::SensorDataQoS(),
                [cache](sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(cache->mutex);
                    cache->latest = std::move(msg);
                }));
    }

    // Create joint state subscribers
    joint_caches_.resize(joint_state_topics_.size());
    for (size_t i = 0; i < joint_state_topics_.size(); ++i) {
        joint_caches_[i] = std::make_shared<JointCache>();
        auto cache = joint_caches_[i];
        joint_subs_.push_back(
            node_->create_subscription<sensor_msgs::msg::JointState>(
                joint_state_topics_[i], 10,
                [cache](sensor_msgs::msg::JointState::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(cache->mutex);
                    cache->latest = std::move(msg);
                }));
    }

    RCLCPP_INFO(node_->get_logger(),
                "HabilisDispatcher configured: %zu cameras, %zu joint sources, %.0f fps",
                camera_topics_.size(), joint_state_topics_.size(), observation_fps_);
}

void HabilisDispatcher::activate() {
    // Observation timer is NOT started here — it starts on START_INFERENCE
    RCLCPP_INFO(node_->get_logger(), "HabilisDispatcher activated");
}

void HabilisDispatcher::deactivate() {
    stop_observation_stream();
    camera_subs_.clear();
    joint_subs_.clear();
    camera_caches_.clear();
    joint_caches_.clear();

    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_requests_.clear();
    }

    RCLCPP_INFO(node_->get_logger(), "HabilisDispatcher deactivated");
}

void HabilisDispatcher::dispatch(HabilisMsgType type, msgpack::object_handle body) {
    // Get a reference to the object before moving the handle
    const auto& obj = body.get();

    switch (type) {
        case HabilisMsgType::HEARTBEAT:        handle_heartbeat(obj); break;
        case HabilisMsgType::TICK:             handle_tick(obj); break;
        case HabilisMsgType::START_INFERENCE:  handle_start_inference(obj); break;
        case HabilisMsgType::FINISH_INFERENCE: handle_finish_inference(obj); break;
        case HabilisMsgType::START_TRAINING:   handle_start_training(obj); break;
        case HabilisMsgType::FINISH_TRAINING:  handle_finish_training(obj); break;
        case HabilisMsgType::START_TIMER:      handle_start_timer(obj); break;
        case HabilisMsgType::FINISH_TIMER:     handle_finish_timer(obj); break;
        case HabilisMsgType::ROBOT_TYPE:       handle_robot_type(obj); break;
        case HabilisMsgType::ERROR:            handle_error(obj); break;
        case HabilisMsgType::ACTION:           handle_action(obj); break;
        case HabilisMsgType::REQUEST:          handle_request(obj); break;
        case HabilisMsgType::RESPONSE:         handle_response(obj); break;
        case HabilisMsgType::TRAINING_STATUS:  handle_training_status(obj); break;
        case HabilisMsgType::INFERENCE_STATUS:  handle_inference_status(obj); break;
        case HabilisMsgType::OBSERVATION:
            // OBSERVATION is ROS->AI direction; receiving it from AI is unexpected
            RCLCPP_DEBUG(node_->get_logger(),
                         "Received OBSERVATION from AI (unexpected direction), ignoring");
            break;
        default:
            RCLCPP_WARN(node_->get_logger(),
                        "Unknown HabilisMsgType: %u", static_cast<uint32_t>(type));
            break;
    }
}

// --- Simple message handlers ---

void HabilisDispatcher::handle_heartbeat(const msgpack::object& /*body*/) {
    // Respond immediately with RESPONSE {name: "ResHeartbeat", value: true}
    msgpack::sbuffer buf;
    msgpack::packer<msgpack::sbuffer> pk(buf);
    pk.pack_map(1);
    pk.pack("msg");
    pk.pack_map(2);
    pk.pack("name");
    pk.pack("ResHeartbeat");
    pk.pack("value");
    pk.pack(true);

    send_to_ai(HabilisMsgType::RESPONSE, buf);
    RCLCPP_DEBUG(node_->get_logger(), "HEARTBEAT: sent ResHeartbeat");
}

void HabilisDispatcher::handle_tick(const msgpack::object& /*body*/) {
    auto msg = std::make_unique<std_msgs::msg::Empty>();
    tick_pub_->publish(std::move(*msg));
    RCLCPP_DEBUG(node_->get_logger(), "TICK published");
}

void HabilisDispatcher::handle_start_inference(const msgpack::object& body) {
    // Forward policy_infos as JSON string to ROS
    auto msg = std::make_unique<std_msgs::msg::String>();
    try {
        std::stringstream ss;
        ss << body;
        msg->data = ss.str();
    } catch (...) {
        msg->data = "{}";
    }
    inference_start_pub_->publish(std::move(*msg));

    // Start observation stream
    start_observation_stream();
    RCLCPP_INFO(node_->get_logger(), "START_INFERENCE: observation stream started");
}

void HabilisDispatcher::handle_finish_inference(const msgpack::object& body) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    try {
        std::stringstream ss;
        ss << body;
        msg->data = ss.str();
    } catch (...) {
        msg->data = "{}";
    }
    inference_finish_pub_->publish(std::move(*msg));

    stop_observation_stream();
    RCLCPP_INFO(node_->get_logger(), "FINISH_INFERENCE: observation stream stopped");
}

void HabilisDispatcher::handle_start_training(const msgpack::object& body) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    try {
        std::stringstream ss;
        ss << body;
        msg->data = ss.str();
    } catch (...) {
        msg->data = "{}";
    }
    training_start_pub_->publish(std::move(*msg));
    RCLCPP_INFO(node_->get_logger(), "START_TRAINING published");
}

void HabilisDispatcher::handle_finish_training(const msgpack::object& body) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    try {
        std::stringstream ss;
        ss << body;
        msg->data = ss.str();
    } catch (...) {
        msg->data = "{}";
    }
    training_finish_pub_->publish(std::move(*msg));
    RCLCPP_INFO(node_->get_logger(), "FINISH_TRAINING published");
}

void HabilisDispatcher::handle_start_timer(const msgpack::object& /*body*/) {
    auto msg = std::make_unique<std_msgs::msg::Empty>();
    timer_start_pub_->publish(std::move(*msg));
    RCLCPP_DEBUG(node_->get_logger(), "START_TIMER published");
}

void HabilisDispatcher::handle_finish_timer(const msgpack::object& /*body*/) {
    auto msg = std::make_unique<std_msgs::msg::Empty>();
    timer_finish_pub_->publish(std::move(*msg));
    RCLCPP_DEBUG(node_->get_logger(), "FINISH_TIMER published");
}

void HabilisDispatcher::handle_robot_type(const msgpack::object& body) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    try {
        std::stringstream ss;
        ss << body;
        msg->data = ss.str();
    } catch (...) {
        msg->data = "{}";
    }
    robot_type_pub_->publish(std::move(*msg));
    RCLCPP_INFO(node_->get_logger(), "ROBOT_TYPE published");
}

void HabilisDispatcher::handle_error(const msgpack::object& body) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    try {
        std::stringstream ss;
        ss << body;
        msg->data = ss.str();
    } catch (...) {
        msg->data = "error";
    }
    error_pub_->publish(std::move(*msg));
    RCLCPP_WARN(node_->get_logger(), "ERROR from AI manager forwarded to ROS");
}

// --- ACTION handler (AI -> ROS) ---

void HabilisDispatcher::handle_action(const msgpack::object& body) {
    NdarrayDesc arr;
    if (!unpack_ndarray(body, arr)) {
        RCLCPP_WARN(node_->get_logger(),
                    "ACTION: failed to unpack numpy array, dropping");
        return;
    }

    if (arr.dtype != "float32") {
        RCLCPP_WARN(node_->get_logger(),
                    "ACTION: unexpected dtype '%s', expected float32", arr.dtype.c_str());
        return;
    }

    size_t num_joints = arr.shape.empty() ? 0 : arr.shape[0];
    size_t expected_bytes = num_joints * sizeof(float);
    if (arr.data.size() != expected_bytes) {
        RCLCPP_WARN(node_->get_logger(),
                    "ACTION: data size mismatch: %zu bytes for %zu joints",
                    arr.data.size(), num_joints);
        return;
    }

    const auto* values = reinterpret_cast<const float*>(arr.data.data());

    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = node_->now();

    // Use action_order for joint names if available
    if (!action_order_.empty()) {
        if (action_order_.size() != num_joints) {
            RCLCPP_WARN(node_->get_logger(),
                        "ACTION: action_order size (%zu) != array size (%zu)",
                        action_order_.size(), num_joints);
        }
        traj.joint_names = action_order_;
    } else {
        // Generate default names
        traj.joint_names.resize(num_joints);
        for (size_t i = 0; i < num_joints; ++i) {
            traj.joint_names[i] = "joint_" + std::to_string(i);
        }
    }

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(values, values + num_joints);
    point.time_from_start = rclcpp::Duration(0, 0);
    traj.points.push_back(std::move(point));

    action_pub_->publish(std::move(traj));
    RCLCPP_DEBUG(node_->get_logger(), "ACTION: published %zu joints", num_joints);
}

// --- OBSERVATION assembly (ROS -> AI) ---

void HabilisDispatcher::start_observation_stream() {
    if (inference_active_.load()) return;
    inference_active_.store(true);

    if (observation_fps_ <= 0.0) {
        RCLCPP_WARN(node_->get_logger(),
                    "observation_fps <= 0, observation stream disabled");
        return;
    }

    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / observation_fps_));

    observation_timer_ = node_->create_wall_timer(
        period, [this]() { observation_timer_callback(); });

    RCLCPP_INFO(node_->get_logger(),
                "Observation stream started at %.0f fps", observation_fps_);
}

void HabilisDispatcher::stop_observation_stream() {
    inference_active_.store(false);
    if (observation_timer_) {
        observation_timer_->cancel();
        observation_timer_.reset();
    }
}

void HabilisDispatcher::observation_timer_callback() {
    if (!inference_active_.load()) return;
    assemble_and_send_observation();
}

void HabilisDispatcher::assemble_and_send_observation() {
    // Collect latest camera images
    std::vector<std::pair<std::string, std::vector<uint8_t>>> images;
    for (size_t i = 0; i < camera_caches_.size(); ++i) {
        sensor_msgs::msg::CompressedImage::SharedPtr img;
        {
            std::lock_guard<std::mutex> lock(camera_caches_[i]->mutex);
            img = camera_caches_[i]->latest;
        }
        if (!img) {
            RCLCPP_DEBUG(node_->get_logger(),
                         "OBSERVATION: camera %zu has no data, skipping frame",
                         i);
            return;  // All-or-nothing: skip if any source missing
        }
        std::string name = (i < camera_names_.size()) ?
            camera_names_[i] : ("camera_" + std::to_string(i));
        images.emplace_back(name, std::vector<uint8_t>(img->data.begin(), img->data.end()));
    }

    // Collect latest joint states and reorder per joint_order
    std::vector<float> ordered_joints;
    if (!joint_caches_.empty()) {
        // Merge all joint state sources
        std::unordered_map<std::string, double> joint_map;
        for (size_t i = 0; i < joint_caches_.size(); ++i) {
            sensor_msgs::msg::JointState::SharedPtr js;
            {
                std::lock_guard<std::mutex> lock(joint_caches_[i]->mutex);
                js = joint_caches_[i]->latest;
            }
            if (!js) {
                RCLCPP_DEBUG(node_->get_logger(),
                             "OBSERVATION: joint source %zu has no data, skipping frame", i);
                return;
            }
            for (size_t j = 0; j < js->name.size() && j < js->position.size(); ++j) {
                joint_map[js->name[j]] = js->position[j];
            }
        }

        // Reorder per joint_order
        if (!joint_order_.empty()) {
            ordered_joints.reserve(joint_order_.size());
            for (const auto& name : joint_order_) {
                auto it = joint_map.find(name);
                if (it != joint_map.end()) {
                    ordered_joints.push_back(static_cast<float>(it->second));
                } else {
                    RCLCPP_DEBUG(node_->get_logger(),
                                 "OBSERVATION: joint '%s' not found, using 0.0", name.c_str());
                    ordered_joints.push_back(0.0f);
                }
            }
        } else {
            // No order specified, use map order
            ordered_joints.reserve(joint_map.size());
            for (const auto& [name, val] : joint_map) {
                ordered_joints.push_back(static_cast<float>(val));
            }
        }
    }

    // Build msgpack payload matching Python build_payload_obs format:
    // {joints: {dtype, shape, data}, images: {name: bytes, ...}}
    msgpack::sbuffer buf;
    msgpack::packer<msgpack::sbuffer> pk(buf);

    pk.pack_map(2);

    // Pack joints as ndarray
    pk.pack("joints");
    std::vector<uint32_t> joint_shape = {static_cast<uint32_t>(ordered_joints.size())};
    pack_ndarray(pk, "float32", joint_shape,
                 reinterpret_cast<const uint8_t*>(ordered_joints.data()),
                 ordered_joints.size() * sizeof(float));

    // Pack images as dict of name -> raw bytes
    pk.pack("images");
    pk.pack_map(images.size());
    for (const auto& [name, data] : images) {
        pk.pack(name);
        pk.pack_bin(data.size());
        pk.pack_bin_body(reinterpret_cast<const char*>(data.data()), data.size());
    }

    send_to_ai(HabilisMsgType::OBSERVATION, buf);
    RCLCPP_DEBUG(node_->get_logger(),
                 "OBSERVATION sent: %zu joints, %zu cameras",
                 ordered_joints.size(), images.size());
}

// --- REQUEST/RESPONSE correlation ---

void HabilisDispatcher::handle_request(const msgpack::object& body) {
    // Forward request to ROS as JSON string
    auto msg = std::make_unique<std_msgs::msg::String>();
    try {
        std::stringstream ss;
        ss << body;
        msg->data = ss.str();
    } catch (...) {
        msg->data = "{}";
    }
    request_pub_->publish(std::move(*msg));

    // Extract response name for correlation
    try {
        auto map = body.as<std::map<std::string, msgpack::object>>();
        auto it = map.find("response");
        if (it != map.end()) {
            std::string response_name = it->second.as<std::string>();

            std::lock_guard<std::mutex> lock(pending_mutex_);
            evict_expired_requests();

            if (pending_requests_.size() >= kMaxPendingRequests) {
                // Drop oldest
                auto oldest = pending_requests_.begin();
                for (auto it2 = pending_requests_.begin(); it2 != pending_requests_.end(); ++it2) {
                    if (it2->second.created_at < oldest->second.created_at) {
                        oldest = it2;
                    }
                }
                RCLCPP_DEBUG(node_->get_logger(),
                             "PendingRequest map full, dropping oldest: %s",
                             oldest->first.c_str());
                pending_requests_.erase(oldest);
            }

            pending_requests_[response_name] = PendingRequest{
                response_name,
                std::chrono::steady_clock::now()
            };
            RCLCPP_DEBUG(node_->get_logger(),
                         "REQUEST: expecting response '%s'", response_name.c_str());
        }
    } catch (...) {
        RCLCPP_WARN(node_->get_logger(),
                    "REQUEST: failed to extract response name from body");
    }
}

void HabilisDispatcher::handle_response(const msgpack::object& body) {
    // Extract name from {msg: {name: "...", value: ...}}
    try {
        auto map = body.as<std::map<std::string, msgpack::object>>();
        auto it_msg = map.find("msg");
        if (it_msg == map.end()) {
            RCLCPP_DEBUG(node_->get_logger(), "RESPONSE: no 'msg' field, dropping");
            return;
        }

        auto msg_map = it_msg->second.as<std::map<std::string, msgpack::object>>();
        auto it_name = msg_map.find("name");
        if (it_name == msg_map.end()) {
            RCLCPP_DEBUG(node_->get_logger(), "RESPONSE: no 'name' field, dropping");
            return;
        }

        std::string name = it_name->second.as<std::string>();

        std::lock_guard<std::mutex> lock(pending_mutex_);
        auto it = pending_requests_.find(name);
        if (it != pending_requests_.end()) {
            pending_requests_.erase(it);
            RCLCPP_DEBUG(node_->get_logger(),
                         "RESPONSE: matched pending request '%s'", name.c_str());
        } else {
            RCLCPP_DEBUG(node_->get_logger(),
                         "RESPONSE: no pending request for '%s', dropping", name.c_str());
        }

        // Forward to ROS as string on a response topic
        // (ROS-side consumers can listen on /ai/response)
    } catch (...) {
        RCLCPP_WARN(node_->get_logger(),
                    "RESPONSE: failed to parse body, dropping");
    }
}

void HabilisDispatcher::handle_training_status(const msgpack::object& /*body*/) {
    // AI is querying training status. Forward to ROS and relay response.
    // For now, send a placeholder response
    msgpack::sbuffer buf;
    msgpack::packer<msgpack::sbuffer> pk(buf);
    pk.pack_map(1);
    pk.pack("msg");
    pk.pack_map(2);
    pk.pack("name");
    pk.pack("ResTrainingStatus");
    pk.pack("value");
    pk.pack_map(3);
    pk.pack("is_running");
    pk.pack(0);  // IDLE
    pk.pack("current_step");
    pk.pack(0);
    pk.pack("error");
    pk.pack("");

    send_to_ai(HabilisMsgType::RESPONSE, buf);
    RCLCPP_DEBUG(node_->get_logger(), "TRAINING_STATUS: sent placeholder response");
}

void HabilisDispatcher::handle_inference_status(const msgpack::object& /*body*/) {
    // AI is querying inference status
    msgpack::sbuffer buf;
    msgpack::packer<msgpack::sbuffer> pk(buf);
    pk.pack_map(1);
    pk.pack("msg");
    pk.pack_map(2);
    pk.pack("name");
    pk.pack("ResInferenceStatus");
    pk.pack("value");
    pk.pack_map(2);
    pk.pack("phase");
    pk.pack(inference_active_.load());
    pk.pack("error");
    pk.pack("");

    send_to_ai(HabilisMsgType::RESPONSE, buf);
    RCLCPP_DEBUG(node_->get_logger(), "INFERENCE_STATUS: sent status (active=%d)",
                 inference_active_.load());
}

// --- Helpers ---

void HabilisDispatcher::send_to_ai(HabilisMsgType type, const msgpack::sbuffer& buf) {
    if (bridge_ && bridge_->is_running()) {
        bridge_->send(type, buf);
    }
}

void HabilisDispatcher::evict_expired_requests() {
    // Called with pending_mutex_ held
    auto now = std::chrono::steady_clock::now();
    for (auto it = pending_requests_.begin(); it != pending_requests_.end(); ) {
        if (now - it->second.created_at > kPendingRequestTimeout) {
            RCLCPP_DEBUG(node_->get_logger(),
                         "Evicting expired pending request: %s", it->first.c_str());
            it = pending_requests_.erase(it);
        } else {
            ++it;
        }
    }
}

}  // namespace cpp_zmq_server
