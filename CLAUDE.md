# TachyBridge — Development Rules

## Build

```bash
# From workspace root (habilis_communicator/)
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | tr '\n' ':')
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
colcon build --packages-up-to cpp_rosbridge_tests \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -Dnlohmann_json_DIR=/home/weed/miniconda3/share/cmake/nlohmann_json
```

> **Note**: Default middleware is Zenoh DDS (`rmw_zenoh_cpp`). To switch to Cyclone DDS at launch:
> ```bash
> ros2 launch cpp_rosbridge_server rosbridge_websocket_launch.py rmw:=rmw_cyclonedds_cpp
> ```
> Ensure the desired middleware is installed:
> `sudo apt install ros-jazzy-rmw-zenoh-cpp ros-jazzy-rmw-cyclonedds-cpp`

## Performance Rules (TSG-001)

These rules prevent recurrence of critical performance issues found in the initial code review.

### NEVER block Boost.Asio I/O threads

- **No `std::this_thread::sleep_for()` in any WebSocket handler or callback**
- **No `wait_for_service()`, `wait_for_action_server()` on I/O threads**
- Any blocking operation must be offloaded to the work pool via `protocol_->post_work()`
- Rationale: A single blocked I/O thread freezes all sessions on that thread

### All queues and buffers MUST have size limits

- Every `std::deque`, `std::vector`, or buffer used for message queuing must define a `kMax*Size` constant
- When the limit is reached, apply backpressure: drop oldest, drop newest, or disconnect the client
- Rationale: Unbounded queues cause OOM crashes under sustained load from slow clients

### Minimize lock scope — no I/O or CPU-heavy work under locks

- Never call `send_fn()`, `json.dump()`, or any network I/O while holding a mutex/shared_mutex
- Pattern: copy data under lock → release lock → process/send outside lock
- Rationale: Lock-held sends cause head-of-line blocking proportional to subscriber count

### Resource allocation must have a matching cleanup path

- Every `subscribe()` must have a corresponding `unsubscribe_all()` on disconnect
- Every client cache (`clients_`, `bridges_`) should have eviction or cleanup on session end
- Use `Session::set_on_disconnect()` to wire cleanup callbacks
- Rationale: Missing cleanup causes zombie subscriptions and memory leaks

### Hot-path logging must use DEBUG level

- `RCLCPP_INFO` is only for lifecycle events (startup, shutdown, first subscription)
- Per-message, per-call, per-subscription logs must use `RCLCPP_DEBUG`
- Never log full message payloads — log size only: `"Received message (%zu bytes)"`
- For periodic status, use `RCLCPP_INFO_THROTTLE` with interval >= 1000ms

### Work pool must be configurable

- Thread pool sizes must be exposed as ROS parameters, never hardcoded
- Default: `work_pool_threads = 4` (minimum for production)
- Consider: `std::thread::hardware_concurrency()` as upper bound reference

## Branching Strategy

- **`dev`**: Development branch with full documentation (`docs/`)
- **`main`**: Production branch — `docs/` is excluded
- **Merge**: Use `./scripts/merge-dev-to-main.sh` to merge dev → main (auto-excludes docs)

## Troubleshooting

See [docs/troubleshooting/README.md](docs/troubleshooting/README.md) for resolved issues and patterns (available on `dev` branch).
