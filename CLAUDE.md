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

Critical guardrails — detailed rationale in graph (`graphify query "TSG-001"`).

- **NEVER** block Boost.Asio I/O threads — no `sleep_for()`, `wait_for_service()` on I/O threads; offload via `post_work()`
- **MUST** size-limit all queues/buffers — define `kMax*Size`, apply backpressure at limit
- **NEVER** do I/O or CPU work under locks — copy under lock, release, then send
- **MUST** pair every resource allocation with cleanup on disconnect (`set_on_disconnect()`)
- **MUST** use `RCLCPP_DEBUG` for hot-path logs — `RCLCPP_INFO` only for lifecycle events
- **MUST** expose work pool thread count as ROS parameter (default 4)

## Branching Strategy

- **`dev`**: Development branch with full logs (`logs/` — troubleshooting, devlogs, TSGs)
- **`main`**: Production branch — `logs/` is excluded, `docs/` contains usage guides only
- **Merge**: Use `./scripts/merge-dev-to-main.sh` to merge dev → main (auto-excludes logs)

## Troubleshooting

See [logs/troubleshooting/README.md](logs/troubleshooting/README.md) for resolved issues and patterns (available on `dev` branch).

## graphify

This project has a graphify knowledge graph at graphify-out/.

Rules:
- Before answering architecture or codebase questions, read graphify-out/GRAPH_REPORT.md for god nodes and community structure
- If graphify-out/wiki/index.md exists, navigate it instead of reading raw files
- After modifying code files in this session, run `python3 -c "from graphify.watch import _rebuild_code; from pathlib import Path; _rebuild_code(Path('.'))"` to keep the graph current
