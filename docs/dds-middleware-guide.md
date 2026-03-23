# DDS Middleware Guide

TachyBridge supports two DDS middleware implementations: **Zenoh** (default) and **Cyclone DDS**.

## Prerequisites

Install the desired middleware packages:

```bash
# Zenoh (default)
sudo apt install ros-jazzy-rmw-zenoh-cpp

# Cyclone DDS
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

## Usage

### Zenoh DDS (Default)

Zenoh is the default middleware. No additional configuration needed:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch cpp_rosbridge_server rosbridge_websocket_launch.py
```

Or explicitly:

```bash
ros2 launch cpp_rosbridge_server rosbridge_websocket_launch.py rmw:=rmw_zenoh_cpp
```

### Cyclone DDS

Pass the `rmw` argument to switch:

```bash
ros2 launch cpp_rosbridge_server rosbridge_websocket_launch.py rmw:=rmw_cyclonedds_cpp
```

## Build

Set the `RMW_IMPLEMENTATION` environment variable before building:

```bash
source /opt/ros/jazzy/setup.bash

# For Zenoh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# For Cyclone DDS
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

colcon build --packages-up-to cpp_rosbridge_tests \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Comparison

| Feature | Zenoh | Cyclone DDS |
|---|---|---|
| Protocol | Zenoh (peer-to-peer + routed) | DDS/RTPS |
| WAN Support | Native (router mode) | Requires DDS bridge |
| Discovery | Scouting + locator-based | Multicast-based |
| Default in TachyBridge | Yes | No |

## Troubleshooting

### Nodes cannot discover each other

- Ensure all nodes use the **same** `RMW_IMPLEMENTATION`
- Mixing Zenoh and Cyclone nodes will prevent communication

### Zenoh router mode

For multi-machine setups, configure the Zenoh router:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### Verify active middleware

```bash
echo $RMW_IMPLEMENTATION
# or check at runtime
ros2 doctor --report | grep middleware
```
