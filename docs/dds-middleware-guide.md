# DDS Middleware Guide

TachyBridge supports two DDS middleware implementations: **Cyclone DDS** (default) and **Zenoh**.

## Prerequisites

Install the desired middleware packages:

```bash
# Cyclone DDS (default)
sudo apt install ros-jazzy-rmw-cyclonedds-cpp

# Zenoh
sudo apt install ros-jazzy-rmw-zenoh-cpp
```

## Usage

### Cyclone DDS (Default)

Cyclone DDS is the default middleware. No additional configuration needed:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch cpp_rosbridge_server rosbridge_websocket_launch.py
```

Or explicitly:

```bash
ros2 launch cpp_rosbridge_server rosbridge_websocket_launch.py rmw:=rmw_cyclonedds_cpp
```

### Zenoh DDS

Pass the `rmw` argument to switch:

```bash
ros2 launch cpp_rosbridge_server rosbridge_websocket_launch.py rmw:=rmw_zenoh_cpp
```

## Build

Set the `RMW_IMPLEMENTATION` environment variable before building:

```bash
source /opt/ros/jazzy/setup.bash

# For Cyclone DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# For Zenoh
# export RMW_IMPLEMENTATION=rmw_zenoh_cpp

colcon build --packages-up-to cpp_rosbridge_tests \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Comparison

| Feature | Cyclone DDS | Zenoh |
|---|---|---|
| Protocol | DDS/RTPS | Zenoh (peer-to-peer + routed) |
| WAN Support | Requires DDS bridge | Native (router mode) |
| Discovery | Multicast-based | Scouting + locator-based |
| Default in TachyBridge | Yes | No |

## Troubleshooting

### Nodes cannot discover each other

- Ensure all nodes use the **same** `RMW_IMPLEMENTATION`
- Mixing Zenoh and Cyclone nodes will prevent communication

### Verify active middleware

```bash
echo $RMW_IMPLEMENTATION
# or check at runtime
ros2 doctor --report | grep middleware
```
