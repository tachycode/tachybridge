# Habilis ZMQ Protocol — Testing Guide

This document explains how to send/receive messages over the Habilis ZMQ protocol to interact with ROS 2 interfaces and the AI frontend.

## Architecture Overview

```
AI Manager (Python)          TachyBridge (C++)              ROS 2
┌─────────────┐          ┌───────────────────┐        ┌──────────────┐
│  PUB :4271  │──ZMQ────▶│ SUB (ai2comm)     │        │              │
│             │          │   HabilisBridge    │───────▶│  /ai/tick    │
│  SUB :4270  │◀──ZMQ───│ PUB (comm2ai)     │        │  /inference/*│
│             │          │   HabilisDispatcher│◀───────│  /joint_states│
└─────────────┘          └───────────────────┘        │  /camera/*   │
                                                       └──────────────┘
```

- **AI → ROS**: AI Manager PUB (:4271) → TachyBridge SUB → ROS 2 topics
- **ROS → AI**: ROS 2 topics → TachyBridge → PUB (:4270) → AI Manager SUB

## Wire Format

Every ZMQ frame follows this structure:

```
[4-byte big-endian header (MsgType value)][MessagePack body]
```

Python:
```python
import struct, msgpack
header = struct.pack("!I", msg_type_value)  # e.g. 16 for HEARTBEAT
body = msgpack.packb(payload, use_bin_type=True)
frame = header + body
```

## Message Types

| Enum | Value | Direction | Body | ROS Topic |
|------|-------|-----------|------|-----------|
| START_INFERENCE | 1 | AI→ROS | policy_infos (map) | `/inference/start` (String) |
| FINISH_INFERENCE | 2 | AI→ROS | policy_infos (map) | `/inference/finish` (String) |
| START_TIMER | 3 | AI→ROS | empty map `{}` | `/timer/start` (Empty) |
| FINISH_TIMER | 4 | AI→ROS | empty map `{}` | `/timer/finish` (Empty) |
| TICK | 5 | AI→ROS | empty map `{}` | `/ai/tick` (Empty) |
| OBSERVATION | 6 | ROS→AI | `{joints, images}` | (assembled from subscribers) |
| ACTION | 7 | AI→ROS | ndarray descriptor | `/joint_commands` (JointTrajectory) |
| ROBOT_TYPE | 8 | AI→ROS | string/map | `/robot_type` (String) |
| START_TRAINING | 9 | AI→ROS | string/map | `/training/start` (String) |
| FINISH_TRAINING | 10 | AI→ROS | string/map | `/training/finish` (String) |
| REQUEST | 11 | AI→ROS | `{response: "name", ...}` | `/ai/request` (String) |
| RESPONSE | 12 | Both | `{msg: {name, value}}` | (correlation) |
| ERROR | 13 | AI→ROS | string/map | `/ai/error` (String) |
| TRAINING_STATUS | 14 | AI→ROS | empty map `{}` | (returns RESPONSE) |
| INFERENCE_STATUS | 15 | AI→ROS | empty map `{}` | (returns RESPONSE) |
| HEARTBEAT | 16 | AI→ROS | empty map `{}` | (returns RESPONSE) |

## Test Scripts

### Prerequisites

```bash
pip install pyzmq msgpack numpy
```

### 1. Heartbeat Test (simplest round-trip)

Verifies basic ZMQ connectivity and the HEARTBEAT→RESPONSE flow.

```python
#!/usr/bin/env python3
"""test_heartbeat.py — Send HEARTBEAT, expect ResHeartbeat RESPONSE."""
import struct, time, zmq, msgpack

HEADER_FMT = "!I"
HEARTBEAT = 16
RESPONSE = 12

ctx = zmq.Context()

# PUB socket: AI → TachyBridge (ai2comm port 4271)
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:4271")

# SUB socket: TachyBridge → AI (comm2ai port 4270)
sub = ctx.socket(zmq.SUB)
sub.connect("tcp://127.0.0.1:4270")
sub.setsockopt(zmq.SUBSCRIBE, b"")  # receive all

time.sleep(1.0)  # wait for PUB/SUB handshake

# Send HEARTBEAT
header = struct.pack(HEADER_FMT, HEARTBEAT)
body = msgpack.packb({}, use_bin_type=True)
pub.send(header + body)
print("Sent HEARTBEAT")

# Receive RESPONSE
poller = zmq.Poller()
poller.register(sub, zmq.POLLIN)
events = dict(poller.poll(timeout=3000))  # 3s timeout

if sub in events:
    frame = sub.recv()
    (msg_type,) = struct.unpack(HEADER_FMT, frame[:4])
    payload = msgpack.unpackb(frame[4:], raw=False)
    assert msg_type == RESPONSE, f"Expected RESPONSE(12), got {msg_type}"
    assert payload["msg"]["name"] == "ResHeartbeat"
    assert payload["msg"]["value"] is True
    print(f"OK: Received ResHeartbeat: {payload}")
else:
    print("FAIL: No response within 3s")

pub.close()
sub.close()
ctx.term()
```

### 2. Inference Lifecycle Test

Sends START_INFERENCE → waits for OBSERVATION stream → sends FINISH_INFERENCE.

```python
#!/usr/bin/env python3
"""test_inference_lifecycle.py — Full inference cycle with observation monitoring."""
import struct, time, zmq, msgpack, numpy as np

HEADER_FMT = "!I"
START_INFERENCE = 1
FINISH_INFERENCE = 2
OBSERVATION = 6

def make_frame(msg_type, payload=None):
    header = struct.pack(HEADER_FMT, msg_type)
    body = msgpack.packb(payload or {}, use_bin_type=True)
    return header + body

def parse_frame(frame):
    (msg_type,) = struct.unpack(HEADER_FMT, frame[:4])
    body = msgpack.unpackb(frame[4:], raw=False)
    return msg_type, body

def unpack_ndarray(d):
    arr = np.frombuffer(d["data"], dtype=np.dtype(d["dtype"]))
    return arr.reshape(d["shape"])

ctx = zmq.Context()

pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:4271")

sub = ctx.socket(zmq.SUB)
sub.connect("tcp://127.0.0.1:4270")
sub.setsockopt(zmq.SUBSCRIBE, b"")

time.sleep(1.0)

# Step 1: Start inference
policy_infos = {"policy_name": "test_policy", "model_path": "/tmp/model.pt"}
pub.send(make_frame(START_INFERENCE, policy_infos))
print("Sent START_INFERENCE")

# Step 2: Receive OBSERVATIONs
# Requires ROS topics to be publishing (joint_states + cameras)
poller = zmq.Poller()
poller.register(sub, zmq.POLLIN)

obs_count = 0
start = time.time()
while time.time() - start < 5.0:  # collect for 5s
    events = dict(poller.poll(timeout=100))
    if sub in events:
        msg_type, body = parse_frame(sub.recv())
        if msg_type == OBSERVATION:
            obs_count += 1
            joints = unpack_ndarray(body["joints"])
            cameras = list(body["images"].keys())
            print(f"  OBS #{obs_count}: joints={joints.shape}, cameras={cameras}")

# Step 3: Finish inference
pub.send(make_frame(FINISH_INFERENCE, {}))
print(f"Sent FINISH_INFERENCE (received {obs_count} observations)")

# Verify stream stops
time.sleep(0.5)
events = dict(poller.poll(timeout=1000))
if sub not in events:
    print("OK: Observation stream stopped correctly")
else:
    print("WARN: Still receiving data after FINISH_INFERENCE")

pub.close(); sub.close(); ctx.term()
```

### 3. Action Command Test

Sends an ACTION (joint trajectory) from AI to ROS.

```python
#!/usr/bin/env python3
"""test_action.py — Send ACTION with numpy joint values."""
import struct, zmq, msgpack, numpy as np, time

HEADER_FMT = "!I"
ACTION = 7

def pack_ndarray(arr):
    arr = np.ascontiguousarray(arr, dtype=np.float32)
    return {"dtype": "float32", "shape": list(arr.shape), "data": arr.tobytes()}

ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:4271")
time.sleep(1.0)

# 6-DOF joint command
joints = np.array([0.1, -0.2, 0.3, 0.0, 1.57, -0.5], dtype=np.float32)
payload = pack_ndarray(joints)

header = struct.pack(HEADER_FMT, ACTION)
body = msgpack.packb(payload, use_bin_type=True)
pub.send(header + body)
print(f"Sent ACTION: {joints}")

# Verify on ROS side:
#   ros2 topic echo /joint_commands trajectory_msgs/msg/JointTrajectory

pub.close(); ctx.term()
```

### 4. Request/Response Correlation Test

```python
#!/usr/bin/env python3
"""test_request_response.py — Send REQUEST, then RESPONSE to match."""
import struct, time, zmq, msgpack

HEADER_FMT = "!I"
REQUEST = 11
RESPONSE = 12

def make_frame(msg_type, payload):
    header = struct.pack(HEADER_FMT, msg_type)
    body = msgpack.packb(payload, use_bin_type=True)
    return header + body

ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:4271")
time.sleep(1.0)

# Send REQUEST (AI asking ROS for something)
request_payload = {
    "response": "ResGetConfig",
    "query": "robot_config"
}
pub.send(make_frame(REQUEST, request_payload))
print(f"Sent REQUEST: expecting 'ResGetConfig'")

# Verify on ROS side:
#   ros2 topic echo /ai/request std_msgs/msg/String

# Later: send matching RESPONSE
time.sleep(0.5)
response_payload = {
    "msg": {
        "name": "ResGetConfig",
        "value": {"dof": 6, "type": "manipulator"}
    }
}
pub.send(make_frame(RESPONSE, response_payload))
print(f"Sent RESPONSE: 'ResGetConfig'")

pub.close(); ctx.term()
```

### 5. Full Message Type Test (all AI→ROS types)

```python
#!/usr/bin/env python3
"""test_all_msg_types.py — Send every AI→ROS message type and verify topics."""
import struct, time, zmq, msgpack, numpy as np

HEADER_FMT = "!I"

def make_frame(msg_type, payload=None):
    header = struct.pack(HEADER_FMT, msg_type)
    body = msgpack.packb(payload or {}, use_bin_type=True)
    return header + body

def pack_ndarray(arr):
    arr = np.ascontiguousarray(arr, dtype=np.float32)
    return {"dtype": "float32", "shape": list(arr.shape), "data": arr.tobytes()}

ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:4271")

sub = ctx.socket(zmq.SUB)
sub.connect("tcp://127.0.0.1:4270")
sub.setsockopt(zmq.SUBSCRIBE, b"")

time.sleep(1.0)

tests = [
    ("HEARTBEAT",        16, {},                          True),   # expects RESPONSE
    ("TICK",             5,  {},                          False),
    ("START_TIMER",      3,  {},                          False),
    ("FINISH_TIMER",     4,  {},                          False),
    ("ROBOT_TYPE",       8,  {"type": "manipulator"},    False),
    ("ERROR",            13, {"error": "test error"},     False),
    ("START_INFERENCE",  1,  {"policy": "test"},          False),
    ("ACTION",           7,  pack_ndarray(np.zeros(6)),   False),
    ("FINISH_INFERENCE", 2,  {},                          False),
    ("START_TRAINING",   9,  {"algo": "ppo"},             False),
    ("FINISH_TRAINING",  10, {},                          False),
    ("TRAINING_STATUS",  14, {},                          True),   # expects RESPONSE
    ("INFERENCE_STATUS", 15, {},                          True),   # expects RESPONSE
]

poller = zmq.Poller()
poller.register(sub, zmq.POLLIN)

for name, msg_type, payload, expects_response in tests:
    pub.send(make_frame(msg_type, payload))
    print(f"Sent {name} ({msg_type})", end="")

    if expects_response:
        events = dict(poller.poll(timeout=2000))
        if sub in events:
            frame = sub.recv()
            (resp_type,) = struct.unpack(HEADER_FMT, frame[:4])
            resp_body = msgpack.unpackb(frame[4:], raw=False)
            print(f" → RESPONSE: {resp_body['msg']['name']}")
        else:
            print(f" → FAIL: no response")
    else:
        print()

    time.sleep(0.2)

pub.close(); sub.close(); ctx.term()
print("\nDone. Verify ROS topics with: ros2 topic list | grep -E 'ai|inference|training|timer|robot'")
```

## ROS-Side Verification Commands

While running the tests above, use these commands in separate terminals to verify ROS integration:

```bash
# Monitor all Habilis-related topics
ros2 topic echo /ai/tick
ros2 topic echo /inference/start
ros2 topic echo /inference/finish
ros2 topic echo /training/start
ros2 topic echo /training/finish
ros2 topic echo /timer/start
ros2 topic echo /timer/finish
ros2 topic echo /robot_type
ros2 topic echo /ai/error
ros2 topic echo /ai/request
ros2 topic echo /joint_commands

# Publish fake sensor data for OBSERVATION assembly
ros2 topic pub /joint_states sensor_msgs/msg/JointState \
  "{name: ['j1','j2','j3'], position: [0.1, 0.2, 0.3]}" -r 30
```

## Frontend Integration

The frontend (habilis_communicator Python) uses the same wire protocol via `CommZMQ`:

```python
from habilis_interface.src.communicator import CommZMQ
from habilis_interface.src.constants import MsgType, PORTS

comm = CommZMQ()

# Create sockets (AI Manager side)
comm.create_socket(name="to_bridge", addr="*", port=PORTS["ai2comm"], pattern="pub")
comm.create_socket(name="from_bridge", addr="127.0.0.1", port=PORTS["comm2ai"], pattern="sub")
comm.set_sockopt("from_bridge", SocketOpt.SET_SUB_PREFIX, prefix="")

# Send a command
comm.send("to_bridge", {"msg_type": MsgType.HEARTBEAT})

# Receive (blocking)
msg_type, payload = comm.recv("from_bridge")
```

## Launching TachyBridge for Testing

```bash
# Terminal 1: Launch the bridge
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch cpp_zmq_server zmq_bridge_launch.py

# Terminal 2: Run a test
python3 test_heartbeat.py
```

### Custom Parameters

```bash
ros2 launch cpp_zmq_server zmq_bridge_launch.py \
  habilis_pub_port:=4270 \
  habilis_sub_port:=4271 \
  habilis_camera_topics:="[/camera/color/image_raw/compressed]" \
  habilis_camera_names:="[rgb]" \
  habilis_joint_state_topics:="[/joint_states]" \
  habilis_joint_order:="[j1,j2,j3,j4,j5,j6]" \
  habilis_observation_fps:=30.0
```

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| No RESPONSE to HEARTBEAT | PUB/SUB not connected | Wait 1s after socket creation for handshake |
| OBSERVATION not received | No sensor data on ROS topics | Publish fake joint_states / camera data |
| ACTION has wrong joint count | `habilis_action_order` mismatch | Match the parameter to your robot's DOF |
| MessagePack decode error | Wrong header format | Use `struct.pack("!I", value)` (big-endian uint32) |
| Connection refused | Bridge not running or wrong port | Check `habilis_pub_port`/`habilis_sub_port` params |
