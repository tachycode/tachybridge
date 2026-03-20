# Habilis Binary Protocol Specification v1.0

**Purpose:** Contract between TachyBridge (C++) and AI Manager (Python) for ZMQ PUB/SUB communication.

**Scope:** This document covers the wire format, message catalogue, port assignments, ROS parameter reference, and sequence diagrams for all 16 message types in the Habilis protocol.

---

## Table of Contents

1. [Wire Format](#wire-format)
2. [Port Configuration](#port-configuration)
3. [Message Type Enum](#message-type-enum)
4. [Fire-and-Forget Messages](#fire-and-forget-messages)
5. [Request-Response Messages](#request-response-messages)
6. [Generic REQUEST / RESPONSE](#generic-request--response)
7. [REQUEST Sub-types](#request-sub-types)
8. [ROS-Assembled Messages](#ros-assembled-messages)
9. [RESPONSE Wire Format Convention](#response-wire-format-convention)
10. [ROS Parameter Configuration](#ros-parameter-configuration)
11. [Sequence Diagrams](#sequence-diagrams)

---

## Wire Format

Every ZMQ frame consists of a fixed 4-byte header followed by a MessagePack body:

```
[ 4-byte big-endian uint32 (MsgType value) ][ MessagePack body ]
```

### Python (sender)

```python
import struct
import msgpack

frame = struct.pack("!I", msg_type.value) + msgpack.packb(payload, use_bin_type=True)
socket.send(frame)
```

### Python (receiver)

```python
frame = socket.recv()
msg_type_value = struct.unpack("!I", frame[:4])[0]
payload = msgpack.unpackb(frame[4:], raw=False)
```

### C++ (TachyBridge)

```cpp
// Serialize
std::vector<uint8_t> frame = serialize_habilis_frame(HabilisMsgType::TICK, body);

// Deserialize
HabilisMessage msg;
bool ok = parse_habilis_frame(data, len, msg);
```

`serialize_habilis_frame` and `parse_habilis_frame` are defined in
`cpp_zmq_server/include/cpp_zmq_server/habilis_types.hpp`.

### Notes

- Binary payload fields (e.g., numpy array `data`) MUST be packed as MessagePack `bin` type, not `str`.
- Empty payloads (e.g., TICK, HEARTBEAT request) are valid — the body is a zero-length MessagePack map `{}`.
- Valid `MsgType` values are 1–16 inclusive. Frames with values outside this range are discarded.

---

## Port Configuration

| Channel  | Direction                       | Port |
|----------|---------------------------------|------|
| comm2ai  | TachyBridge PUB → AI Manager SUB | 4270 |
| ai2comm  | AI Manager PUB → TachyBridge SUB | 4271 |

TachyBridge **binds** its PUB socket on port 4270 and **connects** its SUB socket to the AI Manager's PUB address on port 4271.

The AI Manager's address is configurable via the `habilis_sub_address` ROS parameter (default: `127.0.0.1`).

---

## Message Type Enum

The enum values MUST match the Python `MsgType(Enum)` exactly. Python `auto()` starts at 1.

| Name              | Value | Primary Direction      |
|-------------------|-------|------------------------|
| START_INFERENCE   | 1     | Frontend → AI Manager  |
| FINISH_INFERENCE  | 2     | Frontend → AI Manager  |
| START_TIMER       | 3     | Frontend → AI Manager  |
| FINISH_TIMER      | 4     | Frontend → AI Manager  |
| TICK              | 5     | Frontend → AI Manager  |
| OBSERVATION       | 6     | TachyBridge → AI Manager |
| ACTION            | 7     | AI Manager → TachyBridge |
| ROBOT_TYPE        | 8     | Frontend → AI Manager  |
| START_TRAINING    | 9     | Frontend → AI Manager  |
| FINISH_TRAINING   | 10    | Frontend → AI Manager  |
| REQUEST           | 11    | Frontend → AI Manager  |
| RESPONSE          | 12    | AI Manager → Frontend  |
| ERROR             | 13    | Any → Any              |
| TRAINING_STATUS   | 14    | Frontend → AI Manager  |
| INFERENCE_STATUS  | 15    | Frontend → AI Manager  |
| HEARTBEAT         | 16    | Frontend → TachyBridge |

---

## Fire-and-Forget Messages

These messages carry no correlation ID. No RESPONSE is expected.

---

### TICK (5)

**Flow:** Frontend → TachyBridge → AI Manager (comm2ai, port 4270)

**Payload:**
```python
{}  # empty map
```

**ROS topic:** `/ai/tick` (`std_msgs/Empty`)

TachyBridge publishes an Empty message on `/ai/tick` upon receiving TICK, then forwards the frame to AI Manager.

---

### START_TIMER (3)

**Flow:** Frontend → TachyBridge → AI Manager (comm2ai, port 4270)

**Payload:**
```python
{}
```

**ROS topic:** `/timer/start` (`std_msgs/Empty`)

---

### FINISH_TIMER (4)

**Flow:** Frontend → TachyBridge → AI Manager (comm2ai, port 4270)

**Payload:**
```python
{}
```

**ROS topic:** `/timer/finish` (`std_msgs/Empty`)

---

### ROBOT_TYPE (8)

**Flow:** Frontend → TachyBridge → AI Manager (comm2ai, port 4270)

**Payload:**
```python
{"robot_type": "<string>"}
```

**ROS topic:** `/robot_type` (`std_msgs/String`)

---

### ERROR (13)

**Flow:** Any → Any

**Payload:**
```python
{"error": "<string>"}
```

**ROS topic:** `/ai/error` (`std_msgs/String`)

Used to propagate error conditions across the system. TachyBridge publishes the error string on `/ai/error` and forwards the frame on the appropriate channel.

---

### ACTION (7)

**Flow:** AI Manager → TachyBridge (ai2comm, port 4271), then TachyBridge → ROS

**Payload:** numpy ndarray descriptor
```python
{
    "dtype": "float32",
    "shape": [<num_joints>],
    "data": <bytes>   # raw float32 values, little-endian, packed as msgpack bin
}
```

**ROS topic:** `habilis_action_topic` parameter (default: `/joint_commands`, `trajectory_msgs/JointTrajectory`)

Joint order in the outgoing `JointTrajectory` message is determined by the `habilis_action_order` ROS parameter. If `habilis_action_order` is empty, joints are written in the order they appear in the ACTION payload.

**Data size:** `len(data) == 4 * shape[0]` (4 bytes per float32).

---

## Request-Response Messages

These messages include a `"response"` field containing the name used to correlate the reply. TachyBridge tracks pending requests internally with a 30-second timeout and a maximum of 64 concurrent pending entries.

---

### START_INFERENCE (1)

**Flow:** Frontend → TachyBridge → AI Manager (comm2ai, port 4270)

**Request payload:**
```python
{
    "policy_name": "<str>",
    "model_path": "<str>",
    # ...additional model configuration fields...
    "response": "ResStartInference"
}
```

**Expected RESPONSE payload:**
```python
{
    "msg": {
        "name": "ResStartInference",
        "value": {"success": bool, "msg": "<str>"}
    }
}
```

**Side effects on success (`success == true`):** TachyBridge begins assembling and sending OBSERVATION frames at `habilis_observation_fps` (default 30 Hz). Also publishes inference-start notification on the ROS `/inference/start` topic.

---

### FINISH_INFERENCE (2)

**Flow:** Frontend → TachyBridge → AI Manager (comm2ai, port 4270)

**Request payload:**
```python
{
    "response": "ResFinishInference"
}
```

**Expected RESPONSE payload:**
```python
{
    "msg": {
        "name": "ResFinishInference",
        "value": {"success": bool, "message": "<str>"}
    }
}
```

**Side effects:** TachyBridge stops the OBSERVATION timer regardless of the RESPONSE outcome.

---

### START_TRAINING (9)

**Flow:** Frontend → TachyBridge → AI Manager (comm2ai, port 4270)

**Request payload:**
```python
{
    "algo": "<str>",
    "config": { ... }
}
```

No RESPONSE is currently defined (treated as fire-and-forget despite the request pattern).

---

### FINISH_TRAINING (10)

**Flow:** Frontend → TachyBridge → AI Manager (comm2ai, port 4270)

**Request payload:**
```python
{}
```

No RESPONSE is currently defined.

---

### TRAINING_STATUS (14)

**Flow:** Frontend → TachyBridge → AI Manager (comm2ai, port 4270)

**Request payload:**
```python
{}
```

**Expected RESPONSE payload:**
```python
{
    "msg": {
        "name": "ResTrainingStatus",
        "value": {
            "is_running": int,     # 0 = stopped, 1 = running
            "current_step": int,
            "error": "<str>"       # empty string if no error
        }
    }
}
```

---

### INFERENCE_STATUS (15)

**Flow:** Frontend → TachyBridge → AI Manager (comm2ai, port 4270)

**Request payload:**
```python
{}
```

**Expected RESPONSE payload:**
```python
{
    "msg": {
        "name": "ResInferenceStatus",
        "value": {
            "phase": bool,         # true = inference active
            "error": "<str>"       # empty string if no error
        }
    }
}
```

---

### HEARTBEAT (16)

**Flow:** Frontend → TachyBridge (comm2ai, port 4270)

**Request payload:**
```python
{}
```

**Expected RESPONSE payload:**
```python
{
    "msg": {
        "name": "ResHeartbeat",
        "value": true
    }
}
```

TachyBridge MAY respond to HEARTBEAT directly without forwarding the message to AI Manager. This allows the Frontend to confirm that TachyBridge itself is alive independently of the AI Manager process.

---

## Generic REQUEST / RESPONSE

For operations not covered by the named message types above, TachyBridge provides a generic forwarding path.

### REQUEST (11)

**Flow:** Frontend → TachyBridge → AI Manager (comm2ai, port 4270)

**Payload:**
```python
{
    "request": "<RequestType>",    # identifies the sub-type
    "response": "<ResponseName>",  # correlation key
    # ...additional fields specific to the sub-type...
}
```

TachyBridge forwards the raw MessagePack frame to AI Manager without modification (pass-through). The `"response"` field is stored in the pending-request table for correlation.

### RESPONSE (12)

**Flow:** AI Manager → TachyBridge → Frontend (ai2comm, port 4271)

**Standard payload:**
```python
{
    "msg": {
        "name": "<ResponseName>",   # must match the "response" field of the originating request
        "value": <response_data>    # shape depends on the sub-type (see table below)
    }
}
```

TachyBridge forwards the RESPONSE frame to the Frontend WebSocket session that originated the matching REQUEST, identified by the `"name"` correlation key.

---

## REQUEST Sub-types

| `request` field       | `name` in RESPONSE      | `value` shape                                                               |
|-----------------------|-------------------------|-----------------------------------------------------------------------------|
| GetLerobotDataList    | ResLerobotDataList      | `[{"folder_dir": str, "task_name": str, "status": str}, ...]`               |
| DeleteLerobotData     | ResDeleteLerobotData    | result (implementation-defined)                                             |
| GetTrainedPolicyPaths | ResTrainedPolicyPaths   | `[str, ...]` (list of absolute file paths)                                  |
| GetModelWeightList    | ResModelWeightList      | `[str, ...]` (list of weight names)                                         |
| GetAvailableList      | ResAvailableList        | `[policy_list, device_list]` (two-element list of lists)                    |

---

## ROS-Assembled Messages

### OBSERVATION (6)

**Flow:** TachyBridge → AI Manager (comm2ai, port 4270)

**Trigger:** Sent by TachyBridge's observation timer at `habilis_observation_fps` Hz. Only active between a successful START_INFERENCE and a FINISH_INFERENCE.

**Payload:**
```python
{
    "joints": {
        "dtype": "float32",
        "shape": [<N>],
        "data": <bytes>    # raw float32 values, msgpack bin type
    },
    "images": {
        "<camera_name>": <bytes>,   # raw compressed image bytes (e.g., JPEG)
        # ...one entry per camera in habilis_camera_names
    }
}
```

**Assembly:** TachyBridge subscribes to the topics in `habilis_joint_state_topics` and `habilis_camera_topics`. On each timer tick it snapshots the latest cached value from each topic. Joint values are reordered according to `habilis_joint_order` before packing. Camera data is the raw `data` field from `sensor_msgs/CompressedImage`.

If no data has been received for a topic since the last observation, the previous cached value is reused. If no data has ever been received, that field is omitted from the payload.

---

## RESPONSE Wire Format Convention

All RESPONSE messages sent by AI Manager MUST conform to the following structure:

```python
{
    "msg": {
        "name": "<ResponseName>",   # correlation key — MUST match the request's "response" field
        "value": <response_data>    # MUST use the key "value", not "response" or any other key
    }
}
```

**Enforcement:** TachyBridge uses `"name"` to look up the pending request and route the RESPONSE to the correct WebSocket session. A RESPONSE with a `"name"` that does not match any pending request is logged at DEBUG level and discarded.

**Do not use `"response"` as the key for the result data.** The key MUST be `"value"`.

---

## ROS Parameter Configuration

All parameters are declared on the `ZmqBridgeNode` lifecycle node.

| Parameter                  | Type          | Default         | Description                                                     |
|----------------------------|---------------|-----------------|-----------------------------------------------------------------|
| `habilis_enabled`          | `bool`        | `true`          | Enable/disable the Habilis ZMQ bridge entirely                  |
| `habilis_pub_port`         | `int`         | `4270`          | Port TachyBridge PUB socket binds on (comm2ai)                 |
| `habilis_sub_port`         | `int`         | `4271`          | Port TachyBridge SUB socket connects to (ai2comm)              |
| `habilis_sub_address`      | `string`      | `"127.0.0.1"`   | IP address of the AI Manager's PUB socket                       |
| `habilis_camera_topics`    | `string[]`    | `[]`            | ROS topic names for CompressedImage sources                     |
| `habilis_camera_names`     | `string[]`    | `[]`            | Keys used in OBSERVATION `images` map (must match length of `habilis_camera_topics`) |
| `habilis_joint_state_topics` | `string[]`  | `["/joint_states"]` | ROS topic names for JointState sources                      |
| `habilis_joint_order`      | `string[]`    | `[]`            | Joint names to include in OBSERVATION `joints`, in order. Empty = all joints in arrival order |
| `habilis_action_order`     | `string[]`    | `[]`            | Joint names for ACTION output ordering. Empty = payload order  |
| `habilis_action_topic`     | `string`      | `"/joint_commands"` | ROS topic to publish `trajectory_msgs/JointTrajectory` on   |
| `habilis_observation_fps`  | `double`      | `30.0`          | Rate (Hz) at which OBSERVATION frames are assembled and sent    |

---

## Sequence Diagrams

### 1. Inference Lifecycle

```
Frontend          TachyBridge              AI Manager
   |                   |                       |
   |  START_INFERENCE  |                       |
   |  (comm2ai:4270)   |                       |
   |------------------>|                       |
   |                   |  START_INFERENCE      |
   |                   |  (comm2ai:4270)       |
   |                   |---------------------->|
   |                   |                       | (loads model,
   |                   |                       |  starts policy)
   |                   |       RESPONSE        |
   |                   |  ResStartInference    |
   |                   |  (ai2comm:4271)       |
   |                   |<----------------------|
   |     RESPONSE      |                       |
   |  ResStartInference|                       |
   |<------------------|                       |
   |                   |                       |
   |                   | [observation timer    |
   |                   |  starts @ obs_fps Hz] |
   |                   |                       |
   |                   |  OBSERVATION          |
   |                   |  (comm2ai:4270)       |
   |                   |---------------------->|
   |                   |  OBSERVATION          |
   |                   |---------------------->|  (repeats at obs_fps)
   |                   |  OBSERVATION          |
   |                   |---------------------->|
   |                   |                       |
   |                   |         ACTION        |
   |                   |  (ai2comm:4271)       |
   |                   |<----------------------|
   |                   | [publishes to         |
   |                   |  /joint_commands]     |
   |                   |                       |
   |  FINISH_INFERENCE |                       |
   |  (comm2ai:4270)   |                       |
   |------------------>|                       |
   |                   | [observation timer    |
   |                   |  stops immediately]   |
   |                   |  FINISH_INFERENCE     |
   |                   |  (comm2ai:4270)       |
   |                   |---------------------->|
   |                   |       RESPONSE        |
   |                   |  ResFinishInference   |
   |                   |  (ai2comm:4271)       |
   |                   |<----------------------|
   |     RESPONSE      |                       |
   | ResFinishInference|                       |
   |<------------------|                       |
   |                   |                       |
```

---

### 2. REQUEST / RESPONSE Round-trip

```
Frontend          TachyBridge              AI Manager
   |                   |                       |
   |  REQUEST          |                       |
   |  request: "GetTrainedPolicyPaths"         |
   |  response: "ResTrainedPolicyPaths"        |
   |  (comm2ai:4270)   |                       |
   |------------------>|                       |
   |                   | [stores pending entry:|
   |                   |  "ResTrainedPolicies"]|
   |                   |  REQUEST (forwarded)  |
   |                   |  (comm2ai:4270)       |
   |                   |---------------------->|
   |                   |                       | (processes request)
   |                   |      RESPONSE         |
   |                   |  name: "ResTrainedPolicyPaths"
   |                   |  value: ["/path/a",   |
   |                   |          "/path/b"]   |
   |                   |  (ai2comm:4271)       |
   |                   |<----------------------|
   |                   | [looks up pending,    |
   |                   |  routes to session]   |
   |     RESPONSE      |                       |
   |  name: "ResTrainedPolicyPaths"            |
   |  value: [...]     |                       |
   |<------------------|                       |
   |                   |                       |
```

Pending requests expire after **30 seconds**. A maximum of **64** concurrent pending entries are tracked. If the limit is reached, the oldest entry is evicted.

---

### 3. HEARTBEAT

```
Frontend          TachyBridge              AI Manager
   |                   |                       |
   |  HEARTBEAT        |                       |
   |  (comm2ai:4270)   |                       |
   |------------------>|                       |
   |                   |                       |
   |     RESPONSE      |                       |
   |  name: "ResHeartbeat"                     |
   |  value: true      |                       |
   |<------------------|  (TachyBridge replies |
   |                   |   directly; AI Manager|
   |                   |   not involved)       |
   |                   |                       |
```

TachyBridge responds to HEARTBEAT locally without forwarding to AI Manager. This lets the Frontend distinguish between a TachyBridge outage and an AI Manager outage.

---

## Implementation Notes for AI Manager Team

### Required Changes

1. **RESPONSE format standardization**: Use `"value"` key consistently (not `"response"`)
2. **TRAINING_STATUS handler**: Respond with real training state when receiving TRAINING_STATUS
3. **INFERENCE_STATUS handler**: Respond with real inference state when receiving INFERENCE_STATUS
4. **REQUEST handler**: Process REQUEST types and send RESPONSE with matching `"name"`
5. **START/FINISH_INFERENCE**: Send RESPONSE after processing (success/failure)

### CommCallback Migration Guide

Current `CommCallback` methods map to Habilis message types:

| CommCallback method | Habilis MsgType | Change needed |
|-------------------|-----------------|---------------|
| `observation()` | OBSERVATION (6) | No change — receives from TachyBridge |
| `start_inference()` | START_INFERENCE (1) | Standardize RESPONSE `"value"` key |
| `finish_inference()` | FINISH_INFERENCE (2) | Standardize RESPONSE `"value"` key |
| `start_training()` | START_TRAINING (9) | No change |
| `finish_training()` | FINISH_TRAINING (10) | No change |
| `training_status()` | TRAINING_STATUS (14) | Now triggered by incoming message, not internal |
| `inference_status()` | INFERENCE_STATUS (15) | Now triggered by incoming message, not internal |
| `request()` | REQUEST (11) | Standardize RESPONSE `"value"` key |
| `heartbeat()` | HEARTBEAT (16) | Handled by TachyBridge — no AI Manager action needed |
| `robot_type()` | ROBOT_TYPE (8) | No change |
| `action()` | ACTION (7) | No change — sends to TachyBridge |

### Correlation Rules

- Every message expecting a RESPONSE must include a `"response"` field with the expected response name
- TachyBridge tracks pending requests (max 64, 30s timeout)
- AI Manager must respond within 30 seconds or the request is evicted
- Response `"name"` must exactly match the request's `"response"` field

---

*Document generated from source: `cpp_zmq_server/include/cpp_zmq_server/habilis_types.hpp`, `habilis_bridge.hpp`, `habilis_dispatcher.hpp`.*
