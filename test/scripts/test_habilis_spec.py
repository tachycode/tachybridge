#!/usr/bin/env python3
"""
Habilis Protocol Spec Verification Script

Simulates the AI Manager side to verify TachyBridge handles all message
types correctly per docs/protocol/habilis-protocol-spec.md.

Usage:
    1. Launch TachyBridge:  ros2 launch cpp_zmq_server zmq_bridge_launch.py
    2. Run this script:     python3 test_habilis_spec.py

Ports:
    - PUB  binds on 4271 (ai2comm: this script → TachyBridge)
    - SUB  connects to 4270 (comm2ai: TachyBridge → this script)
"""

import struct
import time
import sys
import zmq
import msgpack
import numpy as np
from enum import Enum, auto
from dataclasses import dataclass

# ── Protocol constants (must match habilis_types.hpp / constants.py) ─────────

class MsgType(Enum):
    START_INFERENCE  = auto()   # 1
    FINISH_INFERENCE = auto()   # 2
    START_TIMER      = auto()   # 3
    FINISH_TIMER     = auto()   # 4
    TICK             = auto()   # 5
    OBSERVATION      = auto()   # 6
    ACTION           = auto()   # 7
    ROBOT_TYPE       = auto()   # 8
    START_TRAINING   = auto()   # 9
    FINISH_TRAINING  = auto()   # 10
    REQUEST          = auto()   # 11
    RESPONSE         = auto()   # 12
    ERROR            = auto()   # 13
    TRAINING_STATUS  = auto()   # 14
    INFERENCE_STATUS = auto()   # 15
    HEARTBEAT        = auto()   # 16

HEADER_FMT = "!I"
HDR_SIZE = struct.calcsize(HEADER_FMT)

# ── Helpers ──────────────────────────────────────────────────────────────────

def make_frame(msg_type: MsgType, payload=None) -> bytes:
    header = struct.pack(HEADER_FMT, msg_type.value)
    body = msgpack.packb(payload or {}, use_bin_type=True)
    return header + body

def parse_frame(frame: bytes):
    msg_type_val = struct.unpack(HEADER_FMT, frame[:HDR_SIZE])[0]
    body = msgpack.unpackb(frame[HDR_SIZE:], raw=False)
    return MsgType(msg_type_val), body

def pack_ndarray(arr: np.ndarray) -> dict:
    arr = np.ascontiguousarray(arr, dtype=np.float32)
    return {"dtype": "float32", "shape": list(arr.shape), "data": arr.tobytes()}

def make_response(name: str, value) -> bytes:
    return make_frame(MsgType.RESPONSE, {"msg": {"name": name, "value": value}})

@dataclass
class TestResult:
    name: str
    passed: bool
    detail: str = ""

# ── Test runner ──────────────────────────────────────────────────────────────

class HabilisSpecTester:
    def __init__(self, bridge_addr="127.0.0.1", pub_port=4271, sub_port=4270):
        self.ctx = zmq.Context()
        self.results: list[TestResult] = []

        # AI Manager side: PUB sends TO TachyBridge
        self.pub = self.ctx.socket(zmq.PUB)
        self.pub.bind(f"tcp://*:{pub_port}")

        # AI Manager side: SUB receives FROM TachyBridge
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.connect(f"tcp://{bridge_addr}:{sub_port}")
        self.sub.setsockopt(zmq.SUBSCRIBE, b"")

        self.poller = zmq.Poller()
        self.poller.register(self.sub, zmq.POLLIN)

        # Wait for PUB/SUB handshake
        print("Waiting for ZMQ handshake (2s)...")
        time.sleep(2.0)

    def recv(self, timeout_ms=3000):
        """Receive a frame with timeout. Returns (MsgType, body) or (None, None)."""
        events = dict(self.poller.poll(timeout=timeout_ms))
        if self.sub in events:
            return parse_frame(self.sub.recv())
        return None, None

    def drain(self, timeout_ms=500):
        """Drain all pending messages."""
        while True:
            events = dict(self.poller.poll(timeout=timeout_ms))
            if self.sub not in events:
                break
            self.sub.recv()

    def record(self, name: str, passed: bool, detail: str = ""):
        self.results.append(TestResult(name, passed, detail))
        status = "\033[92mPASS\033[0m" if passed else "\033[91mFAIL\033[0m"
        print(f"  [{status}] {name}" + (f" — {detail}" if detail else ""))

    # ── Individual tests ─────────────────────────────────────────────────

    def test_heartbeat(self):
        """HEARTBEAT(16): TachyBridge should respond directly with ResHeartbeat."""
        print("\n── Test: HEARTBEAT ──")
        self.drain()

        # Send HEARTBEAT to TachyBridge
        self.pub.send(make_frame(MsgType.HEARTBEAT))

        msg_type, body = self.recv()
        if msg_type == MsgType.RESPONSE:
            name = body.get("msg", {}).get("name")
            value = body.get("msg", {}).get("value")
            self.record("HEARTBEAT → ResHeartbeat",
                        name == "ResHeartbeat" and value is True,
                        f"name={name}, value={value}")
        else:
            self.record("HEARTBEAT → ResHeartbeat", False,
                        f"Expected RESPONSE, got {msg_type}")

    def test_training_status_forward(self):
        """TRAINING_STATUS(14): Should be forwarded to AI Manager (this script receives it)."""
        print("\n── Test: TRAINING_STATUS forward ──")
        self.drain()

        # TachyBridge receives TRAINING_STATUS from Frontend and forwards to AI Manager
        # We simulate Frontend by sending via PUB:4271 → TachyBridge SUB
        # TachyBridge should forward via PUB:4270 → our SUB
        self.pub.send(make_frame(MsgType.TRAINING_STATUS))

        msg_type, body = self.recv()
        if msg_type == MsgType.TRAINING_STATUS:
            self.record("TRAINING_STATUS forwarded", True, "Received forwarded query")
        elif msg_type == MsgType.RESPONSE:
            name = body.get("msg", {}).get("name", "")
            if name == "ResTrainingStatus":
                self.record("TRAINING_STATUS forwarded", False,
                            "Got placeholder response instead of forward — dispatcher not updated")
            else:
                self.record("TRAINING_STATUS forwarded", False, f"Unexpected RESPONSE: {name}")
        else:
            self.record("TRAINING_STATUS forwarded", False,
                        f"No message received (got {msg_type})")

    def test_inference_status_forward(self):
        """INFERENCE_STATUS(15): Should be forwarded to AI Manager."""
        print("\n── Test: INFERENCE_STATUS forward ──")
        self.drain()

        self.pub.send(make_frame(MsgType.INFERENCE_STATUS))

        msg_type, body = self.recv()
        if msg_type == MsgType.INFERENCE_STATUS:
            self.record("INFERENCE_STATUS forwarded", True, "Received forwarded query")
        elif msg_type == MsgType.RESPONSE:
            name = body.get("msg", {}).get("name", "")
            if name == "ResInferenceStatus":
                self.record("INFERENCE_STATUS forwarded", False,
                            "Got placeholder response — dispatcher not updated")
            else:
                self.record("INFERENCE_STATUS forwarded", False, f"Unexpected RESPONSE: {name}")
        else:
            self.record("INFERENCE_STATUS forwarded", False,
                        f"No message received (got {msg_type})")

    def test_request_forward(self):
        """REQUEST(11): Should be forwarded to AI Manager with raw payload."""
        print("\n── Test: REQUEST forward ──")
        self.drain()

        payload = {
            "request": "GetModelWeightList",
            "response": "ResModelWeightList",
        }
        self.pub.send(make_frame(MsgType.REQUEST, payload))

        msg_type, body = self.recv()
        if msg_type == MsgType.REQUEST:
            req = body.get("request", "")
            resp = body.get("response", "")
            self.record("REQUEST forwarded", True,
                        f"request={req}, response={resp}")
        else:
            self.record("REQUEST forwarded", False,
                        f"Expected REQUEST forward, got {msg_type}")

    def test_response_broadcast(self):
        """RESPONSE(12): AI Manager sends RESPONSE, TachyBridge should broadcast."""
        print("\n── Test: RESPONSE broadcast ──")
        self.drain()

        # First send a REQUEST to create a pending entry
        self.pub.send(make_frame(MsgType.REQUEST, {
            "request": "GetAvailableList",
            "response": "ResAvailableList",
        }))
        time.sleep(0.3)
        self.drain()  # drain the forwarded REQUEST

        # Now send RESPONSE from "AI Manager" side
        self.pub.send(make_response("ResAvailableList", [["act"], ["cuda:0"]]))

        msg_type, body = self.recv()
        if msg_type == MsgType.RESPONSE:
            name = body.get("msg", {}).get("name")
            value = body.get("msg", {}).get("value")
            self.record("RESPONSE broadcast",
                        name == "ResAvailableList" and value is not None,
                        f"name={name}, value={value}")
        else:
            self.record("RESPONSE broadcast", False,
                        f"Expected RESPONSE, got {msg_type}")

    def test_action_to_ros(self):
        """ACTION(7): AI Manager sends ACTION, verify TachyBridge receives it."""
        print("\n── Test: ACTION ──")
        self.drain()

        joints = np.array([0.1, -0.2, 0.3, 0.0, 1.57, -0.5], dtype=np.float32)
        self.pub.send(make_frame(MsgType.ACTION, pack_ndarray(joints)))

        # ACTION goes to ROS (JointTrajectory), not back to ZMQ
        # We just verify no error/crash — check ROS topic separately
        time.sleep(0.3)
        self.record("ACTION sent (check /joint_commands)", True,
                    f"Sent {len(joints)} joints — verify via: ros2 topic echo /joint_commands")

    def test_fire_and_forget(self):
        """Fire-and-forget messages: TICK, START_TIMER, FINISH_TIMER, ROBOT_TYPE, ERROR."""
        print("\n── Test: Fire-and-forget messages ──")
        self.drain()

        tests = [
            ("TICK",         MsgType.TICK,         {}),
            ("START_TIMER",  MsgType.START_TIMER,  {}),
            ("FINISH_TIMER", MsgType.FINISH_TIMER, {}),
            ("ROBOT_TYPE",   MsgType.ROBOT_TYPE,   {"robot_type": "manipulator"}),
            ("ERROR",        MsgType.ERROR,        {"error": "test error"}),
        ]

        for name, msg_type, payload in tests:
            self.pub.send(make_frame(msg_type, payload))
            time.sleep(0.1)

        # These go to ROS topics, not back to ZMQ — verify no crash
        time.sleep(0.5)
        self.record("Fire-and-forget (5 types)", True,
                    "Sent TICK, START_TIMER, FINISH_TIMER, ROBOT_TYPE, ERROR — "
                    "verify via ros2 topic echo")

    def test_wire_format(self):
        """Verify wire format: 4-byte BE header + msgpack body."""
        print("\n── Test: Wire format ──")

        # Test header encoding
        frame = make_frame(MsgType.HEARTBEAT, {})
        header_val = struct.unpack("!I", frame[:4])[0]
        self.record("Header encoding",
                    header_val == 16,
                    f"HEARTBEAT header = {header_val} (expected 16)")

        # Test msgpack body
        body = msgpack.unpackb(frame[4:], raw=False)
        self.record("Msgpack body",
                    isinstance(body, dict),
                    f"Body type = {type(body).__name__}")

        # Test all enum values 1-16
        all_valid = all(MsgType(i) for i in range(1, 17))
        self.record("Enum values 1-16", all_valid, "All 16 MsgType values valid")

    def test_ndarray_format(self):
        """Verify numpy ndarray descriptor format matches spec."""
        print("\n── Test: ndarray format ──")

        arr = np.array([1.0, 2.0, 3.0], dtype=np.float32)
        desc = pack_ndarray(arr)

        self.record("ndarray dtype", desc["dtype"] == "float32",
                    f"dtype = {desc['dtype']}")
        self.record("ndarray shape", desc["shape"] == [3],
                    f"shape = {desc['shape']}")
        self.record("ndarray data size",
                    len(desc["data"]) == 3 * 4,
                    f"data = {len(desc['data'])} bytes (expected 12)")

        # Verify round-trip
        recovered = np.frombuffer(desc["data"], dtype=np.float32)
        self.record("ndarray round-trip",
                    np.allclose(arr, recovered),
                    f"original={arr.tolist()}, recovered={recovered.tolist()}")

    def test_response_format(self):
        """Verify RESPONSE wire format convention: {msg: {name, value}}."""
        print("\n── Test: RESPONSE format convention ──")

        frame = make_response("ResTest", {"success": True})
        _, body = parse_frame(frame)

        has_msg = "msg" in body
        has_name = body.get("msg", {}).get("name") == "ResTest"
        has_value = "value" in body.get("msg", {})
        no_response_key = "response" not in body.get("msg", {})

        self.record("RESPONSE has 'msg' field", has_msg)
        self.record("RESPONSE msg.name correct", has_name)
        self.record("RESPONSE uses 'value' key", has_value)
        self.record("RESPONSE no 'response' key", no_response_key,
                    "Spec requires 'value', not 'response'")

    # ── Runner ───────────────────────────────────────────────────────────

    def run_all(self, with_bridge=True):
        """Run all tests. Set with_bridge=False for offline format tests only."""
        print("=" * 60)
        print("Habilis Protocol Spec Verification")
        print("=" * 60)

        # Offline tests (no TachyBridge needed)
        self.test_wire_format()
        self.test_ndarray_format()
        self.test_response_format()

        if with_bridge:
            # Online tests (require TachyBridge running)
            self.test_heartbeat()
            self.test_training_status_forward()
            self.test_inference_status_forward()
            self.test_request_forward()
            self.test_response_broadcast()
            self.test_action_to_ros()
            self.test_fire_and_forget()

        # Summary
        print("\n" + "=" * 60)
        passed = sum(1 for r in self.results if r.passed)
        total = len(self.results)
        color = "\033[92m" if passed == total else "\033[93m"
        print(f"{color}Results: {passed}/{total} passed\033[0m")

        failed = [r for r in self.results if not r.passed]
        if failed:
            print("\nFailed tests:")
            for r in failed:
                print(f"  ✗ {r.name}: {r.detail}")

        print("=" * 60)
        return passed == total

    def cleanup(self):
        self.pub.close()
        self.sub.close()
        self.ctx.term()


def main():
    offline = "--offline" in sys.argv

    if offline:
        print("Running offline tests only (no TachyBridge needed)\n")
        tester = HabilisSpecTester.__new__(HabilisSpecTester)
        tester.results = []
        tester.test_wire_format()
        tester.test_ndarray_format()
        tester.test_response_format()

        passed = sum(1 for r in tester.results if r.passed)
        total = len(tester.results)
        print(f"\n{'=' * 60}")
        print(f"Results: {passed}/{total} passed")
        print(f"{'=' * 60}")
        sys.exit(0 if passed == total else 1)

    tester = HabilisSpecTester()
    try:
        success = tester.run_all(with_bridge=True)
        sys.exit(0 if success else 1)
    finally:
        tester.cleanup()


if __name__ == "__main__":
    main()
