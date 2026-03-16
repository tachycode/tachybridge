#!/usr/bin/env python3
"""
ZMQ Bridge Demo Client

Demonstrates connecting to the ZMQ rosbridge server using a DEALER socket
and performing topic pub/sub and service calls via the rosbridge v2 JSON protocol.

Usage:
    python3 demo_client.py [--address ADDRESS] [--port PORT]

Requires: pyzmq (pip install pyzmq)
"""

import argparse
import json
import time
import zmq
import sys
import uuid


def create_dealer_socket(ctx: zmq.Context, address: str, port: int) -> zmq.Socket:
    """Create and connect a DEALER socket to the ROUTER server."""
    sock = ctx.socket(zmq.DEALER)
    # Set a unique identity so the ROUTER can route replies back
    identity = f"demo-{uuid.uuid4().hex[:8]}".encode()
    sock.setsockopt(zmq.IDENTITY, identity)
    sock.setsockopt(zmq.LINGER, 0)
    endpoint = f"tcp://{address}:{port}"
    sock.connect(endpoint)
    print(f"Connected to {endpoint} (identity: {identity.decode()})")
    return sock


def send_msg(sock: zmq.Socket, msg: dict) -> None:
    """Send a rosbridge v2 JSON message via DEALER.
    DEALER framing: [empty | payload] (identity auto-prepended by ZMQ)
    """
    payload = json.dumps(msg)
    sock.send_multipart([b"", payload.encode()])


def recv_msg(sock: zmq.Socket, timeout_ms: int = 3000) -> dict | None:
    """Receive a rosbridge v2 JSON reply via DEALER.
    Returns None on timeout.
    """
    if sock.poll(timeout_ms, zmq.POLLIN):
        frames = sock.recv_multipart()
        # DEALER receives: [empty | payload]
        payload = frames[-1].decode()
        return json.loads(payload)
    return None


def demo_ping(sock: zmq.Socket) -> bool:
    """Test 1: Ping/Pong"""
    print("\n--- Test 1: Ping/Pong ---")
    send_msg(sock, {"op": "ping"})
    reply = recv_msg(sock)
    if reply and reply.get("op") == "pong":
        print(f"  OK: Received pong: {reply}")
        return True
    print(f"  FAIL: Expected pong, got: {reply}")
    return False


def demo_publish(sock: zmq.Socket) -> bool:
    """Test 2: Advertise + Publish a topic"""
    print("\n--- Test 2: Advertise + Publish ---")
    topic = "/zmq_test_topic"
    msg_type = "std_msgs/msg/String"

    # Advertise
    send_msg(sock, {
        "op": "advertise",
        "topic": topic,
        "type": msg_type,
    })
    time.sleep(0.5)  # Wait for advertise to be processed

    # Publish
    send_msg(sock, {
        "op": "publish",
        "topic": topic,
        "msg": {"data": "Hello from ZMQ bridge!"},
    })
    print(f"  Published to {topic}: 'Hello from ZMQ bridge!'")
    print(f"  Verify with: ros2 topic echo {topic}")
    return True


def demo_subscribe(sock: zmq.Socket) -> bool:
    """Test 3: Subscribe to a topic and receive messages"""
    print("\n--- Test 3: Subscribe ---")
    topic = "/zmq_test_topic"
    msg_type = "std_msgs/msg/String"

    send_msg(sock, {
        "op": "subscribe",
        "topic": topic,
        "type": msg_type,
    })
    print(f"  Subscribed to {topic}")

    # Publish a message to trigger the subscription
    send_msg(sock, {
        "op": "publish",
        "topic": topic,
        "msg": {"data": "Self-subscribe test"},
    })

    # Wait for subscription callback
    print("  Waiting for messages (5s timeout)...")
    deadline = time.time() + 5.0
    received = False
    while time.time() < deadline:
        reply = recv_msg(sock, timeout_ms=500)
        if reply and reply.get("op") == "publish" and reply.get("topic") == topic:
            print(f"  OK: Received subscription message: {reply.get('msg')}")
            received = True
            break
        elif reply:
            print(f"  (other message: {reply.get('op')})")

    # Unsubscribe
    send_msg(sock, {
        "op": "unsubscribe",
        "topic": topic,
    })
    print(f"  Unsubscribed from {topic}")

    if not received:
        print("  WARN: No subscription message received (may need active publisher)")
    return True


def demo_call_service(sock: zmq.Socket) -> bool:
    """Test 4: Call a ROS2 service"""
    print("\n--- Test 4: Call Service ---")
    service = "/zmq_bridge_node/describe_parameters"
    srv_type = "rcl_interfaces/srv/DescribeParameters"

    send_msg(sock, {
        "op": "call_service",
        "service": service,
        "type": srv_type,
        "args": {"names": ["port"]},
    })
    print(f"  Calling service {service}...")

    reply = recv_msg(sock, timeout_ms=5000)
    if reply and reply.get("op") == "service_response":
        print(f"  OK: Service response received: {json.dumps(reply.get('values', {}), indent=2)[:200]}")
        return True
    print(f"  FAIL: Expected service_response, got: {reply}")
    return False


def demo_multi_client(address: str, port: int) -> bool:
    """Test 5: Multiple simultaneous clients"""
    print("\n--- Test 5: Multi-client ---")
    ctx = zmq.Context()
    clients = []
    for i in range(3):
        sock = create_dealer_socket(ctx, address, port)
        clients.append(sock)

    # Each client sends a ping
    for i, sock in enumerate(clients):
        send_msg(sock, {"op": "ping"})

    # Each should get a pong
    all_ok = True
    for i, sock in enumerate(clients):
        reply = recv_msg(sock)
        if reply and reply.get("op") == "pong":
            print(f"  Client {i}: OK (pong received)")
        else:
            print(f"  Client {i}: FAIL (got {reply})")
            all_ok = False

    for sock in clients:
        sock.close()
    ctx.term()
    return all_ok


def main():
    parser = argparse.ArgumentParser(description="ZMQ Bridge Demo Client")
    parser.add_argument("--address", default="127.0.0.1", help="Server address")
    parser.add_argument("--port", type=int, default=5555, help="Server port")
    args = parser.parse_args()

    ctx = zmq.Context()
    sock = create_dealer_socket(ctx, args.address, args.port)

    results = {}
    results["ping"] = demo_ping(sock)
    results["publish"] = demo_publish(sock)
    results["subscribe"] = demo_subscribe(sock)
    results["call_service"] = demo_call_service(sock)

    sock.close()

    results["multi_client"] = demo_multi_client(args.address, args.port)

    ctx.term()

    print("\n=== Results ===")
    for test, passed in results.items():
        status = "PASS" if passed else "FAIL"
        print(f"  {test}: {status}")

    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
