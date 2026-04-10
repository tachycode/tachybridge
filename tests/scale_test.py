#!/usr/bin/env python3
"""
Scale test for TachyBridge Zenoh-WS Gateway.
Simulates N concurrent WebSocket clients, subscribes to all topics,
and reports latency, throughput, and frame drop statistics.

Usage:
    python3 tests/scale_test.py [--url ws://localhost:9090] [--room 01] [--clients 50] [--duration 30]
"""

import argparse
import asyncio
import json
import struct
import time
from dataclasses import dataclass, field
from typing import Optional

try:
    import websockets
except ImportError:
    print("Install websockets: pip install websockets")
    exit(1)


STREAM_TYPE_IMAGE = 0x01
STREAM_TYPE_POSE = 0x02
HEADER_SIZE = 6


@dataclass
class ClientStats:
    client_id: int
    image_frames: int = 0
    pose_frames: int = 0
    total_bytes: int = 0
    first_frame_time: Optional[float] = None
    last_frame_time: Optional[float] = None
    quality_hints: int = 0
    errors: int = 0
    connected: bool = False
    latencies_ms: list = field(default_factory=list)


async def run_client(
    client_id: int,
    url: str,
    room_id: str,
    duration: float,
    stats: ClientStats,
):
    """Single client: connect control + image WS, subscribe all, collect stats."""
    try:
        # 1. Connect control WebSocket
        control_url = f"{url}/room/{room_id}"
        async with websockets.connect(control_url) as control_ws:
            # Wait for welcome
            welcome_raw = await asyncio.wait_for(control_ws.recv(), timeout=5.0)
            welcome = json.loads(welcome_raw)
            if welcome.get("op") != "welcome":
                stats.errors += 1
                return

            stats.connected = True
            topics = welcome.get("topics", [])
            session_id = welcome.get("session_id", 0)

            # 2. Connect image WebSocket
            image_url = f"{url}/room/{room_id}/image"
            async with websockets.connect(image_url) as image_ws:

                # 3. Subscribe to all topics
                for t in topics:
                    await control_ws.send(json.dumps({
                        "op": "subscribe",
                        "topic": t["name"],
                    }))

                # 4. Receive frames for duration
                start = time.monotonic()

                async def recv_control():
                    while time.monotonic() - start < duration:
                        try:
                            msg = await asyncio.wait_for(control_ws.recv(), timeout=1.0)
                            recv_time = time.monotonic()
                            if isinstance(msg, bytes):
                                parse_binary(msg, stats, recv_time)
                            elif isinstance(msg, str):
                                data = json.loads(msg)
                                if data.get("op") == "quality_hint":
                                    stats.quality_hints += 1
                        except asyncio.TimeoutError:
                            continue
                        except Exception:
                            stats.errors += 1
                            break

                async def recv_image():
                    while time.monotonic() - start < duration:
                        try:
                            msg = await asyncio.wait_for(image_ws.recv(), timeout=1.0)
                            recv_time = time.monotonic()
                            if isinstance(msg, bytes):
                                parse_binary(msg, stats, recv_time)
                        except asyncio.TimeoutError:
                            continue
                        except Exception:
                            stats.errors += 1
                            break

                await asyncio.gather(recv_control(), recv_image())

    except Exception as e:
        stats.errors += 1
        print(f"  Client {client_id}: ERROR - {e}")


def parse_binary(data: bytes, stats: ClientStats, recv_time: float):
    if len(data) < HEADER_SIZE:
        return
    stream_type = data[0]
    # topic_id = data[1]
    ts_offset_ms = struct.unpack(">I", data[2:6])[0]

    now = recv_time
    if stats.first_frame_time is None:
        stats.first_frame_time = now
    stats.last_frame_time = now
    stats.total_bytes += len(data)

    if stream_type == STREAM_TYPE_IMAGE:
        stats.image_frames += 1
    elif stream_type == STREAM_TYPE_POSE:
        stats.pose_frames += 1


async def main():
    parser = argparse.ArgumentParser(description="TachyBridge Scale Test")
    parser.add_argument("--url", default="ws://localhost:9090", help="Gateway URL")
    parser.add_argument("--room", default="01", help="Room ID")
    parser.add_argument("--clients", type=int, default=50, help="Number of clients")
    parser.add_argument("--duration", type=float, default=30.0, help="Test duration (seconds)")
    args = parser.parse_args()

    print(f"Scale Test: {args.clients} clients -> {args.url}/room/{args.room}")
    print(f"Duration: {args.duration}s")
    print()

    all_stats = [ClientStats(client_id=i) for i in range(args.clients)]

    # Stagger connections slightly to avoid thundering herd
    tasks = []
    for i in range(args.clients):
        task = asyncio.create_task(
            run_client(i, args.url, args.room, args.duration, all_stats[i])
        )
        tasks.append(task)
        if i % 10 == 9:
            await asyncio.sleep(0.1)  # 10 clients per 100ms

    await asyncio.gather(*tasks)

    # Report
    print("\n" + "=" * 60)
    print("SCALE TEST RESULTS")
    print("=" * 60)

    connected = sum(1 for s in all_stats if s.connected)
    total_images = sum(s.image_frames for s in all_stats)
    total_poses = sum(s.pose_frames for s in all_stats)
    total_bytes = sum(s.total_bytes for s in all_stats)
    total_errors = sum(s.errors for s in all_stats)
    total_quality = sum(s.quality_hints for s in all_stats)

    print(f"Clients connected:  {connected}/{args.clients}")
    print(f"Total image frames: {total_images}")
    print(f"Total pose frames:  {total_poses}")
    print(f"Total data:         {total_bytes / 1024 / 1024:.1f} MB")
    print(f"Quality hints:      {total_quality}")
    print(f"Errors:             {total_errors}")

    if connected > 0:
        avg_images = total_images / connected
        avg_poses = total_poses / connected
        print(f"\nPer-client average:")
        print(f"  Image frames: {avg_images:.0f}")
        print(f"  Pose frames:  {avg_poses:.0f}")
        print(f"  Data:         {total_bytes / connected / 1024 / 1024:.1f} MB")

        # Estimate FPS
        durations = []
        for s in all_stats:
            if s.first_frame_time and s.last_frame_time:
                d = s.last_frame_time - s.first_frame_time
                if d > 0:
                    durations.append(d)

        if durations:
            avg_dur = sum(durations) / len(durations)
            print(f"\nThroughput (avg over {avg_dur:.1f}s):")
            print(f"  Image FPS: {avg_images / avg_dur:.1f}")
            print(f"  Pose Hz:   {avg_poses / avg_dur:.1f}")

    print()
    if total_errors > 0:
        print(f"WARNING: {total_errors} errors occurred")
    if total_quality > 0:
        print(f"NOTE: {total_quality} quality hints received (slow clients detected)")

    # Per-client breakdown for outliers
    if connected > 0:
        min_imgs = min(s.image_frames for s in all_stats if s.connected)
        max_imgs = max(s.image_frames for s in all_stats if s.connected)
        if max_imgs > 0 and min_imgs < max_imgs * 0.5:
            print(f"\nWARNING: Large frame count variance: min={min_imgs}, max={max_imgs}")
            slow = [s for s in all_stats if s.connected and s.image_frames < max_imgs * 0.5]
            print(f"  {len(slow)} slow clients (< 50% of max)")


if __name__ == "__main__":
    asyncio.run(main())
