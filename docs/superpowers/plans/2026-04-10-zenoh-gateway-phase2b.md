# Zenoh-WS Gateway Phase 2b — Process Spawning, JS SDK, Adaptive Quality

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Complete the multi-room YouTube model: gateway spawns/kills mcap_reader processes per room, browser clients use a JS SDK, slow clients get adaptive frame skipping, validated at 50-client scale.

**Architecture:** Gateway manages mcap_reader child processes via fork/exec. JS SDK wraps dual WebSocket with typed event callbacks. StreamLane tracks drain rate for adaptive quality hints.

**Tech Stack:** C++17 (process management), TypeScript (JS SDK), Python (scale test)

---

## Task 1: Room Process Spawning

Gateway spawns mcap_reader_node as a child process when a room is created, kills it when the room is destroyed.

**Files:**
- Modify: `cpp_zenoh_gateway/include/cpp_zenoh_gateway/room.hpp` — add process management
- Modify: `cpp_zenoh_gateway/src/room.cpp` — spawn/kill logic
- Modify: `cpp_zenoh_gateway/src/gateway_server.cpp` — handle create_room control op

## Task 2: TachyBridge-JS SDK

Lightweight TypeScript library for browser clients.

**Files:**
- Create: `tachybridge-js/package.json`
- Create: `tachybridge-js/src/index.ts`
- Create: `tachybridge-js/src/client.ts` — TachyBridgeClient class

## Task 3: Adaptive Quality

Monitor per-client drain rate, emit quality_hint when frames are consistently dropped.

**Files:**
- Modify: `cpp_zenoh_gateway/include/cpp_zenoh_gateway/stream_lane.hpp` — add drop counter
- Modify: `cpp_zenoh_gateway/src/stream_lane.cpp` — track drops
- Modify: `cpp_zenoh_gateway/src/gateway_server.cpp` — periodic quality check

## Task 4: Scale Test Script

Python script simulating 50 concurrent WebSocket clients.

**Files:**
- Create: `tests/scale_test.py`
