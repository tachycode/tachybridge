---
id: DEVLOG-002
title: Habilis ZMQ protocol bridge (PUB/SUB + dispatcher)
task_type: feature
status: completed
complexity: high
created: 2026-03-20
duration_estimate: 4h
tags: [zmq, habilis, dispatcher, observation, action, msgpack, ros2, pub-sub]
---

## 목표 (Goal)
- ZMQ ROUTER 기반 rosbridge v2 서버(DEVLOG-001)에 Habilis 바이너리 프로토콜 지원을 추가
- AI Manager(Python) ↔ TachyBridge(C++) ↔ ROS 2 간 PUB/SUB 브릿지 구현
- 16개 메시지 타입(HEARTBEAT, TICK, ACTION, OBSERVATION 등)의 양방향 라우팅
- ZMQ 프로토콜 테스트 방법 문서화

## 접근 과정 (Approach Log)

### 1차: Habilis 프로토콜 타입 시스템 설계
- **방법**: Python constants.py의 MsgType enum을 C++ enum class로 1:1 매핑, wire format(4-byte BE header + msgpack body)을 habilis_types.hpp에 구현
- **결과**: 성공 — parse/serialize 유틸리티, NdarrayDesc(numpy 호환) 포함
- **교훈**: Python auto() enum은 1부터 시작 — C++ 쪽도 명시적으로 1부터 매핑해야 함

### 2차: HabilisBridge (PUB/SUB transport layer)
- **방법**: 전용 poll 스레드에서 SUB 소켓 수신, bounded queue(1024)로 PUB 전송, lock-and-swap drain 패턴 적용
- **결과**: 성공 — TSG-001 규칙(I/O thread blocking 금지, bounded queue) 준수
- **교훈**: ZMQ 소켓은 단일 스레드에서만 접근 — send()는 큐에 enqueue하고 poll thread에서 drain

### 3차: HabilisDispatcher (message routing)
- **방법**: 16개 메시지 핸들러 구현. AI→ROS: 각 타입별 publisher, ROS→AI: camera/joint subscriber → OBSERVATION 조립(30fps wall timer)
- **결과**: 성공 — ACTION은 numpy ndarray → JointTrajectory 변환, REQUEST/RESPONSE는 correlation tracking(30s timeout, max 64)
- **교훈**: OBSERVATION 조립 시 all-or-nothing 패턴 — 카메라/조인트 중 하나라도 없으면 프레임 skip

### 4차: ZmqBridgeNode 통합 + 테스트 문서
- **방법**: LifecycleNode에 ZmqServer(ROUTER) + HabilisBridge(PUB/SUB) 2개 스레드 통합, Python 테스트 스크립트 5종 작성
- **결과**: 성공 — heartbeat round-trip, inference lifecycle, action command, request/response, full message type 테스트 커버

## 최종 해결 (Final Solution)
- habilis_types.hpp: 16개 MsgType enum + wire format parser/serializer + numpy array helper
- habilis_bridge.hpp/cpp: PUB/SUB transport with bounded queue, dedicated poll thread
- habilis_dispatcher.hpp/cpp: 16개 핸들러, OBSERVATION assembly(camera+joint→msgpack), ACTION→JointTrajectory
- zmq_server_node.cpp: 2개 프로토콜(ROUTER + PUB/SUB) lifecycle 관리
- docs/testing/zmq-habilis-protocol-testing.md: Python 테스트 가이드 5종

## 교훈 (Lessons Learned)
- **msgpack numpy 호환**: `{dtype, shape, data}` 포맷을 C++/Python 양쪽에서 동일하게 유지해야 상호운용 가능
- **PUB/SUB handshake 지연**: ZMQ PUB/SUB는 연결 직후 메시지 유실 가능 — 테스트 시 1초 sleep 필요
- **OBSERVATION은 START_INFERENCE 이후에만 전송**: inference_active 플래그로 스트림 제어, 불필요한 센서 데이터 전송 방지
- **Wire format 통일이 핵심**: 4-byte BE header + msgpack body를 양쪽에서 정확히 맞추면 언어 무관하게 통신 가능

## 변경 파일 (Changed Files)
- `cpp_zmq_server/include/cpp_zmq_server/habilis_types.hpp` — MsgType enum, wire format utilities, ndarray helpers
- `cpp_zmq_server/include/cpp_zmq_server/habilis_bridge.hpp` — PUB/SUB bridge class definition
- `cpp_zmq_server/include/cpp_zmq_server/habilis_dispatcher.hpp` — Message dispatcher class definition
- `cpp_zmq_server/src/habilis_bridge.cpp` — PUB/SUB poll loop, bounded queue implementation
- `cpp_zmq_server/src/habilis_dispatcher.cpp` — 16 message handlers, OBSERVATION assembly
- `cpp_zmq_server/src/zmq_server_node.cpp` — Dual-protocol lifecycle integration
- `cpp_zmq_server/CMakeLists.txt` — Added habilis sources and msgpack dependency
- `cpp_zmq_server/package.xml` — Updated dependencies
- `docs/testing/zmq-habilis-protocol-testing.md` — ZMQ protocol testing guide (NEW)
