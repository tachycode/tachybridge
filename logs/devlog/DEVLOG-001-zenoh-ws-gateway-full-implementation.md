---
id: DEVLOG-001
title: Zenoh-WS Gateway 전체 구현 (Phase 1 + 2a + 2b)
task_type: feature
status: completed
complexity: high
user_validation: pending
created: 2026-04-10
duration_estimate: 4h
tags: [zenoh, websocket, gateway, mcap, ros2, ccg, multi-room, tdd, docker]
---

## 목표 (Goal)
- MCAP(ros2bag) 데이터를 여러 PC에서 독립적으로 재생/탐색할 수 있는 Zenoh-WS Gateway 시스템 구축
- TachyBridge의 ROS bridge 역할은 유지하면서, MCAP 재생 파이프라인만 Direct Zenoh로 분리
- 데이터: 카메라 3대 (CompressedImage), EE Pose 3개 (geometry_msgs/Pose)

## 접근 과정 (Approach Log)

### 1차: CCG 트라이모델 아키텍처 토론
- **방법**: Codex(GPT-5.4) + Gemini + Claude로 아키텍처 병렬 검토
- **결과**: 핵심 결정 합의
  - SharedFrame ref-counting (zero-copy fan-out)
  - Dual WebSocket (image/control 분리 — TCP HOL 방지)
  - Direct Zenoh (ROS RMW 미사용 — CDR 오버헤드 제거)
  - Fixed-layout binary pose (f64[7] — CBOR 대비 CPU 절약)
- **핵심 충돌**: Pose 인코딩 (Codex=CBOR vs Gemini=raw f64[7]) → Gemini 안 채택
- **Gemini 고유 기여**: JS SDK 필요성 제안, Direct Zenoh zero-copy 제안, Room/Session 개념

### 2차: Phase 1 구현 (12 tasks, subagent-driven)
- **방법**: 플랜 작성 → subagent per task 병렬 디스패치 → Opus 코드 리뷰
- **결과**: 성공. 16 unit tests + 1 integration test 통과
- **문제**: 개발 머신에 cmake/ROS 미설치 → Docker 빌드 환경 구축으로 해결
- **문제**: zenoh-cpp API 불일치 (Sample& not const, Subscriber<void>, Bytes move-only) → API 정렬 커밋
- **Opus 리뷰 발견**: C2(shutdown order), C3(dangling this), C4(double disconnect), I2(cout under lock) → 전부 수정

### 3차: Phase 2a (4 tasks — dual WS, rooms, seek fix)
- **방법**: HTTP upgrade path detection → Room struct → path-based routing → MCAP reader reopen
- **결과**: 성공. /room/{id} 경로 기반 라우팅, 룸 격리, late-join, backward seek
- **교훈**: identify_transport 리팩토링이 가장 복잡 — Opus 모델로 디스패치 적절

### 4차: Phase 2b (4 tasks — process spawning, JS SDK, quality, scale test)
- **방법**: 4개 태스크 전부 병렬 (C++, TypeScript, Python 혼합)
- **결과**: fork/exec 프로세스 관리, TachyBridge-JS SDK, 드롭 추적 + quality_hint, 50-client 스케일 테스트

### 5차: E2E 통합 테스트 (TDD)
- **방법**: 6개 e2e 테스트 작성 (Zenoh→Room→Client, late-join, cleanup, backpressure, quality, multi-room isolation)
- **결과**: Docker에서 첫 실행 37/37 tests 전부 통과

## 최종 해결 (Final Solution)
- **cpp_zenoh_gateway**: Zenoh → WebSocket gateway (SharedFrame, StreamLane, FanoutHub, Room, dual WS)
- **cpp_mcap_reader**: MCAP → Zenoh publisher (CDR extractor, playback control, backward seek)
- **tachybridge-js**: TypeScript 브라우저 SDK (dual WS, binary frame parsing, typed callbacks)
- **tests/scale_test.py**: 50-client asyncio 부하 테스트
- **docker/Dockerfile.build-test**: ROS 2 Jazzy + zenoh-cpp 빌드 환경
- 전체 ROS 의존 없음 — Direct Zenoh + manual CDR parsing
- Room 기반 YouTube 모델 (룸당 독립 mcap_reader 프로세스)

## 교훈 (Lessons Learned)
- zenoh-cpp API는 버전별 차이 큼 — 플랜 단계에서 Docker로 실제 API 확인이 이상적
- CCG(3-model consensus)는 아키텍처 결정에 효과적 — 충돌 지점이 가장 가치 있는 논의
- Subagent 병렬 디스패치 시 git 커밋이 분리되므로 통합 스테이징 필요
- Opus는 복잡한 리팩토링에, Sonnet은 독립 컴포넌트 구현에 적합한 모델 라우팅
- Docker 빌드 환경을 프로젝트 초기에 구축하면 빌드 검증 사이클이 빨라짐
- TSG-001 성능 규칙(lock scope, bounded queues, no I/O under locks)이 코드 리뷰에서 효과적 가이드라인

## 변경 파일 (Changed Files)
- `cpp_zenoh_gateway/` — 신규 패키지 (wire_format, stream_lane, fanout_hub, room, gateway_session, gateway_server, main, 7 test files)
- `cpp_mcap_reader/` — 신규 패키지 (cdr_extractor, mcap_publisher, playback_controller, main, 2 test files)
- `tachybridge-js/` — 신규 JS SDK (package.json, tsconfig.json, index.ts, client.ts)
- `tests/scale_test.py` — 50-client 부하 테스트
- `tests/requirements.txt` — websockets dependency
- `docker/Dockerfile.build-test` — 빌드 환경
- `docs/superpowers/plans/` — Phase 1, 2a, 2b 구현 플랜
