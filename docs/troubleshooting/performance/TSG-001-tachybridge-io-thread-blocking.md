---
id: TSG-001
title: TachyBridge WebSocket bridge performance bottlenecks — I/O thread blocking, work pool starvation, OOM risk
status: resolved
category: performance
severity: critical
created: 2026-03-12
updated: 2026-03-12
tags: [websocket, thread-pool, backpressure, fan-out, service-blocking, OOM, subscription-leak, boost-asio]
related: []
---

## 증상
- Service 호출 시 WebSocket 서버 전체가 최대 5초간 응답 불가 (I/O 스레드 차단)
- 동시 서비스 호출 3개 이상 시 work pool 고갈로 무한 대기
- 느린 클라이언트 연결 시 메모리 사용량 무한 증가 (OOM 크래시 위험)
- 클라이언트 연결 끊김 후에도 ROS 구독이 정리되지 않아 좀비 구독 누적
- 고부하 시 과도한 INFO 로그로 추가 성능 저하

## 원인

6개의 독립적인 성능 이슈가 동시에 존재:

| ID | 심각도 | 파일 | 원인 |
|----|--------|------|------|
| C2 | Critical | `service.cpp:83-91` | `sleep_for(100ms)` x 50회 폴링이 WebSocket I/O 스레드에서 직접 실행 |
| C1 | Critical | `rosbridge_server_node.cpp:125` | `work_pool_{2}` 하드코딩. `num_threads` 파라미터가 work pool에 미적용 |
| H1 | High | `websocket_server.hpp:41` | `Session::write_queue_` (std::deque)에 크기 제한 없음 |
| H2 | High | `protocol.cpp:305-318` | Fan-out 시 `shared_lock` 보유 상태에서 순차 전송 + JSON N회 직렬화 |
| H3 | High | `websocket_server.hpp:117` | `on_read` 에러 시 `unsubscribe_all()` 미호출 |
| M1 | Medium | `service.cpp:69`, `action.hpp:109` 등 | 모든 서비스/구독/액션에 `RCLCPP_INFO` 사용 |

## 해결

### C2: Service readiness 폴링을 work pool로 이동
```cpp
// Before: I/O 스레드에서 5초 차단
for (int i = 0; i < 50 && !ready; ++i) {
    std::this_thread::sleep_for(100ms);  // BLOCKS I/O!
}

// After: work pool에서 실행, I/O 스레드 즉시 반환
protocol_->post_work([...]() {
    // polling here — safe on work pool thread
});
```

### C1: work_pool 파라미터화
```cpp
// Before:
boost::asio::thread_pool work_pool_{2};  // hardcoded

// After:
this->declare_parameter<int>("work_pool_threads", 4);
work_pool_ = std::make_unique<boost::asio::thread_pool>(pool_size);
```

### H1: Write queue 크기 제한
```cpp
static constexpr size_t kMaxWriteQueueSize = 1024;
// send_text/send_binary 에서:
if (write_queue_.size() >= kMaxWriteQueueSize) {
    write_queue_.pop_front();  // drop oldest
}
```

### H2: Fan-out lock 최소화 + serialize-once
```cpp
// Before: lock 내에서 순차 send
std::shared_lock lock(mutex_);
for (auto& [sid, send_fn] : senders) { send_fn(msg); }

// After: snapshot 복사 후 lock 해제, fan-out
std::vector<...> snapshot;
{ std::shared_lock lock(mutex_); /* copy senders */ }
for (auto& [sid, send_fn] : snapshot) { send_fn(*serialized); }
```

### H3: Session disconnect 콜백 연결
```cpp
session->set_on_disconnect([this](uint64_t id) {
    protocol_->subscription_manager().unsubscribe_all(id);
});
```

### M1: 로깅 레벨 조정
- Hot-path `RCLCPP_INFO` → `RCLCPP_DEBUG`
- 텍스트 메시지 로그: 전체 내용 → 크기만 출력

## 성능 측정 결과

| 항목 | Before | After | 개선 |
|------|--------|-------|------|
| Service call caller 차단 시간 | 5,004 ms | 0.02 ms | **250,000x** |
| Work pool 동시 처리 | 2 threads (하드코딩) | 4+ threads (설정 가능) | 유연성 확보 |
| Write queue | 무제한 (OOM 위험) | 1024 제한 | OOM 방지 |
| Fan-out lock 범위 | 전체 send 포함 | snapshot 복사만 | 지연 스파이크 제거 |
| Disconnect 정리 | 없음 (좀비 누적) | 자동 unsubscribe_all | 메모리 누수 방지 |

## 핵심 개념
- **Boost.Asio I/O thread에서 blocking 금지**: `sleep_for`, `wait_for_service` 등 차단 호출은 해당 스레드의 모든 세션을 멈춤
- **Backpressure 패턴**: 무제한 큐는 반드시 OOM으로 귀결. High-water mark + drop/disconnect 필수
- **Lock scope 최소화**: Lock 내에서 CPU-heavy 작업(JSON 직렬화) 또는 I/O(send) 금지. Snapshot-copy 패턴 사용
- **RAII 리소스 정리**: 세션 lifecycle에 맞춰 구독/클라이언트 캐시 자동 정리

## 재발 방지
- [ ] WebSocket I/O 핸들러에서 `sleep_for`, `wait_for` 등 blocking 호출 금지 규칙 코드 리뷰 체크리스트 추가
- [ ] 모든 큐/버퍼에 크기 제한 필수 (PR 리뷰 시 확인)
- [ ] 새 리소스 할당 시 해제 경로 반드시 함께 구현
- [ ] Hot-path 로깅은 `RCLCPP_DEBUG` 또는 `RCLCPP_INFO_THROTTLE` 사용
