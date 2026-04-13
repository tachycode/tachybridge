# Docker Dependencies (cyclone branch)

이 문서는 `cyclone` 브랜치를 Docker 컨테이너에서 빌드/실행하는 데 필요한 모든 의존성을 정리합니다.
기반 참조: `habilis_communicator/docker/Dockerfile.amd64`, `Dockerfile.arm64`

---

## Base Image

```
ros:jazzy-ros-base   # Ubuntu 24.04 + ROS 2 Jazzy
```

---

## Build Stage 의존성 (apt)

소스 컴파일에만 필요. 런타임 이미지에는 불필요.

```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    curl ca-certificates gnupg \
    nlohmann-json3-dev \
    libboost-system-dev \
    git
```

| 패키지 | 용도 |
|---|---|
| `python3-rosdep` | `rosdep install`로 ROS 의존성 자동 해소 |
| `nlohmann-json3-dev` | C++ JSON 파싱 (모든 패키지 공통) |
| `libboost-system-dev` | Boost.Asio / thread pool |
| `curl`, `ca-certificates`, `gnupg` | rosdep 업데이트 시 HTTPS 접근 |
| `git` | 소스 클론 |

---

## Runtime Stage 의존성 (apt)

컨테이너 실행 이미지에 포함되어야 하는 패키지.

```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config \
    python3 python3-pip python3-venv python3-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rmw-zenoh-cpp \
    nginx \
    openssl \
    libboost-system-dev \
    nlohmann-json3-dev
```

| 패키지 | 용도 |
|---|---|
| `ros-jazzy-rmw-cyclonedds-cpp` | **cyclone 브랜치 기본 미들웨어** |
| `ros-jazzy-rmw-zenoh-cpp` | Zenoh 미들웨어 (선택적 전환용) |
| `libboost-system-dev` | 런타임 Boost 링크 |
| `nlohmann-json3-dev` | 런타임 JSON 헤더 |
| `nginx` | 리버스 프록시 |
| `openssl` | TLS 인증서 생성 |
| `python3-*` | habilis_communicator Python 패키지 실행 |

---

## libzmq (소스 빌드 필수)

**Draft API(`RADIO`/`DISH` 소켓)가 필요하므로 apt 패키지 사용 불가.**

```dockerfile
ARG LIBZMQ_REF=v4.3.5
RUN git clone --depth 1 --branch ${LIBZMQ_REF} https://github.com/zeromq/libzmq.git /tmp/libzmq && \
    cmake -S /tmp/libzmq -B /tmp/libzmq/build \
        -DENABLE_DRAFTS=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /tmp/libzmq/build -j$(nproc) && \
    cmake --install /tmp/libzmq/build && \
    ldconfig && \
    rm -rf /tmp/libzmq
```

> **ARM64 주의**: 크로스 컴파일 이슈로 별도 `zmq-cross` 스테이지에서 빌드 후 `COPY --from=zmq-cross` 로 가져와야 합니다. (`Dockerfile.arm64` 참고)

---

## Python 패키지 (pip)

```dockerfile
RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel \
        --break-system-packages && \
    python3 -m pip uninstall -y pyzmq --break-system-packages || true && \
    python3 -m pip install --no-cache-dir --no-binary=pyzmq pyzmq \
        --break-system-packages && \
    python3 -m pip install --no-cache-dir --ignore-installed \
        msgpack opencv-python-headless websockets --break-system-packages
```

| 패키지 | 버전 | 비고 |
|---|---|---|
| `pyzmq` | latest | **반드시 소스 빌드** (`--no-binary=pyzmq`). 위에서 빌드한 libzmq v4.3.5와 링크되어야 함 |
| `msgpack` | latest | ZMQ 메시지 직렬화 |
| `opencv-python-headless` | latest | GUI 없는 OpenCV (컨테이너용) |
| `websockets` | latest | Python WebSocket 클라이언트 |

---

## ROS 미들웨어 환경변수

cyclone 브랜치 기본값:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Zenoh로 전환하려면:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

> 컨테이너 내부에서 `ros2 launch cpp_rosbridge_server rosbridge_websocket_launch.py rmw:=rmw_zenoh_cpp` 로도 전환 가능.

---

## rosdep 처리

빌드 시 ROS 의존성(package.xml 기반) 자동 해소:

```bash
rosdep init || true
rosdep update
rosdep install -i --from-paths src --rosdistro jazzy -y \
    --skip-keys='physical_ai_interfaces'
```

`physical_ai_interfaces`는 로컬 패키지이므로 rosdep skip 처리.

---

## 전체 빌드 흐름 요약

```
[Builder stage]
  ros:jazzy-ros-base
  + apt: rosdep, nlohmann-json, boost, git
  + rosdep install (ROS deps)
  + colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

[Runtime stage]
  ros:jazzy-ros-base
  + apt: python3, nginx, openssl, boost, nlohmann-json,
         rmw-cyclonedds-cpp, rmw-zenoh-cpp
  + libzmq v4.3.5 (소스, -DENABLE_DRAFTS=ON)
  + pip: pyzmq(소스), msgpack, opencv-headless, websockets
  + COPY install/ from builder
```

---

## 아키텍처별 참조 Dockerfile

| 아키텍처 | 파일 |
|---|---|
| x86_64 (amd64) | `habilis_communicator/docker/Dockerfile.amd64` |
| ARM64 (Jetson 등) | `habilis_communicator/docker/Dockerfile.arm64` |
