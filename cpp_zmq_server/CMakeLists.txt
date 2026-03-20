cmake_minimum_required(VERSION 3.8)
project(cpp_zmq_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(cpp_rosbridge_core REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)
find_package(nlohmann_json 3.7.0 REQUIRED)

# ZMQ: use pkg-config since no ament package exists
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# msgpack-cxx: header-only library for habilis binary protocol
include(FetchContent)
FetchContent_Declare(
  msgpack-cxx
  GIT_REPOSITORY https://github.com/msgpack/msgpack-c.git
  GIT_TAG        cpp-6.1.1
)
set(MSGPACK_USE_BOOST OFF CACHE BOOL "" FORCE)
set(MSGPACK_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MSGPACK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(msgpack-cxx)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(
  zmq_bridge_node
  src/zmq_server.cpp
  src/zmq_server_node.cpp
  src/habilis_bridge.cpp
  src/habilis_dispatcher.cpp
)

target_include_directories(zmq_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ZMQ_INCLUDE_DIRS}
)

ament_target_dependencies(
  zmq_bridge_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  cpp_rosbridge_core
  std_msgs
  sensor_msgs
  trajectory_msgs
)

target_link_libraries(
  zmq_bridge_node
  Boost::system
  Threads::Threads
  nlohmann_json::nlohmann_json
  ${ZMQ_LIBRARIES}
  msgpack-cxx
)

install(
  TARGETS zmq_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS scripts/demo_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
