cmake_minimum_required(VERSION 3.8)
project(test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)
find_package(cpp_rosbridge_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nlohmann_json 3.7.0 REQUIRED)
find_package(std_msgs REQUIRED)

# ZMQ dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

if(BUILD_TESTING)
  # ── Rosbridge core tests ──────────────────────────────────────────────────

  ament_add_gtest(test_protocol src/test_protocol.cpp)
  if(TARGET test_protocol)
    ament_target_dependencies(test_protocol cpp_rosbridge_core rclcpp_lifecycle std_msgs)
    target_link_libraries(test_protocol nlohmann_json::nlohmann_json)
  endif()

  ament_add_gtest(test_cbor_transport src/test_cbor_transport.cpp)
  if(TARGET test_cbor_transport)
    target_link_libraries(test_cbor_transport nlohmann_json::nlohmann_json)
  endif()

  ament_add_gtest(test_action_capability src/test_action_capability.cpp)
  if(TARGET test_action_capability)
    ament_target_dependencies(test_action_capability cpp_rosbridge_core rclcpp_lifecycle)
    target_link_libraries(test_action_capability nlohmann_json::nlohmann_json)
  endif()

  ament_add_gtest(test_execute_cli src/test_execute_cli.cpp)
  if(TARGET test_execute_cli)
    ament_target_dependencies(test_execute_cli cpp_rosbridge_core rclcpp_lifecycle std_msgs)
    target_link_libraries(test_execute_cli nlohmann_json::nlohmann_json)
  endif()

  ament_add_gtest(bench_performance src/bench_performance.cpp)
  if(TARGET bench_performance)
    ament_target_dependencies(bench_performance cpp_rosbridge_core rclcpp_lifecycle std_msgs)
    target_link_libraries(bench_performance
      nlohmann_json::nlohmann_json
      Boost::system
      Threads::Threads
    )
  endif()

  find_package(cpp_rosbridge_server QUIET)
  ament_add_gtest(test_performance_fixes src/test_performance_fixes.cpp)
  if(TARGET test_performance_fixes)
    ament_target_dependencies(test_performance_fixes cpp_rosbridge_core rclcpp_lifecycle std_msgs)
    target_link_libraries(test_performance_fixes
      nlohmann_json::nlohmann_json
      Boost::system
      Threads::Threads
    )
    if(cpp_rosbridge_server_FOUND)
      ament_target_dependencies(test_performance_fixes cpp_rosbridge_server)
    else()
      target_include_directories(test_performance_fixes PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../cpp_rosbridge_server/include
      )
    endif()
  endif()

  # ── ZMQ server tests ──────────────────────────────────────────────────────

  # Build zmq_server.cpp as a library for test linking
  add_library(zmq_server_lib STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../cpp_zmq_server/src/zmq_server.cpp
  )
  target_include_directories(zmq_server_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../cpp_zmq_server/include
    ${ZMQ_INCLUDE_DIRS}
  )
  ament_target_dependencies(zmq_server_lib rclcpp rclcpp_lifecycle cpp_rosbridge_core)
  target_link_libraries(zmq_server_lib
    Boost::system
    Threads::Threads
    nlohmann_json::nlohmann_json
    ${ZMQ_LIBRARIES}
  )

  # msgpack-cxx: header-only, reuse from cpp_zmq_server build
  include(FetchContent)
  FetchContent_Declare(
    msgpack-cxx
    GIT_REPOSITORY https://github.com/msgpack/msgpack-c.git
    GIT_TAG        cpp-6.1.1
  )
  set(MSGPACK_USE_BOOST OFF CACHE BOOL "" FORCE)
  set(MSGPACK_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(MSGPACK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)
  FetchContent_MakeAvailable(msgpack-cxx)

  add_library(habilis_bridge_lib STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../cpp_zmq_server/src/habilis_bridge.cpp
  )
  target_include_directories(habilis_bridge_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../cpp_zmq_server/include
    ${ZMQ_INCLUDE_DIRS}
  )
  ament_target_dependencies(habilis_bridge_lib rclcpp)
  target_link_libraries(habilis_bridge_lib
    msgpack-cxx
    ${ZMQ_LIBRARIES}
  )

  ament_add_gtest(test_zmq_bridge src/test_zmq_bridge.cpp)
  if(TARGET test_zmq_bridge)
    target_include_directories(test_zmq_bridge PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../cpp_zmq_server/include
      ${ZMQ_INCLUDE_DIRS}
    )
    ament_target_dependencies(test_zmq_bridge rclcpp rclcpp_lifecycle cpp_rosbridge_core)
    target_link_libraries(test_zmq_bridge
      zmq_server_lib
      Boost::system
      Threads::Threads
      nlohmann_json::nlohmann_json
      ${ZMQ_LIBRARIES}
    )
  endif()

  # ── Habilis protocol tests ─────────────────────────────────────────────────

  ament_add_gtest(test_habilis_types src/test_habilis_types.cpp)
  if(TARGET test_habilis_types)
    target_include_directories(test_habilis_types PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../cpp_zmq_server/include
    )
    target_link_libraries(test_habilis_types msgpack-cxx)
  endif()

  ament_add_gtest(test_habilis_bridge src/test_habilis_bridge.cpp)
  if(TARGET test_habilis_bridge)
    target_include_directories(test_habilis_bridge PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../cpp_zmq_server/include
      ${ZMQ_INCLUDE_DIRS}
    )
    ament_target_dependencies(test_habilis_bridge rclcpp)
    target_link_libraries(test_habilis_bridge
      habilis_bridge_lib
      msgpack-cxx
      Threads::Threads
      ${ZMQ_LIBRARIES}
    )
  endif()
endif()

ament_package()
