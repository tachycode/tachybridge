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

add_executable(
  zmq_bridge_node
  src/zmq_server.cpp
  src/zmq_server_node.cpp
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
)

target_link_libraries(
  zmq_bridge_node
  Boost::system
  Threads::Threads
  nlohmann_json::nlohmann_json
  ${ZMQ_LIBRARIES}
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_zmq_bridge test/test_zmq_bridge.cpp src/zmq_server.cpp)
  target_include_directories(test_zmq_bridge PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${ZMQ_INCLUDE_DIRS}
  )
  ament_target_dependencies(test_zmq_bridge rclcpp rclcpp_lifecycle cpp_rosbridge_core)
  target_link_libraries(test_zmq_bridge
    Boost::system
    Threads::Threads
    nlohmann_json::nlohmann_json
    ${ZMQ_LIBRARIES}
  )
endif()

ament_package()
