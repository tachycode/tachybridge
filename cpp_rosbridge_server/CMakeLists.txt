cmake_minimum_required(VERSION 3.8)
project(cpp_rosbridge_server)

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

# Find nlohmann_json via the core package dependency
find_package(nlohmann_json 3.7.0 REQUIRED)

add_executable(
  rosbridge_server_node
  src/rosbridge_server_node.cpp
)

target_include_directories(rosbridge_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  rosbridge_server_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  cpp_rosbridge_core
)

target_link_libraries(
  rosbridge_server_node
  Boost::system
  Threads::Threads
  nlohmann_json::nlohmann_json
)

install(
  TARGETS rosbridge_server_node
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

ament_package()
