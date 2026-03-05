cmake_minimum_required(VERSION 3.8)
project(cpp_rosbridge_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)
find_package(cpp_rosbridge_core REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nlohmann_json 3.7.0 REQUIRED)
find_package(std_msgs REQUIRED)
if(BUILD_TESTING)
  ament_add_gtest(test_protocol src/test_protocol.cpp)
  if(TARGET test_protocol)
    ament_target_dependencies(
      test_protocol
      cpp_rosbridge_core
      rclcpp_lifecycle
      std_msgs
    )
    target_link_libraries(test_protocol nlohmann_json::nlohmann_json)
  endif()

  ament_add_gtest(test_cbor_transport src/test_cbor_transport.cpp)
  if(TARGET test_cbor_transport)
    target_link_libraries(test_cbor_transport nlohmann_json::nlohmann_json)
  endif()

  ament_add_gtest(test_action_capability src/test_action_capability.cpp)
  if(TARGET test_action_capability)
    ament_target_dependencies(
      test_action_capability
      cpp_rosbridge_core
      rclcpp_lifecycle
    )
    target_link_libraries(test_action_capability nlohmann_json::nlohmann_json)
  endif()

  ament_add_gtest(test_execute_cli src/test_execute_cli.cpp)
  if(TARGET test_execute_cli)
    ament_target_dependencies(
      test_execute_cli
      cpp_rosbridge_core
      rclcpp_lifecycle
      std_msgs
    )
    target_link_libraries(test_execute_cli nlohmann_json::nlohmann_json)
  endif()
endif()

ament_package()
