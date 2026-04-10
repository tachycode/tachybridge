cmake_minimum_required(VERSION 3.14)
project(cpp_zenoh_gateway)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)
find_package(nlohmann_json 3.7.0 REQUIRED)
find_package(zenohcxx REQUIRED)

# Gateway library
add_library(zenoh_gateway_lib
  src/stream_lane.cpp
  src/fanout_hub.cpp
  src/gateway_session.cpp
  src/gateway_server.cpp
)

target_include_directories(zenoh_gateway_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(zenoh_gateway_lib
  Boost::system
  Threads::Threads
  nlohmann_json::nlohmann_json
  zenohcxx::zenohcxx
)

# Executable
add_executable(zenoh_gateway_node src/main.cpp)
target_link_libraries(zenoh_gateway_node zenoh_gateway_lib)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_wire_format test/test_wire_format.cpp)
  target_link_libraries(test_wire_format zenoh_gateway_lib)

  ament_add_gtest(test_stream_lane test/test_stream_lane.cpp)
  target_link_libraries(test_stream_lane zenoh_gateway_lib)

  ament_add_gtest(test_fanout_hub test/test_fanout_hub.cpp)
  target_link_libraries(test_fanout_hub zenoh_gateway_lib)

  ament_add_gtest(test_integration_smoke test/test_integration_smoke.cpp)
  target_link_libraries(test_integration_smoke zenoh_gateway_lib)
endif()

# Install
install(TARGETS zenoh_gateway_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

ament_package()
