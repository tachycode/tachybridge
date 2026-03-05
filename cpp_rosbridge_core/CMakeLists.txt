cmake_minimum_required(VERSION 3.8)
project(cpp_rosbridge_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json 3.7.0 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

# Auto-discover actions from workspace interface packages.
include(cmake/generate_action_registration.cmake)

# Resolve discovered interface packages as dependencies.
set(INTERFACE_ACTION_DEPS "")
foreach(interface_pkg IN LISTS AUTO_ACTION_INTERFACE_PACKAGES)
  find_package(${interface_pkg} QUIET)
  if(${interface_pkg}_FOUND)
    list(APPEND INTERFACE_ACTION_DEPS ${interface_pkg})
  endif()
endforeach()

add_library(
  cpp_rosbridge_core
  src/protocol.cpp
  src/introspection.cpp
  src/capabilities/topic.cpp
  src/capabilities/service.cpp
  src/capabilities/action.cpp
  src/capabilities/cli.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/generated/action_registration.cpp
)

set(CORE_DEPS
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  rosidl_typesupport_introspection_cpp
)
if(INTERFACE_ACTION_DEPS)
  list(APPEND CORE_DEPS ${INTERFACE_ACTION_DEPS})
endif()
ament_target_dependencies(cpp_rosbridge_core ${CORE_DEPS})

target_include_directories(cpp_rosbridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(cpp_rosbridge_core
  nlohmann_json::nlohmann_json
  Boost::system
  Threads::Threads
  ${CMAKE_DL_LIBS}
)

install(
  TARGETS cpp_rosbridge_core
  EXPORT export_cpp_rosbridge_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
  EXPORT export_cpp_rosbridge_core
  DESTINATION share/${PROJECT_NAME}/cmake
  NAMESPACE cpp_rosbridge_core::
)

ament_export_include_directories(include)
ament_export_targets(export_cpp_rosbridge_core HAS_LIBRARY_TARGET)
set(EXPORT_DEPS
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  rosidl_typesupport_introspection_cpp
  nlohmann_json
  Boost
  Threads
)
if(INTERFACE_ACTION_DEPS)
  list(APPEND EXPORT_DEPS ${INTERFACE_ACTION_DEPS})
endif()
ament_export_dependencies(${EXPORT_DEPS})

ament_package()
