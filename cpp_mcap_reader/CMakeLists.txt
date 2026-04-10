cmake_minimum_required(VERSION 3.14)
project(cpp_mcap_reader)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(nlohmann_json 3.7.0 REQUIRED)
find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

# mcap is header-only (installed to /usr/local/include/mcap)
# lz4 and zstd for MCAP compression
find_package(PkgConfig REQUIRED)
pkg_check_modules(LZ4 REQUIRED IMPORTED_TARGET liblz4)
pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)

add_library(mcap_reader_lib
  src/cdr_extractor.cpp
  src/mcap_publisher.cpp
  src/playback_controller.cpp
)

target_include_directories(mcap_reader_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include
)

target_link_libraries(mcap_reader_lib
  Threads::Threads
  nlohmann_json::nlohmann_json
  zenohcxx::zenohc
  PkgConfig::LZ4
  PkgConfig::ZSTD
)

add_executable(mcap_reader_node src/main.cpp)
target_link_libraries(mcap_reader_node mcap_reader_lib)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_cdr_extractor test/test_cdr_extractor.cpp)
  target_link_libraries(test_cdr_extractor mcap_reader_lib)
endif()

install(TARGETS mcap_reader_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
