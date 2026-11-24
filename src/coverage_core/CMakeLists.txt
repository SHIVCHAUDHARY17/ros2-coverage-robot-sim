cmake_minimum_required(VERSION 3.16)
project(coverage_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(coverage_core src/planner.cpp)
target_include_directories(coverage_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(coverage_core PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS coverage_core EXPORT coverage_coreTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

include(CTest)
if(BUILD_TESTING)
  # Use the system GoogleTest if present (Docker image), otherwise download it (CI / bare machine).
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip)
    FetchContent_MakeAvailable(googletest)
  endif()

  add_executable(test_planner test/test_planner.cpp)
  target_link_libraries(test_planner PRIVATE coverage_core GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(test_planner)
endif()
