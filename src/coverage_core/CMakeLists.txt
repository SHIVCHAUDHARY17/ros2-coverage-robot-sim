cmake_minimum_required(VERSION 3.16)
project(coverage_core VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(coverage_core
  src/planner.cpp
  src/controller.cpp)
target_include_directories(coverage_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(coverage_core PRIVATE -Wall -Wextra -Wpedantic)
# Static library that gets linked into ROS executables: build it position-independent.
set_target_properties(coverage_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Install + export so other packages (coverage_nodes) can do:
#   find_package(coverage_core REQUIRED)
#   target_link_libraries(my_node coverage_core::coverage_core)
install(TARGETS coverage_core EXPORT coverage_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT coverage_coreTargets
  NAMESPACE coverage_core::
  FILE coverage_coreConfig.cmake
  DESTINATION share/coverage_core/cmake)

include(CTest)
if(BUILD_TESTING)
  # Use the system GoogleTest if present (Docker image, CI), otherwise download it.
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip)
    FetchContent_MakeAvailable(googletest)
  endif()
  include(GoogleTest)

  foreach(t test_planner test_controller)
    add_executable(${t} test/${t}.cpp)
    target_link_libraries(${t} PRIVATE coverage_core GTest::gtest_main)
    target_compile_options(${t} PRIVATE -Wall -Wextra -Wpedantic)
    gtest_discover_tests(${t})
  endforeach()
endif()
