cmake_minimum_required(VERSION 3.16)
project(coverage_nodes LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(coverage_core REQUIRED)  # our ROS-free library (Day 1 + 2)

# One executable per node. Each node is a thin ROS wrapper around tested core logic.
foreach(node planner_node controller_node sim_node coverage_tracker_node)
  add_executable(${node} src/${node}.cpp)
  target_compile_options(${node} PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(${node}
    coverage_core::coverage_core
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS})
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
