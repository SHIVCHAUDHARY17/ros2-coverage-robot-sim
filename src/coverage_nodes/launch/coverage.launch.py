"""Start the whole simulated coverage robot: planner, controller, sim, tracker.

Usage (inside the container, after `source install/setup.bash`):
    ros2 launch coverage_nodes coverage.launch.py
    ros2 launch coverage_nodes coverage.launch.py record:=true      # also write a rosbag2
    ros2 launch coverage_nodes coverage.launch.py params_file:=/path/to/other.yaml
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Topics that describe "what the robot did" (input for the Day 3 run report).
RECORDED_TOPICS = [
    "/target_grid", "/coverage_path", "/nozzle_pose", "/cmd_vel", "/spray_cmd",
    "/spray_state", "/controller_state", "/coverage_percent",
]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("coverage_nodes"), "config", "params.yaml")
    params = LaunchConfiguration("params_file")
    bag_dir = os.path.join("outputs", "bags", datetime.now().strftime("run_%Y%m%d_%H%M%S"))

    def node(name):
        return Node(package="coverage_nodes", executable=name, name=name,
                    parameters=[params], output="screen")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="YAML file with all node parameters"),
        DeclareLaunchArgument("record", default_value="false",
                              description="Record a rosbag2 of the run into outputs/bags/"),
        node("planner_node"),
        node("sim_node"),
        node("controller_node"),
        node("coverage_tracker_node"),
        ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o", bag_dir, "--topics"] + RECORDED_TOPICS,
            output="screen",
            condition=IfCondition(LaunchConfiguration("record"))),
    ])
