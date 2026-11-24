"""Start the whole simulated coverage robot: planner, controller, sim, tracker.

Usage (inside the container, after `source install/setup.bash`):
    ros2 launch coverage_nodes coverage.launch.py
    ros2 launch coverage_nodes coverage.launch.py record:=true            # also write a rosbag2
    ros2 launch coverage_nodes coverage.launch.py scenario:=site_faults   # bad sensor conditions
    ros2 launch coverage_nodes coverage.launch.py params_file:=/path/to/other.yaml
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NODES = ["planner_node", "sim_node", "controller_node", "coverage_tracker_node"]

# Topics that describe "what the robot did" (input for the run report).
RECORDED_TOPICS = [
    "/target_grid", "/coverage_path", "/nozzle_pose", "/nozzle_pose_true", "/nozzle_pose_est",
    "/cmd_vel", "/spray_cmd", "/spray_state", "/controller_state", "/coverage_percent",
]


def _nodes(context):
    """Build the node list once launch arguments are known (scenario file is optional)."""
    share = get_package_share_directory("coverage_nodes")
    params = [LaunchConfiguration("params_file").perform(context)]
    scenario = LaunchConfiguration("scenario").perform(context)
    if scenario:
        scenario_file = os.path.join(share, "config", "scenarios", scenario + ".yaml")
        if not os.path.isfile(scenario_file):
            raise RuntimeError(f"Unknown scenario '{scenario}': {scenario_file} not found")
        params.append(scenario_file)  # later files override earlier ones
    return [Node(package="coverage_nodes", executable=n, name=n, parameters=params,
                 output="screen") for n in NODES]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("coverage_nodes"), "config", "params.yaml")
    bag_dir = os.path.join("outputs", "bags", datetime.now().strftime("run_%Y%m%d_%H%M%S"))

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="YAML file with all node parameters"),
        DeclareLaunchArgument("scenario", default_value="",
                              description="Optional overlay from config/scenarios/<name>.yaml"),
        DeclareLaunchArgument("record", default_value="false",
                              description="Record a rosbag2 of the run into outputs/bags/"),
        OpaqueFunction(function=_nodes),
        ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o", bag_dir, "--topics"] + RECORDED_TOPICS,
            output="screen",
            condition=IfCondition(LaunchConfiguration("record"))),
    ])
