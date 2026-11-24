"""Read a rosbag2 recording of a run into plain RunData (the only ROS-dependent part)."""

from __future__ import annotations

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from .metrics import RunData


def _stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def read_bag(uri: str) -> RunData:
    reader = rosbag2_py.SequentialReader()
    # storage_id "" = detect from the bag's metadata (mcap or sqlite3)
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=""),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}

    run = RunData()
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(data, types[topic])
        t = t_ns * 1e-9  # time the recorder received it
        if topic == "/target_grid":
            run.grid_width, run.grid_height = msg.info.width, msg.info.height
            run.resolution = msg.info.resolution
            run.target = [v > 50 for v in msg.data]
        elif topic == "/coverage_path":
            run.path = [(p.pose.position.x, p.pose.position.y, p.pose.position.z > 0.5)
                        for p in msg.poses]
        elif topic == "/nozzle_pose_true":
            run.true_poses.append((_stamp(msg), msg.pose.position.x, msg.pose.position.y))
        elif topic == "/nozzle_pose":
            run.meas_poses.append((_stamp(msg), msg.pose.position.x, msg.pose.position.y))
            run.meas_arrivals.append(t)
        elif topic == "/nozzle_pose_est":
            run.est_poses.append((_stamp(msg), msg.pose.position.x, msg.pose.position.y))
        elif topic == "/spray_state":
            run.spray.append((t, bool(msg.data)))
        elif topic == "/controller_state":
            run.states.append((t, msg.data))
        elif topic == "/coverage_percent":
            run.coverage.append((t, float(msg.data)))
        elif topic == "/coverage_grid":
            run.final_map = list(msg.data)  # keep the last one = state at the end of the run
    return run
