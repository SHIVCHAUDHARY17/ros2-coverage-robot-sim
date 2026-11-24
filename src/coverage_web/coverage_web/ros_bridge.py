"""ROS 2 side of the operator UI: caches the latest robot data, forwards commands.

Runs in its own executor thread; FastAPI handlers call snapshot()/command() from
other threads, so shared data is protected by a lock.
"""

from __future__ import annotations

import threading
import time
from typing import Dict, Optional, Tuple

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

from .app import COMMANDS


class RosBridge(Node):
    def __init__(self):
        super().__init__("web_operator")
        self.command_timeout = float(self.declare_parameter("command_timeout", 2.0).value)
        self._lock = threading.Lock()
        self._state = "no data"
        self._state_time = 0.0
        self._coverage = 0.0
        self._grid: Optional[Dict] = None
        self._path: list = []
        self._pose: Optional[Tuple[float, float]] = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "controller_state", self._on_state, 10)
        self.create_subscription(Float32, "coverage_percent", self._on_cov, 10)
        self.create_subscription(OccupancyGrid, "coverage_grid", self._on_grid, 1)
        self.create_subscription(Path, "coverage_path", self._on_path, latched)
        # what the controller believes (an operator screen would show this, not sim truth)
        self.create_subscription(PoseStamped, "nozzle_pose_est", self._on_pose, 10)
        # NOT "self._clients": rclpy.Node uses that name internally (overwriting it breaks
        # the executor - found the hard way)
        self._cmd_clients = {n: self.create_client(Trigger, n) for n in COMMANDS}

    # -- subscriptions ---------------------------------------------------------
    def _on_state(self, msg: String):
        with self._lock:
            self._state, self._state_time = msg.data, time.monotonic()

    def _on_cov(self, msg: Float32):
        with self._lock:
            self._coverage = float(msg.data)

    def _on_grid(self, msg: OccupancyGrid):
        # tracker values: 0 not target, 25 overspray (sprayed outside the marking),
        # 50 still to treat, 100 treated  ->  compact string "0", "3", "1", "2"
        code = {100: "2", 50: "1", 25: "3"}
        cells = "".join(code.get(v, "0") for v in msg.data)
        with self._lock:
            self._grid = {"w": msg.info.width, "h": msg.info.height,
                          "res": msg.info.resolution, "cells": cells}

    def _on_path(self, msg: Path):
        with self._lock:
            self._path = [[p.pose.position.x, p.pose.position.y, p.pose.position.z > 0.5]
                          for p in msg.poses]

    def _on_pose(self, msg: PoseStamped):
        with self._lock:
            self._pose = (msg.pose.position.x, msg.pose.position.y)

    # -- bridge interface ------------------------------------------------------
    def snapshot(self) -> dict:
        with self._lock:
            return {
                "state": self._state,
                # controller publishes its state at >= 2 Hz; older than 2 s = link lost
                "connected": time.monotonic() - self._state_time < 2.0,
                "coverage": self._coverage,
                "grid": self._grid,
                "path": self._path,
                "pose": self._pose,
            }

    def command(self, name: str) -> Tuple[bool, str]:
        client = self._cmd_clients[name]
        if not client.wait_for_service(timeout_sec=self.command_timeout):
            return False, f"/{name} service not available (controller not running?)"
        done = threading.Event()
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self.command_timeout):
            return False, f"/{name} timed out after {self.command_timeout:.1f} s"
        res = future.result()
        return bool(res.success), res.message
