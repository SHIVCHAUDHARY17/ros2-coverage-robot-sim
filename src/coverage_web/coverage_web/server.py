"""Entry point: ROS bridge in a background executor thread + uvicorn web server.

    ros2 run coverage_web server            # http://localhost:8080
    ros2 run coverage_web server --ros-args -p port:=9000
"""

from __future__ import annotations

import threading

import rclpy
import uvicorn
from rclpy.executors import MultiThreadedExecutor

from .app import create_app
from .ros_bridge import RosBridge


def main(args=None):
    rclpy.init(args=args)
    bridge = RosBridge()
    host = bridge.declare_parameter("host", "0.0.0.0").value  # all interfaces: reachable from Docker host
    port = int(bridge.declare_parameter("port", 8080).value)

    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    bridge.get_logger().info(f"Operator UI on http://localhost:{port}  (simulation only)")
    try:
        uvicorn.run(create_app(bridge), host=host, port=port, log_level="warning")
    finally:
        executor.shutdown()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
