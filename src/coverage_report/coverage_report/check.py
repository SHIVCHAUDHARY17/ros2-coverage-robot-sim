"""Live acceptance check: wait until the run completes and verify coverage.

Used by CI (scripts/run_scenario.sh) while the robot runs headless:
    ros2 run coverage_report check --min-coverage 99 --timeout 150
Exit code 0 = path completed AND coverage >= threshold, 1 otherwise.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class Check(Node):
    def __init__(self):
        super().__init__("coverage_check")
        self.state = ""
        self.coverage = 0.0
        self.faults = 0
        self.create_subscription(String, "controller_state", self._on_state, 10)
        self.create_subscription(Float32, "coverage_percent", self._on_cov, 10)

    def _on_state(self, msg: String):
        if msg.data != self.state:
            self.get_logger().info(f"state: {msg.data}")
            if msg.data.startswith("FAULT"):
                self.faults += 1
        self.state = msg.data

    def _on_cov(self, msg: Float32):
        self.coverage = float(msg.data)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Wait for the run to finish and check coverage")
    ap.add_argument("--min-coverage", type=float, default=99.0, help="percent (default 99)")
    ap.add_argument("--timeout", type=float, default=150.0, help="seconds (default 150)")
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = Check()
    deadline = time.monotonic() + args.timeout
    done_at = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.state.endswith("path complete") and done_at is None:
            done_at = time.monotonic()
        if node.state == "ESTOP":
            break
        if done_at is not None and time.monotonic() - done_at > 1.5:  # let coverage settle
            break

    ok = done_at is not None and node.coverage >= args.min_coverage
    verdict = "PASS" if ok else "FAIL"
    print(f"{verdict}: state='{node.state}', coverage {node.coverage:.1f} % "
          f"(min {args.min_coverage:.1f} %), faults seen {node.faults}"
          + ("" if done_at is not None else f", path not completed within {args.timeout:.0f} s"))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
