"""perception_node: image of a spray-painted marking -> /target_mask for the planner.

Reads one image (default: the synthetic sample shipped with this package; no
camera or hardware needed), runs the classical pipeline in mask.py, publishes
the result ONCE with latched (transient-local) QoS and saves a debug overlay.

Out: /target_mask nav_msgs/OccupancyGrid (100 = treat, 0 = leave), same grid
     geometry as the planner (grid_cols x grid_rows x cell_size, origin bottom-left)
"""

from __future__ import annotations

import os

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .mask import MaskConfig, mask_to_grid, overlay, paint_mask, summary


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        default_img = os.path.join(get_package_share_directory("coverage_perception"), "data",
                                   "sample_marking.jpg")
        image_path = self.declare_parameter("image_path", "").value or default_img
        cols = int(self.declare_parameter("grid_cols", 40).value)
        rows = int(self.declare_parameter("grid_rows", 30).value)
        cell = float(self.declare_parameter("cell_size", 0.05).value)
        frame = self.declare_parameter("frame_id", "map").value
        overlay_dir = self.declare_parameter("overlay_dir", "outputs/perception").value
        d = MaskConfig()
        cfg = MaskConfig(
            sat_min=int(self.declare_parameter("sat_min", d.sat_min).value),
            val_min=int(self.declare_parameter("val_min", d.val_min).value),
            close_px=int(self.declare_parameter("close_px", d.close_px).value),
            open_px=int(self.declare_parameter("open_px", d.open_px).value),
            min_area_frac=float(self.declare_parameter("min_area_frac", d.min_area_frac).value),
            cell_min_fraction=float(
                self.declare_parameter("cell_min_fraction", d.cell_min_fraction).value),
        )

        img = cv2.imread(image_path)
        if img is None:
            raise RuntimeError(f"cannot read image '{image_path}'")
        mask = paint_mask(img, cfg)
        grid = mask_to_grid(mask, cols, rows, cfg.cell_min_fraction)
        n, area = summary(grid, cell)

        msg = OccupancyGrid()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = cell
        msg.info.width, msg.info.height = cols, rows
        msg.info.origin.orientation.w = 1.0
        msg.data = [100 if v else 0 for v in grid]
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, "target_mask", latched)
        self.pub.publish(msg)

        if overlay_dir:
            os.makedirs(overlay_dir, exist_ok=True)
            out = os.path.join(overlay_dir, "perception_overlay.jpg")
            cv2.imwrite(out, overlay(img, mask, grid, cols, rows))
            self.get_logger().info(f"Debug overlay written to {out}")
        level = self.get_logger().warn if n == 0 else self.get_logger().info
        level(f"Marked area from '{os.path.basename(image_path)}': {n} cells "
              f"({area:.2f} m^2) published on /target_mask")


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)  # keep the latched message available for late subscribers
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
