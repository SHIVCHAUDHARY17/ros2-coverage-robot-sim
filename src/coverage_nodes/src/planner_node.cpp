// planner_node: gets the target region, plans a boustrophedon coverage path with
// coverage_core and publishes both with "latched" (transient-local) QoS, so nodes
// that start later still receive them.
//
// Target source (parameter target_source):
//   "shape"      - built-in test region ("rect" or "L", see target_shape)
//   "mask_topic" - whatever arrives on /target_mask (from coverage_perception);
//                  every new mask is re-planned
//
// Topics out:
//   /target_grid   nav_msgs/OccupancyGrid  (100 = must be treated, 0 = leave alone)
//   /coverage_path nav_msgs/Path           (pose.position.z encodes the nozzle valve:
//                                           1.0 = spray on this segment, 0.0 = travel move;
//                                           the scaled gantry is 2D, so z is free)

#include <memory>
#include <string>

#include "coverage_core/planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

class PlannerNode : public rclcpp::Node {
 public:
  PlannerNode() : Node("planner_node") {
    const auto cols = declare_parameter<int>("grid_cols", 40);
    const auto rows = declare_parameter<int>("grid_rows", 30);
    const auto cell = declare_parameter<double>("cell_size", 0.05);
    const auto source = declare_parameter<std::string>("target_source", "shape");
    const auto shape = declare_parameter<std::string>("target_shape", "L");
    cfg_.nozzle_width = declare_parameter<double>("nozzle_width", 0.25);
    cfg_.overlap = declare_parameter<double>("overlap", 0.1);
    cfg_.edge_inset = declare_parameter<double>("edge_inset", 0.0);
    frame_ = declare_parameter<std::string>("frame_id", "map");

    const auto latched = rclcpp::QoS(1).transient_local().reliable();
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("target_grid", latched);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("coverage_path", latched);

    if (source == "shape") {
      // "rect" = full rectangle, "L" = rectangle with the top-right quarter removed
      // (non-rectangular, like an area around an obstacle).
      coverage::Grid grid(static_cast<std::size_t>(cols), static_cast<std::size_t>(rows), cell, 1);
      if (shape == "L") {
        for (int r = rows * 2 / 3; r < rows; ++r)
          for (int c = cols * 3 / 4; c < cols; ++c)
            grid.set(static_cast<std::size_t>(c), static_cast<std::size_t>(r), false);
      } else if (shape != "rect") {
        throw std::invalid_argument("target_shape must be 'rect' or 'L', got '" + shape + "'");
      }
      plan_and_publish(grid, "shape '" + shape + "'");
    } else if (source == "mask_topic") {
      mask_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
          "target_mask", latched, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {
            coverage::Grid grid(m->info.width, m->info.height, m->info.resolution);
            for (std::size_t r = 0; r < m->info.height; ++r)
              for (std::size_t c = 0; c < m->info.width; ++c)
                grid.set(c, r, m->data[r * m->info.width + c] > 50);
            plan_and_publish(grid, "/target_mask");
          });
      RCLCPP_INFO(get_logger(), "Waiting for the target area on /target_mask");
    } else {
      throw std::invalid_argument("target_source must be 'shape' or 'mask_topic', got '" + source + "'");
    }
  }

 private:
  void plan_and_publish(const coverage::Grid& grid, const std::string& from) {
    const auto path = coverage::plan_coverage(grid, cfg_);  // throws on bad config: fail loudly
    grid_pub_->publish(to_msg(grid));
    path_pub_->publish(to_msg(path));
    if (path.empty())
      RCLCPP_WARN(get_logger(), "Target from %s is EMPTY: nothing to treat", from.c_str());
    RCLCPP_INFO(get_logger(), "Published target (%zu cells, from %s) and path (%zu waypoints, %.2f m)",
                grid.count(), from.c_str(), path.size(), coverage::path_length(path));
  }

  nav_msgs::msg::OccupancyGrid to_msg(const coverage::Grid& g) const {
    nav_msgs::msg::OccupancyGrid m;
    m.header.frame_id = frame_;
    m.header.stamp = now();
    m.info.resolution = static_cast<float>(g.cell_size());
    m.info.width = static_cast<uint32_t>(g.cols());
    m.info.height = static_cast<uint32_t>(g.rows());
    m.info.origin.orientation.w = 1.0;  // origin (0,0) = bottom-left, same as coverage::Grid
    m.data.resize(g.cols() * g.rows());
    for (std::size_t r = 0; r < g.rows(); ++r)
      for (std::size_t c = 0; c < g.cols(); ++c) m.data[r * g.cols() + c] = g.at(c, r) ? 100 : 0;
    return m;
  }

  nav_msgs::msg::Path to_msg(const std::vector<coverage::Waypoint>& path) const {
    nav_msgs::msg::Path m;
    m.header.frame_id = frame_;
    m.header.stamp = now();
    for (const auto& w : path) {
      geometry_msgs::msg::PoseStamped p;
      p.header = m.header;
      p.pose.position.x = w.x;
      p.pose.position.y = w.y;
      p.pose.position.z = w.spray ? 1.0 : 0.0;
      p.pose.orientation.w = 1.0;
      m.poses.push_back(p);
    }
    return m;
  }

  coverage::PlannerConfig cfg_;
  std::string frame_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
