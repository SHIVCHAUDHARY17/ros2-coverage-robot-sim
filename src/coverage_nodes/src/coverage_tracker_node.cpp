// coverage_tracker_node: measures what the nozzle has actually treated.
// Every time a new pose arrives while the valve is open, the segment from the
// previous pose to the new one is stamped into a coverage::CoverageMap.
//
// In:   /target_grid  nav_msgs/OccupancyGrid (latched, from planner_node)
//       /nozzle_pose  geometry_msgs/PoseStamped
//       /spray_state  std_msgs/Bool (actual valve state from the sim)
// Out:  /coverage_grid    nav_msgs/OccupancyGrid  (0 = not target, 50 = target still to treat,
//                                                  100 = treated)
//       /coverage_percent std_msgs/Float32        (0..100, share of target cells treated)

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "coverage_core/planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

class CoverageTrackerNode : public rclcpp::Node {
 public:
  CoverageTrackerNode() : Node("coverage_tracker_node") {
    nozzle_width_ = declare_parameter<double>("nozzle_width", 0.25);
    const double publish_hz = declare_parameter<double>("publish_hz", 2.0);

    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("coverage_grid", 1);
    pct_pub_ = create_publisher<std_msgs::msg::Float32>("coverage_percent", 10);

    target_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "target_grid", rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { on_target(*msg); });
    spray_sub_ = create_subscription<std_msgs::msg::Bool>(
        "spray_state", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) { spray_ = msg->data; });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "nozzle_pose", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { on_pose(*msg); });

    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::duration<double>(1.0 / publish_hz)),
                               [this] { publish(); });
  }

 private:
  void on_target(const nav_msgs::msg::OccupancyGrid& msg) {
    target_.emplace(msg.info.width, msg.info.height, msg.info.resolution);
    for (std::size_t r = 0; r < msg.info.height; ++r)
      for (std::size_t c = 0; c < msg.info.width; ++c)
        target_->set(c, r, msg.data[r * msg.info.width + c] > 50);
    map_.emplace(msg.info.width, msg.info.height, msg.info.resolution);
    header_ = msg.header;
    origin_ = msg.info;
    has_last_ = false;
    RCLCPP_INFO(get_logger(), "Tracking coverage of %zu target cells", target_->count());
  }

  void on_pose(const geometry_msgs::msg::PoseStamped& msg) {
    if (!map_) return;  // no target yet
    const double x = msg.pose.position.x, y = msg.pose.position.y;
    if (has_last_ && spray_) map_->stamp_segment(last_x_, last_y_, x, y, nozzle_width_);
    last_x_ = x;
    last_y_ = y;
    has_last_ = true;
  }

  void publish() {
    if (!map_) return;
    const double pct = 100.0 * map_->fraction_of(*target_);
    std_msgs::msg::Float32 p;
    p.data = static_cast<float>(pct);
    pct_pub_->publish(p);

    nav_msgs::msg::OccupancyGrid g;
    g.header = header_;
    g.header.stamp = now();
    g.info = origin_;
    g.data.resize(target_->cols() * target_->rows());
    for (std::size_t r = 0; r < target_->rows(); ++r)
      for (std::size_t c = 0; c < target_->cols(); ++c)
        g.data[r * target_->cols() + c] = map_->covered(c, r) ? 100 : (target_->at(c, r) ? 50 : 0);
    grid_pub_->publish(g);

    // Log at most every 5 s and only if the value changed, plus once on completion.
    const double t = now().seconds();
    const bool done_now = pct >= 99.95 && !logged_done_;
    if (done_now || (t - last_log_ >= 5.0 && pct != last_logged_pct_)) {
      RCLCPP_INFO(get_logger(), "Coverage: %.1f %%", pct);
      last_log_ = t;
      last_logged_pct_ = pct;
      if (done_now) logged_done_ = true;
    }
  }

  double nozzle_width_ = 0.25;
  std::optional<coverage::Grid> target_;
  std::optional<coverage::CoverageMap> map_;
  std_msgs::msg::Header header_;
  nav_msgs::msg::MapMetaData origin_;
  bool spray_ = false, has_last_ = false, logged_done_ = false;
  double last_x_ = 0.0, last_y_ = 0.0, last_log_ = 0.0, last_logged_pct_ = -1.0;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pct_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr spray_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoverageTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
