// sim_node: 2D kinematic simulation of the nozzle gantry (simulation only, no hardware).
// It integrates the commanded velocity and reports where the nozzle is.
//
// Safety on the "machine" side: a command watchdog. If no /cmd_vel arrives for
// cmd_timeout seconds (controller crashed, link lost), the gantry stops and the
// valve closes. Together with the controller's pose watchdog this guards both
// directions of the link.
//
// In:   /cmd_vel    geometry_msgs/Twist
//       /spray_cmd  std_msgs/Bool
// Out:  /nozzle_pose geometry_msgs/PoseStamped  (measured position)
//       /spray_state std_msgs/Bool              (actual valve state, used for coverage)

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "coverage_core/controller.hpp"  // coverage::Watchdog
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class SimNode : public rclcpp::Node {
 public:
  SimNode()
      : Node("sim_node"),
        watchdog_(declare_parameter<double>("cmd_timeout", 0.2)) {
    const double rate_hz = declare_parameter<double>("rate_hz", 50.0);
    x_ = declare_parameter<double>("start_x", 0.0);
    y_ = declare_parameter<double>("start_y", 0.0);
    max_x_ = declare_parameter<double>("workspace_x", 2.0);
    max_y_ = declare_parameter<double>("workspace_y", 1.5);
    frame_ = declare_parameter<std::string>("frame_id", "map");
    dt_ = 1.0 / rate_hz;

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("nozzle_pose", 10);
    spray_pub_ = create_publisher<std_msgs::msg::Bool>("spray_state", 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          vx_ = msg->linear.x;
          vy_ = msg->linear.y;
          watchdog_.kick(now().seconds());
        });
    spray_sub_ = create_subscription<std_msgs::msg::Bool>(
        "spray_cmd", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) { spray_ = msg->data; });

    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::duration<double>(dt_)),
                               [this] { tick(); });
    RCLCPP_INFO(get_logger(), "Sim gantry %.2f x %.2f m at %.0f Hz, cmd watchdog %.2f s", max_x_,
                max_y_, rate_hz, watchdog_.timeout());
  }

 private:
  void tick() {
    const bool cmd_alive = !watchdog_.expired(now().seconds());
    if (!cmd_alive) {
      if (!stopped_by_watchdog_) RCLCPP_WARN(get_logger(), "No /cmd_vel: gantry stopped, valve closed");
      stopped_by_watchdog_ = true;
      vx_ = vy_ = 0.0;
      spray_ = false;
    } else {
      stopped_by_watchdog_ = false;
    }

    // Kinematic model: position += velocity * dt, clamped to the gantry's travel limits.
    x_ = std::clamp(x_ + vx_ * dt_, 0.0, max_x_);
    y_ = std::clamp(y_ + vy_ * dt_, 0.0, max_y_);

    // Valve state first, then pose: the tracker stamps the segment it just
    // received with the valve state that was active while moving along it.
    std_msgs::msg::Bool valve;
    valve.data = spray_;
    spray_pub_->publish(valve);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = frame_;
    pose.pose.position.x = x_;
    pose.pose.position.y = y_;
    pose.pose.orientation.w = 1.0;
    pose_pub_->publish(pose);
  }

  coverage::Watchdog watchdog_;
  double dt_ = 0.02;
  double x_ = 0.0, y_ = 0.0, vx_ = 0.0, vy_ = 0.0;
  double max_x_ = 2.0, max_y_ = 1.5;
  bool spray_ = false, stopped_by_watchdog_ = false;
  std::string frame_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spray_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr spray_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimNode>());
  rclcpp::shutdown();
  return 0;
}
