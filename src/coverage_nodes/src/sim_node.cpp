// sim_node: 2D kinematic simulation of the nozzle gantry (simulation only, no hardware).
// It integrates the commanded velocity and reports where the nozzle is.
//
// Safety on the "machine" side: a command watchdog. If no /cmd_vel arrives for
// cmd_timeout seconds (controller crashed, link lost), the gantry stops and the
// valve closes. Together with the controller's pose watchdog this guards both
// directions of the link.
//
// Site conditions: the position SENSOR channel goes through coverage::PoseFaultInjector
// (noise, vibration, latency, drops, blackouts; all 0 = clean, set via fault.* params).
//
// In:   /cmd_vel           geometry_msgs/Twist
//       /spray_cmd         std_msgs/Bool
// Out:  /nozzle_pose       geometry_msgs/PoseStamped  measured (faulty) position, stamp = capture time
//       /nozzle_pose_true  geometry_msgs/PoseStamped  ground truth (simulation only; a real robot
//                                                     does not have this, it is used for evaluation)
//       /spray_state       std_msgs/Bool              actual valve state

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "coverage_core/controller.hpp"  // coverage::Watchdog
#include "coverage_core/faults.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class SimNode : public rclcpp::Node {
 public:
  SimNode()
      : Node("sim_node"),
        watchdog_(declare_parameter<double>("cmd_timeout", 0.2)),
        faults_(load_faults()) {
    const double rate_hz = declare_parameter<double>("rate_hz", 50.0);
    x_ = declare_parameter<double>("start_x", 0.0);
    y_ = declare_parameter<double>("start_y", 0.0);
    max_x_ = declare_parameter<double>("workspace_x", 2.0);
    max_y_ = declare_parameter<double>("workspace_y", 1.5);
    frame_ = declare_parameter<std::string>("frame_id", "map");
    dt_ = 1.0 / rate_hz;

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("nozzle_pose", 10);
    true_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("nozzle_pose_true", 10);
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
  coverage::PoseFaultInjector load_faults() {
    coverage::FaultConfig f;
    f.noise_std = declare_parameter<double>("fault.noise_std", 0.0);
    f.vibration_amp = declare_parameter<double>("fault.vibration_amp", 0.0);
    f.vibration_hz = declare_parameter<double>("fault.vibration_hz", 0.0);
    f.latency = declare_parameter<double>("fault.latency", 0.0);
    f.drop_prob = declare_parameter<double>("fault.drop_prob", 0.0);
    f.outage_rate = declare_parameter<double>("fault.outage_rate", 0.0);
    f.outage_duration = declare_parameter<double>("fault.outage_duration", 0.0);
    f.seed = static_cast<unsigned>(declare_parameter<int>("fault.seed", 42));
    RCLCPP_INFO(get_logger(),
                "Sensor faults: noise %.1f mm, vibration %.1f mm @ %.1f Hz, latency %.0f ms, "
                "drop %.0f %%, blackouts %.2f/s x %.2f s",
                f.noise_std * 1e3, f.vibration_amp * 1e3, f.vibration_hz, f.latency * 1e3,
                f.drop_prob * 100.0, f.outage_rate, f.outage_duration);
    return coverage::PoseFaultInjector(f);  // validates (throws on bad values)
  }

  geometry_msgs::msg::PoseStamped pose_msg(double stamp, double x, double y) const {
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(stamp * 1e9)),
                                  get_clock()->get_clock_type());
    p.header.frame_id = frame_;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation.w = 1.0;
    return p;
  }

  void tick() {
    const double t = now().seconds();
    if (watchdog_.expired(t)) {
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

    // Valve state first, then poses: the tracker stamps the segment it just
    // received with the valve state that was active while moving along it.
    std_msgs::msg::Bool valve;
    valve.data = spray_;
    spray_pub_->publish(valve);
    true_pub_->publish(pose_msg(t, x_, y_));

    // The sensor channel: what the controller gets to see.
    const std::size_t outages_before = faults_.outages();
    for (const auto& m : faults_.process(t, x_, y_, dt_)) pose_pub_->publish(pose_msg(m.stamp, m.x, m.y));
    if (faults_.outages() > outages_before)
      RCLCPP_WARN(get_logger(), "Injected sensor blackout #%zu", faults_.outages());
  }

  coverage::Watchdog watchdog_;
  coverage::PoseFaultInjector faults_;
  double dt_ = 0.02;
  double x_ = 0.0, y_ = 0.0, vx_ = 0.0, vy_ = 0.0;
  double max_x_ = 2.0, max_y_ = 1.5;
  bool spray_ = false, stopped_by_watchdog_ = false;
  std::string frame_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_, true_pub_;
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
