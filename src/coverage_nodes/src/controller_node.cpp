// controller_node: ROS wrapper around coverage::Controller (state machine,
// pose watchdog, accel-limited path follower). All decisions happen in the
// tested core class; this node only moves data between ROS and the core.
//
// In:   /coverage_path  nav_msgs/Path (latched; z = spray flag, see planner_node)
//       /nozzle_pose    geometry_msgs/PoseStamped (measured position, kicks the watchdog)
// Out:  /cmd_vel        geometry_msgs/Twist  (linear.x / linear.y in m/s)
//       /spray_cmd      std_msgs/Bool        (nozzle valve command)
//       /controller_state std_msgs/String    (IDLE/RUNNING/PAUSED/ESTOP/FAULT[: reason])
//       /nozzle_pose_est geometry_msgs/PoseStamped (fused position estimate it steers with)
// Services (std_srvs/Trigger): /start /pause /resume /estop /reset
//
// auto_recover (default false): after a FAULT, automatically RESET + START once
// the sensor is back, at most max_auto_recoveries times. Meant for unattended
// SIMULATION tests; on a real machine an operator should decide to restart.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "coverage_core/controller.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

using std_srvs::srv::Trigger;

class ControllerNode : public rclcpp::Node {
 public:
  ControllerNode() : Node("controller_node"), ctrl_(load_config()) {
    const double rate_hz = declare_parameter<double>("rate_hz", 50.0);
    auto_start_ = declare_parameter<bool>("auto_start", false);
    auto_recover_ = declare_parameter<bool>("auto_recover", false);
    max_recoveries_ = declare_parameter<int>("max_auto_recoveries", 3);
    frame_ = declare_parameter<std::string>("frame_id", "map");
    dt_ = 1.0 / rate_hz;

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    spray_pub_ = create_publisher<std_msgs::msg::Bool>("spray_cmd", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("controller_state", 10);
    est_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("nozzle_pose_est", 10);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "coverage_path", rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::Path::ConstSharedPtr msg) { on_path(*msg); });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "nozzle_pose", 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          // Watchdog is kicked with the ARRIVAL time (a late message counts as late);
          // the header stamp (capture time) is used for latency compensation.
          ctrl_.update_pose(msg->pose.position.x, msg->pose.position.y, now().seconds(),
                            rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()).seconds());
        });

    add_command("start", coverage::Command::START);
    add_command("pause", coverage::Command::PAUSE);
    add_command("resume", coverage::Command::RESUME);
    add_command("estop", coverage::Command::ESTOP);
    add_command("reset", coverage::Command::RESET);

    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::duration<double>(dt_)),
                               [this] { tick(); });
    RCLCPP_INFO(get_logger(), "Controller ready at %.0f Hz (auto_start=%s)", rate_hz,
                auto_start_ ? "true" : "false");
  }

 private:
  coverage::ControllerConfig load_config() {
    coverage::ControllerConfig c;
    c.max_speed = declare_parameter<double>("max_speed", c.max_speed);
    c.max_accel = declare_parameter<double>("max_accel", c.max_accel);
    c.goal_tolerance = declare_parameter<double>("goal_tolerance", c.goal_tolerance);
    c.watchdog_timeout = declare_parameter<double>("watchdog_timeout", c.watchdog_timeout);
    c.lateral_gain = declare_parameter<double>("lateral_gain", c.lateral_gain);
    c.pose_filter_alpha = declare_parameter<double>("pose_filter_alpha", c.pose_filter_alpha);
    return c;  // Controller's constructor validates (throws on bad values)
  }

  void on_path(const nav_msgs::msg::Path& msg) {
    std::vector<coverage::Waypoint> path;
    path.reserve(msg.poses.size());
    for (const auto& p : msg.poses)
      path.push_back({p.pose.position.x, p.pose.position.y, p.pose.position.z > 0.5});
    if (ctrl_.set_path(std::move(path)))
      RCLCPP_INFO(get_logger(), "Loaded path with %zu waypoints", msg.poses.size());
    else
      RCLCPP_WARN(get_logger(), "Path rejected: controller is %s (only IDLE accepts a new path)",
                  coverage::to_string(ctrl_.state()));
  }

  void add_command(const std::string& name, coverage::Command cmd) {
    services_.push_back(create_service<Trigger>(
        name, [this, name, cmd](const std::shared_ptr<Trigger::Request>,
                                std::shared_ptr<Trigger::Response> res) {
          res->success = ctrl_.handle(cmd, now().seconds());
          res->message = std::string(res->success ? "accepted" : "refused") + ", state is now " +
                         coverage::to_string(ctrl_.state());
          RCLCPP_INFO(get_logger(), "Command '%s': %s", name.c_str(), res->message.c_str());
        }));
  }

  void tick() {
    const double t = now().seconds();

    if (auto_start_ && !auto_started_ && ctrl_.path_size() > 0 &&
        ctrl_.state() == coverage::State::IDLE) {
      auto_started_ = ctrl_.handle(coverage::Command::START, t);  // retries until pose is fresh
      if (auto_started_) RCLCPP_INFO(get_logger(), "Auto-start: RUNNING");
    }

    if (auto_recover_ && ctrl_.state() == coverage::State::FAULT &&
        recoveries_ < max_recoveries_ && ctrl_.feedback_fresh(t)) {
      const std::string reason = ctrl_.fault_reason();
      if (ctrl_.handle(coverage::Command::RESET, t) && ctrl_.handle(coverage::Command::START, t)) {
        ++recoveries_;
        RCLCPP_WARN(get_logger(), "Auto-recovery %d/%d after FAULT (%s): continuing from waypoint %zu",
                    recoveries_, max_recoveries_, reason.c_str(), ctrl_.target_index());
      }
    }

    const auto out = ctrl_.step(t, dt_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = out.vx;
    cmd.linear.y = out.vy;
    cmd_pub_->publish(cmd);
    std_msgs::msg::Bool spray;
    spray.data = out.spray;
    spray_pub_->publish(spray);

    geometry_msgs::msg::PoseStamped est;
    est.header.stamp = now();
    est.header.frame_id = frame_;
    est.pose.position.x = ctrl_.est_x();
    est.pose.position.y = ctrl_.est_y();
    est.pose.orientation.w = 1.0;
    est_pub_->publish(est);

    publish_state(t);
  }

  // Publish the state on every change, and at 2 Hz as a heartbeat.
  void publish_state(double t) {
    std::string s = coverage::to_string(ctrl_.state());
    if (ctrl_.state() == coverage::State::FAULT) s += ": " + ctrl_.fault_reason();
    if (ctrl_.finished() && ctrl_.state() == coverage::State::IDLE) s += ": path complete";
    if (s != last_state_) {
      RCLCPP_INFO(get_logger(), "State: %s", s.c_str());
    } else if (t - last_state_pub_ < 0.5) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = s;
    state_pub_->publish(msg);
    last_state_ = s;
    last_state_pub_ = t;
  }

  coverage::Controller ctrl_;
  double dt_ = 0.02;
  bool auto_start_ = false, auto_started_ = false, auto_recover_ = false;
  int max_recoveries_ = 3, recoveries_ = 0;
  std::string last_state_, frame_;
  double last_state_pub_ = 0.0;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spray_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr est_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> services_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
