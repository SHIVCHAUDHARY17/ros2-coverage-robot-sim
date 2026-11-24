#pragma once
// ROS-free motion controller for the coverage nozzle.
//
// Design rules (each one is covered by a unit test):
//  * Time is passed in by the caller (`now`, seconds). The class never reads a
//    clock, so tests are deterministic and the ROS node decides where time comes from.
//  * Safety first: ESTOP is accepted in every state and only RESET leaves it.
//    Any state other than RUNNING commands zero velocity with the nozzle off.
//  * Pose-feedback watchdog: if no pose arrives for `watchdog_timeout` seconds
//    while RUNNING, the controller goes to FAULT and stops. START/RESUME are
//    refused while feedback is stale ("never move blind").
//  * Motion limits: speed <= max_speed, |velocity change| <= max_accel * dt, and
//    the speed profile brakes so the nozzle can stop at every waypoint.
//  * Pose estimate (sensor fusion): every cycle the controller predicts where
//    the nozzle is from its own velocity command (dead reckoning) and corrects
//    that prediction with each measurement, weighted by pose_filter_alpha.
//    This smooths sensor noise/vibration and bridges dropped measurements.
//  * Latency compensation: a measurement taken at `stamp` but delivered at `now`
//    describes the past. The controller adds the motion it commanded since
//    `stamp` (from a short command history) before fusing it.
//  * Path following: the nozzle tracks the LINE between waypoints (cross-track
//    control), not just the next point, so it returns to its lane after a
//    disturbance (noisy sensor, latency) instead of spraying a shifted strip.

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

#include "coverage_core/planner.hpp"

namespace coverage {

// Dead-man's switch: must be kicked regularly, otherwise it reports expired.
// A watchdog that was never kicked counts as expired.
class Watchdog {
 public:
  explicit Watchdog(double timeout_s);
  void kick(double now);
  bool expired(double now) const;
  double timeout() const { return timeout_; }

 private:
  double timeout_;
  double last_kick_ = 0.0;
  bool kicked_ = false;
};

enum class State { IDLE, RUNNING, PAUSED, ESTOP, FAULT };
enum class Command { START, PAUSE, RESUME, ESTOP, RESET };

const char* to_string(State s);

struct ControllerConfig {
  double max_speed = 0.5;         // m/s
  double max_accel = 1.0;         // m/s^2
  double goal_tolerance = 0.005;  // m, waypoint counts as reached inside this radius
  double watchdog_timeout = 0.2;  // s, max age of pose feedback while RUNNING
  double lateral_gain = 4.0;      // 1/s, sideways correction speed per metre of lane error
  double pose_filter_alpha = 0.3; // (0,1]: weight of a new measurement vs prediction (1 = raw)
};

// Velocity command for the gantry plus the nozzle valve.
struct ControlOutput {
  double vx = 0.0;
  double vy = 0.0;
  bool spray = false;
};

class Controller {
 public:
  // Throws std::invalid_argument if a limit is <= 0, lateral_gain < 0 or
  // pose_filter_alpha is outside (0, 1].
  explicit Controller(const ControllerConfig& cfg);

  // Load a new path. Only allowed in IDLE; resets progress. Returns false if refused.
  bool set_path(std::vector<Waypoint> path);

  // Operator command. Returns true if the command was accepted in the current state.
  bool handle(Command cmd, double now);

  // Latest measured nozzle position (fused into the estimate). Kicks the watchdog.
  // `stamp` = when it was measured (defaults to `now`, i.e. no latency).
  void update_pose(double x, double y, double now);
  void update_pose(double x, double y, double now, double stamp);

  // One control cycle. `dt` is the cycle period in seconds (> 0).
  ControlOutput step(double now, double dt);

  State state() const { return state_; }
  const std::string& fault_reason() const { return fault_reason_; }
  std::size_t target_index() const { return idx_; }
  std::size_t path_size() const { return path_.size(); }
  bool finished() const { return !path_.empty() && idx_ >= path_.size(); }
  // True if a pose arrived within watchdog_timeout (START/RESUME would be accepted).
  bool feedback_fresh(double now) const { return !watchdog_.expired(now); }
  // Fused position estimate the controller actually steers with.
  double est_x() const { return x_; }
  double est_y() const { return y_; }

 private:
  void fault(const std::string& reason);

  struct SegmentView {
    bool valid = false;  // false only when standing exactly on the target
    double ux = 0.0, uy = 0.0;  // unit direction of the lane
    double along = 0.0;         // distance left to the target, measured along the lane
    double cross = 0.0;         // signed sideways distance from the lane line
  };
  SegmentView view_segment() const;

  ControllerConfig cfg_;
  Watchdog watchdog_;
  std::vector<Waypoint> path_;
  std::size_t idx_ = 0;  // index of the waypoint we are driving towards
  State state_ = State::IDLE;
  std::string fault_reason_;
  double x_ = 0.0, y_ = 0.0;            // fused position estimate
  double meas_x_ = 0.0, meas_y_ = 0.0;  // latest measurement, not yet fused
  bool new_meas_ = false;
  double meas_stamp_ = 0.0;
  struct CmdSample { double t, dt, vx, vy; };  // velocity commanded at t, held for dt
  std::deque<CmdSample> cmd_history_;          // last ~1 s, for latency compensation
  double seg_x_ = 0.0, seg_y_ = 0.0;  // start point of the current segment
  double vx_ = 0.0, vy_ = 0.0;  // last commanded velocity (for the acceleration limit)
};

}  // namespace coverage
