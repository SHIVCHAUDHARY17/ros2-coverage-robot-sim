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
//  * Motion limits: speed <= max_speed, |speed change| <= max_accel * dt, and
//    the speed profile brakes so the nozzle can stop at every waypoint.

#include <cstddef>
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
};

// Velocity command for the gantry plus the nozzle valve.
struct ControlOutput {
  double vx = 0.0;
  double vy = 0.0;
  bool spray = false;
};

class Controller {
 public:
  // Throws std::invalid_argument if any limit is <= 0.
  explicit Controller(const ControllerConfig& cfg);

  // Load a new path. Only allowed in IDLE; resets progress. Returns false if refused.
  bool set_path(std::vector<Waypoint> path);

  // Operator command. Returns true if the command was accepted in the current state.
  bool handle(Command cmd, double now);

  // Latest measured nozzle position. Also kicks the feedback watchdog.
  void update_pose(double x, double y, double now);

  // One control cycle. `dt` is the cycle period in seconds (> 0).
  ControlOutput step(double now, double dt);

  State state() const { return state_; }
  const std::string& fault_reason() const { return fault_reason_; }
  std::size_t target_index() const { return idx_; }
  std::size_t path_size() const { return path_.size(); }
  bool finished() const { return !path_.empty() && idx_ >= path_.size(); }

 private:
  void fault(const std::string& reason);

  ControllerConfig cfg_;
  Watchdog watchdog_;
  std::vector<Waypoint> path_;
  std::size_t idx_ = 0;  // index of the waypoint we are driving towards
  State state_ = State::IDLE;
  std::string fault_reason_;
  double x_ = 0.0, y_ = 0.0;
  double vx_ = 0.0, vy_ = 0.0;  // last commanded velocity (for the acceleration limit)
};

}  // namespace coverage
