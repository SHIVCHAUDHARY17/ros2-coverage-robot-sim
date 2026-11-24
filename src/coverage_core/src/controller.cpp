#include "coverage_core/controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace coverage {

// ------------------------------------------------------------- Watchdog

Watchdog::Watchdog(double timeout_s) : timeout_(timeout_s) {
  if (timeout_s <= 0.0) throw std::invalid_argument("watchdog timeout must be > 0");
}

void Watchdog::kick(double now) {
  last_kick_ = now;
  kicked_ = true;
}

bool Watchdog::expired(double now) const { return !kicked_ || (now - last_kick_) > timeout_; }

// ------------------------------------------------------------- helpers

const char* to_string(State s) {
  switch (s) {
    case State::IDLE: return "IDLE";
    case State::RUNNING: return "RUNNING";
    case State::PAUSED: return "PAUSED";
    case State::ESTOP: return "ESTOP";
    case State::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

// ----------------------------------------------------------- Controller

Controller::Controller(const ControllerConfig& cfg)
    : cfg_(cfg), watchdog_(cfg.watchdog_timeout) {
  if (cfg.max_speed <= 0.0) throw std::invalid_argument("max_speed must be > 0");
  if (cfg.max_accel <= 0.0) throw std::invalid_argument("max_accel must be > 0");
  if (cfg.goal_tolerance <= 0.0) throw std::invalid_argument("goal_tolerance must be > 0");
}

bool Controller::set_path(std::vector<Waypoint> path) {
  if (state_ != State::IDLE) return false;  // never swap the path under a moving robot
  path_ = std::move(path);
  idx_ = 0;
  return true;
}

bool Controller::handle(Command cmd, double now) {
  switch (cmd) {
    case Command::ESTOP:
      // Accepted in every state, no conditions. This is the one rule that must never fail.
      state_ = State::ESTOP;
      vx_ = vy_ = 0.0;
      return true;

    case Command::RESET:
      if (state_ != State::ESTOP && state_ != State::FAULT) return false;
      state_ = State::IDLE;  // progress (idx_) is kept, so START continues where we stopped
      fault_reason_.clear();
      return true;

    case Command::START:
      if (state_ != State::IDLE || path_.empty() || watchdog_.expired(now)) return false;
      if (finished()) idx_ = 0;  // previous run completed: start a new pass
      state_ = State::RUNNING;
      return true;

    case Command::PAUSE:
      if (state_ != State::RUNNING) return false;
      state_ = State::PAUSED;
      vx_ = vy_ = 0.0;
      return true;

    case Command::RESUME:
      if (state_ != State::PAUSED || watchdog_.expired(now)) return false;
      state_ = State::RUNNING;
      return true;
  }
  return false;
}

void Controller::update_pose(double x, double y, double now) {
  x_ = x;
  y_ = y;
  watchdog_.kick(now);
}

void Controller::fault(const std::string& reason) {
  state_ = State::FAULT;
  fault_reason_ = reason;
  vx_ = vy_ = 0.0;
}

ControlOutput Controller::step(double now, double dt) {
  if (state_ != State::RUNNING || dt <= 0.0) {
    vx_ = vy_ = 0.0;
    return {};  // zero velocity, nozzle off
  }
  if (watchdog_.expired(now)) {
    fault("pose feedback timeout");
    return {};
  }

  // Skip every waypoint we are already standing on. The LAST waypoint additionally
  // requires that we are slow enough to stop within one cycle; otherwise finishing
  // would mean an instant jump to zero speed (violating the acceleration limit).
  const double stop_speed = cfg_.max_accel * dt;
  auto reached = [&](std::size_t i) {
    const bool close = std::hypot(path_[i].x - x_, path_[i].y - y_) <= cfg_.goal_tolerance;
    const bool last = (i + 1 == path_.size());
    return close && (!last || std::hypot(vx_, vy_) <= stop_speed);
  };
  while (idx_ < path_.size() && reached(idx_)) ++idx_;
  if (idx_ >= path_.size()) {
    state_ = State::IDLE;  // path complete
    vx_ = vy_ = 0.0;
    return {};
  }

  const Waypoint& target = path_[idx_];
  const double dx = target.x - x_, dy = target.y - y_;
  const double dist = std::hypot(dx, dy);

  // Desired speed: cruise at max_speed, but brake (v = sqrt(2*a*d)) so we can
  // stop at the waypoint, and never plan to jump past it within one cycle.
  const double v_des = std::min({cfg_.max_speed, std::sqrt(2.0 * cfg_.max_accel * dist), dist / dt});
  // Exactly on the target (but still too fast to finish): desired velocity is zero.
  const double des_vx = dist > 1e-12 ? v_des * dx / dist : 0.0;
  const double des_vy = dist > 1e-12 ? v_des * dy / dist : 0.0;

  // Acceleration limit on the velocity VECTOR: |v_new - v_old| <= max_accel * dt.
  // This also limits how sharply the nozzle can turn at lane ends.
  double dvx = des_vx - vx_, dvy = des_vy - vy_;
  const double dv = std::hypot(dvx, dvy);
  const double dv_max = cfg_.max_accel * dt;
  if (dv > dv_max) {
    dvx *= dv_max / dv;
    dvy *= dv_max / dv;
  }
  vx_ += dvx;
  vy_ += dvy;

  // Spray flag belongs to the segment that ENDS at the target waypoint.
  return {vx_, vy_, target.spray};
}

}  // namespace coverage
