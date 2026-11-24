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

  // Desired speed: cruise at max_speed, but brake so we can stop at the waypoint,
  // and never plan to jump past it within one cycle.
  // Braking curve v = sqrt(2*a*d) is the continuous-time formula. In a discrete
  // loop with an acceleration limit the speed lags one cycle behind it and the
  // nozzle overshoots corners (measured: ~3 cm, missed cells). So we brake using
  // the distance that will be left AFTER this cycle's motion: d - |v|*dt.
  const double d_brake = std::max(0.0, dist - std::hypot(vx_, vy_) * dt);
  const double v_des =
      std::min({cfg_.max_speed, std::sqrt(2.0 * cfg_.max_accel * d_brake), dist / dt});
  const double dv_max = cfg_.max_accel * dt;  // velocity change allowed this cycle

  if (dist <= 1e-12) {
    // Exactly on the target but still too fast to finish: just brake.
    const double v = std::hypot(vx_, vy_);
    const double k = v > dv_max ? (v - dv_max) / v : 0.0;
    vx_ *= k;
    vy_ *= k;
    return {vx_, vy_, target.spray};
  }

  // Split the current velocity into "along the direction to the target" (u) and
  // "sideways" (n). Sideways velocity makes the nozzle drift off its lane after a
  // corner, so it is cancelled FIRST; only the remaining acceleration budget is
  // used to speed up / slow down along the lane. Total |dv| stays <= dv_max.
  const double ux = dx / dist, uy = dy / dist;  // unit vector to target
  const double v_along = vx_ * ux + vy_ * uy;
  const double v_side = -vx_ * uy + vy_ * ux;     // component along the left normal

  const double d_side = std::clamp(-v_side, -dv_max, dv_max);
  const double budget = std::sqrt(std::max(0.0, dv_max * dv_max - d_side * d_side));
  const double d_along = std::clamp(v_des - v_along, -budget, budget);

  const double new_along = v_along + d_along;
  const double new_side = v_side + d_side;
  vx_ = new_along * ux - new_side * uy;
  vy_ = new_along * uy + new_side * ux;

  // Spray flag belongs to the segment that ENDS at the target waypoint.
  return {vx_, vy_, target.spray};
}

}  // namespace coverage
