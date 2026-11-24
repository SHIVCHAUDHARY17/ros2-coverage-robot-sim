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
  if (cfg.lateral_gain < 0.0) throw std::invalid_argument("lateral_gain must be >= 0");
  if (cfg.pose_filter_alpha <= 0.0 || cfg.pose_filter_alpha > 1.0)
    throw std::invalid_argument("pose_filter_alpha must be in (0, 1]");
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
      // The first segment runs from wherever the nozzle is now to the target waypoint.
      seg_x_ = x_;
      seg_y_ = y_;
      state_ = State::RUNNING;
      return true;

    case Command::PAUSE:
      if (state_ != State::RUNNING) return false;
      state_ = State::PAUSED;
      vx_ = vy_ = 0.0;
      return true;

    case Command::RESUME:
      if (state_ != State::PAUSED || watchdog_.expired(now)) return false;
      state_ = State::RUNNING;  // same segment as before the pause
      return true;
  }
  return false;
}

void Controller::update_pose(double x, double y, double now) { update_pose(x, y, now, now); }

void Controller::update_pose(double x, double y, double now, double stamp) {
  meas_x_ = x;
  meas_y_ = y;
  meas_stamp_ = std::min(stamp, now);  // a stamp from the future would be a clock error
  new_meas_ = true;
  watchdog_.kick(now);
  if (state_ != State::RUNNING) {  // not moving: nothing to predict, take it as is
    x_ = x;
    y_ = y;
  }
}

void Controller::fault(const std::string& reason) {
  state_ = State::FAULT;
  fault_reason_ = reason;
  vx_ = vy_ = 0.0;
}

// Geometry of the current segment (segment start -> target waypoint), seen from the nozzle.
Controller::SegmentView Controller::view_segment() const {
  const Waypoint& t = path_[idx_];
  SegmentView s;
  double ux = t.x - seg_x_, uy = t.y - seg_y_;
  double len = std::hypot(ux, uy);
  if (len < 1e-9) {  // degenerate segment: steer straight at the target
    ux = t.x - x_;
    uy = t.y - y_;
    len = std::hypot(ux, uy);
    if (len < 1e-12) return s;  // standing on it: zero direction, zero distance
  }
  s.ux = ux / len;
  s.uy = uy / len;
  s.along = (t.x - x_) * s.ux + (t.y - y_) * s.uy;           // distance left along the lane
  s.cross = -(x_ - seg_x_) * s.uy + (y_ - seg_y_) * s.ux;    // signed sideways error (left +)
  s.valid = true;
  return s;
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

  // Pose estimate: predict with the velocity we commanded last cycle, then pull
  // the prediction towards the newest measurement (complementary filter).
  x_ += vx_ * dt;
  y_ += vy_ * dt;
  if (new_meas_) {
    // Latency compensation: move the old measurement forward by the motion we
    // commanded between its stamp and now.
    double mx = meas_x_, my = meas_y_;
    for (const auto& c : cmd_history_) {
      const double from = std::max(c.t, meas_stamp_), to = std::min(c.t + c.dt, now);
      if (to > from) {
        mx += c.vx * (to - from);
        my += c.vy * (to - from);
      }
    }
    x_ += cfg_.pose_filter_alpha * (mx - x_);
    y_ += cfg_.pose_filter_alpha * (my - y_);
    new_meas_ = false;
  }

  // Advance past every waypoint we have reached. "Reached" is measured ALONG the
  // lane (distance left <= tolerance, or already passed), so sideways sensor
  // noise cannot trigger an early switch. The LAST waypoint also requires that we
  // are slow enough to stop within one cycle (acceleration limit).
  const double dv_max = cfg_.max_accel * dt;  // velocity change allowed this cycle
  while (idx_ < path_.size()) {
    const SegmentView s = view_segment();
    const bool close = !s.valid || s.along <= cfg_.goal_tolerance;
    const bool last = (idx_ + 1 == path_.size());
    if (!close || (last && std::hypot(vx_, vy_) > dv_max)) break;
    seg_x_ = path_[idx_].x;  // next segment starts at the waypoint we just reached
    seg_y_ = path_[idx_].y;
    ++idx_;
  }
  if (idx_ >= path_.size()) {
    state_ = State::IDLE;  // path complete
    vx_ = vy_ = 0.0;
    return {};
  }

  const Waypoint& target = path_[idx_];
  const SegmentView s = view_segment();
  if (!s.valid) {
    // Exactly on the target but still too fast to finish: just brake.
    const double v = std::hypot(vx_, vy_);
    const double k = v > dv_max ? (v - dv_max) / v : 0.0;
    vx_ *= k;
    vy_ *= k;
    return {vx_, vy_, target.spray};
  }

  // Current velocity in lane coordinates: along the lane (u) and sideways (n).
  const double nx = -s.uy, ny = s.ux;  // left normal
  const double v_along = vx_ * s.ux + vy_ * s.uy;
  const double v_side = vx_ * nx + vy_ * ny;

  // Along the lane: cruise, but brake so we can stop at the waypoint. The
  // continuous-time braking curve v = sqrt(2*a*d) lags one cycle in a discrete
  // loop (measured: ~3 cm corner overshoot), so brake on the distance that will
  // be left AFTER this cycle's motion. Never plan to jump past the waypoint.
  const double d_left = std::max(0.0, s.along);
  const double d_brake = std::max(0.0, d_left - std::abs(v_along) * dt);
  const double want_along =
      std::min({cfg_.max_speed, std::sqrt(2.0 * cfg_.max_accel * d_brake), d_left / dt});

  // Sideways: steer back onto the lane line, proportional to the error
  // (cross-track control). Capped so the correction never dominates the motion.
  const double side_cap = 0.5 * cfg_.max_speed;
  const double want_side = std::clamp(-cfg_.lateral_gain * s.cross, -side_cap, side_cap);

  // Acceleration budget: the sideways correction gets priority (drifting off the
  // lane leaves untreated strips); only the rest is used along the lane.
  // Total |dv| stays <= max_accel * dt.
  const double d_side = std::clamp(want_side - v_side, -dv_max, dv_max);
  const double budget = std::sqrt(std::max(0.0, dv_max * dv_max - d_side * d_side));
  const double d_along = std::clamp(want_along - v_along, -budget, budget);

  const double new_along = v_along + d_along;
  const double new_side = v_side + d_side;
  vx_ = new_along * s.ux + new_side * nx;
  vy_ = new_along * s.uy + new_side * ny;

  // Along + sideways together must still respect max_speed. Scaling the vector
  // back onto the speed circle is a projection onto a convex set, so it cannot
  // increase the change from last cycle: the acceleration limit still holds.
  const double speed = std::hypot(vx_, vy_);
  if (speed > cfg_.max_speed) {
    vx_ *= cfg_.max_speed / speed;
    vy_ *= cfg_.max_speed / speed;
  }

  // Remember what we commanded (for latency compensation); keep about 1 s.
  cmd_history_.push_back({now, dt, vx_, vy_});
  while (!cmd_history_.empty() && cmd_history_.front().t < now - 1.0) cmd_history_.pop_front();

  // Spray flag belongs to the segment that ENDS at the target waypoint.
  return {vx_, vy_, target.spray};
}

}  // namespace coverage
