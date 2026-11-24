#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "coverage_core/controller.hpp"
#include "coverage_core/planner.hpp"

using coverage::Command;
using coverage::Controller;
using coverage::ControllerConfig;
using coverage::CoverageMap;
using coverage::Grid;
using coverage::PlannerConfig;
using coverage::State;
using coverage::Waypoint;
using coverage::Watchdog;

namespace {

constexpr double kDt = 0.02;  // 50 Hz control loop

// Minimal closed loop: perfect kinematic "robot" that integrates the command.
struct Loop {
  Controller ctrl;
  double x = 0.0, y = 0.0, t = 0.0;
  double max_speed_seen = 0.0, max_accel_seen = 0.0;
  double last_vx = 0.0, last_vy = 0.0;

  explicit Loop(const ControllerConfig& cfg) : ctrl(cfg) { ctrl.update_pose(x, y, t); }

  coverage::ControlOutput tick(bool send_pose = true) {
    t += kDt;
    if (send_pose) ctrl.update_pose(x, y, t);
    auto out = ctrl.step(t, kDt);
    max_speed_seen = std::max(max_speed_seen, std::hypot(out.vx, out.vy));
    max_accel_seen = std::max(max_accel_seen, std::hypot(out.vx - last_vx, out.vy - last_vy) / kDt);
    last_vx = out.vx;
    last_vy = out.vy;
    x += out.vx * kDt;
    y += out.vy * kDt;
    return out;
  }
};

std::vector<Waypoint> line_path() { return {{0.0, 0.0, false}, {1.0, 0.0, true}}; }

}  // namespace

// ------------------------------------------------------------- Watchdog

TEST(Watchdog, NeverKickedCountsAsExpired) {
  Watchdog w(0.2);
  EXPECT_TRUE(w.expired(0.0));
}

TEST(Watchdog, ExpiresOnlyAfterTimeout) {
  Watchdog w(0.2);
  w.kick(1.0);
  EXPECT_FALSE(w.expired(1.19));
  EXPECT_TRUE(w.expired(1.21));
  w.kick(1.21);
  EXPECT_FALSE(w.expired(1.3));
}

// ----------------------------------------------------- state machine

TEST(Controller, RejectsBadConfig) {
  EXPECT_THROW(Controller(ControllerConfig{0.0, 1.0, 0.005, 0.2}), std::invalid_argument);
  EXPECT_THROW(Controller(ControllerConfig{0.5, 0.0, 0.005, 0.2}), std::invalid_argument);
  EXPECT_THROW(Controller(ControllerConfig{0.5, 1.0, 0.0, 0.2}), std::invalid_argument);
  EXPECT_THROW(Controller(ControllerConfig{0.5, 1.0, 0.005, 0.0}), std::invalid_argument);
  EXPECT_THROW(Controller(ControllerConfig{0.5, 1.0, 0.005, 0.2, -1.0}), std::invalid_argument);
  EXPECT_THROW(Controller(ControllerConfig{0.5, 1.0, 0.005, 0.2, 4.0, 0.0}), std::invalid_argument);
  EXPECT_THROW(Controller(ControllerConfig{0.5, 1.0, 0.005, 0.2, 4.0, 1.5}), std::invalid_argument);
}

TEST(Controller, StartNeedsPathAndFreshPose) {
  Controller c{ControllerConfig{}};
  c.update_pose(0, 0, 0.0);
  EXPECT_FALSE(c.handle(Command::START, 0.0));  // no path
  ASSERT_TRUE(c.set_path(line_path()));
  EXPECT_FALSE(c.handle(Command::START, 5.0));  // pose is 5 s old
  c.update_pose(0, 0, 5.0);
  EXPECT_TRUE(c.handle(Command::START, 5.0));
  EXPECT_EQ(c.state(), State::RUNNING);
}

TEST(Controller, InvalidTransitionsAreRefused) {
  Controller c{ControllerConfig{}};
  EXPECT_FALSE(c.handle(Command::PAUSE, 0.0));   // not running
  EXPECT_FALSE(c.handle(Command::RESUME, 0.0));  // not paused
  EXPECT_FALSE(c.handle(Command::RESET, 0.0));   // nothing to reset
  EXPECT_EQ(c.state(), State::IDLE);
}

TEST(Controller, PauseStopsMotionAndResumeContinues) {
  Loop l{ControllerConfig{}};
  l.ctrl.set_path(line_path());
  ASSERT_TRUE(l.ctrl.handle(Command::START, l.t));
  for (int i = 0; i < 20; ++i) l.tick();
  ASSERT_TRUE(l.ctrl.handle(Command::PAUSE, l.t));
  auto out = l.tick();
  EXPECT_EQ(out.vx, 0.0);
  EXPECT_FALSE(out.spray);
  ASSERT_TRUE(l.ctrl.handle(Command::RESUME, l.t));
  for (int i = 0; i < 5; ++i) out = l.tick();
  EXPECT_GT(out.vx, 0.0);
}

TEST(Controller, EstopWinsInEveryStateAndOnlyResetLeavesIt) {
  for (State from : {State::IDLE, State::RUNNING, State::PAUSED}) {
    Loop l{ControllerConfig{}};
    l.ctrl.set_path(line_path());
    if (from != State::IDLE) l.ctrl.handle(Command::START, l.t);
    if (from == State::PAUSED) l.ctrl.handle(Command::PAUSE, l.t);
    ASSERT_EQ(l.ctrl.state(), from);

    EXPECT_TRUE(l.ctrl.handle(Command::ESTOP, l.t));
    EXPECT_EQ(l.ctrl.state(), State::ESTOP);
    EXPECT_FALSE(l.ctrl.handle(Command::START, l.t));
    EXPECT_FALSE(l.ctrl.handle(Command::RESUME, l.t));
    auto out = l.tick();
    EXPECT_EQ(out.vx, 0.0);
    EXPECT_EQ(out.vy, 0.0);
    EXPECT_FALSE(out.spray);
    EXPECT_TRUE(l.ctrl.handle(Command::RESET, l.t));
    EXPECT_EQ(l.ctrl.state(), State::IDLE);
  }
}

TEST(Controller, LostPoseFeedbackTriggersFault) {
  Loop l{ControllerConfig{}};
  l.ctrl.set_path(line_path());
  ASSERT_TRUE(l.ctrl.handle(Command::START, l.t));
  for (int i = 0; i < 10; ++i) l.tick();
  // Sensor drops out: 9 cycles (0.18 s) is still fine, the 11th (0.22 s) is not.
  for (int i = 0; i < 9; ++i) l.tick(/*send_pose=*/false);
  EXPECT_EQ(l.ctrl.state(), State::RUNNING);
  l.tick(false);
  l.tick(false);
  EXPECT_EQ(l.ctrl.state(), State::FAULT);
  EXPECT_EQ(l.ctrl.fault_reason(), "pose feedback timeout");
  auto out = l.tick();
  EXPECT_EQ(out.vx, 0.0);
  EXPECT_FALSE(out.spray);
}

TEST(Controller, CannotSwapPathWhileMoving) {
  Loop l{ControllerConfig{}};
  l.ctrl.set_path(line_path());
  l.ctrl.handle(Command::START, l.t);
  EXPECT_FALSE(l.ctrl.set_path(line_path()));
}

TEST(Controller, ResetAfterFaultContinuesFromSameWaypoint) {
  Loop l{ControllerConfig{}};
  l.ctrl.set_path({{0.0, 0.0, false}, {0.5, 0.0, true}, {0.5, 0.5, false}, {0.0, 0.5, true}});
  l.ctrl.handle(Command::START, l.t);
  while (l.ctrl.target_index() < 2) l.tick();
  for (int i = 0; i < 15; ++i) l.tick(false);  // force a fault
  ASSERT_EQ(l.ctrl.state(), State::FAULT);
  ASSERT_TRUE(l.ctrl.handle(Command::RESET, l.t));
  l.ctrl.update_pose(l.x, l.y, l.t);
  ASSERT_TRUE(l.ctrl.handle(Command::START, l.t));
  EXPECT_EQ(l.ctrl.target_index(), 2u);  // did not restart from waypoint 0
}

// ------------------------------------------------------ motion quality

TEST(Controller, RespectsSpeedAndAccelerationLimits) {
  ControllerConfig cfg{0.4, 0.8, 0.005, 0.2};
  Loop l{cfg};
  Grid g(30, 30, 0.05, 1);  // 1.5 m x 1.5 m
  l.ctrl.set_path(coverage::plan_coverage(g, PlannerConfig{0.25, 0.1}));
  l.ctrl.handle(Command::START, l.t);
  for (int i = 0; i < 20000 && l.ctrl.state() == State::RUNNING; ++i) l.tick();
  EXPECT_LE(l.max_speed_seen, cfg.max_speed + 1e-9);
  EXPECT_LE(l.max_accel_seen, cfg.max_accel + 1e-6);
}

TEST(Controller, NozzleIsOffWhileTravellingToFirstLane) {
  Loop l{ControllerConfig{}};
  l.ctrl.set_path({{1.0, 1.0, false}, {2.0, 1.0, true}});
  l.ctrl.handle(Command::START, l.t);
  auto out = l.tick();
  EXPECT_FALSE(out.spray);
  while (l.ctrl.target_index() == 0) out = l.tick();
  out = l.tick();
  EXPECT_TRUE(out.spray);
}

// The end-to-end check of Day 1 + Day 2 logic: plan -> follow -> measure coverage.
TEST(Controller, ClosedLoopRunCoversWholeTarget) {
  Grid g(40, 30, 0.05, 1);  // 2.0 m x 1.5 m
  for (std::size_t r = 20; r < 30; ++r)  // cut out a corner: non-rectangular target
    for (std::size_t c = 30; c < 40; ++c) g.set(c, r, false);
  const PlannerConfig pcfg{0.25, 0.1};

  Loop l{ControllerConfig{}};
  ASSERT_TRUE(l.ctrl.set_path(coverage::plan_coverage(g, pcfg)));
  ASSERT_TRUE(l.ctrl.handle(Command::START, l.t));

  CoverageMap map(g.cols(), g.rows(), g.cell_size());
  for (int i = 0; i < 50000 && l.ctrl.state() == State::RUNNING; ++i) {
    const double px = l.x, py = l.y;
    if (l.tick().spray) map.stamp_segment(px, py, l.x, l.y, pcfg.nozzle_width);
  }
  EXPECT_EQ(l.ctrl.state(), State::IDLE);
  EXPECT_TRUE(l.ctrl.finished());
  EXPECT_DOUBLE_EQ(map.fraction_of(g), 1.0);  // every target cell treated
}

// Regression test: the first ROS run showed 99.5 % coverage because the nozzle
// overshot the first corner by ~3 cm. While spraying, it must stay on its lane.
TEST(Controller, StaysOnLaneWhileSpraying) {
  Grid g(40, 30, 0.05, 1);
  auto path = coverage::plan_coverage(g, PlannerConfig{0.25, 0.1});
  Loop l{ControllerConfig{}};
  l.ctrl.set_path(path);
  l.ctrl.handle(Command::START, l.t);
  double worst = 0.0;
  for (int i = 0; i < 50000 && l.ctrl.state() == State::RUNNING; ++i) {
    const std::size_t idx = l.ctrl.target_index();
    if (l.tick().spray && idx < path.size())  // spray lanes are horizontal: y must match
      worst = std::max(worst, std::abs(l.y - path[idx].y));
  }
  EXPECT_LT(worst, 0.005);  // < 5 mm off the lane centre line
}

TEST(Controller, RectangleIsCoveredCompletely) {
  Grid g(40, 30, 0.05, 1);
  const PlannerConfig pcfg{0.25, 0.1};
  Loop l{ControllerConfig{}};
  l.ctrl.set_path(coverage::plan_coverage(g, pcfg));
  l.ctrl.handle(Command::START, l.t);
  CoverageMap map(g.cols(), g.rows(), g.cell_size());
  for (int i = 0; i < 50000 && l.ctrl.state() == State::RUNNING; ++i) {
    const double px = l.x, py = l.y;
    if (l.tick().spray) map.stamp_segment(px, py, l.x, l.y, pcfg.nozzle_width);
  }
  EXPECT_DOUBLE_EQ(map.fraction_of(g), 1.0);
}

TEST(Controller, NoNaNWhenExactlyOnLastWaypointWhileMoving) {
  Loop l{ControllerConfig{}};
  l.ctrl.set_path({{0.5, 0.0, true}});
  l.ctrl.handle(Command::START, l.t);
  for (int i = 0; i < 30; ++i) l.tick();  // build up speed towards x = 0.5
  l.ctrl.update_pose(0.5, 0.0, l.t);      // teleport exactly onto the target
  auto out = l.ctrl.step(l.t, kDt);
  EXPECT_FALSE(std::isnan(out.vx));
  EXPECT_FALSE(std::isnan(out.vy));
}

TEST(Controller, StateNames) {
  EXPECT_STREQ(coverage::to_string(State::ESTOP), "ESTOP");
  EXPECT_STREQ(coverage::to_string(State::FAULT), "FAULT");
}

// Coverage-vs-overspray trade-off on a round marking (slanted edges everywhere):
// edge_inset 0.25 must keep 100 % coverage while spraying less outside the target.
TEST(Controller, EdgeInsetCutsOversprayWithoutLosingCoverage) {
  Grid g(40, 30, 0.05);
  for (std::size_t r = 0; r < 30; ++r)
    for (std::size_t c = 0; c < 40; ++c)
      g.set(c, r, std::hypot((c + 0.5) * 0.05 - 1.0, (r + 0.5) * 0.05 - 0.75) < 0.6);
  auto run = [&](double inset, double& overspray) {
    const PlannerConfig pcfg{0.25, 0.1, inset};
    Loop l{ControllerConfig{}};
    l.ctrl.set_path(coverage::plan_coverage(g, pcfg));
    l.ctrl.handle(Command::START, l.t);
    CoverageMap map(g.cols(), g.rows(), g.cell_size());
    for (int i = 0; i < 50000 && l.ctrl.state() == State::RUNNING; ++i) {
      const double px = l.x, py = l.y;
      if (l.tick().spray) map.stamp_segment(px, py, l.x, l.y, pcfg.nozzle_width);
    }
    overspray = 0;
    for (std::size_t r = 0; r < g.rows(); ++r)
      for (std::size_t c = 0; c < g.cols(); ++c) overspray += map.covered(c, r) && !g.at(c, r);
    return map.fraction_of(g);
  };
  double over0 = 0, over25 = 0;
  EXPECT_DOUBLE_EQ(run(0.0, over0), 1.0);
  EXPECT_DOUBLE_EQ(run(0.25, over25), 1.0);
  EXPECT_LT(over25, 0.85 * over0);  // at least 15 % less concrete sprayed by mistake
}
