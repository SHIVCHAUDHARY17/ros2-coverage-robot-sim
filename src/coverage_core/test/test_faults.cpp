#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "coverage_core/controller.hpp"
#include "coverage_core/faults.hpp"
#include "coverage_core/planner.hpp"

using coverage::Command;
using coverage::Controller;
using coverage::ControllerConfig;
using coverage::CoverageMap;
using coverage::FaultConfig;
using coverage::Grid;
using coverage::PlannerConfig;
using coverage::PoseFaultInjector;
using coverage::State;

namespace {

constexpr double kDt = 0.02;

FaultConfig faults(double noise, double vib_amp, double vib_hz, double latency, double drop,
                   double outage_rate, double outage_dur, unsigned seed = 1) {
  FaultConfig f;
  f.noise_std = noise;
  f.vibration_amp = vib_amp;
  f.vibration_hz = vib_hz;
  f.latency = latency;
  f.drop_prob = drop;
  f.outage_rate = outage_rate;
  f.outage_duration = outage_dur;
  f.seed = seed;
  return f;
}

// Closed loop: planner -> controller -> perfect kinematics, but the controller
// only sees measurements that went through the fault injector.
struct FaultyRun {
  double coverage = 0.0;
  double max_lane_dev = 0.0;  // worst |y - lane y| while spraying (true position)
  double rms_meas_err = 0.0;  // sensor error  (measurement vs truth)
  double rms_est_err = 0.0;   // estimate error (controller's fused pose vs truth)
  State final_state = State::IDLE;
};

// use_stamps=false simulates a controller that ignores message timestamps (no latency compensation).
FaultyRun run_with_faults(const FaultConfig& f, bool use_stamps = true) {
  Grid g(40, 30, 0.05, 1);  // 2.0 m x 1.5 m, L-shaped target
  for (std::size_t r = 20; r < 30; ++r)
    for (std::size_t c = 30; c < 40; ++c) g.set(c, r, false);
  const PlannerConfig pcfg{0.25, 0.1};
  const auto path = coverage::plan_coverage(g, pcfg);

  Controller ctrl{ControllerConfig{}};
  PoseFaultInjector inj(f);
  double x = 0, y = 0, t = 0;
  ctrl.update_pose(x, y, t);
  ctrl.set_path(path);
  ctrl.handle(Command::START, t);

  CoverageMap map(g.cols(), g.rows(), g.cell_size());
  FaultyRun r;
  double sum_meas = 0, sum_est = 0;
  std::size_t n_meas = 0, n_est = 0;
  for (int i = 0; i < 20000 && ctrl.state() == State::RUNNING; ++i) {
    t += kDt;
    for (const auto& m : inj.process(t, x, y, kDt)) {
      if (use_stamps)
        ctrl.update_pose(m.x, m.y, t, m.stamp);
      else
        ctrl.update_pose(m.x, m.y, t);
      if (m.stamp == t) {  // only compare with truth at the same instant (no latency)
        sum_meas += std::pow(m.x - x, 2) + std::pow(m.y - y, 2);
        ++n_meas;
      }
    }
    const std::size_t idx = ctrl.target_index();
    const auto out = ctrl.step(t, kDt);
    sum_est += std::pow(ctrl.est_x() - x, 2) + std::pow(ctrl.est_y() - y, 2);
    ++n_est;
    const double px = x, py = y;
    x += out.vx * kDt;
    y += out.vy * kDt;
    if (out.spray) {
      map.stamp_segment(px, py, x, y, pcfg.nozzle_width);
      if (idx < path.size()) r.max_lane_dev = std::max(r.max_lane_dev, std::abs(y - path[idx].y));
    }
  }
  r.coverage = map.fraction_of(g);
  r.final_state = ctrl.state();
  r.rms_meas_err = n_meas ? std::sqrt(sum_meas / n_meas) : 0.0;
  r.rms_est_err = n_est ? std::sqrt(sum_est / n_est) : 0.0;
  return r;
}

}  // namespace

// ------------------------------------------------------ injector itself

TEST(FaultInjector, RejectsBadConfig) {
  EXPECT_THROW(PoseFaultInjector(faults(-0.1, 0, 0, 0, 0, 0, 0)), std::invalid_argument);
  EXPECT_THROW(PoseFaultInjector(faults(0, 0, 0, -0.1, 0, 0, 0)), std::invalid_argument);
  EXPECT_THROW(PoseFaultInjector(faults(0, 0, 0, 0, 1.5, 0, 0)), std::invalid_argument);
}

TEST(FaultInjector, CleanConfigPassesThroughUnchanged) {
  PoseFaultInjector inj{FaultConfig{}};
  auto out = inj.process(1.0, 0.3, 0.7, kDt);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_DOUBLE_EQ(out[0].x, 0.3);
  EXPECT_DOUBLE_EQ(out[0].y, 0.7);
  EXPECT_DOUBLE_EQ(out[0].arrival, out[0].stamp);
}

TEST(FaultInjector, LatencyDelaysDeliveryButKeepsStamp) {
  PoseFaultInjector inj(faults(0, 0, 0, 0.1, 0, 0, 0));
  std::vector<coverage::Measurement> got;
  for (int i = 1; i <= 10; ++i)
    for (const auto& m : inj.process(i * kDt, i, 0, kDt)) got.push_back(m);
  ASSERT_FALSE(got.empty());
  EXPECT_NEAR(got[0].stamp, 1 * kDt, 1e-12);           // measured at the first cycle
  EXPECT_NEAR(got[0].arrival - got[0].stamp, 0.1, 1e-9);  // delivered 100 ms later
  EXPECT_EQ(got.size(), 5u);  // by cycle 10, cycles 1..5 have arrived (each 5 cycles late)
}

TEST(FaultInjector, DropProbabilityIsRespected) {
  PoseFaultInjector inj(faults(0, 0, 0, 0, 0.3, 0, 0));
  std::size_t delivered = 0;
  for (int i = 0; i < 10000; ++i) delivered += inj.process(i * kDt, 0, 0, kDt).size();
  EXPECT_NEAR(static_cast<double>(inj.dropped()) / 10000.0, 0.3, 0.02);
  EXPECT_EQ(delivered + inj.dropped(), 10000u);
}

TEST(FaultInjector, NoiseHasConfiguredSpread) {
  PoseFaultInjector inj(faults(0.01, 0, 0, 0, 0, 0, 0));
  double sum2 = 0;
  for (int i = 0; i < 10000; ++i) sum2 += std::pow(inj.process(i * kDt, 0, 0, kDt)[0].x, 2);
  EXPECT_NEAR(std::sqrt(sum2 / 10000.0), 0.01, 0.0005);
}

TEST(FaultInjector, OutagesBlackOutForTheirFullDuration) {
  PoseFaultInjector inj(faults(0, 0, 0, 0, 0, 0.5, 0.3));
  double last_arrival = 0.0, longest_gap = 0.0;
  for (int i = 1; i <= 10000; ++i)  // 200 s
    for (const auto& m : inj.process(i * kDt, 0, 0, kDt)) {
      longest_gap = std::max(longest_gap, m.arrival - last_arrival);
      last_arrival = m.arrival;
    }
  EXPECT_GT(inj.outages(), 50u);         // ~0.5/s over 200 s
  EXPECT_GE(longest_gap, 0.3 - 1e-9);    // a blackout really silences the sensor
}

TEST(FaultInjector, SameSeedGivesSameFaults) {
  PoseFaultInjector a(faults(0.01, 0, 0, 0, 0.2, 0.1, 0.2, 7));
  PoseFaultInjector b(faults(0.01, 0, 0, 0, 0.2, 0.1, 0.2, 7));
  for (int i = 0; i < 500; ++i) {
    auto ma = a.process(i * kDt, 1, 1, kDt), mb = b.process(i * kDt, 1, 1, kDt);
    ASSERT_EQ(ma.size(), mb.size());
    for (std::size_t k = 0; k < ma.size(); ++k) EXPECT_DOUBLE_EQ(ma[k].x, mb[k].x);
  }
}

// ----------------------------------- controller behaviour under faults

TEST(ControllerUnderFaults, PoseFilterReducesSensorNoise) {
  const auto r = run_with_faults(faults(0.01, 0, 0, 0, 0, 0, 0));
  EXPECT_LT(r.rms_est_err, 0.5 * r.rms_meas_err);  // fused estimate beats the raw sensor
}

TEST(ControllerUnderFaults, NoisySensorStillFullCoverageOnLane) {
  const auto r = run_with_faults(faults(0.003, 0, 0, 0, 0, 0, 0));
  EXPECT_DOUBLE_EQ(r.coverage, 1.0);
  EXPECT_LT(r.max_lane_dev, 0.01);
}

TEST(ControllerUnderFaults, DroppedMessagesAreBridgedByPrediction) {
  const auto r = run_with_faults(faults(0, 0, 0, 0, 0.3, 0, 0));
  EXPECT_DOUBLE_EQ(r.coverage, 1.0);
  EXPECT_LT(r.max_lane_dev, 0.01);
}

TEST(ControllerUnderFaults, CombinedSiteConditionsStillFullCoverage) {
  // noise 3 mm, vibration 2 mm @ 8 Hz, 40 ms latency, 10 % drops
  const auto r = run_with_faults(faults(0.003, 0.002, 8.0, 0.04, 0.1, 0, 0));
  EXPECT_EQ(r.final_state, State::IDLE);
  EXPECT_DOUBLE_EQ(r.coverage, 1.0);
  EXPECT_LT(r.max_lane_dev, 0.01);
}

TEST(ControllerUnderFaults, LatencyIsCompensatedUsingTimestamps) {
  const auto f = faults(0, 0, 0, 0.1, 0, 0, 0);  // 100 ms late measurements
  const auto with = run_with_faults(f, /*use_stamps=*/true);
  const auto without = run_with_faults(f, /*use_stamps=*/false);
  EXPECT_LT(with.max_lane_dev, 0.01);     // compensated: stays on the lane
  EXPECT_GT(without.max_lane_dev, 0.02);  // ignoring stamps: drifts off the lane
  EXPECT_LT(with.rms_est_err, 0.5 * without.rms_est_err);
}

TEST(ControllerUnderFaults, BlackoutLongerThanWatchdogStopsTheRobot) {
  // 0.5 s blackouts, frequent: the controller must FAULT instead of moving blind.
  const auto r = run_with_faults(faults(0, 0, 0, 0, 0, 0.5, 0.5));
  EXPECT_EQ(r.final_state, State::FAULT);
}
