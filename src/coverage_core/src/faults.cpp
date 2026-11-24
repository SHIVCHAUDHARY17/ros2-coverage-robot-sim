#include "coverage_core/faults.hpp"

#include <cmath>
#include <stdexcept>

namespace coverage {

namespace {
constexpr double kTwoPi = 6.283185307179586;
}

PoseFaultInjector::PoseFaultInjector(const FaultConfig& cfg)
    : cfg_(cfg), rng_(cfg.seed), noise_(0.0, 1.0) {
  if (cfg.noise_std < 0.0 || cfg.vibration_amp < 0.0 || cfg.vibration_hz < 0.0 ||
      cfg.latency < 0.0 || cfg.outage_rate < 0.0 || cfg.outage_duration < 0.0)
    throw std::invalid_argument("fault parameters must be >= 0");
  if (cfg.drop_prob < 0.0 || cfg.drop_prob > 1.0)
    throw std::invalid_argument("drop_prob must be in [0, 1]");
}

std::vector<Measurement> PoseFaultInjector::process(double now, double x, double y, double dt) {
  // 1) Maybe start a blackout (Poisson process: probability rate*dt per cycle).
  if (!in_outage(now) && cfg_.outage_rate > 0.0 && uniform_(rng_) < cfg_.outage_rate * dt) {
    outage_until_ = now + cfg_.outage_duration;
    ++outages_;
  }

  // 2) Lose this measurement (blackout or random drop) ...
  if (in_outage(now) || (cfg_.drop_prob > 0.0 && uniform_(rng_) < cfg_.drop_prob)) {
    ++dropped_;
  } else {
    // 3) ... or corrupt it with noise + vibration and put it "on the wire".
    const double phase = kTwoPi * cfg_.vibration_hz * now;
    const double mx = x + cfg_.noise_std * noise_(rng_) + cfg_.vibration_amp * std::sin(phase);
    const double my = y + cfg_.noise_std * noise_(rng_) + cfg_.vibration_amp * std::cos(phase);
    in_flight_.push_back({now, now + cfg_.latency, mx, my});
  }

  // 4) Deliver everything whose arrival time has come (constant latency keeps FIFO order).
  std::vector<Measurement> out;
  while (!in_flight_.empty() && in_flight_.front().arrival <= now + 1e-9) {
    out.push_back(in_flight_.front());
    in_flight_.pop_front();
  }
  return out;
}

}  // namespace coverage
