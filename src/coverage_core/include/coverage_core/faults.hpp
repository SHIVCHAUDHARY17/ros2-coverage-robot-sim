#pragma once
// Fault injection for the position feedback channel (simulation only).
//
// Models what a real site does to a clean sensor signal:
//   noise      - Gaussian measurement noise (noisy sensor, dust)
//   vibration  - sinusoidal offset (machine vibration shaking the sensor)
//   latency    - every measurement arrives `latency` seconds late (slow network/bus)
//   drop_prob  - each measurement is lost independently with this probability
//   outages    - occasional complete blackouts (bad connectivity): they start at
//                random (Poisson, `outage_rate` per second) and last `outage_duration`
//
// Deterministic: the same seed gives the same faults, so failures are reproducible.

#include <cstddef>
#include <deque>
#include <random>
#include <vector>

namespace coverage {

struct FaultConfig {
  double noise_std = 0.0;        // m, standard deviation per axis
  double vibration_amp = 0.0;    // m
  double vibration_hz = 0.0;     // Hz
  double latency = 0.0;          // s
  double drop_prob = 0.0;        // 0..1
  double outage_rate = 0.0;      // outages per second
  double outage_duration = 0.0;  // s
  unsigned seed = 42;
};

// A measurement as the controller would receive it.
// `stamp` = when it was measured, `arrival` = when it is delivered.
struct Measurement {
  double stamp;
  double arrival;
  double x;
  double y;
};

class PoseFaultInjector {
 public:
  // Throws std::invalid_argument for negative values or drop_prob outside [0, 1].
  explicit PoseFaultInjector(const FaultConfig& cfg);

  // Feed the TRUE position measured at time `now` (call once per sim cycle, period dt).
  // Returns every corrupted measurement whose delivery time has been reached.
  std::vector<Measurement> process(double now, double x, double y, double dt);

  std::size_t dropped() const { return dropped_; }   // lost to random drops or outages
  std::size_t outages() const { return outages_; }   // number of blackouts started
  bool in_outage(double now) const { return now < outage_until_; }

 private:
  FaultConfig cfg_;
  std::mt19937 rng_;
  std::normal_distribution<double> noise_;
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};
  std::deque<Measurement> in_flight_;  // delayed measurements, ordered by arrival
  double outage_until_ = -1.0;
  std::size_t dropped_ = 0, outages_ = 0;
};

}  // namespace coverage
