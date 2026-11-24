#include "coverage_core/planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace coverage {

namespace {
constexpr double kEps = 1e-9;
}

// ---------------------------------------------------------------- Grid

Grid::Grid(std::size_t cols, std::size_t rows, double cell_size, unsigned char fill)
    : cols_(cols), rows_(rows), cell_size_(cell_size), data_(cols * rows, fill ? 1 : 0) {
  if (cell_size <= 0.0) throw std::invalid_argument("cell_size must be > 0");
}

bool Grid::at(std::size_t c, std::size_t r) const { return data_.at(r * cols_ + c) != 0; }

void Grid::set(std::size_t c, std::size_t r, bool value) {
  data_.at(r * cols_ + c) = value ? 1 : 0;
}

std::size_t Grid::count() const {
  return static_cast<std::size_t>(std::count(data_.begin(), data_.end(), 1));
}

// ------------------------------------------------------------- planner

std::vector<Waypoint> plan_coverage(const Grid& target, const PlannerConfig& cfg) {
  if (cfg.nozzle_width <= 0.0) throw std::invalid_argument("nozzle_width must be > 0");
  if (cfg.overlap < 0.0 || cfg.overlap >= 1.0)
    throw std::invalid_argument("overlap must be in [0, 1)");

  std::vector<Waypoint> path;
  if (target.count() == 0) return path;

  const double w = cfg.nozzle_width;
  const double cell = target.cell_size();
  const double H = target.height();
  const double max_spacing = w * (1.0 - cfg.overlap);

  // Lane centres. Lane k sweeps the band [y-w/2, y+w/2]. We choose the fewest
  // lanes whose spacing is <= max_spacing and then re-space them evenly so the
  // first lane touches the bottom edge and the last touches the top edge.
  std::size_t n_lanes = 1;
  if (H > w) n_lanes = static_cast<std::size_t>(std::ceil((H - w) / max_spacing - kEps)) + 1;
  std::vector<double> lane_y(n_lanes);
  if (n_lanes == 1) {
    lane_y[0] = H / 2.0;
  } else {
    const double step = (H - w) / static_cast<double>(n_lanes - 1);
    for (std::size_t k = 0; k < n_lanes; ++k) lane_y[k] = w / 2.0 + step * static_cast<double>(k);
  }

  bool left_to_right = true;
  for (double y : lane_y) {
    // Rows of cells that overlap this lane's band.
    const double lo = std::max(0.0, y - w / 2.0);
    const double hi = std::min(H, y + w / 2.0);
    const auto r0 = static_cast<std::size_t>(std::floor(lo / cell + kEps));
    auto r1 = static_cast<std::size_t>(std::ceil(hi / cell - kEps));
    r1 = std::min(r1, target.rows());  // exclusive

    // A column is "needed" if any target cell in the band sits in it.
    std::vector<std::pair<std::size_t, std::size_t>> runs;  // [c_begin, c_end) per run
    bool in_run = false;
    std::size_t run_start = 0;
    for (std::size_t c = 0; c <= target.cols(); ++c) {
      bool needed = false;
      if (c < target.cols()) {
        for (std::size_t r = r0; r < r1 && !needed; ++r) needed = target.at(c, r);
      }
      if (needed && !in_run) {
        in_run = true;
        run_start = c;
      } else if (!needed && in_run) {
        in_run = false;
        runs.emplace_back(run_start, c);
      }
    }
    if (runs.empty()) continue;

    if (!left_to_right) std::reverse(runs.begin(), runs.end());
    for (const auto& run : runs) {
      double x_begin = static_cast<double>(run.first) * cell;
      double x_end = static_cast<double>(run.second) * cell;
      if (!left_to_right) std::swap(x_begin, x_end);
      // Travel (nozzle off) to the start of the run, then spray along it.
      path.push_back({x_begin, y, false});
      path.push_back({x_end, y, true});
    }
    left_to_right = !left_to_right;
  }
  return path;
}

double path_length(const std::vector<Waypoint>& path) {
  double total = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i)
    total += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  return total;
}

// ---------------------------------------------------------- CoverageMap

CoverageMap::CoverageMap(std::size_t cols, std::size_t rows, double cell_size)
    : cols_(cols), rows_(rows), cell_size_(cell_size), data_(cols * rows, 0) {
  if (cell_size <= 0.0) throw std::invalid_argument("cell_size must be > 0");
}

void CoverageMap::stamp_segment(double ax, double ay, double bx, double by, double nozzle_width) {
  const double half = nozzle_width / 2.0;
  const double dx = bx - ax, dy = by - ay;
  const double len2 = dx * dx + dy * dy;

  // Only visit cells inside the segment's bounding box grown by the nozzle radius.
  const auto clamp_idx = [](double v, std::size_t n) {
    return static_cast<std::size_t>(std::clamp(v, 0.0, static_cast<double>(n)));
  };
  const std::size_t c0 = clamp_idx(std::floor((std::min(ax, bx) - half) / cell_size_), cols_);
  const std::size_t c1 = clamp_idx(std::ceil((std::max(ax, bx) + half) / cell_size_), cols_);
  const std::size_t r0 = clamp_idx(std::floor((std::min(ay, by) - half) / cell_size_), rows_);
  const std::size_t r1 = clamp_idx(std::ceil((std::max(ay, by) + half) / cell_size_), rows_);

  for (std::size_t r = r0; r < r1; ++r) {
    for (std::size_t c = c0; c < c1; ++c) {
      const double px = (static_cast<double>(c) + 0.5) * cell_size_;
      const double py = (static_cast<double>(r) + 0.5) * cell_size_;
      double t = len2 > 0.0 ? ((px - ax) * dx + (py - ay) * dy) / len2 : 0.0;
      t = std::clamp(t, 0.0, 1.0);
      const double d = std::hypot(px - (ax + t * dx), py - (ay + t * dy));
      if (d <= half + kEps) data_[r * cols_ + c] = 1;
    }
  }
}

double CoverageMap::fraction_of(const Grid& target) const {
  std::size_t total = 0, hit = 0;
  for (std::size_t r = 0; r < target.rows(); ++r)
    for (std::size_t c = 0; c < target.cols(); ++c)
      if (target.at(c, r)) {
        ++total;
        if (covered(c, r)) ++hit;
      }
  return total == 0 ? 1.0 : static_cast<double>(hit) / static_cast<double>(total);
}

bool CoverageMap::covered(std::size_t c, std::size_t r) const { return data_.at(r * cols_ + c) != 0; }

}  // namespace coverage
