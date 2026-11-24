#pragma once
// Pure C++17 coverage-planning core. Deliberately has NO ROS dependency:
// the ROS 2 nodes (Day 2) are thin wrappers around this library, so all the
// logic can be unit-tested fast and without a running robot graph.

#include <cstddef>
#include <vector>

namespace coverage {

// A single path waypoint. spray=false means "travel move, nozzle off".
struct Waypoint {
  double x;
  double y;
  bool spray;
};

// Row-major boolean grid. Origin (0,0) is the bottom-left corner.
// value 1 = cell must be treated ("target"), 0 = leave alone.
class Grid {
 public:
  Grid(std::size_t cols, std::size_t rows, double cell_size, unsigned char fill = 0);

  std::size_t cols() const { return cols_; }
  std::size_t rows() const { return rows_; }
  double cell_size() const { return cell_size_; }
  double width() const { return static_cast<double>(cols_) * cell_size_; }
  double height() const { return static_cast<double>(rows_) * cell_size_; }

  bool at(std::size_t c, std::size_t r) const;
  void set(std::size_t c, std::size_t r, bool value);
  std::size_t count() const;  // number of target cells

 private:
  std::size_t cols_, rows_;
  double cell_size_;
  std::vector<unsigned char> data_;
};

struct PlannerConfig {
  double nozzle_width = 0.2;  // metres, width of the treated strip
  double overlap = 0.1;       // 0..<1, fraction of width shared by adjacent lanes
};

// Boustrophedon ("lawn-mower") coverage over the target cells of `target`.
// Throws std::invalid_argument for a bad config. Empty target -> empty path.
std::vector<Waypoint> plan_coverage(const Grid& target, const PlannerConfig& cfg);

// Total length of spraying + travel moves, in metres.
double path_length(const std::vector<Waypoint>& path);

// Tracks which cells of a grid have been treated by a moving nozzle.
// Used by the planner tests now, and by the live coverage tracker node later.
class CoverageMap {
 public:
  CoverageMap(std::size_t cols, std::size_t rows, double cell_size);

  // Mark every cell whose centre is within nozzle_width/2 of the segment a->b.
  void stamp_segment(double ax, double ay, double bx, double by, double nozzle_width);

  // Fraction (0..1) of `target` cells that have been treated. 1.0 if target is empty.
  double fraction_of(const Grid& target) const;

  bool covered(std::size_t c, std::size_t r) const;

 private:
  std::size_t cols_, rows_;
  double cell_size_;
  std::vector<unsigned char> data_;
};

}  // namespace coverage
