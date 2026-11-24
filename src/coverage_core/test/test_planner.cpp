#include <gtest/gtest.h>

#include <stdexcept>

#include "coverage_core/planner.hpp"

using coverage::CoverageMap;
using coverage::Grid;
using coverage::PlannerConfig;
using coverage::Waypoint;

namespace {

// Replay only the spraying moves of a path onto a CoverageMap.
double coverage_achieved(const Grid& target, const std::vector<Waypoint>& path, double width) {
  CoverageMap map(target.cols(), target.rows(), target.cell_size());
  for (std::size_t i = 1; i < path.size(); ++i)
    if (path[i].spray)
      map.stamp_segment(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y, width);
  return map.fraction_of(target);
}

Grid full_grid(std::size_t cols, std::size_t rows, double cell) { return Grid(cols, rows, cell, 1); }

}  // namespace

TEST(Planner, EmptyTargetGivesEmptyPath) {
  Grid g(10, 10, 0.1);
  EXPECT_TRUE(coverage::plan_coverage(g, PlannerConfig{}).empty());
}

TEST(Planner, RejectsBadConfig) {
  Grid g = full_grid(5, 5, 0.1);
  EXPECT_THROW(coverage::plan_coverage(g, PlannerConfig{0.0, 0.1}), std::invalid_argument);
  EXPECT_THROW(coverage::plan_coverage(g, PlannerConfig{0.2, 1.0}), std::invalid_argument);
  EXPECT_THROW(coverage::plan_coverage(g, PlannerConfig{0.2, -0.1}), std::invalid_argument);
  EXPECT_THROW(coverage::plan_coverage(g, PlannerConfig{0.2, 0.1, -0.1}), std::invalid_argument);
  EXPECT_THROW(coverage::plan_coverage(g, PlannerConfig{0.2, 0.1, 0.6}), std::invalid_argument);
}

TEST(Planner, FullRectangleIsFullyCovered) {
  Grid g = full_grid(40, 30, 0.05);  // 2.0 m x 1.5 m
  PlannerConfig cfg{0.25, 0.1};
  auto path = coverage::plan_coverage(g, cfg);
  ASSERT_FALSE(path.empty());
  EXPECT_GE(coverage_achieved(g, path, cfg.nozzle_width), 0.999);
}

TEST(Planner, LanesAlternateDirection) {
  Grid g = full_grid(20, 20, 0.1);  // 2 m x 2 m
  auto path = coverage::plan_coverage(g, PlannerConfig{0.5, 0.0});
  // Spray moves are the odd-indexed waypoints; consecutive ones must reverse x-direction.
  ASSERT_GE(path.size(), 4u);
  const double d0 = path[1].x - path[0].x;
  const double d1 = path[3].x - path[2].x;
  EXPECT_LT(d0 * d1, 0.0);
}

TEST(Planner, NonRectangularMaskIsCovered) {
  // L-shaped region plus an isolated island, to exercise multi-run lanes.
  Grid g(40, 40, 0.05);
  for (std::size_t r = 0; r < 40; ++r)
    for (std::size_t c = 0; c < 15; ++c) g.set(c, r, true);
  for (std::size_t r = 0; r < 12; ++r)
    for (std::size_t c = 15; c < 40; ++c) g.set(c, r, true);
  for (std::size_t r = 28; r < 34; ++r)
    for (std::size_t c = 30; c < 36; ++c) g.set(c, r, true);

  PlannerConfig cfg{0.2, 0.15};
  auto path = coverage::plan_coverage(g, cfg);
  EXPECT_GE(coverage_achieved(g, path, cfg.nozzle_width), 0.999);
}

TEST(Planner, SingleLaneWhenRegionNarrowerThanNozzle) {
  Grid g = full_grid(30, 2, 0.05);  // 1.5 m x 0.1 m, nozzle 0.3 m
  auto path = coverage::plan_coverage(g, PlannerConfig{0.3, 0.1});
  EXPECT_EQ(path.size(), 2u);
  EXPECT_NEAR(coverage::path_length(path), 1.5, 1e-9);
}

TEST(Planner, MoreOverlapMeansLongerPath) {
  Grid g = full_grid(40, 40, 0.05);
  const double lo = coverage::path_length(coverage::plan_coverage(g, PlannerConfig{0.25, 0.0}));
  const double hi = coverage::path_length(coverage::plan_coverage(g, PlannerConfig{0.25, 0.5}));
  EXPECT_GT(hi, lo);
}

TEST(CoverageMap, EmptyTargetCountsAsCovered) {
  CoverageMap map(5, 5, 0.1);
  EXPECT_DOUBLE_EQ(map.fraction_of(Grid(5, 5, 0.1)), 1.0);
}

TEST(CoverageMap, StampMarksOnlyCellsInsideNozzleRadius) {
  CoverageMap map(10, 10, 0.1);  // 1 m x 1 m
  map.stamp_segment(0.0, 0.55, 1.0, 0.55, 0.2);  // horizontal strip at y=0.55, +-0.1
  EXPECT_TRUE(map.covered(5, 5));   // centre y = 0.55
  EXPECT_FALSE(map.covered(5, 8));  // centre y = 0.85, outside
}

// Regression: lanes used to be spread over the whole GRID height, so for a target
// in the middle of the image the outer lanes overhung it (found via the overspray
// metric in the run report).
TEST(Planner, LanesAreLaidOverTheTargetNotTheWholeGrid) {
  Grid g(40, 30, 0.05);  // 2.0 m x 1.5 m grid ...
  for (std::size_t r = 10; r < 20; ++r)
    for (std::size_t c = 5; c < 35; ++c) g.set(c, r, true);  // ... target only y in [0.5, 1.0)
  const PlannerConfig cfg{0.25, 0.1};
  const auto path = coverage::plan_coverage(g, cfg);
  ASSERT_FALSE(path.empty());
  for (const auto& w : path) {
    EXPECT_GE(w.y, 0.5 + cfg.nozzle_width / 2 - 1e-9);  // first lane's strip starts at the target
    EXPECT_LE(w.y, 1.0 - cfg.nozzle_width / 2 + 1e-9);  // last lane's strip ends at the target
  }
  EXPECT_GE(coverage_achieved(g, path, cfg.nozzle_width), 0.999);
}

TEST(Planner, EdgeInsetStopsSprayRunsShortOfTheMarkedEdge) {
  Grid g = full_grid(40, 30, 0.05);  // 2.0 m wide
  const auto path = coverage::plan_coverage(g, PlannerConfig{0.25, 0.1, 0.25});
  ASSERT_GE(path.size(), 2u);
  EXPECT_NEAR(path[0].x, 0.25 * 0.25, 1e-9);        // starts 6.25 cm inside the left edge
  EXPECT_NEAR(path[1].x, 2.0 - 0.25 * 0.25, 1e-9);  // ends 6.25 cm inside the right edge
}
