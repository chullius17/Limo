#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

#include "limo_dwb_critics/obstacle_guidance.hpp"

using limo_dwb_critics::ObstacleGuidance;


namespace
{
// Frozen pre-optimization implementation. Keep its arithmetic, neighbor order,
// heap ordering and obstacle inflation independent of production helpers.
// It is intentionally straightforward: these tests compare public behavior of
// the optimized implementation against the algorithm used before optimization.
constexpr double infinity = std::numeric_limits<double>::infinity();

class FrozenObstacleGuidance
{
public:
  void update(
    unsigned int width, unsigned int height, double resolution,
    const unsigned char * costs, double clearance, double cost_weight);
  bool traversable(int x, int y) const;
  bool setTarget(int x, int y);
  double distance(double x, double y) const;
  double heading(double x, double y, double current_heading) const;

private:
  unsigned int width_{0};
  unsigned int height_{0};
  double resolution_{0.0};
  std::vector<bool> blocked_;
  std::vector<double> penalties_;
  std::vector<double> distances_;
};

void FrozenObstacleGuidance::update(
  unsigned int width, unsigned int height, double resolution,
  const unsigned char * costs, double clearance, double cost_weight)
{
  if (!width || !height || !costs || !std::isfinite(resolution) || resolution <= 0.0 ||
    !std::isfinite(clearance) || clearance < 0.0 ||
    !std::isfinite(cost_weight) || cost_weight < 0.0)
  {
    throw std::invalid_argument("Invalid obstacle guidance grid");
  }
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  const std::size_t count = static_cast<std::size_t>(width) * height;
  blocked_.assign(count, false);
  penalties_.resize(count);
  distances_.assign(count, infinity);
  // Include a half-cell diagonal so clearance applies to obstacle cell area.
  const double radius = clearance / resolution + std::sqrt(0.5);
  const int cells = static_cast<int>(std::ceil(radius));
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      const auto index = static_cast<std::size_t>(y) * width + x;
      blocked_[index] = blocked_[index] || costs[index] >= 253 ||
        (x + 0.5) * resolution < clearance || (y + 0.5) * resolution < clearance ||
        (width - x - 0.5) * resolution < clearance ||
        (height - y - 0.5) * resolution < clearance;
      penalties_[index] = 1.0 + cost_weight * std::min<int>(costs[index], 252) / 252.0;
      if (costs[index] < 254) {
        continue;
      }
      for (int dy = -cells; dy <= cells; ++dy) {
        for (int dx = -cells; dx <= cells; ++dx) {
          const int nx = static_cast<int>(x) + dx;
          const int ny = static_cast<int>(y) + dy;
          if (nx >= 0 && ny >= 0 && nx < static_cast<int>(width) &&
            ny < static_cast<int>(height) && std::hypot(dx, dy) <= radius)
          {
            blocked_[static_cast<std::size_t>(ny) * width + nx] = true;
          }
        }
      }
    }
  }
}

bool FrozenObstacleGuidance::traversable(int x, int y) const
{
  return x >= 0 && y >= 0 && x < static_cast<int>(width_) &&
         y < static_cast<int>(height_) &&
         !blocked_[static_cast<std::size_t>(y) * width_ + x];
}

bool FrozenObstacleGuidance::setTarget(int x, int y)
{
  std::fill(distances_.begin(), distances_.end(), infinity);
  if (!traversable(x, y)) {
    return false;
  }
  using Entry = std::pair<double, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  const auto target = static_cast<std::size_t>(y) * width_ + x;
  distances_[target] = 0.0;
  queue.emplace(0.0, target);
  while (!queue.empty()) {
    const auto entry = queue.top();
    queue.pop();
    if (entry.first > distances_[entry.second]) {
      continue;
    }
    const int cx = entry.second % width_;
    const int cy = entry.second / width_;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const int nx = cx + dx;
        const int ny = cy + dy;
        if ((!dx && !dy) || !traversable(nx, ny) ||
          (dx && dy && (!traversable(cx + dx, cy) || !traversable(cx, cy + dy))))
        {
          continue;
        }
        const auto next = static_cast<std::size_t>(ny) * width_ + nx;
        const double candidate = entry.first + resolution_ * std::hypot(dx, dy) *
          0.5 * (penalties_[entry.second] + penalties_[next]);
        if (candidate < distances_[next]) {
          distances_[next] = candidate;
          queue.emplace(candidate, next);
        }
      }
    }
  }
  return true;
}

double FrozenObstacleGuidance::distance(double x, double y) const
{
  if (!std::isfinite(x) || !std::isfinite(y) || x < -0.5 || y < -0.5 ||
    x >= width_ - 0.5 || y >= height_ - 0.5)
  {
    return infinity;
  }
  const int nearest_x = static_cast<int>(std::floor(x + 0.5));
  const int nearest_y = static_cast<int>(std::floor(y + 0.5));
  if (!traversable(nearest_x, nearest_y) ||
    !std::isfinite(distances_[static_cast<std::size_t>(nearest_y) * width_ + nearest_x]))
  {
    return infinity;
  }
  // Smooth the finite grid values so sub-cell MPC progress receives a reward.
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  double sum = 0.0;
  double weights = 0.0;
  for (int dy = 0; dy <= 1; ++dy) {
    for (int dx = 0; dx <= 1; ++dx) {
      const int nx = x0 + dx;
      const int ny = y0 + dy;
      if (!traversable(nx, ny)) {
        continue;
      }
      const double value = distances_[static_cast<std::size_t>(ny) * width_ + nx];
      const double weight = (dx ? x - x0 : 1.0 - (x - x0)) *
        (dy ? y - y0 : 1.0 - (y - y0));
      if (std::isfinite(value) && weight > 0.0) {
        sum += weight * value;
        weights += weight;
      }
    }
  }
  return weights > 0.0 ? sum / weights : infinity;
}

double FrozenObstacleGuidance::heading(double x, double y, double current_heading) const
{
  if (!std::isfinite(x) || !std::isfinite(y) || x < -0.5 || y < -0.5 ||
    x >= width_ - 0.5 || y >= height_ - 0.5)
  {
    return current_heading;
  }
  int cx = static_cast<int>(std::floor(x + 0.5));
  int cy = static_cast<int>(std::floor(y + 0.5));
  if (!traversable(cx, cy)) {
    return current_heading;
  }
  const int start_x = cx;
  const int start_y = cy;
  double length = 0.0;
  double direction = current_heading;
  // Follow a short section of the optimal grid route rather than using one
  // neighbor's 45-degree direction. This suppresses cell-to-cell heading jumps.
  while (length < 0.35) {
    const auto index = static_cast<std::size_t>(cy) * width_ + cx;
    const double current = distances_[index];
    double best_cost = infinity;
    double best_angle = infinity;
    int step_x = 0;
    int step_y = 0;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if ((!dx && !dy) || !traversable(cx + dx, cy + dy) ||
          (dx && dy && (!traversable(cx + dx, cy) || !traversable(cx, cy + dy))))
        {
          continue;
        }
        const auto next = static_cast<std::size_t>(cy + dy) * width_ + cx + dx;
        if (distances_[next] >= current) {
          continue;
        }
        const double cost = distances_[next] + resolution_ * std::hypot(dx, dy) *
          0.5 * (penalties_[index] + penalties_[next]);
        const double heading = std::atan2(dy, dx);
        const double angle = std::abs(std::atan2(
            std::sin(heading - direction), std::cos(heading - direction)));
        if (cost < best_cost - 1e-9 ||
          (std::abs(cost - best_cost) <= 1e-9 && angle < best_angle))
        {
          best_cost = cost;
          best_angle = angle;
          step_x = dx;
          step_y = dy;
        }
      }
    }
    if (!step_x && !step_y) {
      break;
    }
    cx += step_x;
    cy += step_y;
    length += resolution_ * std::hypot(step_x, step_y);
    direction = std::atan2(step_y, step_x);
  }
  return length > 0.0 ? std::atan2(cy - start_y, cx - start_x) : current_heading;
}


void expectSameDistance(double actual, double expected)
{
  if (std::isinf(expected)) {
    EXPECT_EQ(actual, expected);
  } else {
    EXPECT_NEAR(actual, expected, 1e-12);
  }
}

void expectSameQueries(
  const ObstacleGuidance & actual, const FrozenObstacleGuidance & expected,
  int width, int height, bool subcell = true)
{
  const std::vector<std::pair<double, double>> offsets = subcell ?
    std::vector<std::pair<double, double>>{{0.0, 0.0}, {0.23, 0.41}, {-0.49, 0.49}} :
    std::vector<std::pair<double, double>>{{0.0, 0.0}};
  const std::vector<double> headings = subcell ?
    std::vector<double>{-2.7, 0.13, 2.9} : std::vector<double>{0.41};
  for (int y = -1; y <= height; ++y) {
    for (int x = -1; x <= width; ++x) {
      SCOPED_TRACE(::testing::Message() << "cell (" << x << ", " << y << ")");
      EXPECT_EQ(actual.traversable(x, y), expected.traversable(x, y));
      for (const auto & offset : offsets) {
        const double qx = x + offset.first;
        const double qy = y + offset.second;
        expectSameDistance(actual.distance(qx, qy), expected.distance(qx, qy));
        for (const double heading : headings) {
          EXPECT_NEAR(
            actual.heading(qx, qy, heading), expected.heading(qx, qy, heading), 1e-12);
        }
      }
    }
  }
  // Explicit rounding and map boundaries, including non-finite inputs.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<std::pair<double, double>> boundaries{
    {-0.5, 0.0}, {std::nextafter(-0.5, -infinity), 0.0},
    {width - 0.5, 0.0}, {std::nextafter(width - 0.5, 0.0), 0.0},
    {0.0, -0.5}, {0.0, height - 0.5}, {0.5, 0.5},
    {nan, 0.0}, {0.0, nan}, {infinity, 0.0}, {0.0, -infinity}};
  for (const auto & point : boundaries) {
    expectSameDistance(
      actual.distance(point.first, point.second), expected.distance(point.first, point.second));
    EXPECT_NEAR(
      actual.heading(point.first, point.second, -0.72),
      expected.heading(point.first, point.second, -0.72), 1e-12);
  }
}

void expectSameTarget(
  ObstacleGuidance & actual, FrozenObstacleGuidance & expected,
  int width, int height, int x, int y, bool subcell = true)
{
  SCOPED_TRACE(::testing::Message() << "target (" << x << ", " << y << ")");
  EXPECT_EQ(actual.setTarget(x, y), expected.setTarget(x, y));
  expectSameQueries(actual, expected, width, height, subcell);
}
}  // namespace


TEST(ObstacleGuidance, RewardsGoingAroundAnObstacleInsteadOfWaitingInFront)
{
  constexpr int size = 100;
  std::vector<unsigned char> costs(size * size, 0);
  // A metre-wide cube in a 5 m map, seen after planning a straight path.
  for (int y = 40; y <= 60; ++y) {
    for (int x = 45; x <= 65; ++x) {
      costs[y * size + x] = 254;
    }
  }
  ObstacleGuidance guidance;
  guidance.update(size, size, 0.05, costs.data(), 0.25, 2.0);
  ASSERT_TRUE(guidance.setTarget(80, 50));
  const double waiting = guidance.distance(37, 50);
  ASSERT_TRUE(std::isfinite(waiting));
  // A lateral move increases straight-line distance to the goal, but reduces
  // distance through free space. This is the previous stopping local minimum.
  EXPECT_LT(guidance.distance(37, 37), waiting);
  EXPECT_LT(guidance.distance(37, 63), waiting);
  EXPECT_TRUE(std::isinf(guidance.distance(50, 50)));
  EXPECT_FALSE(guidance.setTarget(50, 50));
}

TEST(ObstacleGuidance, RejectsUnknownCellsAndRoutesThroughNarrowGaps)
{
  std::vector<unsigned char> costs(40 * 40, 0);
  for (int y = 0; y < 40; ++y) {
    if (y < 18 || y > 22) {
      costs[y * 40 + 20] = 254;
    }
  }
  ObstacleGuidance guidance;
  guidance.update(40, 40, 0.05, costs.data(), 0.20, 0.0);
  ASSERT_TRUE(guidance.setTarget(30, 20));
  EXPECT_TRUE(std::isinf(guidance.distance(10, 20)));
  costs[20 * 40 + 30] = 255;
  guidance.update(40, 40, 0.05, costs.data(), 0.20, 0.0);
  EXPECT_FALSE(guidance.setTarget(30, 20));
}

TEST(ObstacleGuidance, DoesNotCutDiagonallyBetweenTouchingObstacles)
{
  std::vector<unsigned char> costs(5 * 5, 254);
  costs[1 * 5 + 1] = 0;
  costs[2 * 5 + 2] = 0;
  ObstacleGuidance guidance;
  guidance.update(5, 5, 0.1, costs.data(), 0.0, 0.0);
  ASSERT_TRUE(guidance.setTarget(2, 2));
  EXPECT_TRUE(std::isinf(guidance.distance(1, 1)));
  EXPECT_TRUE(std::isinf(guidance.distance(1.4, 1.4)));
}

TEST(ObstacleGuidance, ClearsOldObstaclesAndSmoothlyRewardsSubcellProgress)
{
  std::vector<unsigned char> costs(40 * 40, 0);
  for (int y = 0; y < 40; ++y) {
    costs[y * 40 + 20] = 254;
  }
  ObstacleGuidance guidance;
  guidance.update(40, 40, 0.05, costs.data(), 0.2, 0.0);
  ASSERT_TRUE(guidance.setTarget(30, 20));
  EXPECT_TRUE(std::isinf(guidance.distance(10, 20)));
  std::fill(costs.begin(), costs.end(), 0);
  guidance.update(40, 40, 0.05, costs.data(), 0.2, 0.0);
  ASSERT_TRUE(guidance.setTarget(30, 20));
  EXPECT_NEAR(guidance.distance(10, 20), 1.0, 1e-12);
  EXPECT_NEAR(guidance.distance(10.1, 20), 0.995, 1e-12);
  EXPECT_TRUE(std::isinf(guidance.distance(-1, 20)));
}

TEST(ObstacleGuidanceReference, ExhaustiveSmallMapsPreserveRoutesAndTieBreaking)
{
  // All 512 obstacle arrangements and all nine targets include disconnected
  // regions, diagonal corner cutting, blocked targets and equal-cost routes.
  ObstacleGuidance actual;
  FrozenObstacleGuidance expected;
  std::vector<unsigned char> costs(9);
  for (unsigned int mask = 0; mask < (1u << 9); ++mask) {
    SCOPED_TRACE(::testing::Message() << "obstacle mask " << mask);
    for (unsigned int cell = 0; cell < costs.size(); ++cell) {
      costs[cell] = (mask & (1u << cell)) ? 254 : 0;
    }
    actual.update(3, 3, 0.1, costs.data(), 0.0, 0.0);
    expected.update(3, 3, 0.1, costs.data(), 0.0, 0.0);
    for (int target = 0; target < 9; ++target) {
      expectSameTarget(actual, expected, 3, 3, target % 3, target / 3, false);
    }
  }
}

TEST(ObstacleGuidanceReference, DeterministicWeightedMapsPreserveSubcellValuesAndHeadings)
{
  std::mt19937 random(0x79179u);
  ObstacleGuidance actual;
  FrozenObstacleGuidance expected;
  const unsigned char values[]{0, 0, 0, 0, 1, 57, 128, 251, 252, 253, 254, 255};
  for (int map = 0; map < 12; ++map) {
    const int width = 12 + random() % 7;
    const int height = 10 + random() % 5;
    const double resolution = map % 2 ? 0.02 : 0.05;
    const double clearance = (map % 4) * resolution * 0.45;
    const double weight = (map % 5) * 0.73;
    SCOPED_TRACE(::testing::Message() << "weighted map " << map);
    std::vector<unsigned char> costs(width * height, 0);
    for (auto & cost : costs) {
      // Include every cost threshold without making each inflated map solid.
      if (random() % 4 == 0) {
        cost = values[random() % (sizeof(values) / sizeof(values[0]))];
      }
    }
    actual.update(width, height, resolution, costs.data(), clearance, weight);
    expected.update(width, height, resolution, costs.data(), clearance, weight);
    expectSameQueries(actual, expected, width, height);
    expectSameTarget(actual, expected, width, height, -1, height / 2);
    expectSameTarget(actual, expected, width, height, width / 2, height / 2);
    // Choose a free target as well, so the random map cannot test only infinity.
    for (int cell = 0; cell < width * height; ++cell) {
      if (expected.traversable(cell % width, cell / width)) {
        expectSameTarget(actual, expected, width, height, cell % width, cell / width);
        break;
      }
    }
  }
}

TEST(ObstacleGuidanceReference, PreservesInflationAndMapBoundaryThresholds)
{
  constexpr int width = 15;
  constexpr int height = 13;
  constexpr double resolution = 0.05;
  std::vector<unsigned char> costs(width * height, 0);
  costs[6 * width + 7] = 254;
  std::vector<double> clearances{0.0};
  // Exercise the exact disk inclusion boundary and each adjacent double.
  for (const auto & offset : std::vector<std::pair<int, int>>{
      {1, 0}, {1, 1}, {2, 0}, {2, 1}, {2, 2}, {3, 1}})
  {
    const double boundary = (std::hypot(offset.first, offset.second) -
      std::sqrt(0.5)) * resolution;
    clearances.push_back(std::nextafter(boundary, 0.0));
    clearances.push_back(boundary);
    clearances.push_back(std::nextafter(boundary, infinity));
  }
  // The independent clearance from map edges uses strict '<', not '<='.
  for (const double cells : {0.5, 1.5, 2.5}) {
    const double boundary = cells * resolution;
    clearances.push_back(std::nextafter(boundary, 0.0));
    clearances.push_back(boundary);
    clearances.push_back(std::nextafter(boundary, infinity));
  }
  ObstacleGuidance actual;
  FrozenObstacleGuidance expected;
  for (const double clearance : clearances) {
    SCOPED_TRACE(::testing::Message() << "clearance " << clearance);
    actual.update(width, height, resolution, costs.data(), clearance, 2.0);
    expected.update(width, height, resolution, costs.data(), clearance, 2.0);
    expectSameTarget(actual, expected, width, height, 3, 3);
    expectSameTarget(actual, expected, width, height, 7, 6, false);
  }
}

TEST(ObstacleGuidanceReference, DistinguishesInscribedLethalAndUnknownCells)
{
  constexpr int size = 11;
  ObstacleGuidance actual;
  FrozenObstacleGuidance expected;
  std::vector<unsigned char> costs(size * size, 0);
  for (const unsigned char cost : {252, 253, 254, 255}) {
    SCOPED_TRACE(::testing::Message() << "center cost " << static_cast<int>(cost));
    costs[5 * size + 5] = cost;
    actual.update(size, size, 0.05, costs.data(), 0.05, 1.8);
    expected.update(size, size, 0.05, costs.data(), 0.05, 1.8);
    EXPECT_EQ(actual.traversable(5, 5), cost < 253);
    EXPECT_EQ(actual.traversable(6, 5), cost < 254);
    expectSameTarget(actual, expected, size, size, 5, 5);
    expectSameTarget(actual, expected, size, size, 2, 2);
  }
}

TEST(ObstacleGuidanceReference, ReusedStorageClearsOldFieldsAndAdaptsToChangedGeometry)
{
  struct Geometry
  {
    int width;
    int height;
    double resolution;
    double clearance;
    double weight;
  };
  const std::vector<Geometry> grids{
    {19, 13, 0.05, 0.07, 3.0}, {19, 13, 0.05, 0.07, 3.0},
    {8, 17, 0.02, 0.012, 0.5}, {17, 8, 0.1, 0.13, 1.7},
    {19, 13, 0.05, 0.07, 0.0}, {19, 13, 0.05, 0.0, 2.0}};
  ObstacleGuidance actual;
  FrozenObstacleGuidance expected;
  for (std::size_t update = 0; update < grids.size(); ++update) {
    const auto & grid = grids[update];
    SCOPED_TRACE(::testing::Message() << "grid update " << update);
    std::vector<unsigned char> costs(grid.width * grid.height, 0);
    if (update % 2 == 0) {
      for (int y = 0; y < grid.height; ++y) {
        costs[y * grid.width + grid.width / 2] = 254;
      }
      costs[(grid.height / 2) * grid.width + 2] = 253;
    }
    actual.update(
      grid.width, grid.height, grid.resolution, costs.data(), grid.clearance, grid.weight);
    expected.update(
      grid.width, grid.height, grid.resolution, costs.data(), grid.clearance, grid.weight);
    // update() must invalidate the old target field before a new target is set.
    expectSameQueries(actual, expected, grid.width, grid.height);
    expectSameTarget(
      actual, expected, grid.width, grid.height, grid.width - 3, grid.height / 2);
    expectSameTarget(actual, expected, grid.width, grid.height, 2, grid.height / 2);
    expectSameTarget(actual, expected, grid.width, grid.height, grid.width, 0, false);
  }
}
