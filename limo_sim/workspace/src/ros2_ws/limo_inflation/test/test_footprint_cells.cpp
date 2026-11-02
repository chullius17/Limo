#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "limo_inflation/footprint_cells.hpp"

namespace
{
using Cells = std::vector<nav2_costmap_2d::MapLocation>;
using Footprint = std::vector<geometry_msgs::msg::Point>;

geometry_msgs::msg::Pose2D pose(double x, double y, double theta)
{
  geometry_msgs::msg::Pose2D result;
  result.x = x;
  result.y = y;
  result.theta = theta;
  return result;
}

Footprint footprint(const std::vector<std::pair<double, double>> & vertices)
{
  Footprint result;
  for (const auto & vertex : vertices) {
    geometry_msgs::msg::Point point;
    point.x = vertex.first;
    point.y = vertex.second;
    result.push_back(point);
  }
  return result;
}

// Baseline before caller-owned scratch storage was added. Keep rasterization,
// cell ordering, failure results and the integral-image shortcut identical.
bool legacyFootprintCells(
  nav2_costmap_2d::Costmap2D & map, const geometry_msgs::msg::Pose2D & position,
  const Footprint & shape, Cells & cells,
  const std::vector<std::uint32_t> * blocked_prefix = nullptr)
{
  cells.clear();
  if (shape.size() < 3 || !std::isfinite(position.x) || !std::isfinite(position.y) ||
    !std::isfinite(position.theta)) {return false;}
  const double c = std::cos(position.theta), s = std::sin(position.theta);
  Cells polygon;
  polygon.reserve(shape.size());
  unsigned int min_x = map.getSizeInCellsX(), min_y = map.getSizeInCellsY();
  unsigned int max_x = 0, max_y = 0;
  for (const auto & vertex : shape) {
    nav2_costmap_2d::MapLocation cell;
    if (!map.worldToMap(position.x + c * vertex.x - s * vertex.y,
      position.y + s * vertex.x + c * vertex.y, cell.x, cell.y)) {return false;}
    polygon.push_back(cell);
    min_x = std::min(min_x, cell.x); min_y = std::min(min_y, cell.y);
    max_x = std::max(max_x, cell.x); max_y = std::max(max_y, cell.y);
  }
  if (blocked_prefix) {
    const std::size_t stride = map.getSizeInCellsX() + 1;
    const auto & p = *blocked_prefix;
    if (p[(max_y + 1) * stride + max_x + 1] + p[min_y * stride + min_x] ==
      p[min_y * stride + max_x + 1] + p[(max_y + 1) * stride + min_x]) {return true;}
  }
  map.convexFillCells(polygon, cells);
  return !cells.empty();
}

std::vector<std::uint32_t> prefix(
  nav2_costmap_2d::Costmap2D & map, const Cells & blocked)
{
  const std::size_t width = map.getSizeInCellsX();
  const std::size_t height = map.getSizeInCellsY();
  const std::size_t stride = width + 1;
  std::vector<std::uint32_t> result(stride * (height + 1), 0);
  for (const auto & cell : blocked) {
    result[(cell.y + 1) * stride + cell.x + 1] = 1;
  }
  for (std::size_t y = 1; y <= height; ++y) {
    for (std::size_t x = 1; x <= width; ++x) {
      result[y * stride + x] += result[(y - 1) * stride + x] +
        result[y * stride + x - 1] - result[(y - 1) * stride + x - 1];
    }
  }
  return result;
}

void expectSameCells(const Cells & expected, const Cells & actual)
{
  ASSERT_EQ(expected.size(), actual.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(expected[i].x, actual[i].x) << "cell " << i;
    EXPECT_EQ(expected[i].y, actual[i].y) << "cell " << i;
  }
}

const Footprint robot = footprint({
    {-0.161, -0.110}, {0.161, -0.110}, {0.161, 0.110}, {-0.161, 0.110}});
}  // namespace

TEST(FootprintCells, ReusedAndCompatibleOverloadsExactlyMatchLegacyRasterization)
{
  nav2_costmap_2d::Costmap2D map(80, 72, 0.05, -2.0, -1.8, 0);
  const std::vector<Footprint> shapes{
    robot,
    footprint({{-0.31, -0.14}, {0.27, 0.0}, {-0.31, 0.19}}),
    footprint({{-0.34, -0.10}, {-0.14, -0.24}, {0.31, -0.13},
        {0.35, 0.10}, {0.14, 0.26}, {-0.30, 0.17}}),
    footprint({{-0.001, -0.001}, {0.001, -0.001}, {0.0, 0.001}})};
  const auto empty_prefix = prefix(map, {});
  const auto sparse_prefix = prefix(map, {{0, 0}, {40, 36}, {79, 71}, {71, 7}});
  const std::vector<const std::vector<std::uint32_t> *> prefixes{
    nullptr, &empty_prefix, &sparse_prefix};
  Cells expected, actual, compatible, polygon;
  for (const auto & shape : shapes) {
    for (const double x : {-1.99, -1.70, -0.03, 0.0, 1.70, 1.99}) {
      for (const double y : {-1.79, -1.50, 0.0, 1.50, 1.79}) {
        for (const double theta : {0.0, 0.4, 1.5707963267948966, -2.3}) {
          for (const auto * blocked : prefixes) {
            const auto position = pose(x, y, theta);
            const bool valid = legacyFootprintCells(map, position, shape, expected, blocked);
            EXPECT_EQ(valid, limo_inflation::footprintCells(
                map, position, shape, actual, polygon, blocked));
            EXPECT_EQ(valid, limo_inflation::footprintCells(
                map, position, shape, compatible, blocked));
            expectSameCells(expected, actual);
            expectSameCells(expected, compatible);
          }
        }
      }
    }
  }
}

TEST(FootprintCells, PrefixShortcutAndFailuresDoNotLeaveCellsFromEarlierPose)
{
  nav2_costmap_2d::Costmap2D map(80, 72, 0.05, -2.0, -1.8, 0);
  Cells cells, polygon, expected;
  const auto position = pose(0.0, 0.0, 0.3);
  const auto free_prefix = prefix(map, {});
  const auto occupied_prefix = prefix(map, {{40, 36}});
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<geometry_msgs::msg::Pose2D> invalid_poses{
    pose(nan, 0.0, 0.0), pose(0.0, nan, 0.0), pose(0.0, 0.0, nan),
    pose(inf, 0.0, 0.0), pose(0.0, -inf, 0.0), pose(0.0, 0.0, inf),
    pose(2.0, 0.0, 0.0), pose(0.0, -1.8, 0.0)};
  for (const auto & invalid : invalid_poses) {
    ASSERT_TRUE(limo_inflation::footprintCells(map, position, robot, cells, polygon));
    ASSERT_FALSE(cells.empty());
    EXPECT_FALSE(limo_inflation::footprintCells(map, invalid, robot, cells, polygon));
    EXPECT_TRUE(cells.empty());

    ASSERT_TRUE(limo_inflation::footprintCells(
        map, position, robot, cells, polygon, &free_prefix));
    EXPECT_TRUE(cells.empty());

    ASSERT_TRUE(legacyFootprintCells(map, position, robot, expected, &occupied_prefix));
    ASSERT_TRUE(limo_inflation::footprintCells(
        map, position, robot, cells, polygon, &occupied_prefix));
    ASSERT_FALSE(cells.empty());
    expectSameCells(expected, cells);
  }
  for (std::size_t count = 0; count < 3; ++count) {
    const Footprint invalid_shape(robot.begin(), robot.begin() + count);
    EXPECT_FALSE(limo_inflation::footprintCells(
        map, position, invalid_shape, cells, polygon));
    EXPECT_TRUE(cells.empty());
    EXPECT_TRUE(polygon.empty());
    ASSERT_TRUE(limo_inflation::footprintCells(map, position, robot, cells, polygon));
    ASSERT_FALSE(cells.empty());
  }
}

TEST(FootprintCells, WarmCallerBuffersRetainStorageAcrossChecksAndEarlyReturns)
{
  nav2_costmap_2d::Costmap2D map(80, 72, 0.05, -2.0, -1.8, 0);
  Cells cells, polygon;
  cells.reserve(80 * 72);
  polygon.reserve(robot.size());
  ASSERT_TRUE(limo_inflation::footprintCells(map, pose(0.0, 0.0, 0.0), robot, cells, polygon));
  const auto cells_capacity = cells.capacity();
  const auto polygon_capacity = polygon.capacity();
  const auto * cells_data = cells.data();
  const auto * polygon_data = polygon.data();
  const auto free_prefix = prefix(map, {});
  for (int i = 0; i < 200; ++i) {
    const auto position = pose(0.5 * std::sin(i * 0.1), 0.5 * std::cos(i * 0.1), i * 0.1);
    ASSERT_TRUE(limo_inflation::footprintCells(map, position, robot, cells, polygon));
    ASSERT_TRUE(limo_inflation::footprintCells(
        map, position, robot, cells, polygon, &free_prefix));
    EXPECT_FALSE(limo_inflation::footprintCells(
        map, pose(5.0, 5.0, 0.0), robot, cells, polygon));
    EXPECT_EQ(cells_capacity, cells.capacity());
    EXPECT_EQ(polygon_capacity, polygon.capacity());
    EXPECT_EQ(cells_data, cells.data());
    EXPECT_EQ(polygon_data, polygon.data());
  }
}
