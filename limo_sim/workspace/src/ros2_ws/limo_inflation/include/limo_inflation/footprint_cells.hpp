#ifndef LIMO_INFLATION__FOOTPRINT_CELLS_HPP_
#define LIMO_INFLATION__FOOTPRINT_CELLS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"

namespace limo_inflation
{

// Nav2's convex fill includes the boundary and interior of the robot polygon.
// An optional integral image skips rasterization if its bounding box is empty.
inline bool footprintCells(
  nav2_costmap_2d::Costmap2D & map, const geometry_msgs::msg::Pose2D & pose,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  std::vector<nav2_costmap_2d::MapLocation> & cells,
  const std::vector<std::uint32_t> * blocked_prefix = nullptr)
{
  cells.clear();
  if (footprint.size() < 3 || !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
    !std::isfinite(pose.theta)) {return false;}
  const double c = std::cos(pose.theta), s = std::sin(pose.theta);
  std::vector<nav2_costmap_2d::MapLocation> polygon;
  polygon.reserve(footprint.size());
  unsigned int min_x = map.getSizeInCellsX(), min_y = map.getSizeInCellsY();
  unsigned int max_x = 0, max_y = 0;
  for (const auto & vertex : footprint) {
    nav2_costmap_2d::MapLocation cell;
    if (!map.worldToMap(pose.x + c * vertex.x - s * vertex.y,
      pose.y + s * vertex.x + c * vertex.y, cell.x, cell.y)) {return false;}
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

}  // namespace limo_inflation
#endif  // LIMO_INFLATION__FOOTPRINT_CELLS_HPP_
