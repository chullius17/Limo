#ifndef LIMO_MAP__CLOUD_HPP_
#define LIMO_MAP__CLOUD_HPP_
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "limo_map/grid.hpp"

namespace limo_map
{
// Honor arbitrary field offsets, point/row padding and wire endianness.
// Nonfinite XYZ and unconfigured IDs supply no evidence. Apply a planar TF pose.
std::vector<Point> read_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud, const Grid & grid,
  Pose cloud_to_map, Pose submap_to_map);
}  // namespace limo_map
#endif  // LIMO_MAP__CLOUD_HPP_
