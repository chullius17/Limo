#include "limo_map/cloud.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace limo_map
{
namespace
{
float read_float(const uint8_t * data, bool bigendian)
{
  const uint16_t one = 1;
  const bool host_big = reinterpret_cast<const uint8_t *>(&one)[0] == 0;
  uint8_t bytes[4];
  for (size_t i = 0; i < 4; ++i) {bytes[i] = data[host_big == bigendian ? i : 3 - i];}
  float value;
  std::memcpy(&value, bytes, 4);
  return value;
}
}  // namespace

std::vector<Point> read_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud, const Grid & grid,
  Pose cloud_to_map, Pose submap_to_map)
{
  std::array<uint32_t, 4> offsets{};
  const std::array<std::string, 4> names{{"x", "y", "z", "class_id"}};
  for (size_t i = 0; i < names.size(); ++i) {
    size_t matches = 0;
    for (const auto & field : cloud.fields) {
      if (field.name != names[i]) {continue;}
      ++matches;
      const uint8_t expected = i == 3 ? 2 : 7;  // UINT8 / FLOAT32
      const uint64_t size = i == 3 ? 1 : 4;
      if (field.datatype != expected || field.count != 1 ||
        static_cast<uint64_t>(field.offset) + size > cloud.point_step)
      {
        throw std::invalid_argument("Invalid PointCloud2 field: " + names[i]);
      }
      offsets[i] = field.offset;
    }
    if (matches != 1) {throw std::invalid_argument("PointCloud2 requires one field: " + names[i]);}
  }
  if (cloud.row_step < static_cast<uint64_t>(cloud.width) * cloud.point_step ||
    cloud.data.size() < static_cast<uint64_t>(cloud.height) * cloud.row_step)
  {
    throw std::invalid_argument("Invalid PointCloud2 row_step or truncated buffer");
  }
  std::vector<Point> points;
  points.reserve(static_cast<size_t>(cloud.width) * cloud.height);
  const auto translation = transform({cloud_to_map.x, cloud_to_map.y, 0}, submap_to_map, true);
  const double c = std::cos(cloud_to_map.yaw - submap_to_map.yaw);
  const double s = std::sin(cloud_to_map.yaw - submap_to_map.yaw);
  for (uint32_t row = 0; row < cloud.height; ++row) {
    for (uint32_t col = 0; col < cloud.width; ++col) {
      const auto * data = cloud.data.data() + static_cast<size_t>(row) * cloud.row_step +
        static_cast<size_t>(col) * cloud.point_step;
      const auto id = data[offsets[3]];
      if (grid.group(id) < 0) {continue;}
      const double x = read_float(data + offsets[0], cloud.is_bigendian);
      const double y = read_float(data + offsets[1], cloud.is_bigendian);
      const double z = read_float(data + offsets[2], cloud.is_bigendian);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {continue;}
      points.push_back({c * x - s * y + translation.x, s * x + c * y + translation.y, id});
    }
  }
  return points;
}
}  // namespace limo_map
