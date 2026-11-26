// Copyright (c) 2026 Giulio Cataldo
// Licensed under the Apache License, Version 2.0.
#ifndef LIMO_AMCL__SENSORS__CV__CV_POINT_CLOUD_HPP_
#define LIMO_AMCL__SENSORS__CV__CV_POINT_CLOUD_HPP_

#include <string>
#include <array>
#include <cstdint>
#include <vector>

#include "limo_amcl/sensors/cv/cv_likelihood_model.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"

namespace limo_amcl
{

/// Configurable input semantics. -1 ignores a class, 0 observes road/free space,
/// 1 observes an obstacle. Lists must be disjoint and contain unique uint8 IDs.
struct CvClassRoles
{
  CvClassRoles(
    const std::vector<int64_t> & road = {1, 5},
    const std::vector<int64_t> & obstacles = {2, 4, 6});
  std::array<int8_t, 256> polarity;
};

/// Select the configured road and obstacle classes; ignore all other IDs.
/// Transform to the robot frame at laser time, then merge into XY voxels per
/// polarity. Each centroid votes once, with occupancy 1 (obstacle) or 0 (road).
/// Supports organized clouds, padded rows, arbitrary field offsets and endian.
/// Invalid layouts return false; empty or wholly invalid observations return
/// true with an empty template (no evidence).
bool voxelizeCvCloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const tf2::Transform & cloud_to_base, double voxel_size,
  std::vector<CvTemplateCell2D> & cells, std::string & error,
  const CvClassRoles & roles = CvClassRoles());

}  // namespace limo_amcl
#endif  // LIMO_AMCL__SENSORS__CV__CV_POINT_CLOUD_HPP_
