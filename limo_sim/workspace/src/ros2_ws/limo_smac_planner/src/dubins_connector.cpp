#include "limo_smac_planner/dubins_connector.hpp"
#include <cmath>
#include <stdexcept>
#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"

namespace limo_smac_planner
{
nav_msgs::msg::Path dubinsConnector(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & end,
  double turning_radius, double sample_step, double max_length)
{
  if (start.header.frame_id != end.header.frame_id || turning_radius <= 0.0 ||
    sample_step <= 0.0 || max_length <= 0.0)
  {
    throw std::invalid_argument("Invalid Dubins connector frame or dimensions");
  }
  auto space = std::make_shared<ompl::base::DubinsStateSpace>(turning_radius, false);
  ompl::base::ScopedState<> from(space), to(space), point(space);
  from[0] = start.pose.position.x;
  from[1] = start.pose.position.y;
  from[2] = tf2::getYaw(start.pose.orientation);
  to[0] = end.pose.position.x;
  to[1] = end.pose.position.y;
  to[2] = tf2::getYaw(end.pose.orientation);
  const double length = space->distance(from(), to());
  if (!std::isfinite(length) || length > max_length) {
    throw std::runtime_error("Initial Dubins connector exceeds start_connector_max_length");
  }
  nav_msgs::msg::Path path;
  path.header = start.header;
  const auto steps = std::max(1, static_cast<int>(std::ceil(length / sample_step)));
  for (int i = 0; i <= steps; ++i) {
    space->interpolate(from(), to(), static_cast<double>(i) / steps, point());
    auto pose = start;
    pose.pose.position.x = point[0];
    pose.pose.position.y = point[1];
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, point[2]);
    pose.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(pose);
  }
  path.poses.front() = start;
  path.poses.back() = end;
  return path;
}
}
