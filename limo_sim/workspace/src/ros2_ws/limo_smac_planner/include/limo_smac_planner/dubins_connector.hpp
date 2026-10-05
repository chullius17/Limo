#ifndef LIMO_SMAC_PLANNER__DUBINS_CONNECTOR_HPP_
#define LIMO_SMAC_PLANNER__DUBINS_CONNECTOR_HPP_
#include "nav_msgs/msg/path.hpp"
namespace limo_smac_planner
{
nav_msgs::msg::Path dubinsConnector(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & end,
  double turning_radius, double sample_step, double max_length);
}
#endif
