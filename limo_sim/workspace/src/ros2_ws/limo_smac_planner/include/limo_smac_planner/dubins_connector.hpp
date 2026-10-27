#ifndef LIMO_SMAC_PLANNER__DUBINS_CONNECTOR_HPP_
#define LIMO_SMAC_PLANNER__DUBINS_CONNECTOR_HPP_
#include <stdexcept>
#include "nav_msgs/msg/path.hpp"
namespace limo_smac_planner
{
class DubinsLengthError : public std::runtime_error
{
public:
  DubinsLengthError()
  : std::runtime_error("Initial Dubins connector exceeds start_connector_max_length") {}
};

// measured_length is populated even when the maximum-length check fails.
nav_msgs::msg::Path dubinsConnector(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & end,
  double turning_radius, double sample_step, double max_length,
  double * measured_length = nullptr);
}
#endif
