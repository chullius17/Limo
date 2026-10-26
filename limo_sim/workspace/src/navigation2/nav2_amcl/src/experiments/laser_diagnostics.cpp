// Copyright (c) 2026 Giulio Cataldo
// Licensed under the Apache License, Version 2.0.
#include "nav2_amcl/amcl_node.hpp"

#include <iomanip>
#include <sstream>
#include <string>

namespace nav2_amcl
{
void AmclNode::publishLaserDiagnostics(
  const sensor_msgs::msg::LaserScan & scan, const LaserData * data,
  const std::string & reason)
{
  if (!laser_diagnostics_pub_ || !laser_diagnostics_pub_->is_activated() ||
    laser_diagnostics_pub_->get_subscription_count() == 0)
  {
    return;
  }
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = scan.header.stamp;
  message.header.frame_id = global_frame_id_;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "amcl/laser_update";
  status.hardware_id = scan.header.frame_id;
  status.message = reason;
  status.level = reason == "accepted" || reason == "disabled" ? status.OK : status.WARN;
  const auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue entry;
      entry.key = key;
      entry.value = value;
      status.values.push_back(entry);
    };
  std::ostringstream weight;
  weight << std::setprecision(17) << laser_weight_factor_;
  add("laser_weight_factor", weight.str());
  add("input_points", std::to_string(scan.ranges.size()));
  add("candidate_points", std::to_string(data ? data->candidate_points : 0));
  add("used_points", std::to_string(data ? data->used_points : 0));
  add("particles", std::to_string(data ? data->particles : 0));
  add("evaluations", std::to_string(data ? data->evaluations : 0));
  message.status.push_back(status);
  laser_diagnostics_pub_->publish(message);
}
}  // namespace nav2_amcl
