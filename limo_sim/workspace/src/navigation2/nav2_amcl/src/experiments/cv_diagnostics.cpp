// Copyright (c) 2026 Giulio Cataldo
// Licensed under the Apache License, Version 2.0.
#include "nav2_amcl/amcl_node.hpp"

#include <iomanip>
#include <sstream>
#include <string>

namespace nav2_amcl
{
void AmclNode::publishCvDiagnostics(
  const builtin_interfaces::msg::Time & stamp, const CvFusionDiagnostics & data)
{
  if (!cv_diagnostics_pub_ || !cv_diagnostics_pub_->is_activated() ||
    cv_diagnostics_pub_->get_subscription_count() == 0)
  {
    return;
  }
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = stamp;
  message.header.frame_id = global_frame_id_;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "amcl/cv_fusion";
  status.hardware_id = "semantic_cloud";
  status.level = data.accepted ? status.OK : status.WARN;
  status.message = data.reason;
  const auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue entry;
      entry.key = key;
      entry.value = value;
      status.values.push_back(entry);
    };
  const auto precise = [](double value) {
      std::ostringstream stream;
      stream << std::setprecision(17) << value;
      return stream.str();
    };
  add("accepted", data.accepted ? "true" : "false");
  add("reused", data.reused ? "true" : "false");
  add("input_points", std::to_string(data.input_points));
  add("candidate_points", std::to_string(data.candidate_points));
  add("used_points", std::to_string(data.accepted ? data.candidate_points : 0));
  add("used_non_road_points", std::to_string(data.accepted ? data.non_road_points : 0));
  add(
    "used_road_points", std::to_string(
      data.accepted ? data.candidate_points - data.non_road_points : 0));
  add("particles", std::to_string(data.particles));
  add("evaluations", std::to_string(data.evaluations));
  add("cloud_stamp", precise(data.cloud_stamp));
  add("sync_error", precise(data.sync_error));
  message.status.push_back(status);
  cv_diagnostics_pub_->publish(message);
}
}  // namespace nav2_amcl
