#ifndef LIMO_DWB_CRITICS__START_CONNECTOR_HPP_
#define LIMO_DWB_CRITICS__START_CONNECTOR_HPP_
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include "limo_interfaces/msg/start_connector.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"

namespace limo_dwb_critics
{
// Plan-bound, one-way handoff: proximity to the virtual start alone cannot
// skip a Dubins loop or reactivate the exception after its completion.
class StartConnectorGate
{
public:
  static bool samePath(const nav_msgs::msg::Path & a, const nav_msgs::msg::Path & b)
  {
    if (a.header.frame_id != b.header.frame_id || a.header.stamp != b.header.stamp ||
      a.poses.size() != b.poses.size()) {
      return false;
    }
    for (std::size_t i = 0; i < a.poses.size(); ++i) {
      const auto & p = a.poses[i].pose;
      const auto & q = b.poses[i].pose;
      if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
        !std::isfinite(q.position.x) || !std::isfinite(q.position.y) ||
        std::hypot(p.position.x - q.position.x, p.position.y - q.position.y) > 1e-8 ||
        angleError(tf2::getYaw(p.orientation), tf2::getYaw(q.orientation)) > 1e-8)
      {
        return false;
      }
    }
    return true;
  }
  void setPlan(const nav_msgs::msg::Path & path)
  {
    if (!samePath(plan_, path)) {resetProgress();}
    plan_ = path;
  }
  void setMetadata(const limo_interfaces::msg::StartConnector & metadata)
  {
    if (metadata_.path.header.stamp != metadata.path.header.stamp ||
      metadata_.end_index != metadata.end_index || !samePath(metadata_.path, metadata.path))
    {
      resetProgress();
    }
    metadata_ = metadata;
  }
  nav_msgs::msg::Path prefix() const
  {
    auto path = plan_;
    path.poses.resize(metadata_.end_index + 1);
    return path;
  }
  nav_msgs::msg::Path normalPlan() const
  {
    auto path = plan_;
    if (finished_ && metadata_.end_index < path.poses.size() &&
      samePath(plan_, metadata_.path))
    {
      path.poses.erase(path.poses.begin(), path.poses.begin() + metadata_.end_index);
    }
    return path;
  }
  bool update(const geometry_msgs::msg::Pose & pose, double position_tolerance, double yaw_tolerance)
  {
    if (finished_ || plan_.poses.empty() || metadata_.end_index == 0 ||
      metadata_.end_index >= plan_.poses.size() || !samePath(plan_, metadata_.path))
    {
      return false;
    }
    if (lengths_.empty()) {
      lengths_.push_back(0.0);
      for (std::size_t i = 1; i <= metadata_.end_index; ++i) {
        lengths_.push_back(lengths_.back() + distance(
          plan_.poses[i - 1].pose.position, plan_.poses[i].pose.position));
      }
      previous_ = plan_.poses.front().pose.position;
    }
    const double available = lengths_[index_] + distance(previous_, pose.position) + 0.10;
    previous_ = pose.position;
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t i = index_; i < lengths_.size() && lengths_[i] <= available; ++i) {
      const double d = distance(pose.position, plan_.poses[i].pose.position);
      if (d < nearest) {nearest = d; index_ = i;}
    }
    const auto & end = plan_.poses[metadata_.end_index].pose;
    if (lengths_.back() - lengths_[index_] <= position_tolerance &&
      distance(pose.position, end.position) <= position_tolerance &&
      angleError(tf2::getYaw(pose.orientation), tf2::getYaw(end.orientation)) <= yaw_tolerance)
    {
      finished_ = true;
      return false;
    }
    return true;
  }
private:
  static double distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
  {return std::hypot(a.x - b.x, a.y - b.y);}
  static double angleError(double a, double b)
  {return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));}
  void resetProgress() {finished_ = false; index_ = 0; lengths_.clear();}
  nav_msgs::msg::Path plan_;
  limo_interfaces::msg::StartConnector metadata_;
  std::vector<double> lengths_;
  geometry_msgs::msg::Point previous_;
  std::size_t index_{0};
  bool finished_{false};
};
}
#endif
