#ifndef LIMO_INFLATION__PATH_POSE_ASSOCIATION_HPP_
#define LIMO_INFLATION__PATH_POSE_ASSOCIATION_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"

namespace limo_inflation
{

using PathPosePoint = std::array<double, 4>;  // x, y, arc length, chassis yaw.

inline double wrappedAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// Only spatially close alternatives may compete on orientation. A distant,
// aligned branch must never beat a nearby reference solely because of yaw.
inline double poseAssociationCost(double distance, double angle)
{
  constexpr double heading_scale = 0.25;  // meters per radian
  return distance * distance + std::pow(heading_scale * wrappedAngle(angle), 2);
}

inline std::size_t closestPathPoseIndex(
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  const geometry_msgs::msg::Pose2D & pose, std::size_t begin, std::size_t end)
{
  double nearest = std::numeric_limits<double>::infinity();
  for (std::size_t i = begin; i < end; ++i) {
    nearest = std::min(nearest, std::hypot(pose.x - poses[i].x, pose.y - poses[i].y));
  }
  std::size_t index = begin;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = begin; i < end; ++i) {
    const double distance = std::hypot(pose.x - poses[i].x, pose.y - poses[i].y);
    if (distance > nearest + 0.10) {continue;}
    const double cost = poseAssociationCost(distance, pose.theta - poses[i].theta);
    if (cost < best - 1e-9) {best = cost; index = i;}
  }
  return index;
}

struct PathPoseProjection
{
  double progress{0.0};
  double heading{0.0};
  double distance{0.0};
  std::size_t segment{0};
};

inline std::vector<PathPosePoint> pathPosePoints(
  const std::vector<geometry_msgs::msg::Pose2D> & poses)
{
  std::vector<PathPosePoint> points;
  double arc = 0.0;
  for (const auto & pose : poses) {
    if (!points.empty()) {
      arc += std::hypot(pose.x - points.back()[0], pose.y - points.back()[1]);
    }
    points.push_back({pose.x, pose.y, arc, pose.theta});
  }
  return points;
}

inline bool projectPathPose(
  const std::vector<PathPosePoint> & points, const geometry_msgs::msg::Pose2D & pose,
  PathPoseProjection & result, double minimum = -1e30, double maximum = 1e30,
  double preferred_progress = 0.0)
{
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta) ||
    minimum > maximum)
  {return false;}
  if (points.size() == 1 && minimum <= 0.0 && maximum >= 0.0) {
    result = {0.0, points[0][3], std::hypot(pose.x - points[0][0], pose.y - points[0][1]), 0};
    return true;
  }
  double nearest = std::numeric_limits<double>::infinity();
  double best = nearest, continuity = nearest;
  // First establish the spatial neighborhood, then compare complete poses.
  // Two passes keep the result independent of segment iteration order.
  for (int pass = 0; pass < 2; ++pass) {
    for (std::size_t i = 1; i < points.size(); ++i) {
      const auto & a = points[i - 1]; const auto & b = points[i];
      const double length = b[2] - a[2];
      if (length <= 1e-9 || b[2] < minimum || a[2] > maximum) {continue;}
      const double low = std::clamp((minimum - a[2]) / length, 0.0, 1.0);
      const double high = std::clamp((maximum - a[2]) / length, 0.0, 1.0);
      const double dx = b[0] - a[0], dy = b[1] - a[1];
      const double f = std::clamp(
        ((pose.x - a[0]) * dx + (pose.y - a[1]) * dy) /
        (length * length), low, high);
      const double distance = std::hypot(pose.x - a[0] - f * dx, pose.y - a[1] - f * dy);
      if (pass == 0) {
        nearest = std::min(nearest, distance);
        continue;
      }
      if (distance > nearest + 0.10) {continue;}
      const double heading = wrappedAngle(a[3] + f * wrappedAngle(b[3] - a[3]));
      const double progress = a[2] + f * length;
      const double cost = poseAssociationCost(distance, pose.theta - heading);
      const double delta = std::abs(progress - preferred_progress);
      if (cost < best - 1e-9 || (std::abs(cost - best) <= 1e-9 && delta < continuity)) {
        best = cost;
        continuity = delta;
        result = {progress, heading, distance, i - 1};
      }
    }
  }
  return std::isfinite(best);
}

// Robot history belongs to the controller, never to candidate projections.
// Keep a high-water mark, with 10 cm tolerance for localization noise. The
// forward window permits motion between updates but excludes a remote lap.
class PathProgressTracker
{
public:
  void reset() {valid_ = false; progress_ = 0.0;}
  bool valid() const {return valid_;}
  double progress() const {return progress_;}
  bool update(
    const std::vector<PathPosePoint> & points,
    const geometry_msgs::msg::Pose2D & pose, PathPoseProjection & result)
  {
    const double movement = std::hypot(pose.x - previous_.x, pose.y - previous_.y);
    const double minimum = valid_ ? progress_ - 0.10 : -1e30;
    const double maximum = valid_ ? progress_ + std::max(0.50, 2.0 * movement) : 1e30;
    if (!projectPathPose(points, pose, result, minimum, maximum, progress_)) {return false;}
    progress_ = std::max(progress_, result.progress);
    previous_ = pose;
    valid_ = true;
    return true;
  }

private:
  bool valid_{false};
  double progress_{0.0};
  geometry_msgs::msg::Pose2D previous_;
};

}  // namespace limo_inflation

#endif  // LIMO_INFLATION__PATH_POSE_ASSOCIATION_HPP_
