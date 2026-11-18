#include "limo_dwb_critics/mppi_cost_critics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "dwb_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "limo_inflation/footprint_cells.hpp"
#include "limo_inflation/path_pose_association.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace limo_dwb_critics
{
namespace
{
double distance(const geometry_msgs::msg::Pose2D & a, const geometry_msgs::msg::Pose2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double angleError(double a, double b)
{
  return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

double readNonnegative(
  const nav2_util::LifecycleNode::SharedPtr & node, const std::string & name, double value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, rclcpp::ParameterValue(value));
  }
  value = node->get_parameter(name).as_double();
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument("Cost parameter must be finite and nonnegative: " + name);
  }
  return value;
}
}  // namespace

void MppiObstacleCritic::onInit()
{
  dwb_critics::ObstacleFootprintCritic::onInit();
  const auto prefix = dwb_plugin_name_ + "." + name_ + ".";
  critical_cost_ = readNonnegative(nh_, prefix + "critical_cost", critical_cost_);
  near_goal_distance_ = readNonnegative(nh_, prefix + "near_goal_distance", near_goal_distance_);
  if (!nh_->has_parameter(prefix + "full_footprint_check")) {
    nh_->declare_parameter(prefix + "full_footprint_check", false);
  }
  full_footprint_check_ = nh_->get_parameter(prefix + "full_footprint_check").as_bool();
}

bool MppiObstacleCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & velocity,
  const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & path)
{
  near_goal_ = distance(pose, goal) < near_goal_distance_;
  const bool ready = dwb_critics::ObstacleFootprintCritic::prepare(pose, velocity, goal, path);
  if (ready && full_footprint_check_) {
    collision_footprint_ = costmap_ros_->getRobotFootprint();
    footprint_radius_ = 0.0;
    for (const auto & vertex : collision_footprint_) {
      footprint_radius_ = std::max(footprint_radius_, std::hypot(vertex.x, vertex.y));
    }
    const std::size_t stride = costmap_->getSizeInCellsX() + 1;
    blocked_prefix_.assign(stride * (costmap_->getSizeInCellsY() + 1), 0);
    for (unsigned int y = 0; y < costmap_->getSizeInCellsY(); ++y) {
      std::uint32_t row = 0;
      for (unsigned int x = 0; x < costmap_->getSizeInCellsX(); ++x) {
        row += costmap_->getCost(x, y) >= nav2_costmap_2d::LETHAL_OBSTACLE;
        blocked_prefix_[(y + 1) * stride + x + 1] = blocked_prefix_[y * stride + x + 1] + row;
      }
    }
  }
  return ready;
}

double MppiObstacleCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (trajectory.poses.size() < 2) {
    throw dwb_core::IllegalTrajectoryException(name_, "Empty prediction");
  }
  if (full_footprint_check_ && blocked_prefix_.size() !=
    (costmap_->getSizeInCellsX() + 1) * (costmap_->getSizeInCellsY() + 1))
  {throw dwb_core::IllegalTrajectoryException(name_, "Footprint collision data not prepared");}
  double sum = 0.0;
  const auto check_footprint = [&](const geometry_msgs::msg::Pose2D & pose) {
      if (!limo_inflation::footprintCells(
          *costmap_, pose, collision_footprint_,
          collision_cells_, collision_polygon_, &blocked_prefix_))
      {throw dwb_core::IllegalTrajectoryException(name_, "Footprint outside costmap");}
      for (const auto & cell : collision_cells_) {
        if (costmap_->getCost(cell.x, cell.y) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
          throw dwb_core::IllegalTrajectoryException(
                  name_,
                  "Full footprint in lethal/unknown cell");
        }
      }
    };
  for (std::size_t t = 0; t < trajectory.poses.size(); ++t) {
    const auto & pose = trajectory.poses[t];
    if (full_footprint_check_) {
      if (t == 0) {check_footprint(pose);} else {
        const auto & previous = trajectory.poses[t - 1];
        const double angle = std::atan2(
          std::sin(pose.theta - previous.theta),
          std::cos(pose.theta - previous.theta));
        const double travel = distance(pose, previous) + footprint_radius_ * std::abs(angle);
        const int steps = std::max(
          1, static_cast<int>(std::ceil(
            travel / (0.5 * costmap_->getResolution()))));
        for (int step = 1; step <= steps; ++step) {
          const double f = static_cast<double>(step) / steps;
          geometry_msgs::msg::Pose2D intermediate;
          intermediate.x = previous.x + f * (pose.x - previous.x);
          intermediate.y = previous.y + f * (pose.y - previous.y);
          intermediate.theta = previous.theta + f * angle;
          check_footprint(intermediate);
        }
      }
    } else {scorePose(pose);}
    unsigned int x, y;
    if (!costmap_->worldToMap(pose.x, pose.y, x, y)) {
      throw dwb_core::IllegalTrajectoryException(name_, "Prediction leaves costmap");
    }
    const auto cost = costmap_->getCost(x, y);
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::NO_INFORMATION) {
      throw dwb_core::IllegalTrajectoryException(name_, "Prediction center in obstacle or unknown");
    }
    if (t == 0) {
      continue;  // Match the mean over future states, not the shared initial pose.
    }
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      sum += critical_cost_;
    } else if (!near_goal_) {
      sum += cost;
    }
  }
  return sum / (254.0 * (trajectory.poses.size() - 1));
}

void MppiPathCritic::onInit()
{
  const auto prefix = dwb_plugin_name_ + "." + name_ + ".";
  const auto read = [&](const std::string & key, double & value) {
      value = readNonnegative(nh_, prefix + key, value);
    };
  read("GoalCritic.cost_weight", goal_weight_);
  read("GoalCritic.threshold_to_consider", goal_distance_);
  read("GoalAngleCritic.cost_weight", goal_angle_weight_);
  read("GoalAngleCritic.threshold_to_consider", goal_angle_distance_);
  read("PathAlignCritic.cost_weight", align_weight_);
  read("PathAlignCritic.threshold_to_consider", align_distance_);
  read("PathAlignCritic.max_path_occupancy_ratio", max_path_occupancy_ratio_);
  read("PathFollowCritic.cost_weight", follow_weight_);
  read("PathFollowCritic.threshold_to_consider", follow_distance_);
  read("PathFollowCritic.lookahead_distance", lookahead_distance_);
  read("PathAngleCritic.cost_weight", angle_weight_);
  read("PathAngleCritic.threshold_to_consider", angle_distance_);
  read("PathAngleCritic.max_angle_to_furthest", max_angle_to_furthest_);
  const auto read_bool = [&](const std::string & key, bool & value) {
      if (!nh_->has_parameter(prefix + key)) {
        nh_->declare_parameter(prefix + key, rclcpp::ParameterValue(value));
      }
      value = nh_->get_parameter(prefix + key).as_bool();
    };
  read_bool("PathAlignCritic.use_path_orientations", use_path_orientations_);
  read_bool("PathAngleCritic.forward_preference", forward_preference_);
  if (max_path_occupancy_ratio_ > 1.0 || lookahead_distance_ <= 0.0) {
    throw std::invalid_argument("Invalid MPPI path occupancy ratio or lookahead");
  }
  configureObstacleGuidance();
}

bool MppiPathCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & path)
{
  resetTiming();
  path_lookahead_.reset();
  guidance_lookahead_.reset();
  guidance_candidate_target_.reset();
  goal_ = goal;
  distance_to_goal_ = distance(pose, goal);
  path_ = path.poses;
  const bool was_detouring = guidance_active_;
  guidance_active_ = false;
  if (path_.empty()) {
    resetSmacCache();
    return false;
  }
  path_lengths_.assign(path_.size(), 0.0);
  for (std::size_t i = 0; i < path_.size(); ++i) {
    if (i > 0) {
      path_lengths_[i] = path_lengths_[i - 1] + distance(path_[i - 1], path_[i]);
    }
  }
  // The controller already selected this branch using persistent history.
  // A second unrestricted search here could skip to a future nearby lap.
  const std::size_t start_end = tracked_reference_ ?
    static_cast<std::size_t>(std::upper_bound(
      path_lengths_.begin(), path_lengths_.end(), 0.50) - path_lengths_.begin()) : path_.size();
  start_index_ = limo_inflation::closestPathPoseIndex(path_, pose, 0, start_end);
  const double nearest = distance(pose, path_[start_index_]);
  target_index_ = start_index_;
  while (target_index_ + 1 < path_.size() &&
    path_lengths_[target_index_] - path_lengths_[start_index_] < lookahead_distance_)
  {
    ++target_index_;
  }
  path_lookahead_ = path_[target_index_];
  auto * costmap = costmap_ros_->getCostmap();
  std::size_t occupied = 0;
  for (std::size_t i = start_index_; i <= target_index_; ++i) {
    unsigned int x, y;
    if (!costmap->worldToMap(path_[i].x, path_[i].y, x, y) ||
      costmap->getCost(x, y) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      ++occupied;
    }
  }
  path_blocked_ = static_cast<double>(occupied) / (target_index_ - start_index_ + 1) >
    max_path_occupancy_ratio_;
  if (guidance_enabled_) {prepareObstacleGuidance(pose, was_detouring, nearest);}
  const auto & target = path_[target_index_];
  double angle = angleError(pose.theta, std::atan2(target.y - pose.y, target.x - pose.x));
  if (!forward_preference_) {
    angle = std::min(angle, std::acos(-1.0) - angle);
  }
  apply_path_angle_ = angle > max_angle_to_furthest_;
  return true;
}

double MppiPathCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (path_.empty() || trajectory.poses.size() < 2) {
    throw dwb_core::IllegalTrajectoryException(name_, "Missing path or prediction");
  }
  double goal_cost = 0.0;
  double goal_angle_cost = 0.0;
  double align_cost = 0.0;
  std::size_t index = start_index_;
  double traveled = 0.0;
  for (std::size_t t = 1; t < trajectory.poses.size(); ++t) {
    const auto & pose = trajectory.poses[t];
    if (guidance_active_) {validateGuidancePose(pose);}
    if (!guidance_active_ && distance_to_goal_ < goal_distance_) {
      goal_cost += distance(pose, goal_);
    }
    if (!guidance_active_ && distance_to_goal_ < goal_angle_distance_) {
      goal_angle_cost += angleError(pose.theta, goal_.theta);
    }
    if (!path_blocked_ && distance_to_goal_ > align_distance_) {
      traveled += distance(pose, trajectory.poses[t - 1]);
      // Monotone, distance-limited association avoids jumping far ahead onto
      // a different branch when a path folds back near the robot.
      std::size_t end = index + 1;
      while (end < path_.size() &&
        path_lengths_[end] - path_lengths_[start_index_] <= traveled + 0.5)
      {
        ++end;
      }
      index = limo_inflation::closestPathPoseIndex(path_, pose, index, end);
      const double closest = distance(pose, path_[index]);
      const double angle =
        use_path_orientations_ ? angleError(pose.theta, path_[index].theta) : 0.0;
      align_cost += std::hypot(closest, angle);
    }
  }
  const double count = trajectory.poses.size() - 1;
  double result = (goal_weight_ * goal_cost + goal_angle_weight_ * goal_angle_cost +
    align_weight_ * align_cost) / count;
  const auto & last = trajectory.poses.back();
  const auto & target = path_[target_index_];
  if (guidance_active_) {
    result += scoreObstacleGuidance(trajectory);
  } else if (distance_to_goal_ >= follow_distance_) {
    result += follow_weight_ * distance(last, target);
  }
  if (!guidance_active_ && distance_to_goal_ > angle_distance_ && apply_path_angle_) {
    double angle = angleError(last.theta, std::atan2(target.y - last.y, target.x - last.x));
    if (!forward_preference_) {
      angle = std::min(angle, std::acos(-1.0) - angle);
    }
    result += angle_weight_ * angle;
  }
  return result;
}

}  // namespace limo_dwb_critics

PLUGINLIB_EXPORT_CLASS(limo_dwb_critics::MppiObstacleCritic, dwb_core::TrajectoryCritic)
PLUGINLIB_EXPORT_CLASS(limo_dwb_critics::MppiPathCritic, dwb_core::TrajectoryCritic)
