#include "limo_dwb_critics/mppi_cost_critics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "dwb_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
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
}

bool MppiObstacleCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & velocity,
  const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & path)
{
  near_goal_ = distance(pose, goal) < near_goal_distance_;
  return dwb_critics::ObstacleFootprintCritic::prepare(pose, velocity, goal, path);
}

double MppiObstacleCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (trajectory.poses.size() < 2) {
    throw dwb_core::IllegalTrajectoryException(name_, "Empty prediction");
  }
  if (ignore_obstacles_) {
    return 0.0;  // Explicitly authorized only on a plan-bound initial Dubins prefix.
  }
  double sum = 0.0;
  for (std::size_t t = 0; t < trajectory.poses.size(); ++t) {
    const auto & pose = trajectory.poses[t];
    scorePose(pose);  // Footprint collision rejection at EVERY pose, including t=0.
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
  read("ObstacleGuidance.clearance_margin", guidance_clearance_margin_);
  read("ObstacleGuidance.cost_weight", guidance_cost_weight_);
  read("ObstacleGuidance.heading_weight", guidance_heading_weight_);
  read("ObstacleGuidance.rejoin_distance", guidance_rejoin_distance_);
  const auto read_bool = [&](const std::string & key, bool & value) {
      if (!nh_->has_parameter(prefix + key)) {
        nh_->declare_parameter(prefix + key, rclcpp::ParameterValue(value));
      }
      value = nh_->get_parameter(prefix + key).as_bool();
    };
  read_bool("PathAlignCritic.use_path_orientations", use_path_orientations_);
  read_bool("PathAngleCritic.forward_preference", forward_preference_);
  read_bool("ObstacleGuidance.enabled", guidance_enabled_);
  if (max_path_occupancy_ratio_ > 1.0 || lookahead_distance_ <= 0.0) {
    throw std::invalid_argument("Invalid MPPI path occupancy ratio or lookahead");
  }
}

bool MppiPathCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & path)
{
  goal_ = goal;
  distance_to_goal_ = distance(pose, goal);
  path_ = path.poses;
  const bool was_detouring = guidance_active_;
  guidance_active_ = false;
  if (path_.empty()) {
    return false;
  }
  start_index_ = 0;
  double nearest = std::numeric_limits<double>::infinity();
  path_lengths_.assign(path_.size(), 0.0);
  for (std::size_t i = 0; i < path_.size(); ++i) {
    if (i > 0) {
      path_lengths_[i] = path_lengths_[i - 1] + distance(path_[i - 1], path_[i]);
    }
    const double d = distance(pose, path_[i]);
    if (d < nearest) {
      nearest = d;
      start_index_ = i;
    }
  }
  target_index_ = start_index_;
  while (target_index_ + 1 < path_.size() &&
    path_lengths_[target_index_] - path_lengths_[start_index_] < lookahead_distance_)
  {
    ++target_index_;
  }
  if (ignore_obstacles_) {
    path_blocked_ = false;
    apply_path_angle_ = false;
    return true;  // No obstacle detour may replace the initial Dubins reference.
  }
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
  if (guidance_enabled_) {
    const auto footprint = costmap_ros_->getRobotFootprint();
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);
    // The grid supplies a center reference, not an orientation-independent
    // collision envelope. The full oriented footprint is checked separately.
    const double radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
    guidance_.update(
      costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), costmap->getResolution(),
      costmap->getCharMap(), radius + guidance_clearance_margin_, guidance_cost_weight_);
    const auto free_point = [&](const geometry_msgs::msg::Pose2D & point) {
        unsigned int x, y;
        return costmap->worldToMap(point.x, point.y, x, y) && guidance_.traversable(x, y);
      };
    const auto collision_free = [&](const geometry_msgs::msg::Pose2D & point) {
        unsigned int x, y;
        if (!costmap->worldToMap(point.x, point.y, x, y) ||
          costmap->getCost(x, y) >= nav2_costmap_2d::LETHAL_OBSTACLE)
        {
          return false;
        }
        const double cost = checker.footprintCostAtPose(point.x, point.y, point.theta, footprint);
        return cost >= 0.0 && cost < nav2_costmap_2d::LETHAL_OBSTACLE;
      };
    bool blocked = false;
    // Check between path samples too; a sparse path must not miss a cube.
    auto previous = pose;
    for (std::size_t i = start_index_; i <= target_index_ && !blocked; ++i) {
      const double length = distance(previous, path_[i]);
      const int steps = std::max(1, static_cast<int>(std::ceil(length / costmap->getResolution())));
      for (int step = 0; step <= steps; ++step) {
        auto point = previous;
        const double ratio = static_cast<double>(step) / steps;
        point.x += ratio * (path_[i].x - previous.x);
        point.y += ratio * (path_[i].y - previous.y);
        const double angle = path_[i].theta - previous.theta;
        point.theta += ratio * std::atan2(std::sin(angle), std::cos(angle));
        // A valid planned footprint must not trigger a detour merely because
        // its center lies inside the larger circular guidance margin.
        if (!collision_free(point)) {
          blocked = true;
          break;
        }
      }
      previous = path_[i];
    }
    // Finish an ongoing detour before restoring the strong path alignment;
    // cell-level observation changes must not flip the scoring mode each cycle.
    if (blocked || (was_detouring && nearest > 0.15)) {
      path_blocked_ = true;
      // The nominal lookahead may lie inside the new obstacle. Advance to a
      // free path point after it, rather than attracting MPC into that cell.
      std::size_t rejoin = target_index_;
      while (rejoin < path_.size() && !free_point(path_[rejoin])) {
        ++rejoin;
      }
      // Leave room to straighten after the obstacle. The first free cell on
      // its exit is too early a rejoin target for an Ackermann S maneuver.
      if (rejoin < path_.size()) {
        const auto first_free = rejoin;
        while (rejoin + 1 < path_.size() && free_point(path_[rejoin + 1]) &&
          path_lengths_[rejoin + 1] - path_lengths_[first_free] <= guidance_rejoin_distance_)
        {
          ++rejoin;
        }
      }
      unsigned int x, y;
      if (rejoin < path_.size() &&
        costmap->worldToMap(path_[rejoin].x, path_[rejoin].y, x, y) &&
        guidance_.setTarget(x, y))
      {
        const double px = (pose.x - costmap->getOriginX()) / costmap->getResolution() - 0.5;
        const double py = (pose.y - costmap->getOriginY()) / costmap->getResolution() - 0.5;
        guidance_active_ = std::isfinite(guidance_.distance(px, py));
        if (guidance_active_) {
          target_index_ = rejoin;
        }
      }
    }
  }
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
  if (ignore_obstacles_ && trajectory.velocity.x < -1e-6) {
    throw dwb_core::IllegalTrajectoryException(name_, "Initial Dubins connector is forward only");
  }
  double goal_cost = 0.0;
  double goal_angle_cost = 0.0;
  double align_cost = 0.0;
  std::size_t index = start_index_;
  double traveled = 0.0;
  for (std::size_t t = 1; t < trajectory.poses.size(); ++t) {
    const auto & pose = trajectory.poses[t];
    if (guidance_active_) {
      auto * map = costmap_ros_->getCostmap();
      unsigned int x, y;
      if (!map->worldToMap(pose.x, pose.y, x, y) || !guidance_.traversable(x, y)) {
        throw dwb_core::IllegalTrajectoryException(name_, "Prediction leaves detour clearance");
      }
    }
    if (!guidance_active_ && distance_to_goal_ < goal_distance_) {
      goal_cost += distance(pose, goal_);
    }
    if (!guidance_active_ && distance_to_goal_ < goal_angle_distance_) {
      goal_angle_cost += angleError(pose.theta, goal_.theta);
    }
    if (!path_blocked_ && (distance_to_goal_ > align_distance_ || ignore_obstacles_)) {
      traveled += distance(pose, trajectory.poses[t - 1]);
      double closest = std::numeric_limits<double>::infinity();
      // Monotone, distance-limited association avoids jumping far ahead onto
      // a different branch when a path folds back near the robot.
      for (std::size_t i = index; i < path_.size(); ++i) {
        if (i > index && path_lengths_[i] - path_lengths_[start_index_] > traveled + 0.5) {
          break;
        }
        const double d = distance(pose, path_[i]);
        if (d < closest) {
          closest = d;
          index = i;
        }
      }
      const double angle = use_path_orientations_ ? angleError(pose.theta, path_[index].theta) : 0.0;
      align_cost += std::hypot(closest, angle);
    }
  }
  const double count = trajectory.poses.size() - 1;
  double result = (goal_weight_ * goal_cost + goal_angle_weight_ * goal_angle_cost +
    align_weight_ * align_cost) / count;
  const auto & last = trajectory.poses.back();
  const auto & target = path_[target_index_];
  if (guidance_active_) {
    auto * map = costmap_ros_->getCostmap();
    const double x = (last.x - map->getOriginX()) / map->getResolution() - 0.5;
    const double y = (last.y - map->getOriginY()) / map->getResolution() - 0.5;
    const double remaining = guidance_.distance(x, y);
    if (!std::isfinite(remaining)) {
      throw dwb_core::IllegalTrajectoryException(name_, "No obstacle-free route from prediction");
    }
    result += follow_weight_ * remaining;
    // Ackermann cannot move sideways: reward turning toward a free passage
    // before the robot reaches the face of the obstacle.
    if (remaining > angle_distance_) {
      double motion_heading = last.theta;
      for (std::size_t t = trajectory.poses.size() - 1; t > 0; --t) {
        const auto & before = trajectory.poses[t - 1];
        const auto & after = trajectory.poses[t];
        if (distance(before, after) > 1e-6) {
          motion_heading = std::atan2(after.y - before.y, after.x - before.x);
          break;
        }
      }
      // For a reverse segment, motion heading is opposite to chassis heading.
      // Reward progress along the passage, not rotating the chassis while
      // actually travelling away from it.
      result += guidance_heading_weight_ * angleError(
        motion_heading, guidance_.heading(x, y, motion_heading));
    }
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
