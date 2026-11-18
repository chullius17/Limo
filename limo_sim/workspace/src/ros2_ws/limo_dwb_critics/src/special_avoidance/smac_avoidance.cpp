#include "limo_dwb_critics/mppi_cost_critics.hpp"

#include <algorithm>
#include <cmath>

#include "limo_dwb_critics/guidance_unavailable.hpp"
#include "limo_dwb_critics/mpc_timing_scope.hpp"
#include "limo_inflation/footprint_cells.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace limo_dwb_critics
{
namespace
{
double distance(const geometry_msgs::msg::Pose2D & a, const geometry_msgs::msg::Pose2D & b)
{return std::hypot(a.x - b.x, a.y - b.y);}

geometry_msgs::msg::Pose2D interpolate(
  const geometry_msgs::msg::Pose2D & a, const geometry_msgs::msg::Pose2D & b, double t)
{
  auto p = a;
  p.x += t * (b.x - a.x); p.y += t * (b.y - a.y);
  p.theta += t * limo_inflation::wrappedAngle(b.theta - a.theta);
  return p;
}
}  // namespace

void MppiPathCritic::reset()
{
  path_lookahead_.reset();
  guidance_lookahead_.reset();
  guidance_candidate_target_.reset();
  resetSmacCache();
}

void MppiPathCritic::resetSmacCache()
{
  ++smac_generation_;  // In-flight work may finish, but cannot repopulate this cache.
  smac_detour_pending_ = false;
  guidance_active_ = false;
  smac_path_.clear(); smac_lengths_.clear();
  smac_tracking_points_.clear(); smac_tracking_lengths_.clear();
  smac_progress_tracker_.reset();
  smac_last_plan_start_.reset();
  smac_last_target_warning_.reset();
  smac_failed_goal_.reset();
  smac_advanced_goal_.reset();
  smac_failed_goal_progress_ = smac_advanced_goal_progress_ = 0.0;
}

void MppiPathCritic::recordSmacCompletion(const SmacPlanResult & result, bool discarded)
{
  // A reset invalidates the route, not CPU work already started by the worker.
  smac_last_search_start_ = result.started;
  guidance_plan_called_ = true;
  guidance_plan_failed_ = !result.planned;
  guidance_plan_sample_epoch_ = result.epoch;
  guidance_plan_discarded_ = discarded;
  if (timing_enabled_) {
    guidance_plan_start_steady_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      result.started.time_since_epoch()).count();
    guidance_plan_ms_ = result.duration_ms;
  }
}

void MppiPathCritic::prepareSmacAvoidance(
  const geometry_msgs::msg::Pose2D & pose, const geometry_msgs::msg::Pose2D * desired_goal,
  double target_progress)
{
  smac_detour_pending_ = true;
  auto * map = costmap_ros_->getCostmap();
  const auto global = path_;
  const auto global_points = limo_inflation::pathPosePoints(global);
  const auto footprint = costmap_ros_->getRobotFootprint();
  double footprint_radius = 0.0;
  for (const auto & p : footprint) {footprint_radius = std::max(footprint_radius, std::hypot(p.x, p.y));}
  std::vector<nav2_costmap_2d::MapLocation> cells, polygon;
  std::string invalid_reason = desired_goal ? "no cached path" :
    "no cached path or valid rejoin target";
  const auto safe_segment = [&](const auto & a, const auto & b) {
      const double sweep = distance(a, b) + footprint_radius *
        std::abs(limo_inflation::wrappedAngle(b.theta - a.theta));
      const int steps = std::max(1, static_cast<int>(std::ceil(sweep / map->getResolution())));
      for (int step = 0; step <= steps; ++step) {
        const auto p = interpolate(a, b, static_cast<double>(step) / steps);
        unsigned int x, y;
        const auto location = [&]() {
            return " at (" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")";
          };
        if (!map->worldToMap(p.x, p.y, x, y)) {
          invalid_reason = "reference outside local map" + location();
          return false;
        }
        if (!guidance_.traversable(x, y)) {
          invalid_reason = "reference clearance blocked" + location();
          return false;
        }
        if (!limo_inflation::footprintCells(*map, p, footprint, cells, polygon)) {
          invalid_reason = "footprint outside local map" + location();
          return false;
        }
        for (const auto & cell : cells) {
          if (map->getCost(cell.x, cell.y) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
            invalid_reason = "footprint collision at (" + std::to_string(p.x) + ", " +
              std::to_string(p.y) + "), cell " + std::to_string(cell.x) + ", " +
              std::to_string(cell.y);
            return false;
          }
        }
      }
      return true;
    };
  const auto valid_route = [&]() {
      if (smac_path_.size() < 2) {return false;}
      const auto points = limo_inflation::pathPosePoints(smac_path_);
      limo_inflation::PathPoseProjection projection;
      if (!smac_progress_tracker_.update(points, pose, projection)) {
        invalid_reason = "cannot associate robot with cached path";
        return false;
      }
      const auto i = projection.segment;
      const double length = points[i + 1][2] - points[i][2];
      auto previous = interpolate(smac_path_[i], smac_path_[i + 1], length > 1e-9 ?
        (projection.progress - points[i][2]) / length : 1.0);
      if (!safe_segment(pose, previous)) {
        invalid_reason = "robot-to-reference connection: " + invalid_reason;
        return false;
      }
      for (std::size_t j = i + 1; j < smac_path_.size(); ++j) {
        if (!safe_segment(previous, smac_path_[j])) {return false;}
        previous = smac_path_[j];
      }
      return safe_segment(previous, smac_goal_);
    };
  bool valid = valid_route();
  bool completed_current = false;
  const auto consume = [&](SmacPlanResult result) {
      recordSmacCompletion(result, result.generation != smac_generation_);
      if (result.generation != smac_generation_) {
        RCLCPP_INFO(nh_->get_logger(), "SMAC result discarded: superseded reference/detour");
        return;
      }
      completed_current = true;
      smac_last_plan_start_ = result.started;
      const auto old_path = smac_path_;
      const auto old_lengths = smac_lengths_;
      const auto old_goal = smac_goal_;
      const auto old_tracker = smac_progress_tracker_;
      const auto old_reason = invalid_reason;
      const bool old_valid = valid;
      if (result.planned) {
        smac_path_ = std::move(result.path);
        smac_lengths_ = std::move(result.lengths);
        smac_goal_ = result.goal;
        smac_progress_tracker_.reset();
        valid = valid_route();  // Validate against this cycle's live map and pose.
        guidance_plan_failed_ = !valid;
        if (!valid) {
          result.failure_reason = "current_map_validation";
          result.detail = invalid_reason;
        }
      }
      if (guidance_plan_failed_) {
        smac_failed_goal_ = result.goal;
        smac_failed_goal_progress_ = result.target_progress;
        limo_inflation::PathPoseProjection failed;
        if (limo_inflation::projectPathPose(global_points, result.goal, failed,
            0.0, global_points.back()[2], result.target_progress))
        {smac_failed_goal_progress_ = failed.progress;}
        smac_path_ = old_path; smac_lengths_ = old_lengths; smac_goal_ = old_goal;
        smac_progress_tracker_ = old_tracker;
        valid = old_valid;
        invalid_reason = old_reason;
        RCLCPP_WARN(nh_->get_logger(),
          "SMAC rejected: reason=%s detail=%s goal=(%.3f, %.3f, %.3f) "
          "worker=%.2f ms cached_safe=%s",
          result.failure_reason.c_str(), result.detail.c_str(), result.goal.x, result.goal.y,
          result.goal.theta, result.duration_ms, old_valid ? "true" : "false");
      } else {
        smac_failed_goal_.reset();
        RCLCPP_INFO(nh_->get_logger(),
          "SMAC accepted: goal=(%.3f, %.3f, %.3f) worker=%.2f ms",
          result.goal.x, result.goal.y, result.goal.theta, result.duration_ms);
      }
    };
  if (smac_async_planning_ && smac_worker_) {
    if (auto result = smac_worker_->takeResult()) {consume(std::move(*result));}
  }
  // Reuse a safe route until its goal moves significantly. Compare against
  // the last accepted goal so small changes accumulate and failed updates retry.
  const bool goal_changed = desired_goal && valid &&
    (distance(*desired_goal, smac_goal_) >= smac_replan_goal_distance_ ||
    std::abs(limo_inflation::wrappedAngle(desired_goal->theta - smac_goal_.theta)) >=
    smac_replan_goal_angle_);
  // Validate the route every cycle; rate-limit only the searches it needs,
  // including retries after a failure. Expiring the cooldown alone is no trigger.
  const auto now = std::chrono::steady_clock::now();
  const bool due = !smac_last_search_start_ || smac_replan_frequency_ == 0.0 ||
    std::chrono::duration<double>(now - *smac_last_search_start_).count() >=
    1.0 / smac_replan_frequency_;
  if (!completed_current && due && desired_goal &&
    (!valid || goal_changed || smac_failed_goal_) &&
    (!smac_async_planning_ || (smac_worker_ && !smac_worker_->busy())))
  {
    if (smac_failed_goal_) {
      RCLCPP_INFO(nh_->get_logger(),
        "SMAC retry target: failed=(%.3f, %.3f), next=(%.3f, %.3f)",
        smac_failed_goal_->x, smac_failed_goal_->y, desired_goal->x, desired_goal->y);
    }
    SmacPlanRequest request;
    request.generation = smac_generation_;
    request.epoch = guidance_plan_epoch_ + (!smac_last_plan_start_ ? 1 : 0);
    request.start = pose; request.goal = *desired_goal;
    request.target_progress = target_progress;
    request.map = std::make_shared<const nav2_costmap_2d::Costmap2D>(*map);
    request.footprint = footprint;
    request.clearance_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius() +
      guidance_clearance_margin_;
    request.clearance_margin = guidance_clearance_margin_;
    request.cost_weight = guidance_cost_weight_;
    request.frame = costmap_ros_->getGlobalFrameID();
    request.stamp = nh_->now();
    const auto submitted = std::chrono::steady_clock::now();
    const auto epoch = request.epoch;
    if (smac_async_planning_) {
      if (smac_worker_->submit(std::move(request))) {
        guidance_plan_epoch_ = epoch;
        // Busy prevents a second submission. Completion replaces this provisional
        // stamp with the actual worker start before any subsequent rate check.
        smac_last_plan_start_ = submitted;
        smac_last_search_start_ = submitted;
      }
    } else {
      // Synchronous mode remains available for deterministic backend benchmarks.
      guidance_plan_epoch_ = epoch;
      auto result = computeSmacPlan(request);
      result.generation = request.generation; result.epoch = epoch;
      result.goal = request.goal; result.target_progress = target_progress;
      result.started = submitted; result.finished = std::chrono::steady_clock::now();
      result.duration_ms = std::chrono::duration<double, std::milli>(
        result.finished - result.started).count();
      consume(std::move(result));
    }
  }
  if (!valid) {
    throw GuidanceUnavailable(name_,
            "No collision-free SMAC reference available: " + invalid_reason);
  }
  guidance_active_ = true;
  // Append the original suffix: the 50 cm rejoin point is not a stopping goal.
  path_ = smac_path_;
  if (distance(path_.back(), smac_goal_) > 1e-6) {path_.push_back(smac_goal_);}
  limo_inflation::PathPoseProjection rejoin;
  if (limo_inflation::projectPathPose(global_points, smac_goal_, rejoin)) {
    for (std::size_t i = rejoin.segment + 1; i < global.size(); ++i) {
      if (global_points[i][2] > rejoin.progress + 1e-6) {path_.push_back(global[i]);}
    }
  }
  smac_tracking_points_ = limo_inflation::pathPosePoints(path_);
  path_lengths_.clear(); smac_tracking_lengths_.assign(path_.size(), 0.0);
  for (const auto & point : smac_tracking_points_) {path_lengths_.push_back(point[2]);}
  for (std::size_t i = 1; i < path_.size(); ++i) {
    unsigned int x, y;
    const double penalty = map->worldToMap(path_[i].x, path_[i].y, x, y) ?
      1.0 + guidance_cost_weight_ * std::min(252, static_cast<int>(map->getCost(x, y))) / 252.0 : 1.0;
    smac_tracking_lengths_[i] = smac_tracking_lengths_[i - 1] +
      (path_lengths_[i] - path_lengths_[i - 1]) * penalty;
  }
  start_index_ = limo_inflation::closestPathPoseIndex(path_, pose, 0, path_.size());
  target_index_ = start_index_;
  while (target_index_ + 1 < path_.size() &&
    path_lengths_[target_index_] - path_lengths_[start_index_] < lookahead_distance_) {++target_index_;}
  path_blocked_ = false;  // Alignment now follows SMAC, rather than the global path.
}

bool MppiPathCritic::guidanceTubeContains(
  const geometry_msgs::msg::Pose2D & pose, double radius) const
{
  limo_inflation::PathPoseProjection projection;
  if (!limo_inflation::projectPathPose(smac_tracking_points_, pose, projection)) {return false;}
  const auto footprint = costmap_ros_->getRobotFootprint();
  double reach = 0.0;
  for (const auto & p : footprint) {reach = std::max(reach, std::hypot(p.x, p.y));}
  auto * map = costmap_ros_->getCostmap();
  const double cell_radius = map->getResolution() * std::sqrt(0.5);
  if (projection.distance + reach + cell_radius <= radius) {return true;}
  std::vector<nav2_costmap_2d::MapLocation> cells, polygon;
  if (!limo_inflation::footprintCells(*map, pose, footprint, cells, polygon)) {return false;}
  for (const auto & cell : cells) {
    auto point = pose;
    map->mapToWorld(cell.x, cell.y, point.x, point.y);
    if (!limo_inflation::projectPathPose(smac_tracking_points_, point, projection) ||
      projection.distance + cell_radius > radius) {return false;}
  }
  return true;
}

}  // namespace limo_dwb_critics
