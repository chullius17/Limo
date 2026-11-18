#include "limo_dwb_critics/mppi_cost_critics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "dwb_core/exceptions.hpp"
#include "limo_dwb_critics/guidance_unavailable.hpp"
#include "limo_dwb_critics/mpc_timing_scope.hpp"
#include "limo_inflation/footprint_cells.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"

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

enum class TargetBlocker : std::size_t
{CenterOutside, Clearance, FootprintOutside, FootprintLethal, FootprintUnknown, Invalid, Count};

struct TargetRejection
{
  std::size_t index;
  TargetBlocker reason;
  std::optional<nav2_costmap_2d::MapLocation> collision_cell;
  std::optional<geometry_msgs::msg::Point> outside_vertex;
};

std::string describeTargetRejection(
  const TargetRejection & rejected, const std::vector<geometry_msgs::msg::Pose2D> & path,
  const std::vector<double> & lengths, std::size_t start,
  const nav2_costmap_2d::Costmap2D & map)
{
  constexpr std::array<const char *, static_cast<std::size_t>(TargetBlocker::Count)> reasons = {
    "center_outside", "clearance", "footprint_outside", "footprint_lethal",
    "footprint_unknown", "invalid_footprint"};
  const auto & p = path[rejected.index];
  std::ostringstream out;
  out.precision(9);
  out << "index=" << rejected.index << ", reason=" << reasons[static_cast<std::size_t>(
      rejected.reason)] << ", pose=(" << p.x << ',' << p.y << ',' << p.theta <<
    "), s_rel=" << lengths[rejected.index] - lengths[start];
  unsigned int x, y;
  if (map.worldToMap(p.x, p.y, x, y)) {
    out << ", center_cell=(" << x << ',' << y << "), center_cost=" <<
      static_cast<unsigned int>(map.getCost(x, y));
  } else {
    out << ", center=outside_map";
  }
  if (rejected.collision_cell) {
    const auto & cell = *rejected.collision_cell;
    double wx, wy;
    map.mapToWorld(cell.x, cell.y, wx, wy);
    out << ", collision_cell=(" << cell.x << ',' << cell.y << "), collision_cost=" <<
      static_cast<unsigned int>(map.getCost(cell.x, cell.y)) <<
      ", collision_world=(" << wx << ',' << wy << ')';
  }
  if (rejected.outside_vertex) {
    out << ", outside_vertex=(" << rejected.outside_vertex->x << ',' <<
      rejected.outside_vertex->y << ')';
  }
  return out.str();
}

// The cell's circumscribed disk conservatively covers its occupied area.
// This trigger only selects detour scoring; it never rejects a rollout.
bool footprintIntersectsObstacle(
  const geometry_msgs::msg::Pose2D & pose,
  const nav2_costmap_2d::Footprint & footprint,
  const nav2_costmap_2d::Costmap2D & map, double margin,
  nav2_costmap_2d::Footprint & oriented, bool marked_only = false)
{
  if (footprint.size() < 3) {
    return false;
  }
  oriented.clear();
  nav2_costmap_2d::transformFootprint(pose.x, pose.y, pose.theta, footprint, oriented);
  const double reach = margin + map.getResolution() * std::sqrt(0.5);
  double min_x = oriented.front().x, max_x = min_x;
  double min_y = oriented.front().y, max_y = min_y;
  for (const auto & vertex : oriented) {
    min_x = std::min(min_x, vertex.x);
    max_x = std::max(max_x, vertex.x);
    min_y = std::min(min_y, vertex.y);
    max_y = std::max(max_y, vertex.y);
  }
  int first_x, first_y, last_x, last_y;
  map.worldToMapEnforceBounds(min_x - reach, min_y - reach, first_x, first_y);
  map.worldToMapEnforceBounds(max_x + reach, max_y + reach, last_x, last_y);
  for (int y = first_y; y <= last_y; ++y) {
    for (int x = first_x; x <= last_x; ++x) {
      const auto cost = map.getCost(x, y);
      if (cost < nav2_costmap_2d::LETHAL_OBSTACLE ||
        (marked_only && cost != nav2_costmap_2d::LETHAL_OBSTACLE))
      {
        continue;
      }
      double wx, wy;
      map.mapToWorld(x, y, wx, wy);
      bool inside = false;
      for (std::size_t i = 0, j = oriented.size() - 1; i < oriented.size(); j = i++) {
        const auto & a = oriented[j];
        const auto & b = oriented[i];
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double length_squared = dx * dx + dy * dy;
        const double fraction = length_squared > 0.0 ?
          std::clamp(((wx - a.x) * dx + (wy - a.y) * dy) / length_squared, 0.0, 1.0) :
          0.0;
        const double ex = wx - a.x - fraction * dx;
        const double ey = wy - a.y - fraction * dy;
        if (ex * ex + ey * ey <= reach * reach) {
          return true;
        }
        if ((a.y > wy) != (b.y > wy) && wx < a.x + (wy - a.y) * dx / dy) {
          inside = !inside;
        }
      }
      if (inside) {
        return true;
      }
    }
  }
  return false;
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

void MppiPathCritic::configureObstacleGuidance()
{
  const auto prefix = dwb_plugin_name_ + "." + name_ + ".";
  const auto read = [&](const std::string & key, double & value) {
      value = readNonnegative(nh_, prefix + key, value);
    };
  const auto read_bool = [&](const std::string & key, bool & value) {
      if (!nh_->has_parameter(prefix + key)) {
        nh_->declare_parameter(prefix + key, rclcpp::ParameterValue(value));
      }
      value = nh_->get_parameter(prefix + key).as_bool();
    };
  read("ObstacleGuidance.clearance_margin", guidance_clearance_margin_);
  read("ObstacleGuidance.trigger_margin", guidance_trigger_margin_);
  read("ObstacleGuidance.trigger_front_extension", guidance_trigger_front_extension_);
  read("ObstacleGuidance.lookahead_distance", guidance_lookahead_distance_);
  if (guidance_lookahead_distance_ <= 0.0) {
    throw std::invalid_argument("ObstacleGuidance.lookahead_distance must be positive");
  }
  read("ObstacleGuidance.cost_weight", guidance_cost_weight_);
  read("ObstacleGuidance.heading_weight", guidance_heading_weight_);
  read("ObstacleGuidance.rejoin_distance", guidance_rejoin_distance_);
  read("ObstacleGuidance.Smac.replan_frequency", smac_replan_frequency_);
  read("ObstacleGuidance.Smac.replan_goal_distance", smac_replan_goal_distance_);
  read("ObstacleGuidance.Smac.replan_goal_angle", smac_replan_goal_angle_);
  read("ObstacleGuidance.Smac.failure_advance_distance", smac_failure_advance_distance_);
  read_bool("ObstacleGuidance.enabled", guidance_enabled_);
  read_bool("ObstacleGuidance.use_smac", guidance_use_smac_);
  read_bool("ObstacleGuidance.Smac.async_planning", smac_async_planning_);
  const auto trigger_parameter = dwb_plugin_name_ + "." + name_ +
    ".ObstacleGuidance.trigger_layer";
  if (!nh_->has_parameter(trigger_parameter)) {
    nh_->declare_parameter(trigger_parameter, std::string());
  }
  guidance_trigger_layer_name_ = nh_->get_parameter(trigger_parameter).as_string();
  if (guidance_enabled_ && !guidance_trigger_layer_name_.empty()) {
    for (const auto & layer : *costmap_ros_->getLayeredCostmap()->getPlugins()) {
      if (layer->getName() == guidance_trigger_layer_name_) {
        guidance_trigger_layer_ = std::dynamic_pointer_cast<nav2_costmap_2d::ObstacleLayer>(layer);
        break;
      }
    }
    if (!guidance_trigger_layer_) {
      throw std::invalid_argument("ObstacleGuidance.trigger_layer must name an ObstacleLayer: " +
              guidance_trigger_layer_name_);
    }
  }
  if (guidance_enabled_ && guidance_use_smac_) {configureSmac();}
}

void MppiPathCritic::prepareObstacleGuidance(
  const geometry_msgs::msg::Pose2D & pose, bool was_detouring, double nearest)
{
  auto * costmap = costmap_ros_->getCostmap();
  const auto footprint = costmap_ros_->getRobotFootprint();
  auto trigger_footprint = footprint;
  if (!trigger_footprint.empty()) {
    const double front = std::max_element(
      footprint.begin(), footprint.end(),
      [](const auto & a, const auto & b) {return a.x < b.x;})->x;
    for (auto & vertex : trigger_footprint) {
      if (std::abs(vertex.x - front) <= 1e-9) {
        vertex.x += guidance_trigger_front_extension_;
      }
    }
  }
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);
  // The grid supplies a center reference, not an orientation-independent
  // collision envelope. The full oriented footprint is checked separately.
  const double radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  {
    MpcTimingScope timing(timing_enabled_ ? &guidance_grid_ms_ : nullptr);
    guidance_.update(
      costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), costmap->getResolution(),
      costmap->getCharMap(), radius + guidance_clearance_margin_, guidance_cost_weight_);
    grid_updated_ = true;
  }
  const auto free_point = [&](const geometry_msgs::msg::Pose2D & point) {
      unsigned int x, y;
      if (!costmap->worldToMap(point.x, point.y, x, y) || !guidance_.traversable(x, y)) {
        return false;
      }
      return true;
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
  // The laser trigger's range must not shorten ordinary MPC path tracking.
  std::size_t guidance_lookahead = start_index_;
  while (guidance_lookahead + 1 < path_.size() &&
    path_lengths_[guidance_lookahead] - path_lengths_[start_index_] < guidance_lookahead_distance_)
  {
    ++guidance_lookahead;
  }
  guidance_lookahead_ = path_[guidance_lookahead];
  // The sensor layer preserves provenance lost when the master combines
  // laser, semantic/static maps, inflation and path-distance costs.
  auto * trigger_map = guidance_trigger_layer_ ? guidance_trigger_layer_.get() : costmap;
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> trigger_lock;
  bool trigger_enabled = true;
  if (guidance_trigger_layer_) {
    trigger_lock = std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t>(*trigger_map->getMutex());
    costmap_ros_->get_parameter(guidance_trigger_layer_name_ + ".enabled", trigger_enabled);
  }
  // Check between path samples too; a sparse path must not miss a cube.
  auto previous = pose;
  for (std::size_t i = start_index_; trigger_enabled && i <= guidance_lookahead && !blocked; ++i) {
    const double length = distance(previous, path_[i]);
    const int steps = std::max(1, static_cast<int>(std::ceil(length / costmap->getResolution())));
    for (int step = 0; step <= steps; ++step) {
      auto point = previous;
      const double ratio = static_cast<double>(step) / steps;
      point.x += ratio * (path_[i].x - previous.x);
      point.y += ratio * (path_[i].y - previous.y);
      const double angle = path_[i].theta - previous.theta;
      point.theta += ratio * std::atan2(std::sin(angle), std::cos(angle));
      // Trigger clearance surrounds the footprint. Optional front extension
      // is applied in the robot frame; neither changes collision checking.
      const bool nearby = footprintIntersectsObstacle(point, trigger_footprint, *trigger_map,
        guidance_trigger_margin_, proximity_footprint_, !!guidance_trigger_layer_);
      if (nearby || (!guidance_trigger_layer_ && !collision_free(point)))
      {
        blocked = true;
        break;
      }
    }
    previous = path_[i];
  }
  if (trigger_lock.owns_lock()) {trigger_lock.unlock();}
  // Finish an ongoing detour before restoring the strong path alignment;
  // cell-level observation changes must not flip the scoring mode each cycle.
  const bool detour_pending = was_detouring ||
    (guidance_use_smac_ && (smac_last_plan_start_.has_value() || smac_detour_pending_));
  const bool should_detour = blocked || (detour_pending && nearest > 0.15);
  if (should_detour) {
    path_blocked_ = true;
    if (!guidance_use_smac_) {
      guidance_plan_called_ = true;
      guidance_plan_failed_ = true;
      if (timing_enabled_) {
        guidance_plan_start_steady_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      }
    }
  }
  MpcTimingScope plan_timing(
    should_detour && !guidance_use_smac_ && timing_enabled_ ? &guidance_plan_ms_ : nullptr);
  const bool diagnose_target = should_detour && guidance_use_smac_;
  std::array<std::size_t, static_cast<std::size_t>(TargetBlocker::Count)> rejected_counts{};
  std::optional<TargetRejection> first_rejected, last_rejected;
  const auto reject = [&](std::size_t index, TargetBlocker reason,
    std::optional<nav2_costmap_2d::MapLocation> cell = std::nullopt,
    std::optional<geometry_msgs::msg::Point> vertex = std::nullopt) {
      if (!diagnose_target) {return;}
      ++rejected_counts[static_cast<std::size_t>(reason)];
      last_rejected = TargetRejection{index, reason, cell, vertex};
      if (!first_rejected) {first_rejected = last_rejected;}
    };
  const auto reject_center = [&](std::size_t index) {
      unsigned int x, y;
      reject(index, costmap->worldToMap(path_[index].x, path_[index].y, x, y) ?
        TargetBlocker::Clearance : TargetBlocker::CenterOutside);
    };
  // Select the prospective target even without a trigger so debug consumers
  // can show the points under consideration without invoking either planner.
  // The nominal lookahead may lie inside the new obstacle. Advance to a
  // free path point after it, rather than attracting MPC into that cell.
  std::size_t rejoin = guidance_lookahead;
  while (rejoin < path_.size() && !free_point(path_[rejoin])) {
    if (diagnose_target) {reject_center(rejoin);}
    ++rejoin;
  }
  const auto initial_skipped = rejoin - guidance_lookahead;
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
  std::size_t floor_bypassed = 0, floor_blocked = 0;
  if (should_detour && guidance_use_smac_ && rejoin < path_.size() &&
    (smac_failed_goal_ || smac_advanced_goal_))
  {
    const auto points = limo_inflation::pathPosePoints(path_);
    double minimum = path_lengths_[rejoin];
    const auto advance_from = [&](const auto & target, double & progress, double step) {
        if (!target) {return;}
        limo_inflation::PathPoseProjection projection;
        if (limo_inflation::projectPathPose(points, *target, projection,
            0.0, path_lengths_.back(), progress))
        {
          progress = projection.progress;
          minimum = std::max(minimum,
            std::min(path_lengths_.back(), projection.progress + step));
        }
      };
    advance_from(smac_advanced_goal_, smac_advanced_goal_progress_, 0.0);
    // The failed pose changes only after an actual attempt. Recomputing this
    // bound during cooldown therefore does not repeatedly increment the target.
    advance_from(smac_failed_goal_, smac_failed_goal_progress_, smac_failure_advance_distance_);
    while (rejoin + 1 < path_.size() && path_lengths_[rejoin] + 1e-9 < minimum) {
      ++rejoin; ++floor_bypassed;
    }
    while (rejoin < path_.size() && !free_point(path_[rejoin])) {
      reject_center(rejoin); ++rejoin; ++floor_blocked;
    }
  }
  const auto footprint_scan_begin = rejoin;
  if (guidance_use_smac_) {
    // Preserve the center-based lookahead, straightening distance and failure
    // floor above. Only the final endpoint must fit the full oriented polygon;
    // an obstructed intermediate original-path pose does not shorten the rejoin.
    std::vector<nav2_costmap_2d::MapLocation> candidate_cells, candidate_polygon;
    const auto safe_target = [&](std::size_t index) {
        const auto & point = path_[index];
        if (!free_point(point)) {
          if (diagnose_target) {reject_center(index);}
          return false;
        }
        if (!limo_inflation::footprintCells(
            *costmap, point, footprint, candidate_cells, candidate_polygon))
        {
          if (diagnose_target) {
            // The rasterizer also rejects invalid polygons. Distinguish those
            // from an actual off-map vertex without inventing a map cell.
            std::optional<geometry_msgs::msg::Point> outside;
            if (footprint.size() >= 3 && std::isfinite(point.theta)) {
              const double c = std::cos(point.theta), s = std::sin(point.theta);
              for (const auto & vertex : footprint) {
                geometry_msgs::msg::Point world;
                world.x = point.x + c * vertex.x - s * vertex.y;
                world.y = point.y + s * vertex.x + c * vertex.y;
                unsigned int x, y;
                if (std::isfinite(world.x) && std::isfinite(world.y) &&
                  !costmap->worldToMap(world.x, world.y, x, y))
                {outside = world; break;}
              }
            }
            reject(index, outside ? TargetBlocker::FootprintOutside : TargetBlocker::Invalid,
              std::nullopt, outside);
          }
          return false;
        }
        for (const auto & cell : candidate_cells) {
          const auto cost = costmap->getCost(cell.x, cell.y);
          if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
            reject(index, cost == nav2_costmap_2d::NO_INFORMATION ?
              TargetBlocker::FootprintUnknown : TargetBlocker::FootprintLethal, cell);
            return false;
          }
        }
        return true;
      };
    while (rejoin < path_.size() && !safe_target(rejoin)) {++rejoin;}
    if (should_detour && smac_failed_goal_ && rejoin < path_.size()) {
      smac_advanced_goal_ = path_[rejoin];
      smac_advanced_goal_progress_ = path_lengths_[rejoin];
    }
  }
  unsigned int x = 0, y = 0;
  const bool valid_target = rejoin < path_.size() &&
    costmap->worldToMap(path_[rejoin].x, path_[rejoin].y, x, y);
  if (valid_target) {guidance_candidate_target_ = path_[rejoin];}
  if (diagnose_target) {
    if (valid_target) {
      if (smac_last_target_warning_) {
        RCLCPP_INFO(nh_->get_logger(),
          "SMAC target available again: index=%zu pose=(%.3f, %.3f, %.3f) s_rel=%.3f",
          rejoin, path_[rejoin].x, path_[rejoin].y, path_[rejoin].theta,
          path_lengths_[rejoin] - path_lengths_[start_index_]);
        smac_last_target_warning_.reset();
      }
    } else {
      const auto now = std::chrono::steady_clock::now();
      if (!smac_last_target_warning_ || now - *smac_last_target_warning_ >=
        std::chrono::seconds(2))
      {
        smac_last_target_warning_ = now;
        const auto count = [&](TargetBlocker reason) {
            return rejected_counts[static_cast<std::size_t>(reason)];
          };
        const auto goal_hint = [](const auto & target, double stored_arc) {
            if (!target) {return std::string("none");}
            std::ostringstream out;
            out.precision(9);
            out << "pose=(" << target->x << ',' << target->y << ',' << target->theta <<
              "), stored_arc_hint=" << stored_arc;
            return out.str();
          };
        const auto & end = path_.back();
        RCLCPP_WARN(nh_->get_logger(),
          "SMAC target unavailable: initial_skipped=%zu floor_bypassed=%zu floor_blocked=%zu "
          "footprint_scan_begin=%zu/%zu rejected={center_outside=%zu clearance=%zu "
          "footprint_outside=%zu footprint_lethal=%zu footprint_unknown=%zu invalid_footprint=%zu} "
          "trigger_blocked=%s continuing_detour=%s worker_busy=%s cached_points=%zu",
          initial_skipped, floor_bypassed, floor_blocked, footprint_scan_begin, path_.size(),
          count(TargetBlocker::CenterOutside), count(TargetBlocker::Clearance),
          count(TargetBlocker::FootprintOutside), count(TargetBlocker::FootprintLethal),
          count(TargetBlocker::FootprintUnknown), count(TargetBlocker::Invalid),
          blocked ? "true" : "false", detour_pending ? "true" : "false",
          guidancePlanRunning() ? "true" : "false", smac_path_.size());
        RCLCPP_WARN(nh_->get_logger(),
          "SMAC target context: robot=(%.3f, %.3f, %.3f) lookahead_index=%zu "
          "reference_end=(%.3f, %.3f, %.3f) end_s_rel=%.3f end_to_goal=%.3f "
          "failed_hint={%s} advanced_hint={%s} "
          "map_origin=(%.3f, %.3f) map_cells=(%u, %u) resolution=%.4f "
          "clearance_radius=%.3f; s_rel is arc distance from the clipped reference start index",
          pose.x, pose.y, pose.theta, guidance_lookahead, end.x, end.y, end.theta,
          path_lengths_.back() - path_lengths_[start_index_], distance(end, goal_),
          goal_hint(smac_failed_goal_, smac_failed_goal_progress_).c_str(),
          goal_hint(smac_advanced_goal_, smac_advanced_goal_progress_).c_str(),
          costmap->getOriginX(), costmap->getOriginY(), costmap->getSizeInCellsX(),
          costmap->getSizeInCellsY(), costmap->getResolution(), radius + guidance_clearance_margin_);
        if (first_rejected) {
          RCLCPP_WARN(nh_->get_logger(), "SMAC target first rejected: %s",
            describeTargetRejection(*first_rejected, path_, path_lengths_, start_index_,
            *costmap).c_str());
          RCLCPP_WARN(nh_->get_logger(), "SMAC target last rejected: %s",
            describeTargetRejection(*last_rejected, path_, path_lengths_, start_index_,
            *costmap).c_str());
        }
      }
    }
  }
  if (should_detour) {
    if (guidance_use_smac_) {
      prepareSmacAvoidance(pose, valid_target ? &path_[rejoin] : nullptr,
        valid_target ? path_lengths_[rejoin] : 0.0);
    } else if (valid_target && guidance_.setTarget(x, y)) {
      const double px = (pose.x - costmap->getOriginX()) / costmap->getResolution() - 0.5;
      const double py = (pose.y - costmap->getOriginY()) / costmap->getResolution() - 0.5;
      guidance_active_ = std::isfinite(guidance_.distance(px, py));
      if (guidance_active_) {
        target_index_ = rejoin;
        guidance_plan_failed_ = false;
      }
    }
  } else if (guidance_use_smac_) {
    resetSmacCache();
    if (smac_async_planning_ && smac_worker_) {
      if (auto result = smac_worker_->takeResult()) {
        recordSmacCompletion(*result, true);
        RCLCPP_INFO(nh_->get_logger(), "SMAC result discarded: detour ended");
      }
    }
  }
}

void MppiPathCritic::validateGuidancePose(const geometry_msgs::msg::Pose2D & pose) const
{
  auto * map = costmap_ros_->getCostmap();
  unsigned int x, y;
  if (!map->worldToMap(pose.x, pose.y, x, y) || !guidance_.traversable(x, y)) {
    throw dwb_core::IllegalTrajectoryException(name_, "Prediction leaves detour clearance");
  }
}

double MppiPathCritic::scoreObstacleGuidance(const dwb_msgs::msg::Trajectory2D & trajectory) const
{
  const auto & last = trajectory.poses.back();
  double result = 0.0;
  auto * map = costmap_ros_->getCostmap();
  const double x = (last.x - map->getOriginX()) / map->getResolution() - 0.5;
  const double y = (last.y - map->getOriginY()) / map->getResolution() - 0.5;
  double remaining, reference_heading = last.theta;
  if (guidance_use_smac_) {smacReference(last, remaining, reference_heading);} else {
    remaining = guidance_.distance(x, y);
  }
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
      motion_heading, guidance_use_smac_ ? reference_heading : guidance_.heading(x, y, motion_heading));
  }
  return result;
}

}  // namespace limo_dwb_critics
