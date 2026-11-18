#include "limo_dwb_critics/mppi_cost_critics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "limo_dwb_critics/special_avoidance/smac_worker.hpp"
#include "limo_inflation/footprint_cells.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav_msgs/msg/path.hpp"

namespace limo_dwb_critics
{

MppiPathCritic::~MppiPathCritic()
{
  // The native planner and private costmap belong to the worker while a job
  // runs. Joining is a lifecycle operation, never part of a control-cycle reset.
  smac_worker_.reset();
  if (smac_planner_) {
    // Foxy 0.4.7 cleanup() dereferences its null downsampler when downsampling
    // is disabled. Deactivate, then let the plugin's RAII members release it.
    // navigation2/0.4.7/smac_planner/src/smac_planner.cpp, cleanup().
    try {smac_planner_->deactivate();} catch (...) {}
    smac_planner_.reset();
  }
  if (smac_costmap_) {
    try {smac_costmap_->on_cleanup(rclcpp_lifecycle::State());} catch (...) {}
  }
}

std::vector<geometry_msgs::msg::Pose2D> MppiPathCritic::guidanceReference(
  const geometry_msgs::msg::Pose2D & pose) const
{
  if (!guidance_active_) {return {};}
  if (guidance_use_smac_) {return smac_path_;}
  auto * map = costmap_ros_->getCostmap();
  unsigned int x, y;
  if (!map->worldToMap(pose.x, pose.y, x, y)) {return {};}
  const auto cells = guidance_.route(x, y, pose.theta);
  std::vector<geometry_msgs::msg::Pose2D> result;
  result.reserve(cells.size());
  for (const auto & cell : cells) {
    geometry_msgs::msg::Pose2D point;
    map->mapToWorld(cell.first, cell.second, point.x, point.y);
    result.push_back(point);
  }
  for (std::size_t i = 0; i + 1 < result.size(); ++i) {
    result[i].theta = std::atan2(result[i + 1].y - result[i].y, result[i + 1].x - result[i].x);
  }
  if (!result.empty()) {result.back().theta = path_[target_index_].theta;}
  return result;
}

void MppiPathCritic::configureSmac()
{
  const auto source = dwb_plugin_name_ + "." + name_ + ".ObstacleGuidance.Smac.";
  // Keep this private parameter namespace compatible with ROS naming rules.
  const auto plugin_name = dwb_plugin_name_ + "_" + name_ + "_local_smac";
  const auto parameter = [&](const std::string & key, const rclcpp::ParameterValue & value) {
      if (!nh_->has_parameter(source + key)) {nh_->declare_parameter(source + key, value);}
      const auto configured = nh_->get_parameter(source + key).get_parameter_value();
      const auto internal = plugin_name + "." + key;
      if (!nh_->has_parameter(internal)) {nh_->declare_parameter(internal, configured);}
      nh_->set_parameter(rclcpp::Parameter(internal, configured));
    };
  const auto radius_name = dwb_plugin_name_ + ".AckermannKinematics.min_turning_radius";
  const double radius = nh_->has_parameter(radius_name) ?
    nh_->get_parameter(radius_name).as_double() : 0.55;
  parameter("minimum_turning_radius", rclcpp::ParameterValue(radius));
  parameter("angle_quantization_bins", rclcpp::ParameterValue(64));
  parameter("max_iterations", rclcpp::ParameterValue(20000));
  parameter("tolerance", rclcpp::ParameterValue(0.05));
  parameter("cost_penalty", rclcpp::ParameterValue(guidance_cost_weight_));
  parameter("analytic_expansion_ratio", rclcpp::ParameterValue(3.5));
  parameter("change_penalty", rclcpp::ParameterValue(0.0));
  parameter("non_straight_penalty", rclcpp::ParameterValue(1.15));
  // The structured MPC moves forward. Preserve SMAC's curvature with no smoother.
  parameter("motion_model_for_search", rclcpp::ParameterValue(std::string("DUBIN")));
  parameter("smooth_path", rclcpp::ParameterValue(false));
  parameter("downsample_costmap", rclcpp::ParameterValue(false));
  parameter("downsampling_factor", rclcpp::ParameterValue(1));
  parameter("allow_unknown", rclcpp::ParameterValue(false));
  if (nh_->get_parameter(source + "minimum_turning_radius").as_double() < radius ||
    nh_->get_parameter(source + "max_iterations").as_int() <= 0 ||
    nh_->get_parameter(source + "angle_quantization_bins").as_int() < 8 ||
    nh_->get_parameter(source + "motion_model_for_search").as_string() != "DUBIN" ||
    nh_->get_parameter(source + "smooth_path").as_bool() ||
    nh_->get_parameter(source + "downsample_costmap").as_bool() ||
    nh_->get_parameter(source + "allow_unknown").as_bool())
  {throw std::invalid_argument("Local SMAC requires bounded forward, unsmoothed planning");}
  smac_loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::GlobalPlanner>>(
    "nav2_core", "nav2_core::GlobalPlanner");
  smac_planner_ = smac_loader_->createSharedInstance("smac_planner/SmacPlanner");
  // Use a private snapshot so the extra clearance never changes the live map.
  smac_costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(plugin_name + "_costmap");
  std::ostringstream footprint;
  footprint.precision(17);
  footprint << '[';
  bool first = true;
  for (const auto & point : costmap_ros_->getRobotFootprint()) {
    if (!first) {footprint << ',';}
    footprint << '[' << point.x << ',' << point.y << ']';
    first = false;
  }
  footprint << ']';
  smac_costmap_->set_parameters({
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("global_frame", costmap_ros_->getGlobalFrameID()),
    rclcpp::Parameter("robot_base_frame", costmap_ros_->getBaseFrameID()),
    rclcpp::Parameter("footprint", footprint.str()),
    rclcpp::Parameter("footprint_padding", 0.0)});
  if (smac_costmap_->on_configure(rclcpp_lifecycle::State()) != nav2_util::CallbackReturn::SUCCESS) {
    throw std::runtime_error("Unable to configure the local SMAC costmap snapshot");
  }
  smac_costmap_->setRobotFootprint(costmap_ros_->getRobotFootprint());
  *smac_costmap_->getCostmap() = *costmap_ros_->getCostmap();
  smac_planner_->configure(nh_, plugin_name, nullptr, smac_costmap_);
  smac_planner_->activate();
  if (smac_async_planning_) {
    smac_worker_ = std::make_unique<SmacPlannerWorker>(
      [this](const SmacPlanRequest & request) {return computeSmacPlan(request);});
  }
}

SmacPlanResult MppiPathCritic::computeSmacPlan(const SmacPlanRequest & request)
{
  SmacPlanResult result;
  result.generation = request.generation;
  result.epoch = request.epoch;
  result.goal = request.goal;
  result.target_progress = request.target_progress;
  const auto fail = [&](const std::string & reason, const std::string & detail) {
      result.failure_reason = reason;
      result.detail = detail;
      result.path.clear();
      result.lengths.clear();
      return result;
    };
  const auto finite_pose = [](const geometry_msgs::msg::Pose2D & pose) {
      return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.theta);
    };
  if (!request.map || !smac_costmap_ || !smac_planner_) {
    return fail("invalid_snapshot", "Missing request snapshot or configured native planner");
  }
  if (!finite_pose(request.start) || !finite_pose(request.goal) ||
    !std::isfinite(request.target_progress) || !std::isfinite(request.clearance_radius) ||
    !std::isfinite(request.clearance_margin) || !std::isfinite(request.cost_weight))
  {return fail("nonfinite", "Non-finite request pose, progress, clearance, or cost weight");}
  if (request.frame.empty() || request.frame != smac_costmap_->getGlobalFrameID()) {
    return fail("frame_mismatch", "Request frame='" + request.frame +
      "', native frame='" + smac_costmap_->getGlobalFrameID() + "'");
  }
  if (request.clearance_radius < 0.0 || request.clearance_margin < 0.0 ||
    request.cost_weight < 0.0 || request.footprint.size() < 3)
  {return fail("invalid_snapshot", "Invalid clearance, cost weight, or footprint polygon");}
  double footprint_radius = 0.0;
  for (const auto & point : request.footprint) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return fail("nonfinite", "Non-finite request footprint vertex");
    }
    footprint_radius = std::max(footprint_radius, std::hypot(point.x, point.y));
  }

  // footprintCells uses Nav2's non-const rasterization API. This local RAW copy
  // preserves the immutable request and the exact full-polygon collision check.
  nav2_costmap_2d::Costmap2D raw(*request.map);
  auto * map = &raw;
  if (map->getSizeInCellsX() == 0 || map->getSizeInCellsY() == 0 ||
    !std::isfinite(map->getResolution()) || map->getResolution() <= 0.0)
  {return fail("invalid_snapshot", "Empty snapshot or invalid map resolution");}
  ObstacleGuidance clearance;
  clearance.update(map->getSizeInCellsX(), map->getSizeInCellsY(), map->getResolution(),
    map->getCharMap(), request.clearance_radius, request.cost_weight);
  const auto context = [](const nav2_costmap_2d::Costmap2D & checked_map,
    const geometry_msgs::msg::Pose2D & pose) {
      std::ostringstream out;
      out.precision(9);
      out << "world=(" << pose.x << ',' << pose.y << "), yaw=" << pose.theta;
      unsigned int x, y;
      if (checked_map.worldToMap(pose.x, pose.y, x, y)) {
        out << ", cell=(" << x << ',' << y << "), cost=" <<
          static_cast<unsigned int>(checked_map.getCost(x, y));
      } else {
        out << ", outside_map origin=(" << checked_map.getOriginX() << ',' <<
          checked_map.getOriginY() << "), size=(" << checked_map.getSizeInCellsX() <<
          ',' << checked_map.getSizeInCellsY() << "), resolution=" << checked_map.getResolution();
      }
      return out.str();
    };
  std::vector<nav2_costmap_2d::MapLocation> footprint_cells, polygon;
  const auto footprint_failure = [&](nav2_costmap_2d::Costmap2D & checked_map,
    const geometry_msgs::msg::Pose2D & pose) -> std::string {
      unsigned int x, y;
      if (!checked_map.worldToMap(pose.x, pose.y, x, y)) {
        return "Center outside map: " + context(checked_map, pose);
      }
      if (checked_map.getCost(x, y) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        return "Center in lethal/unknown cell: " + context(checked_map, pose);
      }
      if (!limo_inflation::footprintCells(
          checked_map, pose, request.footprint, footprint_cells, polygon))
      {return "Full footprint outside map: " + context(checked_map, pose);}
      for (const auto & cell : footprint_cells) {
        const auto cost = checked_map.getCost(cell.x, cell.y);
        if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
          return "Full footprint in lethal/unknown cell=(" + std::to_string(cell.x) + ',' +
            std::to_string(cell.y) + "), cost=" + std::to_string(cost) + "; " +
            context(checked_map, pose);
        }
      }
      return {};
    };
  const auto endpoint_failure = [&](const geometry_msgs::msg::Pose2D & pose,
    const std::string & endpoint) -> bool {
      const auto collision = footprint_failure(raw, pose);
      if (!collision.empty()) {
        result.failure_reason = endpoint + "_footprint";
        result.detail = "Raw snapshot: " + collision;
        return true;
      }
      unsigned int x, y;
      if (!map->worldToMap(pose.x, pose.y, x, y) || !clearance.traversable(x, y)) {
        result.failure_reason = endpoint + "_clearance";
        result.detail = "Raw snapshot clearance blocked: " + context(raw, pose) +
          ", clearance_radius=" + std::to_string(request.clearance_radius);
        return true;
      }
      return false;
    };
  if (endpoint_failure(request.start, "start") || endpoint_failure(request.goal, "goal")) {
    return result;
  }

  auto * snapshot = smac_costmap_->getCostmap();
  *snapshot = *map;
  smac_costmap_->setRobotFootprint(request.footprint);
  // SMAC already checks the full footprint, so do not add the robot radius
  // again. Cover both the obstacle and reference cell's half diagonals: the
  // common center grid rejects the whole reference cell, whereas SMAC checks
  // continuous poses against rasterized footprint edges.
  const double radius = request.clearance_margin / map->getResolution() + std::sqrt(2.0);
  const int cells = static_cast<int>(std::ceil(radius));
  for (unsigned int y = 0; y < map->getSizeInCellsY(); ++y) {
    for (unsigned int x = 0; x < map->getSizeInCellsX(); ++x) {
      if (map->getCost(x, y) < nav2_costmap_2d::LETHAL_OBSTACLE) {continue;}
      for (int dy = -cells; dy <= cells; ++dy) {
        for (int dx = -cells; dx <= cells; ++dx) {
          const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
          if (std::hypot(dx, dy) <= radius && nx >= 0 && ny >= 0 &&
            nx < static_cast<int>(map->getSizeInCellsX()) && ny < static_cast<int>(map->getSizeInCellsY()))
          {snapshot->setCost(nx, ny, nav2_costmap_2d::LETHAL_OBSTACLE);}
        }
      }
    }
  }
  // Report the native inflated-map conditions when SMAC rejects a request.
  // These diagnostics do not relax its existing quantization/clearance margin.
  const auto native_start_collision = footprint_failure(*snapshot, request.start);
  const auto native_goal_collision = footprint_failure(*snapshot, request.goal);
  const auto native_context = [&]() {
      return "native_start={" + context(*snapshot, request.start) + "; " +
        (native_start_collision.empty() ? "full_footprint=clear" : native_start_collision) +
        "}; native_goal={" + context(*snapshot, request.goal) + "; " +
        (native_goal_collision.empty() ? "full_footprint=clear" : native_goal_collision) +
        "}; clearance_margin=" + std::to_string(request.clearance_margin) +
        ", quantization_margin_cells=" + std::to_string(std::sqrt(2.0));
    };
  geometry_msgs::msg::PoseStamped from, to;
  from.header.frame_id = to.header.frame_id = request.frame;
  from.header.stamp = to.header.stamp = request.stamp;
  const auto set_pose = [](const auto & pose, auto & stamped) {
      stamped.pose.position.x = pose.x;
      stamped.pose.position.y = pose.y;
      stamped.pose.orientation.z = std::sin(0.5 * pose.theta);
      stamped.pose.orientation.w = std::cos(0.5 * pose.theta);
    };
  set_pose(request.start, from);
  set_pose(request.goal, to);  // Reuse the reference's arrival orientation as well as position.
  nav_msgs::msg::Path route;
  try {
    route = smac_planner_->createPlan(from, to);
  } catch (const std::exception & error) {
    return fail("native_exception", std::string(error.what()) + "; " + native_context());
  } catch (...) {
    return fail("native_exception", "Unknown native planner exception; " + native_context());
  }
  if (route.poses.empty()) {return fail("empty_route", native_context());}
  if (route.header.frame_id != request.frame) {
    return fail("frame_mismatch", "Route frame='" + route.header.frame_id +
      "', request frame='" + request.frame + "'");
  }
  const auto validate_segment = [&](const geometry_msgs::msg::Pose2D & a,
    const geometry_msgs::msg::Pose2D & b, const std::string & segment) -> bool {
      const double angle = std::atan2(std::sin(b.theta - a.theta), std::cos(b.theta - a.theta));
      const double sweep = std::hypot(b.x - a.x, b.y - a.y) + footprint_radius * std::abs(angle);
      const int steps = std::max(1, static_cast<int>(std::ceil(sweep / map->getResolution())));
      for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / steps;
        auto point = a;
        point.x += t * (b.x - a.x);
        point.y += t * (b.y - a.y);
        point.theta += t * angle;
        unsigned int x, y;
        if (!map->worldToMap(point.x, point.y, x, y) || !clearance.traversable(x, y)) {
          result.failure_reason = "clearance";
          result.detail = segment + ": raw snapshot clearance blocked: " + context(raw, point);
          return false;
        }
        const auto collision = footprint_failure(raw, point);
        if (!collision.empty()) {
          result.failure_reason = "footprint";
          result.detail = segment + ": raw snapshot: " + collision;
          return false;
        }
      }
      return true;
    };
  auto previous = request.start;
  result.path.push_back(request.start);
  result.lengths.push_back(0.0);
  for (std::size_t route_index = 0; route_index < route.poses.size(); ++route_index) {
    const auto & stamped = route.poses[route_index];
    const auto segment = "route_pose=" + std::to_string(route_index);
    if (!stamped.header.frame_id.empty() && stamped.header.frame_id != request.frame) {
      return fail("frame_mismatch", segment + ": frame='" + stamped.header.frame_id + "'");
    }
    const auto & p = stamped.pose.position;
    const auto & q = stamped.pose.orientation;
    const double quaternion_norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
      !std::isfinite(q.w) || !std::isfinite(quaternion_norm) || quaternion_norm < 1e-12)
    {return fail("nonfinite", segment + ": invalid position or quaternion");}
    geometry_msgs::msg::Pose2D point;
    point.x = p.x; point.y = p.y;
    point.theta = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.theta)) {
      return fail("nonfinite", segment + ": invalid planar pose");
    }
    if (!validate_segment(previous, point, segment)) {
      return fail(result.failure_reason, result.detail);
    }
    const double length = std::hypot(point.x - previous.x, point.y - previous.y);
    if (length < 1e-9) {continue;}
    const int steps = std::max(1, static_cast<int>(std::ceil(length / map->getResolution())));
    double segment_cost = 0.0;
    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      unsigned int x, y;
      const double wx = previous.x + t * (point.x - previous.x);
      const double wy = previous.y + t * (point.y - previous.y);
      if (!map->worldToMap(wx, wy, x, y) || !clearance.traversable(x, y)) {
        auto invalid = point;
        invalid.x = wx; invalid.y = wy;
        return fail("clearance", segment + ": raw snapshot clearance blocked: " +
          context(raw, invalid));
      }
      const double penalty = 1.0 + request.cost_weight *
        std::min(252, static_cast<int>(map->getCost(x, y))) / 252.0;
      segment_cost += penalty * (i == 0 || i == steps ? 0.5 : 1.0);
    }
    result.path.push_back(point);
    result.lengths.push_back(result.lengths.back() + length * segment_cost / steps);
    previous = point;
  }
  if (result.path.size() < 2) {
    return fail("empty_route", "Native route contained no distinct planar points; " + native_context());
  }
  if (!validate_segment(previous, request.goal, "route_to_goal")) {
    return fail(result.failure_reason, result.detail);
  }
  result.planned = true;
  return result;
}

void MppiPathCritic::smacReference(
  const geometry_msgs::msg::Pose2D & pose, double & remaining, double & heading) const
{
  remaining = std::numeric_limits<double>::infinity();
  heading = pose.theta;
  double best_distance = std::numeric_limits<double>::infinity();
  std::size_t best_segment = 0;
  double best_t = 0.0;
  for (std::size_t i = 1; i < path_.size(); ++i) {
    const auto & a = path_[i - 1];
    const auto & b = path_[i];
    const double dx = b.x - a.x, dy = b.y - a.y;
    if (dx * dx + dy * dy < 1e-18) {continue;}
    const double t = std::clamp(
      ((pose.x - a.x) * dx + (pose.y - a.y) * dy) / (dx * dx + dy * dy), 0.0, 1.0);
    const double distance = std::hypot(pose.x - a.x - t * dx, pose.y - a.y - t * dy);
    // Equal-distance crossings retain the earlier branch instead of skipping ahead.
    if (distance < best_distance - 1e-9) {
      best_distance = distance; best_segment = i; best_t = t;
    }
  }
  if (best_segment == 0) {return;}
  const auto & a = path_[best_segment - 1];
  const auto & b = path_[best_segment];
  const double x = a.x + best_t * (b.x - a.x), y = a.y + best_t * (b.y - a.y);
  const double progress = smac_tracking_lengths_[best_segment - 1] + best_t *
    (smac_tracking_lengths_[best_segment] - smac_tracking_lengths_[best_segment - 1]);
  remaining = best_distance + smac_tracking_lengths_.back() - progress;
  double lookahead = 0.35, from_x = x, from_y = y, target_x = b.x, target_y = b.y;
  for (std::size_t i = best_segment; i < path_.size(); ++i) {
    const auto & point = path_[i];
    const double length = std::hypot(point.x - from_x, point.y - from_y);
    const double t = length > 1e-9 ? std::min(1.0, lookahead / length) : 1.0;
    target_x = from_x + t * (point.x - from_x);
    target_y = from_y + t * (point.y - from_y);
    if (length >= lookahead) {break;}
    lookahead -= length;
    from_x = point.x; from_y = point.y;
  }
  heading = std::hypot(target_x - pose.x, target_y - pose.y) > 1e-9 ?
    std::atan2(target_y - pose.y, target_x - pose.x) : b.theta;
}

}  // namespace limo_dwb_critics
