#ifndef LIMO_DWB_CRITICS__MPPI_COST_CRITICS_HPP_
#define LIMO_DWB_CRITICS__MPPI_COST_CRITICS_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "dwb_core/trajectory_critic.hpp"
#include "dwb_critics/obstacle_footprint.hpp"
#include "limo_dwb_critics/obstacle_guidance.hpp"
#include "limo_dwb_critics/special_avoidance/smac_worker.hpp"
#include "limo_inflation/path_pose_association.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_core/global_planner.hpp"
#include "pluginlib/class_loader.hpp"

namespace limo_dwb_critics
{

// MPPI-style mean normalized center cost, with Foxy's footprint collision
// rejection at every pose. getScale deliberately bypasses DWB's resolution
// multiplier: normalization is explicit and independent of map resolution.
class MppiObstacleCritic : public dwb_critics::ObstacleFootprintCritic
{
public:
  void onInit() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & path) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override;
  double getScale() const override {return scale_;}
  bool enforcesFullFootprint() const {return full_footprint_check_;}

protected:
  double critical_cost_{300.0};
  double near_goal_distance_{0.5};
  bool near_goal_{false};
  bool full_footprint_check_{false};
  std::vector<std::uint32_t> blocked_prefix_;
  std::vector<geometry_msgs::msg::Point> collision_footprint_;
  // Reused across all intermediate poses and candidate trajectories.
  std::vector<nav2_costmap_2d::MapLocation> collision_cells_;
  std::vector<nav2_costmap_2d::MapLocation> collision_polygon_;
  double footprint_radius_{0.0};
};

// Distances in meters and angles in radians, averaged over predicted poses.
// Goal and path terms have the same weights and distance gates as the LIMO
// Humble MPPI profile. Path following uses a fixed metric lookahead instead of
// MPPI's batch-wide furthest-trajectory index, which DWB critics do not expose.
class MppiPathCritic : public dwb_core::TrajectoryCritic
{
public:
  ~MppiPathCritic() override;
  void setTimingEnabled(bool enabled) {timing_enabled_ = enabled;}
  void resetTiming()
  {
    guidance_grid_ms_ = guidance_plan_ms_ = 0.0;
    grid_updated_ = guidance_plan_called_ = guidance_plan_failed_ = false;
    guidance_plan_start_steady_ns_ = 0;
    guidance_plan_sample_epoch_ = 0;
    guidance_plan_discarded_ = false;
  }
  double guidanceGridMs() const {return guidance_grid_ms_;}
  double guidancePlanMs() const {return guidance_plan_ms_;}
  bool guidancePlanCalled() const {return guidance_plan_called_;}
  bool guidancePlanFailed() const {return guidance_plan_failed_;}
  std::int64_t guidancePlanStartSteadyNs() const {return guidance_plan_start_steady_ns_;}
  std::uint64_t guidancePlanEpoch() const
  {return guidance_use_smac_ ?
    (guidance_plan_sample_epoch_ ? guidance_plan_sample_epoch_ : guidance_plan_epoch_) : 0;}
  double guidanceReplanHz() const {return guidance_use_smac_ ? smac_replan_frequency_ : 0.0;}
  bool guidanceAsync() const {return guidance_use_smac_ && smac_async_planning_;}
  bool guidancePlanRunning() const {return smac_worker_ && smac_worker_->busy();}
  bool guidancePlanDiscarded() const {return guidance_plan_discarded_;}
  bool usesSmac() const {return guidance_use_smac_;}
  std::vector<geometry_msgs::msg::Pose2D> guidanceReference(
    const geometry_msgs::msg::Pose2D & pose) const;
  bool gridUpdated() const {return grid_updated_;}
  bool detouring() const {return guidance_active_;}
  void setTrackedReference(bool enabled) {tracked_reference_ = enabled;}
  void onInit() override;
  void reset() override;
  bool followsSmac() const {return guidance_active_ && guidance_use_smac_;}
  // DWB prepare supplies the transformed final global goal, not the local
  // reference endpoint or the temporary obstacle-avoidance target.
  double distanceToGoal() const {return distance_to_goal_;}
  const geometry_msgs::msg::Pose2D * guidanceTarget() const
  {
    if (!guidance_active_) {return nullptr;}
    return guidance_use_smac_ ? &smac_goal_ : &path_[target_index_];
  }
  const geometry_msgs::msg::Pose2D * pathLookahead() const
  {return path_lookahead_ ? &*path_lookahead_ : nullptr;}
  const geometry_msgs::msg::Pose2D * guidanceLookahead() const
  {return guidance_lookahead_ ? &*guidance_lookahead_ : nullptr;}
  const geometry_msgs::msg::Pose2D * guidanceCandidateTarget() const
  {return guidance_candidate_target_ ? &*guidance_candidate_target_ : nullptr;}
  const auto & activeGuidancePoints() const {return smac_tracking_points_;}
  bool guidanceTubeContains(const geometry_msgs::msg::Pose2D & pose, double radius) const;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & path) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override;

protected:
  void configureObstacleGuidance();
  void prepareObstacleGuidance(
    const geometry_msgs::msg::Pose2D & pose, bool was_detouring, double nearest);
  void validateGuidancePose(const geometry_msgs::msg::Pose2D & pose) const;
  double scoreObstacleGuidance(const dwb_msgs::msg::Trajectory2D & trajectory) const;
  void configureSmac();
  void resetSmacCache();
  void recordSmacCompletion(const SmacPlanResult & result, bool discarded);
  void prepareSmacAvoidance(
    const geometry_msgs::msg::Pose2D & pose, const geometry_msgs::msg::Pose2D * target,
    double target_progress);
  SmacPlanResult computeSmacPlan(const SmacPlanRequest & request);
  void smacReference(
    const geometry_msgs::msg::Pose2D & pose, double & remaining, double & heading) const;
  bool timing_enabled_{false};
  bool grid_updated_{false};
  double guidance_grid_ms_{0.0};
  double guidance_plan_ms_{0.0};
  bool guidance_plan_called_{false};
  bool guidance_plan_failed_{false};
  std::int64_t guidance_plan_start_steady_ns_{0};
  // Incremented on the first SMAC submission in a new detour.
  // This survives cache resets so consumers can delimit independent detours.
  std::uint64_t guidance_plan_epoch_{0};
  std::uint64_t guidance_plan_sample_epoch_{0};
  bool guidance_plan_discarded_{false};
  bool tracked_reference_{false};
  double goal_weight_{5.0};
  double goal_angle_weight_{3.0};
  double align_weight_{10.0};
  double follow_weight_{5.0};
  double angle_weight_{2.0};
  double goal_distance_{1.0};
  double goal_angle_distance_{0.5};
  double align_distance_{0.5};
  double follow_distance_{1.0};
  double angle_distance_{0.5};
  double max_path_occupancy_ratio_{0.15};
  double lookahead_distance_{1.25};
  double max_angle_to_furthest_{1.0};
  double guidance_clearance_margin_{0.0};
  double guidance_trigger_margin_{0.05};
  double guidance_trigger_front_extension_{0.0};
  double guidance_lookahead_distance_{1.0};
  double guidance_cost_weight_{2.0};
  double guidance_heading_weight_{2.0};
  double guidance_rejoin_distance_{0.50};
  bool guidance_enabled_{false};
  std::string guidance_trigger_layer_name_;
  std::shared_ptr<nav2_costmap_2d::CostmapLayer> guidance_trigger_layer_;
  bool guidance_use_smac_{false};
  bool guidance_active_{false};
  ObstacleGuidance guidance_;
  // The loader outlives the plugin. Dijkstra mode never constructs this plugin.
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GlobalPlanner>> smac_loader_;
  nav2_core::GlobalPlanner::Ptr smac_planner_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> smac_costmap_;
  bool smac_async_planning_{true};
  std::unique_ptr<SmacPlannerWorker> smac_worker_;
  std::uint64_t smac_generation_{1};
  bool smac_detour_pending_{false};
  double smac_replan_frequency_{4.0};
  double smac_replan_goal_distance_{0.25};
  double smac_replan_goal_angle_{0.35};
  double smac_failure_advance_distance_{0.25};
  // Per-detour submission marker; resetting it starts a new reporting epoch.
  std::optional<std::chrono::steady_clock::time_point> smac_last_plan_start_;
  // Search cadence survives detour/reference resets. Completed results replace
  // the provisional submission stamp with the actual worker start, even stale ones.
  std::optional<std::chrono::steady_clock::time_point> smac_last_search_start_;
  // Diagnostic throttle only; never participates in target selection or search cadence.
  std::optional<std::chrono::steady_clock::time_point> smac_last_target_warning_;
  std::optional<geometry_msgs::msg::Pose2D> smac_failed_goal_;
  double smac_failed_goal_progress_{0.0};
  // Retain an advanced target after success so nominal lookahead cannot undo it.
  std::optional<geometry_msgs::msg::Pose2D> smac_advanced_goal_;
  double smac_advanced_goal_progress_{0.0};
  std::vector<geometry_msgs::msg::Pose2D> smac_path_;
  std::vector<double> smac_lengths_;
  geometry_msgs::msg::Pose2D smac_goal_;
  limo_inflation::PathProgressTracker smac_progress_tracker_;
  std::vector<limo_inflation::PathPosePoint> smac_tracking_points_;
  std::vector<double> smac_tracking_lengths_;
  std::vector<geometry_msgs::msg::Point> proximity_footprint_;
  bool use_path_orientations_{true};
  bool forward_preference_{false};
  geometry_msgs::msg::Pose2D goal_;
  // Copies survive replacement of path_ by an active SMAC reference.
  std::optional<geometry_msgs::msg::Pose2D> path_lookahead_;
  std::optional<geometry_msgs::msg::Pose2D> guidance_lookahead_;
  std::optional<geometry_msgs::msg::Pose2D> guidance_candidate_target_;
  std::vector<geometry_msgs::msg::Pose2D> path_;
  std::vector<double> path_lengths_;
  std::size_t start_index_{0};
  std::size_t target_index_{0};
  double distance_to_goal_{0.0};
  bool path_blocked_{false};
  bool apply_path_angle_{false};
};

}  // namespace limo_dwb_critics
#endif  // LIMO_DWB_CRITICS__MPPI_COST_CRITICS_HPP_
