#ifndef LIMO_DWB_CRITICS__ACKERMANN_MPC_CONTROLLER_HPP_
#define LIMO_DWB_CRITICS__ACKERMANN_MPC_CONTROLLER_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "dwb_core/dwb_local_planner.hpp"
#include "limo_dwb_critics/sampling_mpc.hpp"
#include "limo_dwb_critics/mpc_debug.hpp"
#include "limo_dwb_critics/start_connector.hpp"
#include "limo_inflation/path_distance_layer.hpp"

namespace limo_dwb_critics
{

class AckermannMPCController : public dwb_core::DWBLocalPlanner
{
public:
  using dwb_core::DWBLocalPlanner::computeVelocityCommands;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;

protected:
  nav_2d_msgs::msg::Path2D transformGlobalPlan(
    const nav_2d_msgs::msg::Pose2DStamped & pose) override;
  dwb_msgs::msg::TrajectoryScore coreScoringAlgorithm(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D velocity,
    std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results) override;

  std::unique_ptr<SamplingMpc> mpc_;
  virtual std::int64_t controlTimeNs() const;
  bool open_loop_{true};
  double steering_feedback_min_velocity_{0.05};

private:
  void setTrackingPlan(const nav_msgs::msg::Path & path);
  nav_2d_msgs::msg::Path2D tracking_plan_;
  std::vector<limo_inflation::PathPosePoint> tracking_points_;
  limo_inflation::PathProgressTracker progress_tracker_;
  std::mutex connector_mutex_;
  StartConnectorGate connector_gate_;
  nav_msgs::msg::Path complete_plan_;
  bool connector_enabled_{false};
  bool connector_active_{false};
  double connector_position_tolerance_{0.05};
  double connector_yaw_tolerance_{0.10};
  rclcpp::Subscription<limo_interfaces::msg::StartConnector>::SharedPtr connector_subscription_;
  void resetPrediction();
  dwb_msgs::msg::Trajectory2D toTrajectory(const MpcRollout & rollout) const;
  std::int64_t last_time_ns_{0};
  std::int64_t last_ros_time_ns_{0};
  bool have_time_{false};
  bool have_command_{false};
  MpcControl previous_command_;
  std::unique_ptr<MpcDebugPublisher> debug_publisher_;
  std::shared_ptr<limo_inflation::PathDistanceLayer> tube_layer_;
  bool tube_required_{false};
  bool tube_start_pending_{true};
  double progress_weight_{2.0};
  double progress_distance_{1.0};
  double terminal_heading_weight_{1.0};
};

}  // namespace limo_dwb_critics
#endif  // LIMO_DWB_CRITICS__ACKERMANN_MPC_CONTROLLER_HPP_
