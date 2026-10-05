#ifndef LIMO_DWB_CRITICS__MPC_DEBUG_HPP_
#define LIMO_DWB_CRITICS__MPC_DEBUG_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "limo_dwb_critics/sampling_mpc.hpp"
#include "limo_interfaces/msg/mpc_debug.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace limo_dwb_critics
{

// Optional, bounded telemetry. No image rendering or TF work in the control loop.
class MpcDebugPublisher
{
public:
  MpcDebugPublisher(
    const nav2_util::LifecycleNode::SharedPtr & node, const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap, const MpcConfig & config);
  void activate();
  void deactivate();
  MpcObserver begin(const MpcState & initial);
  void finish(const MpcSolution & solution);

private:
  limo_interfaces::msg::MpcCandidate candidate(
    const MpcRollout & rollout, MpcFamily family, std::size_t id, double cost) const;
  nav2_util::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  rclcpp_lifecycle::LifecyclePublisher<limo_interfaces::msg::MpcDebug>::SharedPtr publisher_;
  MpcConfig config_;
  std::array<int, 5> totals_{};
  std::array<int, 5> seen_{};
  int samples_per_family_{6};
  int pose_stride_{2};
  std::int64_t period_ns_{0};
  std::int64_t last_publish_ns_{0};
  bool collecting_{false};
  limo_interfaces::msg::MpcDebug frame_;
};

}  // namespace limo_dwb_critics
#endif  // LIMO_DWB_CRITICS__MPC_DEBUG_HPP_
