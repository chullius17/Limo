/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

// Adapted from DWBLocalPlanner::computeVelocityCommands (Foxy):
// https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp
// Keep Foxy's ordering, mutex lifetime, publications and exception handling.
// Only the 2D preparation boundary is reproduced; the base 3D wrapper and the
// MPC scoring implementation remain responsible for their existing work.
#include "limo_dwb_critics/ackermann_mpc_controller.hpp"

#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "limo_dwb_critics/mpc_timing_scope.hpp"

namespace limo_dwb_critics
{
nav_2d_msgs::msg::Twist2DStamped AckermannMPCController::computeVelocityCommands(
  const nav_2d_msgs::msg::Pose2DStamped & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results)
{
  if (!timing_sample_active_) {
    return DWBLocalPlanner::computeVelocityCommands(pose, velocity, results);
  }
  if (results) {
    results->header.frame_id = pose.header.frame_id;
    results->header.stamp = node_->now();
  }

  nav_2d_msgs::msg::Path2D transformed_plan;
  nav_2d_msgs::msg::Pose2DStamped goal_pose;
  {
    // Includes plan projection/clipping, its publication and goal transform;
    // it is not a measurement of TF waiting alone.
    MpcTimingScope timing(&timing_sample_.plan_prepare_ms, &timing_sample_.plan_prepare_cpu_ms);
    prepareGlobalPlan(pose, transformed_plan, goal_pose);
  }

  auto * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
    *(costmap->getMutex()), std::defer_lock);
  {
    MpcTimingScope timing(&timing_sample_.costmap_lock_wait_ms);
    lock.lock();
  }
  {
    MpcTimingScope timing(
      &timing_sample_.critics_prepare_ms, &timing_sample_.critics_prepare_cpu_ms);
    for (std::size_t i = 0; i < critics_.size(); ++i) {
      auto & critic = critics_[i];
      // Storage is allocated once during configure, never in this loop.
      auto * sample = i < timing_sample_.critic_prepare.size() ?
        &timing_sample_.critic_prepare[i] : nullptr;
      bool prepared;
      {
        if (sample) {sample->called = true;}
        MpcTimingScope critic_timing(
          sample ? &sample->prepare_ms : nullptr,
          sample ? &sample->prepare_cpu_ms : nullptr);
        prepared = critic->prepare(pose.pose, velocity, goal_pose.pose, transformed_plan);
      }
      if (!prepared) {
        RCLCPP_WARN(rclcpp::get_logger("DWBLocalPlanner"), "A scoring function failed to prepare");
      }
    }
  }

  try {
    dwb_msgs::msg::TrajectoryScore best = coreScoringAlgorithm(pose.pose, velocity, results);
    nav_2d_msgs::msg::Twist2DStamped cmd_vel;
    cmd_vel.header.stamp = node_->now();
    cmd_vel.velocity = best.traj.velocity;
    {
      MpcTimingScope timing(&timing_sample_.debrief_ms);
      for (const auto & critic : critics_) {critic->debrief(cmd_vel.velocity);}
    }
    lock.unlock();
    {
      MpcTimingScope timing(&timing_sample_.dwb_publish_ms);
      pub_->publishLocalPlan(pose.header, best.traj);
      pub_->publishCostGrid(costmap_ros_, critics_);
    }
    return cmd_vel;
  } catch (const dwb_core::NoLegalTrajectoriesException &) {
    nav_2d_msgs::msg::Twist2D empty_cmd;
    dwb_msgs::msg::Trajectory2D empty_traj;
    {
      MpcTimingScope timing(&timing_sample_.debrief_ms);
      for (const auto & critic : critics_) {critic->debrief(empty_cmd);}
    }
    lock.unlock();
    {
      MpcTimingScope timing(&timing_sample_.dwb_publish_ms);
      pub_->publishLocalPlan(pose.header, empty_traj);
      pub_->publishCostGrid(costmap_ros_, critics_);
    }
    throw;
  }
}
}  // namespace limo_dwb_critics
