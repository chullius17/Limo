# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Locus Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

"""Translation of the timed 2D DWB preparation boundary.

The base 2D path also calls this helper with collection disabled, selecting
None accumulators. Lock ownership and exception ordering are identical.
"""
from copy import deepcopy
from ...include.python_ver.mpc_timing_scope import MpcTimingScope
from ...include.python_ver.ros_compat import (
    Header, NoLegalTrajectoriesException, Trajectory2D, Twist2D, Twist2DStamped,
)


def compute_velocity_commands_2d(controller, pose, velocity, results=None, delegate=True):
    if not controller.timing_sample_active_ and delegate:
        return controller.baseComputeVelocityCommands2D(pose, velocity, results)
    sample = controller.timing_sample_ if controller.timing_sample_active_ else None
    node = controller.node_
    if results is not None:
        results.header = Header(stamp=node.now(), frame_id=pose.header.frame_id)
    with MpcTimingScope(sample, 'plan_prepare_ms', sample, 'plan_prepare_cpu_ms'):
        transformed_plan, goal_pose = controller.prepareGlobalPlan(pose)
    mutex = controller.costmap_ros_.getCostmap().getMutex()
    locked = False
    try:
        with MpcTimingScope(sample, 'costmap_lock_wait_ms'):
            mutex.acquire()
            locked = True
        with MpcTimingScope(sample, 'critics_prepare_ms', sample, 'critics_prepare_cpu_ms'):
            for index, critic in enumerate(controller.critics_):
                entry = sample.critic_prepare[index] if sample is not None and index < len(sample.critic_prepare) else None
                if entry is not None:
                    entry.called = True
                with MpcTimingScope(entry, 'prepare_ms', entry, 'prepare_cpu_ms'):
                    ready = critic.prepare(pose.pose, velocity, goal_pose.pose, transformed_plan)
                if not ready:
                    node.get_logger().warning('A scoring function failed to prepare')
        try:
            best = controller.coreScoringAlgorithm(pose.pose, velocity, results)
            command = Twist2DStamped(Header(stamp=node.now()), deepcopy(best.traj.velocity))
            with MpcTimingScope(sample, 'debrief_ms'):
                for critic in controller.critics_:
                    critic.debrief(command.velocity)
            mutex.release()
            locked = False
            with MpcTimingScope(sample, 'dwb_publish_ms'):
                controller.pub_.publishLocalPlan(pose.header, best.traj)
                controller.pub_.publishCostGrid(controller.costmap_ros_, controller.critics_)
            return command
        except NoLegalTrajectoriesException:
            with MpcTimingScope(sample, 'debrief_ms'):
                for critic in controller.critics_:
                    critic.debrief(Twist2D())
            if locked:
                mutex.release()
                locked = False
            with MpcTimingScope(sample, 'dwb_publish_ms'):
                controller.pub_.publishLocalPlan(pose.header, Trajectory2D())
                controller.pub_.publishCostGrid(controller.costmap_ros_, controller.critics_)
            raise
    finally:
        if locked:
            mutex.release()
