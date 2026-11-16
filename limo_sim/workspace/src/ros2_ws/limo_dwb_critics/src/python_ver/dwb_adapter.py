"""Standalone host for the external DWBLocalPlanner calls in the translation.

This is an explicit teaching adapter, not a port of all of Nav2 DWB. It supports
critic lifecycle/scoring, frame transforms through an injected transform_pose
function, and a contiguous costmap clip. A real integration must replace it
with a host matching Foxy's transform/prune/publication behavior and ROS QoS.
"""
from copy import deepcopy
from ...include.python_ver.ros_compat import (
    Path2D, Pose2DStamped, Header, TrajectoryScore, CriticScore, PlannerException,
    read_parameter,
)
from ...include.python_ver.nav2_compat import closestPathPoseIndex


class DwbPublisher:
    def __init__(self):
        self.local_plans, self.cost_grids = [], []

    def publishLocalPlan(self, header, trajectory):
        self.local_plans.append((deepcopy(header), deepcopy(trajectory)))
        self.local_plans[:] = self.local_plans[-1:]

    def publishCostGrid(self, costmap_ros, critics):
        self.cost_grids[:] = [list(costmap_ros.getCostmap().getCharMap())]


class DwbAdapter:
    def configure(self, controller, node, name, tf, costmap_ros):
        controller.node_, controller.tf_, controller.costmap_ros_ = node, tf, costmap_ros
        controller.name_, controller.pub_ = name, DwbPublisher()
        controller.transform_tolerance_ = read_parameter(node, name + '.transform_tolerance', 0.1)
        controller.debug_trajectory_details_ = read_parameter(node, name + '.debug_trajectory_details', False)
        controller.short_circuit_trajectory_evaluation_ = read_parameter(node, name + '.short_circuit_trajectory_evaluation', True)
        for critic in controller.critics_:
            critic.nh_, critic.dwb_plugin_name_, critic.costmap_ros_ = node, name, costmap_ros
            critic.costmap_ = costmap_ros.getCostmap()
            critic.scale_ = read_parameter(node, name + '.' + critic.name_ + '.scale', critic.scale_)
            critic.onInit()
        controller.active_ = False

    def activate(self, controller): controller.active_ = True
    def deactivate(self, controller): controller.active_ = False

    def cleanup(self, controller):
        controller.active_ = False
        controller.global_plan_ = Path2D()

    def setPlan(self, controller, path):
        from ...include.python_ver.ros_compat import pathToPath2D
        controller.global_plan_ = pathToPath2D(path)
        for critic in controller.critics_:
            critic.reset()

    def transformPose(self, controller, pose, frame):
        if pose.header.frame_id == frame:
            return deepcopy(pose)
        if controller.tf_ is None:
            raise PlannerException('Cannot transform robot into reference frame')
        return controller.tf_.transform_pose(pose, frame, controller.transform_tolerance_)

    def transformGlobalPlan(self, controller, pose):
        plan = controller.global_plan_
        if not plan.poses:
            raise PlannerException('Received plan with zero length')
        global_pose = self.transformPose(controller, pose, plan.header.frame_id)
        first = closestPathPoseIndex(plan.poses, global_pose.pose, 0, len(plan.poses))
        frame = controller.costmap_ros_.getGlobalFrameID()
        grid = controller.costmap_ros_.getCostmap()
        transformed, entered = [], False
        for point in plan.poses[first:]:
            local = self.transformPose(controller, Pose2DStamped(deepcopy(plan.header), deepcopy(point)), frame).pose
            if grid.worldToMap(local.x, local.y) is None:
                if entered:
                    break
            else:
                entered = True
                transformed.append(local)
        if not transformed:
            raise PlannerException('Resulting plan has zero poses in the costmap')
        return Path2D(Header(stamp=pose.header.stamp, frame_id=frame), transformed)

    def prepareGlobalPlan(self, controller, pose):
        transformed = controller.transformGlobalPlan(pose)
        goal = Pose2DStamped(deepcopy(controller.tracking_plan_.header), deepcopy(controller.tracking_plan_.poses[-1]))
        goal = self.transformPose(controller, goal, controller.costmap_ros_.getGlobalFrameID())
        return transformed, goal

    def scoreTrajectory(self, controller, trajectory, best_score=-1.0):
        result = TrajectoryScore(traj=deepcopy(trajectory))
        for critic in controller.critics_:
            scale = critic.getScale()
            raw = critic.scoreTrajectory(trajectory) if scale != 0 else 0.0
            result.scores.append(CriticScore(critic.getName(), raw, scale))
            result.total += raw * scale
            if (controller.short_circuit_trajectory_evaluation_ and best_score > 0 and
                    result.total > best_score):
                break
        return result
