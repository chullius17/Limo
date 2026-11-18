"""Translation of mppi_cost_critics.cpp, using explicit costmap adapters.

Obstacle costs average future center costs / 254. Collision checks also cover
initial poses. Path scoring uses meters and radians and optional grid detours.
"""
from bisect import bisect_right
from copy import deepcopy
from contextlib import nullcontext
import math
import time
from ...include.python_ver.mppi_cost_critics import MppiObstacleParameters, MppiPathParameters
from ...include.python_ver.mpc_timing_scope import MpcTimingScope
from ...include.python_ver.ros_compat import (
    TrajectoryCritic, Pose2D, ObstacleLayer, IllegalTrajectoryException, read_parameter,
    LETHAL_OBSTACLE, NO_INFORMATION, INSCRIBED_INFLATED_OBSTACLE, clamp, wrappedAngle,
)
from ...include.python_ver.nav2_compat import (
    closestPathPoseIndex, footprintCells, footprintCostAtPose, transformFootprint,
)
from .obstacle_guidance import ObstacleGuidance
from .smac_guidance import plan_smac, smac_reference


def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def angleError(a, b):
    return abs(wrappedAngle(a - b))


def readNonnegative(node, name, value):
    value = read_parameter(node, name, value)
    if not math.isfinite(value) or value < 0:
        raise ValueError('Cost parameter must be finite and nonnegative: ' + name)
    return value


def footprintNearObstacle(pose, footprint, grid, margin, oriented, marked_only=False):
    if (margin <= 0 and not marked_only) or len(footprint) < 3:
        return False
    oriented[:] = transformFootprint(pose, footprint)
    reach = margin + grid.getResolution() * math.sqrt(0.5)
    first_x, first_y = grid.worldToMapEnforceBounds(min(p.x for p in oriented) - reach, min(p.y for p in oriented) - reach)
    last_x, last_y = grid.worldToMapEnforceBounds(max(p.x for p in oriented) + reach, max(p.y for p in oriented) + reach)
    for y in range(first_y, last_y + 1):
        for x in range(first_x, last_x + 1):
            cost = grid.getCost(x, y)
            if cost < LETHAL_OBSTACLE or (marked_only and cost != LETHAL_OBSTACLE):
                continue
            wx, wy = grid.mapToWorld(x, y)
            inside = False
            for i, b in enumerate(oriented):
                a = oriented[i - 1]
                dx, dy = b.x - a.x, b.y - a.y
                length_squared = dx * dx + dy * dy
                fraction = clamp(((wx - a.x) * dx + (wy - a.y) * dy) / length_squared, 0, 1) if length_squared > 0 else 0
                ex, ey = wx - a.x - fraction * dx, wy - a.y - fraction * dy
                if ex * ex + ey * ey <= reach * reach:
                    return True
                if (a.y > wy) != (b.y > wy) and wx < a.x + (wy - a.y) * dx / dy:
                    inside = not inside
            if inside:
                return True
    return False


class MppiObstacleCritic(TrajectoryCritic):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.__dict__.update(vars(MppiObstacleParameters()))

    def getScale(self): return self.scale_
    def enforcesFullFootprint(self): return self.full_footprint_check_

    def onInit(self):
        prefix = self.dwb_plugin_name_ + '.' + self.name_ + '.'
        self.critical_cost_ = readNonnegative(self.nh_, prefix + 'critical_cost', self.critical_cost_)
        self.near_goal_distance_ = readNonnegative(self.nh_, prefix + 'near_goal_distance', self.near_goal_distance_)
        self.full_footprint_check_ = read_parameter(self.nh_, prefix + 'full_footprint_check', False)

    def prepare(self, pose, velocity, goal, path):
        self.near_goal_ = distance(pose, goal) < self.near_goal_distance_
        ready = self.costmap_ros_ is not None and len(self.costmap_ros_.getRobotFootprint()) >= 3
        if ready and self.full_footprint_check_:
            self.collision_footprint_ = self.costmap_ros_.getRobotFootprint()
            self.footprint_radius_ = max((math.hypot(v.x, v.y) for v in self.collision_footprint_), default=0.0)
            grid = self.costmap_
            stride = grid.getSizeInCellsX() + 1
            self.blocked_prefix_ = [0] * (stride * (grid.getSizeInCellsY() + 1))
            for y in range(grid.getSizeInCellsY()):
                row = 0
                for x in range(grid.getSizeInCellsX()):
                    row += grid.getCost(x, y) >= LETHAL_OBSTACLE
                    self.blocked_prefix_[(y + 1) * stride + x + 1] = self.blocked_prefix_[y * stride + x + 1] + row
        return ready

    def scorePose(self, pose):
        cost = footprintCostAtPose(self.costmap_, pose, self.costmap_ros_.getRobotFootprint())
        if cost < 0 or cost >= LETHAL_OBSTACLE:
            raise IllegalTrajectoryException(self.name_, 'Footprint collision or outside costmap')
        return cost

    def scoreTrajectory(self, trajectory):
        poses, grid = trajectory.poses, self.costmap_
        if len(poses) < 2:
            raise IllegalTrajectoryException(self.name_, 'Empty prediction')
        if self.full_footprint_check_ and len(self.blocked_prefix_) != (grid.getSizeInCellsX() + 1) * (grid.getSizeInCellsY() + 1):
            raise IllegalTrajectoryException(self.name_, 'Footprint collision data not prepared')

        def check_footprint(pose):
            cells = footprintCells(grid, pose, self.collision_footprint_, self.blocked_prefix_)
            if cells is None:
                raise IllegalTrajectoryException(self.name_, 'Footprint outside costmap')
            self.collision_cells_ = cells
            if any(grid.getCost(x, y) >= LETHAL_OBSTACLE for x, y in cells):
                raise IllegalTrajectoryException(self.name_, 'Full footprint in lethal/unknown cell')

        total = 0.0
        for t, pose in enumerate(poses):
            if self.full_footprint_check_:
                if t == 0:
                    check_footprint(pose)
                else:
                    previous = poses[t - 1]
                    angle = wrappedAngle(pose.theta - previous.theta)
                    travel = distance(pose, previous) + self.footprint_radius_ * abs(angle)
                    steps = max(1, math.ceil(travel / (0.5 * grid.getResolution())))
                    for step in range(1, steps + 1):
                        f = step / steps
                        check_footprint(Pose2D(previous.x + f * (pose.x - previous.x),
                            previous.y + f * (pose.y - previous.y), previous.theta + f * angle))
            else:
                self.scorePose(pose)
            cell = grid.worldToMap(pose.x, pose.y)
            if cell is None:
                raise IllegalTrajectoryException(self.name_, 'Prediction leaves costmap')
            cost = grid.getCost(*cell)
            if cost in (LETHAL_OBSTACLE, NO_INFORMATION):
                raise IllegalTrajectoryException(self.name_, 'Prediction center in obstacle or unknown')
            if t == 0:
                continue
            if cost >= INSCRIBED_INFLATED_OBSTACLE:
                total += self.critical_cost_
            elif not self.near_goal_:
                total += cost
        return total / (254.0 * (len(poses) - 1))


class MppiPathCritic(TrajectoryCritic):
    def __init__(self, *args, smac_planner=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.__dict__.update(vars(MppiPathParameters()))
        self.guidance_ = ObstacleGuidance()
        self.smac_planner_ = smac_planner

    def setTimingEnabled(self, enabled): self.timing_enabled_ = enabled
    def resetTiming(self):
        self.guidance_grid_ms_ = self.guidance_plan_ms_ = 0.0
        self.grid_updated_ = self.guidance_plan_called_ = self.guidance_plan_failed_ = False
    def guidanceGridMs(self): return self.guidance_grid_ms_
    def guidancePlanMs(self): return self.guidance_plan_ms_
    def guidancePlanCalled(self): return self.guidance_plan_called_
    def guidancePlanFailed(self): return self.guidance_plan_failed_
    def usesSmac(self): return self.guidance_use_smac_
    def gridUpdated(self): return self.grid_updated_
    def detouring(self): return self.guidance_active_
    def setTrackedReference(self, enabled): self.tracked_reference_ = enabled
    def reset(self):
        self.guidance_active_ = False
        self.smac_path_, self.smac_lengths_ = [], []

    def guidanceReference(self, pose):
        if not self.guidance_active_:
            return []
        if self.guidance_use_smac_:
            return deepcopy(self.smac_path_)
        grid = self.costmap_ros_.getCostmap()
        cell = grid.worldToMap(pose.x, pose.y)
        if cell is None:
            return []
        result = [Pose2D(*grid.mapToWorld(x, y)) for x, y in self.guidance_.route(*cell, pose.theta)]
        for a, b in zip(result, result[1:]):
            a.theta = math.atan2(b.y-a.y, b.x-a.x)
        if result:
            result[-1].theta = self.path_[self.target_index_].theta
        return result

    def onInit(self):
        prefix = self.dwb_plugin_name_ + '.' + self.name_ + '.'
        parameters = {
            'GoalCritic.cost_weight': 'goal_weight_', 'GoalCritic.threshold_to_consider': 'goal_distance_',
            'GoalAngleCritic.cost_weight': 'goal_angle_weight_', 'GoalAngleCritic.threshold_to_consider': 'goal_angle_distance_',
            'PathAlignCritic.cost_weight': 'align_weight_', 'PathAlignCritic.threshold_to_consider': 'align_distance_',
            'PathAlignCritic.max_path_occupancy_ratio': 'max_path_occupancy_ratio_',
            'PathFollowCritic.cost_weight': 'follow_weight_', 'PathFollowCritic.threshold_to_consider': 'follow_distance_',
            'PathFollowCritic.lookahead_distance': 'lookahead_distance_',
            'PathAngleCritic.cost_weight': 'angle_weight_', 'PathAngleCritic.threshold_to_consider': 'angle_distance_',
            'PathAngleCritic.max_angle_to_furthest': 'max_angle_to_furthest_',
            'ObstacleGuidance.clearance_margin': 'guidance_clearance_margin_',
            'ObstacleGuidance.trigger_margin': 'guidance_trigger_margin_',
            'ObstacleGuidance.cost_weight': 'guidance_cost_weight_',
            'ObstacleGuidance.heading_weight': 'guidance_heading_weight_',
            'ObstacleGuidance.rejoin_distance': 'guidance_rejoin_distance_',
        }
        for key, attribute in parameters.items():
            setattr(self, attribute, readNonnegative(self.nh_, prefix + key, getattr(self, attribute)))
        for key, attribute in (('PathAlignCritic.use_path_orientations', 'use_path_orientations_'),
                ('PathAngleCritic.forward_preference', 'forward_preference_'), ('ObstacleGuidance.enabled', 'guidance_enabled_'),
                ('ObstacleGuidance.use_smac', 'guidance_use_smac_')):
            setattr(self, attribute, read_parameter(self.nh_, prefix + key, getattr(self, attribute)))
        self.guidance_trigger_layer_name_ = read_parameter(self.nh_, prefix + 'ObstacleGuidance.trigger_layer', '')
        if self.guidance_enabled_ and self.guidance_trigger_layer_name_:
            self.guidance_trigger_layer_ = next((layer for layer in self.costmap_ros_.getLayeredCostmap().getPlugins()
                if isinstance(layer, ObstacleLayer) and layer.getName() == self.guidance_trigger_layer_name_), None)
            if self.guidance_trigger_layer_ is None:
                raise ValueError('ObstacleGuidance.trigger_layer must name an ObstacleLayer: ' + self.guidance_trigger_layer_name_)
        if self.max_path_occupancy_ratio_ > 1 or self.lookahead_distance_ <= 0:
            raise ValueError('Invalid MPPI path occupancy ratio or lookahead')
        if self.guidance_enabled_ and self.guidance_use_smac_ and self.smac_planner_ is None:
            raise ValueError('Python SMAC mode requires an injected Nav2 createPlan adapter; use the C++ plugin for ROS')

    def prepare(self, pose, velocity, goal, path):
        self.resetTiming()
        self.goal_, self.distance_to_goal_, self.path_ = deepcopy(goal), distance(pose, goal), deepcopy(path.poses)
        was_detouring, self.guidance_active_ = self.guidance_active_, False
        if not self.path_:
            return False
        self.path_lengths_ = [0.0] * len(self.path_)
        for i in range(1, len(self.path_)):
            self.path_lengths_[i] = self.path_lengths_[i - 1] + distance(self.path_[i - 1], self.path_[i])
        start_end = bisect_right(self.path_lengths_, 0.50) if self.tracked_reference_ else len(self.path_)
        self.start_index_ = closestPathPoseIndex(self.path_, pose, 0, start_end)
        nearest = distance(pose, self.path_[self.start_index_])
        self.target_index_ = self.start_index_
        while (self.target_index_ + 1 < len(self.path_) and
                self.path_lengths_[self.target_index_] - self.path_lengths_[self.start_index_] < self.lookahead_distance_):
            self.target_index_ += 1
        grid = self.costmap_ros_.getCostmap()
        occupied = 0
        for point in self.path_[self.start_index_:self.target_index_ + 1]:
            cell = grid.worldToMap(point.x, point.y)
            occupied += cell is None or grid.getCost(*cell) >= INSCRIBED_INFLATED_OBSTACLE
        self.path_blocked_ = occupied / (self.target_index_ - self.start_index_ + 1) > self.max_path_occupancy_ratio_
        if self.guidance_enabled_:
            footprint = self.costmap_ros_.getRobotFootprint()
            radius = self.costmap_ros_.getLayeredCostmap().getInscribedRadius()
            with MpcTimingScope(self if self.timing_enabled_ else None, 'guidance_grid_ms_'):
                self.guidance_.update(grid.getSizeInCellsX(), grid.getSizeInCellsY(), grid.getResolution(),
                    grid.getCharMap(), radius + self.guidance_clearance_margin_, self.guidance_cost_weight_)
                self.grid_updated_ = True

            def free_point(point):
                cell = grid.worldToMap(point.x, point.y)
                return (cell is not None and self.guidance_.traversable(*cell) and
                    (not self.guidance_use_smac_ or 0 <= footprintCostAtPose(grid, point, footprint) < LETHAL_OBSTACLE))

            def collision_free(point):
                cell = grid.worldToMap(point.x, point.y)
                if cell is None or grid.getCost(*cell) >= LETHAL_OBSTACLE:
                    return False
                cost = footprintCostAtPose(grid, point, footprint)
                return 0 <= cost < LETHAL_OBSTACLE

            blocked, previous = False, pose
            sensor = self.guidance_trigger_layer_
            trigger_grid = sensor if sensor is not None else grid
            with sensor.getMutex() if sensor is not None else nullcontext():
                for i in range(self.start_index_, self.target_index_ + 1):
                    if blocked or (sensor is not None and not sensor.enabled):
                        break
                    target = self.path_[i]
                    steps = max(1, math.ceil(distance(previous, target) / grid.getResolution()))
                    for step in range(steps + 1):
                        f = step / steps
                        point = Pose2D(previous.x + f * (target.x - previous.x), previous.y + f * (target.y - previous.y),
                                       previous.theta + f * wrappedAngle(target.theta - previous.theta))
                        nearby = footprintNearObstacle(point, footprint, trigger_grid,
                            self.guidance_trigger_margin_, self.proximity_footprint_, sensor is not None)
                        if nearby or (sensor is None and not collision_free(point)):
                            blocked = True
                            break
                    previous = target
            if blocked or (was_detouring and nearest > 0.15):
                self.guidance_plan_called_ = self.guidance_plan_failed_ = True
                plan_start = time.monotonic() if self.timing_enabled_ else 0.0
                self.path_blocked_ = True
                rejoin = self.target_index_
                while rejoin < len(self.path_) and not free_point(self.path_[rejoin]):
                    rejoin += 1
                if rejoin < len(self.path_):
                    first_free = rejoin
                    while (rejoin + 1 < len(self.path_) and free_point(self.path_[rejoin + 1]) and
                            self.path_lengths_[rejoin + 1] - self.path_lengths_[first_free] <= self.guidance_rejoin_distance_):
                        rejoin += 1
                cell = grid.worldToMap(self.path_[rejoin].x, self.path_[rejoin].y) if rejoin < len(self.path_) else None
                try:
                    if cell is not None:
                        if self.guidance_use_smac_:
                            self.guidance_active_ = plan_smac(self, pose, self.path_[rejoin])
                        elif self.guidance_.setTarget(*cell):
                            px = (pose.x - grid.getOriginX()) / grid.getResolution() - 0.5
                            py = (pose.y - grid.getOriginY()) / grid.getResolution() - 0.5
                            self.guidance_active_ = math.isfinite(self.guidance_.distance(px, py))
                    if self.guidance_active_:
                        self.target_index_ = rejoin
                        self.guidance_plan_failed_ = False
                    elif self.guidance_use_smac_:
                        raise IllegalTrajectoryException(self.name_, 'Local SMAC found no valid rejoin route')
                finally:
                    if self.timing_enabled_:
                        self.guidance_plan_ms_ = (time.monotonic()-plan_start)*1000
        target = self.path_[self.target_index_]
        angle = angleError(pose.theta, math.atan2(target.y - pose.y, target.x - pose.x))
        if not self.forward_preference_:
            angle = min(angle, math.pi - angle)
        self.apply_path_angle_ = angle > self.max_angle_to_furthest_
        return True

    def scoreTrajectory(self, trajectory):
        if not self.path_ or len(trajectory.poses) < 2:
            raise IllegalTrajectoryException(self.name_, 'Missing path or prediction')
        goal_cost = goal_angle_cost = align_cost = traveled = 0.0
        index = self.start_index_
        for t in range(1, len(trajectory.poses)):
            pose = trajectory.poses[t]
            if self.guidance_active_:
                grid = self.costmap_ros_.getCostmap()
                cell = grid.worldToMap(pose.x, pose.y)
                if cell is None or not self.guidance_.traversable(*cell):
                    raise IllegalTrajectoryException(self.name_, 'Prediction leaves detour clearance')
            if not self.guidance_active_ and self.distance_to_goal_ < self.goal_distance_:
                goal_cost += distance(pose, self.goal_)
            if not self.guidance_active_ and self.distance_to_goal_ < self.goal_angle_distance_:
                goal_angle_cost += angleError(pose.theta, self.goal_.theta)
            if not self.path_blocked_ and self.distance_to_goal_ > self.align_distance_:
                traveled += distance(pose, trajectory.poses[t - 1])
                end = index + 1
                while (end < len(self.path_) and
                        self.path_lengths_[end] - self.path_lengths_[self.start_index_] <= traveled + 0.5):
                    end += 1
                index = closestPathPoseIndex(self.path_, pose, index, end)
                closest = distance(pose, self.path_[index])
                angle = angleError(pose.theta, self.path_[index].theta) if self.use_path_orientations_ else 0.0
                align_cost += math.hypot(closest, angle)
        count = len(trajectory.poses) - 1
        result = (self.goal_weight_ * goal_cost + self.goal_angle_weight_ * goal_angle_cost + self.align_weight_ * align_cost) / count
        last, target = trajectory.poses[-1], self.path_[self.target_index_]
        if self.guidance_active_:
            grid = self.costmap_ros_.getCostmap()
            x = (last.x - grid.getOriginX()) / grid.getResolution() - 0.5
            y = (last.y - grid.getOriginY()) / grid.getResolution() - 0.5
            if self.guidance_use_smac_:
                remaining, reference_heading = smac_reference(self, last)
            else:
                remaining = self.guidance_.distance(x, y)
            if not math.isfinite(remaining):
                raise IllegalTrajectoryException(self.name_, 'No obstacle-free route from prediction')
            result += self.follow_weight_ * remaining
            if remaining > self.angle_distance_:
                motion_heading = last.theta
                for t in range(len(trajectory.poses) - 1, 0, -1):
                    before, after = trajectory.poses[t - 1], trajectory.poses[t]
                    if distance(before, after) > 1e-6:
                        motion_heading = math.atan2(after.y - before.y, after.x - before.x)
                        break
                result += self.guidance_heading_weight_ * angleError(motion_heading,
                    reference_heading if self.guidance_use_smac_ else self.guidance_.heading(x, y, motion_heading))
        elif self.distance_to_goal_ >= self.follow_distance_:
            result += self.follow_weight_ * distance(last, target)
        if not self.guidance_active_ and self.distance_to_goal_ > self.angle_distance_ and self.apply_path_angle_:
            angle = angleError(last.theta, math.atan2(target.y - last.y, target.x - last.x))
            if not self.forward_preference_:
                angle = min(angle, math.pi - angle)
            result += self.angle_weight_ * angle
        return result
