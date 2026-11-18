"""SMAC reference helpers corresponding to smac_guidance.cpp.

The standalone translation needs an injected, configured Nav2 planner exposing
createPlan(start, goal). Actual SMAC runs in the C++ ROS plugin; no substitute
search is silently presented as SMAC here.
"""
import math
from ...include.python_ver.ros_compat import Header, Pose, PoseStamped, Point, Quaternion, poseToPose2D


def plan_smac(critic, start, goal):
    critic.smac_path_, critic.smac_lengths_ = [], []
    def stamped(pose):
        return PoseStamped(Header(stamp=critic.nh_.now(), frame_id=critic.costmap_ros_.getGlobalFrameID()),
            Pose(Point(pose.x, pose.y), Quaternion(z=math.sin(pose.theta / 2), w=math.cos(pose.theta / 2))))
    route = critic.smac_planner_.createPlan(stamped(start), stamped(goal))
    if not route.poses or route.header.frame_id != critic.costmap_ros_.getGlobalFrameID():
        return False
    grid = critic.costmap_ros_.getCostmap()
    previous = start
    critic.smac_path_, critic.smac_lengths_ = [start], [0.0]
    for stamped_pose in route.poses:
        point = poseToPose2D(stamped_pose.pose)
        if not all(math.isfinite(v) for v in (point.x, point.y, point.theta)):
            return False
        length = math.hypot(point.x - previous.x, point.y - previous.y)
        if length < 1e-9:
            continue
        steps = max(1, math.ceil(length / grid.getResolution()))
        segment_cost = 0.0
        for i in range(steps + 1):
            t = i / steps
            cell = grid.worldToMap(previous.x + t * (point.x - previous.x), previous.y + t * (point.y - previous.y))
            if cell is None or not critic.guidance_.traversable(*cell):
                return False
            penalty = 1 + critic.guidance_cost_weight_ * min(252, grid.getCost(*cell)) / 252
            segment_cost += penalty * (0.5 if i in (0, steps) else 1.0)
        critic.smac_path_.append(point)
        critic.smac_lengths_.append(critic.smac_lengths_[-1] + length * segment_cost / steps)
        previous = point
    return len(critic.smac_path_) >= 2


def smac_reference(critic, pose):
    best_distance, best_segment, best_t = math.inf, 0, 0.0
    for i in range(1, len(critic.smac_path_)):
        a, b = critic.smac_path_[i - 1:i + 1]
        dx, dy = b.x - a.x, b.y - a.y
        t = min(1.0, max(0.0, ((pose.x-a.x)*dx + (pose.y-a.y)*dy)/(dx*dx+dy*dy)))
        distance = math.hypot(pose.x-a.x-t*dx, pose.y-a.y-t*dy)
        if distance < best_distance - 1e-9:
            best_distance, best_segment, best_t = distance, i, t
    if not best_segment:
        return math.inf, pose.theta
    a, b = critic.smac_path_[best_segment - 1:best_segment + 1]
    x, y = a.x + best_t*(b.x-a.x), a.y + best_t*(b.y-a.y)
    progress = critic.smac_lengths_[best_segment-1] + best_t*(critic.smac_lengths_[best_segment]-critic.smac_lengths_[best_segment-1])
    remaining = best_distance + critic.smac_lengths_[-1] - progress
    lookahead, from_x, from_y = 0.35, x, y
    target_x, target_y = b.x, b.y
    for point in critic.smac_path_[best_segment:]:
        length = math.hypot(point.x-from_x, point.y-from_y)
        t = min(1.0, lookahead/length) if length > 1e-9 else 1.0
        target_x, target_y = from_x+t*(point.x-from_x), from_y+t*(point.y-from_y)
        if length >= lookahead:
            break
        lookahead -= length
        from_x, from_y = point.x, point.y
    heading = math.atan2(target_y-pose.y, target_x-pose.x) if math.hypot(target_x-pose.x, target_y-pose.y)>1e-9 else b.theta
    return remaining, heading
