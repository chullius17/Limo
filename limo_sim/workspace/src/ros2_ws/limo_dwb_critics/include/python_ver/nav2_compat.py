"""Local geometry helpers for external Nav2 and limo_inflation dependencies.

Path association follows path_pose_association.hpp. Rasterization uses integer
polygon outlines and vertical spans for convex footprints. These educational
substitutes require validation before use with a real Nav2 costmap adapter.
"""
from copy import deepcopy
from dataclasses import dataclass
import math
from .ros_compat import Point, Pose2D, clamp, wrappedAngle


def closestPathPoseIndex(poses, pose, begin, end):
    nearest = min((math.hypot(pose.x - p.x, pose.y - p.y) for p in poses[begin:end]), default=math.inf)
    index, best = begin, math.inf
    for i in range(begin, end):
        d = math.hypot(pose.x - poses[i].x, pose.y - poses[i].y)
        if d > nearest + 0.10:
            continue
        cost = d * d + (0.25 * wrappedAngle(pose.theta - poses[i].theta)) ** 2
        if cost < best - 1e-9:
            best, index = cost, i
    return index


def pathPosePoints(poses):
    points, arc = [], 0.0
    for pose in poses:
        if points:
            arc += math.hypot(pose.x - points[-1][0], pose.y - points[-1][1])
        points.append((pose.x, pose.y, arc, pose.theta))
    return points


@dataclass
class PathPoseProjection:
    progress: float = 0.0
    heading: float = 0.0
    distance: float = 0.0
    segment: int = 0


def projectPathPose(points, pose, minimum=-1e30, maximum=1e30, preferred_progress=0.0):
    """Return a projection or None; the C++ result output reference becomes a return value."""
    if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.theta)) or minimum > maximum:
        return None
    if len(points) == 1 and minimum <= 0 <= maximum:
        p = points[0]
        return PathPoseProjection(0.0, p[3], math.hypot(pose.x - p[0], pose.y - p[1]), 0)
    nearest = best = continuity = math.inf
    result = None
    for pass_index in range(2):
        for i in range(1, len(points)):
            a, b = points[i - 1], points[i]
            length = b[2] - a[2]
            if length <= 1e-9 or b[2] < minimum or a[2] > maximum:
                continue
            low, high = clamp((minimum - a[2]) / length, 0, 1), clamp((maximum - a[2]) / length, 0, 1)
            dx, dy = b[0] - a[0], b[1] - a[1]
            f = clamp(((pose.x - a[0]) * dx + (pose.y - a[1]) * dy) / (length * length), low, high)
            d = math.hypot(pose.x - a[0] - f * dx, pose.y - a[1] - f * dy)
            if pass_index == 0:
                nearest = min(nearest, d)
                continue
            if d > nearest + 0.10:
                continue
            heading = wrappedAngle(a[3] + f * wrappedAngle(b[3] - a[3]))
            progress = a[2] + f * length
            cost = d * d + (0.25 * wrappedAngle(pose.theta - heading)) ** 2
            delta = abs(progress - preferred_progress)
            if cost < best - 1e-9 or (abs(cost - best) <= 1e-9 and delta < continuity):
                best, continuity = cost, delta
                result = PathPoseProjection(progress, heading, d, i - 1)
    return result


class PathProgressTracker:
    def __init__(self):
        self.previous_ = Pose2D()
        self.reset()

    def reset(self):
        self.valid_, self.progress_ = False, 0.0

    def valid(self): return self.valid_
    def progress(self): return self.progress_

    def update(self, points, pose):
        movement = math.hypot(pose.x - self.previous_.x, pose.y - self.previous_.y)
        minimum = self.progress_ - 0.10 if self.valid_ else -1e30
        maximum = self.progress_ + max(0.50, 2 * movement) if self.valid_ else 1e30
        result = projectPathPose(points, pose, minimum, maximum, self.progress_)
        if result is None:
            return None
        self.progress_ = max(self.progress_, result.progress)
        self.previous_, self.valid_ = deepcopy(pose), True
        return result


def transformFootprint(pose, footprint):
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    return [Point(pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y) for p in footprint]


def lineCells(a, b):
    """Rasterize a segment with Nav2's dominant-axis Bresenham convention."""
    x, y = a
    dx, dy = b[0] - x, b[1] - y
    abs_dx, abs_dy = abs(dx), abs(dy)
    sx, sy = (1 if dx >= 0 else -1), (1 if dy >= 0 else -1)
    if abs_dx >= abs_dy:
        error = abs_dx // 2
        for _ in range(abs_dx):
            yield x, y
            x += sx
            error += abs_dy
            if error >= abs_dx:
                y += sy
                error -= abs_dx
    else:
        error = abs_dy // 2
        for _ in range(abs_dy):
            yield x, y
            y += sy
            error += abs_dx
            if error >= abs_dy:
                x += sx
                error -= abs_dy
    yield b


def footprintCells(grid, pose, footprint, blocked_prefix=None, fill=True):
    """Return occupied cell coordinates, or None for an invalid/outside polygon."""
    if len(footprint) < 3 or not all(math.isfinite(v) for v in (pose.x, pose.y, pose.theta)):
        return None
    polygon = [grid.worldToMap(p.x, p.y) for p in transformFootprint(pose, footprint)]
    if any(p is None for p in polygon):
        return None
    if blocked_prefix is not None:
        xs, ys = zip(*polygon)
        x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
        stride, p = grid.getSizeInCellsX() + 1, blocked_prefix
        if p[(y1 + 1) * stride + x1 + 1] + p[y0 * stride + x0] == p[y0 * stride + x1 + 1] + p[(y1 + 1) * stride + x0]:
            return []
    boundary = set()
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        boundary.update(lineCells(a, b))
    if not fill:
        return sorted(boundary)
    columns = {}
    for x, y in boundary:
        columns.setdefault(x, []).append(y)
    return [(x, y) for x in sorted(columns) for y in range(min(columns[x]), max(columns[x]) + 1)]


def footprintCostAtPose(grid, pose, footprint):
    cells = footprintCells(grid, pose, footprint, fill=False)
    if cells is None:
        return -1.0
    # The footprint checker inspects the outline; center costs are checked by callers.
    maximum = 0
    for x, y in cells:
        cost = grid.getCost(x, y)
        if cost >= 254:
            return -1.0
        maximum = max(maximum, cost)
    return float(maximum)
