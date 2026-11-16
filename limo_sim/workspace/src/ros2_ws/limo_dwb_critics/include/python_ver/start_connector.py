"""Translation of the header-only start_connector.hpp.

The gate belongs to a complete plan and only hands off once. Being close to the
virtual start cannot skip a Dubins loop or reactivate a completed exception.
"""
from copy import deepcopy
import math
from .ros_compat import Path, StartConnector, Point, getYaw, wrappedAngle


class StartConnectorGate:
    def __init__(self):
        self.plan_, self.metadata_, self.previous_ = Path(), StartConnector(), Point()
        self.lengths_, self.index_, self.finished_ = [], 0, False

    @staticmethod
    def distance(a, b):
        return math.hypot(a.x - b.x, a.y - b.y)

    @staticmethod
    def angleError(a, b):
        return abs(wrappedAngle(a - b))

    @staticmethod
    def samePath(a, b):
        if (a.header.frame_id != b.header.frame_id or a.header.stamp != b.header.stamp or
                len(a.poses) != len(b.poses)):
            return False
        for pa, pb in zip(a.poses, b.poses):
            p, q = pa.pose, pb.pose
            if (not all(math.isfinite(v) for v in (p.position.x, p.position.y, q.position.x, q.position.y)) or
                    StartConnectorGate.distance(p.position, q.position) > 1e-8 or
                    StartConnectorGate.angleError(getYaw(p.orientation), getYaw(q.orientation)) > 1e-8):
                return False
        return True

    def resetProgress(self):
        self.finished_, self.index_, self.lengths_ = False, 0, []

    def setPlan(self, path):
        if not self.samePath(self.plan_, path):
            self.resetProgress()
        self.plan_ = deepcopy(path)

    def setMetadata(self, metadata):
        if (self.metadata_.path.header.stamp != metadata.path.header.stamp or
                self.metadata_.end_index != metadata.end_index or
                not self.samePath(self.metadata_.path, metadata.path)):
            self.resetProgress()
        self.metadata_ = deepcopy(metadata)

    def prefix(self):
        path = deepcopy(self.plan_)
        end = self.metadata_.end_index + 1
        path.poses = path.poses[:end]
        # C++ vector.resize also creates default poses when called out of sequence.
        if end > len(path.poses):
            from .ros_compat import PoseStamped
            path.poses.extend(PoseStamped() for _ in range(end - len(path.poses)))
        return path

    def normalPlan(self):
        path = deepcopy(self.plan_)
        if (self.finished_ and self.metadata_.end_index < len(path.poses) and
                self.samePath(self.plan_, self.metadata_.path)):
            path.poses = path.poses[self.metadata_.end_index:]
        return path

    def update(self, pose, position_tolerance, yaw_tolerance):
        end_index = self.metadata_.end_index
        if (self.finished_ or not self.plan_.poses or end_index == 0 or
                end_index >= len(self.plan_.poses) or not self.samePath(self.plan_, self.metadata_.path)):
            return False
        if not self.lengths_:
            self.lengths_.append(0.0)
            for i in range(1, end_index + 1):
                self.lengths_.append(self.lengths_[-1] + self.distance(
                    self.plan_.poses[i - 1].pose.position, self.plan_.poses[i].pose.position))
            self.previous_ = deepcopy(self.plan_.poses[0].pose.position)
        available = self.lengths_[self.index_] + self.distance(self.previous_, pose.position) + 0.10
        self.previous_ = deepcopy(pose.position)
        nearest = math.inf
        for i in range(self.index_, len(self.lengths_)):
            if self.lengths_[i] > available:
                break
            d = self.distance(pose.position, self.plan_.poses[i].pose.position)
            if d < nearest:
                nearest, self.index_ = d, i
        end = self.plan_.poses[end_index].pose
        if (self.lengths_[-1] - self.lengths_[self.index_] <= position_tolerance and
                self.distance(pose.position, end.position) <= position_tolerance and
                self.angleError(getYaw(pose.orientation), getYaw(end.orientation)) <= yaw_tolerance):
            self.finished_ = True
            return False
        return True
