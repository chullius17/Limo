"""Small, explicit substitutes for external ROS/Nav2 types used in this port.

These objects keep the algorithms executable without ROS. They do not implement
ROS transport, TF lookup, or the C++ plugin ABI. Adapters can provide those services
through the same methods. Costmap conversion returns tuples instead of C++ output
references; a failed worldToMap returns None.
"""
from copy import deepcopy
from dataclasses import dataclass, field
import logging
import math
import threading
import time
from typing import Any, List

FREE_SPACE = 0
INSCRIBED_INFLATED_OBSTACLE = 253
LETHAL_OBSTACLE = 254
NO_INFORMATION = 255


@dataclass
class Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class Pose:
    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass
class Header:
    stamp: Any = 0
    frame_id: str = ""


@dataclass
class PoseStamped:
    header: Header = field(default_factory=Header)
    pose: Pose = field(default_factory=Pose)


@dataclass
class Pose2DStamped:
    header: Header = field(default_factory=Header)
    pose: Pose2D = field(default_factory=Pose2D)


@dataclass
class Path:
    header: Header = field(default_factory=Header)
    poses: List[PoseStamped] = field(default_factory=list)


@dataclass
class Path2D:
    header: Header = field(default_factory=Header)
    poses: List[Pose2D] = field(default_factory=list)


@dataclass
class Twist2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass
class Twist:
    linear: Point = field(default_factory=Point)
    angular: Point = field(default_factory=Point)


@dataclass
class TwistStamped:
    header: Header = field(default_factory=Header)
    twist: Twist = field(default_factory=Twist)


@dataclass
class Twist2DStamped:
    header: Header = field(default_factory=Header)
    velocity: Twist2D = field(default_factory=Twist2D)


@dataclass
class Trajectory2D:
    velocity: Twist2D = field(default_factory=Twist2D)
    poses: List[Pose2D] = field(default_factory=list)
    time_offsets: List[float] = field(default_factory=list)  # Seconds.


@dataclass
class CriticScore:
    name: str = ""
    raw_score: float = 0.0
    scale: float = 0.0


@dataclass
class TrajectoryScore:
    traj: Trajectory2D = field(default_factory=Trajectory2D)
    scores: List[CriticScore] = field(default_factory=list)
    total: float = 0.0


@dataclass
class LocalPlanEvaluation:
    header: Header = field(default_factory=Header)
    twists: List[TrajectoryScore] = field(default_factory=list)
    best_index: int = 0
    worst_index: int = 0


@dataclass
class KeyValue:
    key: str = ""
    value: str = ""


@dataclass
class DiagnosticStatus:
    OK = 0
    WARN = 1
    name: str = ""
    hardware_id: str = ""
    level: int = 0
    message: str = ""
    values: List[KeyValue] = field(default_factory=list)


@dataclass
class DiagnosticArray:
    header: Header = field(default_factory=Header)
    status: List[DiagnosticStatus] = field(default_factory=list)


class PlannerException(RuntimeError):
    """Equivalent of nav2_core::PlannerException."""


class IllegalTrajectoryException(PlannerException):
    def __init__(self, critic_name, message):
        self.critic_name = critic_name
        super().__init__(message)

    def getCriticName(self):
        return self.critic_name


class NoLegalTrajectoriesException(PlannerException):
    """Raised when no sampled candidate has a finite valid score."""


def clamp(value, low, high):
    return min(max(value, low), high)


def wrappedAngle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def getYaw(q):
    # tf2's normalized-quaternion expression also handles non-unit quaternions.
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)


def yawQuaternion(yaw):
    return Quaternion(z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def poseToPose2D(pose):
    return Pose2D(pose.position.x, pose.position.y, getYaw(pose.orientation))


def pathToPath2D(path):
    return Path2D(deepcopy(path.header), [poseToPose2D(p.pose) for p in path.poses])


def read_parameter(node, name, default):
    """Accept a parameter dict, a LocalNode, or an rclpy-like node."""
    if isinstance(node, dict):
        return node.setdefault(name, deepcopy(default))
    if not node.has_parameter(name):
        node.declare_parameter(name, deepcopy(default))
    value = node.get_parameter(name)
    return value.value if hasattr(value, "value") else value


class MemoryPublisher:
    """Lifecycle publisher storing message snapshots for examples and tests."""
    def __init__(self):
        self.active = False
        self.messages = []
        self.subscription_count = 1

    def on_activate(self):
        self.active = True

    def on_deactivate(self):
        self.active = False

    def get_subscription_count(self):
        return self.subscription_count

    def publish(self, message):
        if self.active:
            self.messages[:] = [deepcopy(message)]  # Match the depth-one queues.


class LocalNode:
    """In-process parameter, clock and publisher host; no network transport."""
    def __init__(self, parameters=None, name="python_mpc", now=time.time_ns):
        self.parameters = dict(parameters or {})
        self.publishers = {}
        self.subscriptions = {}
        self._name = name
        self._now = now

    def now(self):
        return self._now()

    def has_parameter(self, name):
        return name in self.parameters

    def declare_parameter(self, name, value):
        self.parameters.setdefault(name, deepcopy(value))

    def get_parameter(self, name):
        return self.parameters[name]

    def get_name(self):
        return self._name

    def get_logger(self):
        return logging.getLogger(self._name)

    def create_publisher(self, topic, qos=1):
        publisher = MemoryPublisher()
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, topic, callback, qos=1):
        self.subscriptions[topic] = callback
        return callback


class Costmap2D:
    """Row-major byte grid; tuple coordinates replace C++ output arguments."""
    def __init__(self, width, height, resolution, costs=None, origin_x=0.0, origin_y=0.0):
        if (width <= 0 or height <= 0 or not math.isfinite(resolution) or resolution <= 0):
            raise ValueError("Invalid costmap dimensions")
        self.width, self.height, self.resolution = width, height, resolution
        self.origin_x, self.origin_y = origin_x, origin_y
        self.costs = list(costs) if costs is not None else [0] * (width * height)
        if len(self.costs) != width * height or any(c < 0 or c > 255 for c in self.costs):
            raise ValueError("Costs must contain one byte per map cell")
        self.mutex = threading.RLock()

    def getSizeInCellsX(self): return self.width
    def getSizeInCellsY(self): return self.height
    def getResolution(self): return self.resolution
    def getOriginX(self): return self.origin_x
    def getOriginY(self): return self.origin_y
    def getCharMap(self): return self.costs
    def getMutex(self): return self.mutex
    def getCost(self, x, y): return self.costs[y * self.width + x]
    def setCost(self, x, y, cost): self.costs[y * self.width + x] = cost

    def worldToMap(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        mx = math.floor((x - self.origin_x) / self.resolution)
        my = math.floor((y - self.origin_y) / self.resolution)
        return (mx, my) if 0 <= mx < self.width and 0 <= my < self.height else None

    def worldToMapEnforceBounds(self, x, y):
        return (clamp(math.floor((x - self.origin_x) / self.resolution), 0, self.width - 1),
                clamp(math.floor((y - self.origin_y) / self.resolution), 0, self.height - 1))

    def mapToWorld(self, x, y):
        return (self.origin_x + (x + 0.5) * self.resolution,
                self.origin_y + (y + 0.5) * self.resolution)


class Costmap2DROS:
    """Footprint and frame metadata around the standalone grid."""
    def __init__(self, costmap, footprint=None, global_frame="map", base_frame="base_link", plugins=None):
        self.map = costmap
        self.footprint = deepcopy(footprint) if footprint is not None else [
            Point(-0.156, -0.105), Point(0.156, -0.105),
            Point(0.156, 0.105), Point(-0.156, 0.105)]
        self.global_frame, self.base_frame = global_frame, base_frame
        self.plugins = list(plugins or [])

    def getCostmap(self): return self.map
    def getRobotFootprint(self): return deepcopy(self.footprint)
    def getGlobalFrameID(self): return self.global_frame
    def getBaseFrameID(self): return self.base_frame
    def getLayeredCostmap(self): return self
    def getPlugins(self): return self.plugins

    def getInscribedRadius(self):
        distances = []
        for a, b in zip(self.footprint, self.footprint[1:] + self.footprint[:1]):
            dx, dy = b.x - a.x, b.y - a.y
            f = clamp(-(a.x * dx + a.y * dy) / (dx * dx + dy * dy), 0, 1) if dx or dy else 0
            distances.append(math.hypot(a.x + f * dx, a.y + f * dy))
        return min(distances, default=0.0)


class TrajectoryCritic:
    """Minimal DWB critic lifecycle for the translated algorithms."""
    def __init__(self, node=None, name=None, dwb_plugin_name="FollowPath", costmap_ros=None, scale=1.0):
        self.nh_ = node if node is not None else LocalNode()
        self.name_ = name or type(self).__name__
        self.dwb_plugin_name_ = dwb_plugin_name
        self.costmap_ros_ = costmap_ros
        self.costmap_ = costmap_ros.getCostmap() if costmap_ros is not None else None
        self.scale_ = scale

    def getName(self): return self.name_
    def getScale(self): return self.scale_
    def prepare(self, pose, velocity, goal, path): return True
    def reset(self): return None
    def debrief(self, velocity): return None
