"""Translation of mppi_cost_critics.hpp: critic state, defaults and interfaces."""
from dataclasses import dataclass, field
from typing import List, Protocol
from .ros_compat import Point, Pose2D, Path2D, Trajectory2D, Twist2D


@dataclass
class MppiObstacleParameters:
    critical_cost_: float = 300.0
    near_goal_distance_: float = 0.5
    near_goal_: bool = False
    full_footprint_check_: bool = False
    blocked_prefix_: List[int] = field(default_factory=list)
    collision_footprint_: List[Point] = field(default_factory=list)
    collision_cells_: list = field(default_factory=list)
    collision_polygon_: list = field(default_factory=list)
    footprint_radius_: float = 0.0


@dataclass
class MppiPathParameters:
    timing_enabled_: bool = False
    grid_updated_: bool = False
    guidance_grid_ms_: float = 0.0
    guidance_plan_ms_: float = 0.0
    guidance_plan_called_: bool = False
    guidance_plan_failed_: bool = False
    tracked_reference_: bool = False
    goal_weight_: float = 5.0
    goal_angle_weight_: float = 3.0
    align_weight_: float = 10.0
    follow_weight_: float = 5.0
    angle_weight_: float = 2.0
    goal_distance_: float = 1.0
    goal_angle_distance_: float = 0.5
    align_distance_: float = 0.5
    follow_distance_: float = 1.0
    angle_distance_: float = 0.5
    max_path_occupancy_ratio_: float = 0.15
    lookahead_distance_: float = 1.25
    max_angle_to_furthest_: float = 1.0
    guidance_clearance_margin_: float = 0.0
    guidance_trigger_margin_: float = 0.0
    guidance_cost_weight_: float = 2.0
    guidance_heading_weight_: float = 2.0
    guidance_rejoin_distance_: float = 0.50
    guidance_enabled_: bool = False
    guidance_trigger_layer_name_: str = ''
    guidance_trigger_layer_: object = None
    guidance_use_smac_: bool = False
    guidance_active_: bool = False
    smac_path_: list = field(default_factory=list)
    smac_lengths_: list = field(default_factory=list)
    proximity_footprint_: List[Point] = field(default_factory=list)
    use_path_orientations_: bool = True
    forward_preference_: bool = False
    goal_: Pose2D = field(default_factory=Pose2D)
    path_: List[Pose2D] = field(default_factory=list)
    path_lengths_: List[float] = field(default_factory=list)
    start_index_: int = 0
    target_index_: int = 0
    distance_to_goal_: float = 0.0
    path_blocked_: bool = False
    apply_path_angle_: bool = False


class MppiObstacleCritic(Protocol):
    def onInit(self) -> None: ...
    def prepare(self, pose: Pose2D, velocity: Twist2D, goal: Pose2D, path: Path2D) -> bool: ...
    def scoreTrajectory(self, trajectory: Trajectory2D) -> float: ...
    def getScale(self) -> float: ...
    def enforcesFullFootprint(self) -> bool: ...


class MppiPathCritic(Protocol):
    def setTimingEnabled(self, enabled: bool) -> None: ...
    def resetTiming(self) -> None: ...
    def guidanceGridMs(self) -> float: ...
    def guidancePlanMs(self) -> float: ...
    def guidancePlanCalled(self) -> bool: ...
    def guidancePlanFailed(self) -> bool: ...
    def usesSmac(self) -> bool: ...
    def guidanceReference(self, pose: Pose2D) -> List[Pose2D]: ...
    def gridUpdated(self) -> bool: ...
    def detouring(self) -> bool: ...
    def setTrackedReference(self, enabled: bool) -> None: ...
    def onInit(self) -> None: ...
    def reset(self) -> None: ...
    def prepare(self, pose: Pose2D, velocity: Twist2D, goal: Pose2D, path: Path2D) -> bool: ...
    def scoreTrajectory(self, trajectory: Trajectory2D) -> float: ...
