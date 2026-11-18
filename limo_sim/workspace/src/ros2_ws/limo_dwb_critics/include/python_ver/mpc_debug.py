"""Translation of mpc_debug.hpp and the referenced limo_interfaces messages."""
from dataclasses import dataclass, field
from typing import ClassVar, List, Optional, Protocol
from .ros_compat import Header, Point, Pose, Pose2D
from .sampling_mpc import MpcObserver, MpcSolution, MpcState


@dataclass
class MpcCandidate:
    COST_PRUNED: ClassVar[int] = 0
    SCORED: ClassVar[int] = 1
    REJECTED: ClassVar[int] = 2
    candidate_id: int = 0
    family: int = 0
    total_cost: float = 0.0
    score_status: int = 0
    poses: List[Pose2D] = field(default_factory=list)


@dataclass
class CostmapMetadata:
    update_time: int = 0
    layer: str = ''
    resolution: float = 0.0
    size_x: int = 0
    size_y: int = 0
    origin: Pose = field(default_factory=Pose)


@dataclass
class CostmapSnapshot:
    header: Header = field(default_factory=Header)
    metadata: CostmapMetadata = field(default_factory=CostmapMetadata)
    data: List[int] = field(default_factory=list)


@dataclass
class FootprintSnapshot:
    points: List[Point] = field(default_factory=list)


@dataclass
class MpcDebug:
    header: Header = field(default_factory=Header)
    base_frame: str = ''
    robot_pose: Pose2D = field(default_factory=Pose2D)
    footprint: FootprintSnapshot = field(default_factory=FootprintSnapshot)
    costmap: CostmapSnapshot = field(default_factory=CostmapSnapshot)
    wheelbase: float = 0.0
    rear_axle_to_base: float = 0.0
    model_dt: float = 0.0
    time_steps: int = 0
    generated_count: int = 0
    initial_velocity: float = 0.0
    initial_steering: float = 0.0
    command_velocity: float = 0.0
    command_steering: float = 0.0
    selected_id: int = 0
    candidates: List[MpcCandidate] = field(default_factory=list)
    guidance_backend: str = ''
    guidance_path: List[Pose2D] = field(default_factory=list)


class MpcDebugPublisher(Protocol):
    def activate(self) -> None: ...
    def deactivate(self) -> None: ...
    def begin(self, initial: MpcState) -> Optional[MpcObserver]: ...
    def setGuidance(self, backend: str, path: List[Pose2D]) -> None: ...
    def finish(self, solution: MpcSolution) -> None: ...
