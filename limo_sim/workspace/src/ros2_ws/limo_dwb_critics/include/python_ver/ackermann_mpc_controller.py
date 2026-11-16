"""Translation of ackermann_mpc_controller.hpp: state and adapter interfaces.

C++ overloads become a single dispatching computeVelocityCommands method.
The external DWB and path-distance layer implementations are injected adapters.
"""
from dataclasses import dataclass
from typing import Optional, Protocol, Tuple
from .ros_compat import Path, Pose2D


@dataclass
class AckermannMPCParameters:
    open_loop_: bool = True
    steering_feedback_min_velocity_: float = 0.05
    last_time_ns_: int = 0
    last_ros_time_ns_: int = 0
    have_time_: bool = False
    have_command_: bool = False
    timing_core_started_: bool = False
    timing_sample_active_: bool = False
    tube_required_: bool = False
    tube_start_pending_: bool = True
    progress_weight_: float = 2.0
    progress_distance_: float = 1.0
    terminal_heading_weight_: float = 1.0


class PathDistanceLayer(Protocol):
    """Interface supplied by limo_inflation, which is outside this package.

    projectProgress returns (progress, heading), or None on failure. This
    replaces the C++ bool result and its two output-reference arguments.
    """
    def matchesPath(self, path: Path) -> bool: ...
    def containsFootprint(self, pose: Pose2D) -> bool: ...
    def projectProgress(self, pose: Pose2D, minimum: float = -1e30,
                        maximum: float = 1e30, preferred_progress: float = 0.0) -> Optional[Tuple[float, float]]: ...
    def tubeRadius(self) -> float: ...


class AckermannMPCController(Protocol):
    def configure(self, node, name, tf, costmap_ros) -> None: ...
    def computeVelocityCommands(self, pose, velocity, results=None): ...
    def setPlan(self, path: Path) -> None: ...
    def activate(self) -> None: ...
    def deactivate(self) -> None: ...
    def cleanup(self) -> None: ...
    def transformGlobalPlan(self, pose): ...
    def coreScoringAlgorithm(self, pose, velocity, results=None): ...
