"""Translation of ackermann_kinematics_critic.hpp: defaults and public API."""
from dataclasses import dataclass
from typing import Protocol
from .ros_compat import Trajectory2D


@dataclass
class AckermannKinematicsParameters:
    min_turning_radius_: float = 0.462
    lateral_tolerance_: float = 1e-6
    stopped_velocity_: float = 1e-3


class AckermannKinematicsCritic(Protocol):
    def onInit(self) -> None: ...
    def scoreTrajectory(self, trajectory: Trajectory2D) -> float: ...


def commandRespectsAckermann(linear_x: float, linear_y: float, angular_z: float,
                             min_turning_radius: float, lateral_tolerance: float,
                             stopped_velocity: float) -> bool:
    """Forward the header's free-function declaration to its implementation."""
    from ...src.python_ver.ackermann_kinematics_critic import commandRespectsAckermann as implementation
    return implementation(linear_x, linear_y, angular_z, min_turning_radius,
                          lateral_tolerance, stopped_velocity)
