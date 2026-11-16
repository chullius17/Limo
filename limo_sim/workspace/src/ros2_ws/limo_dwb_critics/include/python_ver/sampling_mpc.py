"""Translation of sampling_mpc.hpp: model configuration, values and interface.

C++ structs become dataclasses; each mutable default has its own factory.
Velocity is measured at the rear axle; steering is the bicycle-model angle.
"""
from dataclasses import dataclass, field
from enum import IntEnum
import math
from typing import Callable, List, Optional, Protocol


@dataclass
class MpcConfig:
    dt: float = 0.05
    time_steps: int = 50
    control_segments: int = 5
    batch_size: int = 768
    velocity_samples: int = 8
    curvature_samples: int = 15
    structured_actions: bool = False
    deduplicate: bool = False
    action_variants: int = 3
    action_noise: float = 0.20
    startup_enabled: bool = False
    startup_max_initial_speed: float = 0.05
    startup_target_speed: float = 0.03
    startup_acceleration: float = 0.05
    block_fractions: List[float] = field(default_factory=lambda: [0.25, 0.50, 0.25])
    min_velocity: float = -0.10
    max_velocity: float = 0.50
    max_yaw_rate: float = 1.10
    acceleration: float = 1.3
    deceleration: float = 1.3
    yaw_acceleration: float = 3.4
    steering_rate: float = 0.8
    wheelbase: float = 0.20
    min_turning_radius: float = 0.462
    rear_axle_to_base: float = 0.10
    velocity_std: float = 0.12
    steering_std: float = 0.15
    acceleration_weight: float = 0.5
    steering_weight: float = 0.5
    steering_command_weight: float = 0.2
    steering_rate_change_weight: float = 0.1
    reverse_distance_weight: float = 0.0
    direction_change_weight: float = 0.0

    def validate(self):
        if self.startup_enabled and (
            not self.structured_actions or not math.isfinite(self.startup_max_initial_speed) or
            not 0 < self.startup_max_initial_speed <= self.max_velocity or
            not math.isfinite(self.startup_target_speed) or
            not 0 < self.startup_target_speed <= self.startup_max_initial_speed or
            not math.isfinite(self.startup_acceleration) or
            not 0 < self.startup_acceleration <= min(self.acceleration, self.deceleration)):
            raise ValueError("Invalid MPC startup speed, acceleration or strategy")
        if self.structured_actions:
            if any(not math.isfinite(f) or f <= 0 or f * self.time_steps < 1
                   for f in self.block_fractions):
                raise ValueError("Each MPC block must contain at least one model step")
            if (len(self.block_fractions) != self.control_segments or
                    abs(sum(self.block_fractions) - 1.0) > 1e-9):
                raise ValueError("Invalid MPC block fractions")
        positive = (self.dt, self.max_velocity, self.max_yaw_rate, self.acceleration,
                    self.deceleration, self.yaw_acceleration, self.steering_rate,
                    self.wheelbase, self.min_turning_radius, self.velocity_std, self.steering_std)
        if any(not math.isfinite(v) or v <= 0 for v in positive):
            raise ValueError("MPC rates, dimensions and sampling deviations must be positive")
        nonnegative = (self.rear_axle_to_base, self.acceleration_weight, self.steering_weight,
                       self.steering_command_weight, self.steering_rate_change_weight,
                       self.reverse_distance_weight, self.direction_change_weight)
        counts = (self.time_steps, self.control_segments, self.velocity_samples,
                  self.curvature_samples, self.batch_size, self.action_variants)
        if (any(not isinstance(n, int) or isinstance(n, bool) for n in counts) or
                not math.isfinite(self.min_velocity) or self.min_velocity > 0 or
                any(not math.isfinite(v) or v < 0 for v in nonnegative) or
                not 2 <= self.time_steps <= 1000 or not 1 <= self.control_segments <= self.time_steps or
                not 2 <= self.velocity_samples <= 100 or not 3 <= self.curvature_samples <= 100 or
                self.batch_size > 10000 or not math.isfinite(self.action_noise) or
                not 0 <= self.action_noise < 1 or not 1 <= self.action_variants <= 100):
            raise ValueError("Invalid MPC horizon, sample count, limits or weights")
        if self.structured_actions:
            invalid = (self.batch_size != 2 + 18 * self.action_variants + (3 if self.startup_enabled else 0)
                       or self.min_velocity != 0.0)
        else:
            invalid = self.batch_size < 2 + self.velocity_samples * self.curvature_samples
        if invalid:
            raise ValueError("Invalid MPC horizon, sample count, limits or weights")


@dataclass
class MpcControl:
    velocity: float = 0.0
    steering: float = 0.0


@dataclass
class MpcState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    control: MpcControl = field(default_factory=MpcControl)


@dataclass
class MpcRollout:
    states: List[MpcState] = field(default_factory=list)  # Initial state, then N predictions.
    targets: List[MpcControl] = field(default_factory=list)
    effort_cost: float = 0.0


class MpcFamily(IntEnum):
    Braking = 0
    Nominal = 1
    ConstantCurvature = 2
    PerturbedNominal = 3
    BroadExploration = 4
    Structured = 5
    Monophase = 6
    Startup = 7


# NaN means effort-pruned; infinity means rejected. Observers cannot select a winner.
MpcObserver = Callable[[MpcRollout, MpcFamily, int, float], None]


@dataclass
class MpcSolution:
    rollout: MpcRollout = field(default_factory=MpcRollout)
    cost: float = math.inf
    family: MpcFamily = MpcFamily.Braking
    candidate_id: int = 0
    generated_count: int = 0
    duplicate_count: int = 0


class SamplingMpc(Protocol):
    """Public contract; instantiate the implementation from src/python_ver."""
    def config(self) -> MpcConfig: ...
    def reset(self) -> None: ...
    def hasWarmStart(self) -> bool: ...
    def yawRate(self, control: MpcControl) -> float: ...
    def advance(self, current: MpcControl, target: MpcControl) -> MpcControl: ...
    def rollout(self, initial: MpcState, targets: List[MpcControl],
                previous_steering_rate: float = 0.0, previous_direction: int = 0) -> MpcRollout: ...
    def solve(self, initial: MpcState, environment_cost: Callable[[MpcRollout, float], float],
              shift_steps: int = 1, observer: Optional[MpcObserver] = None) -> MpcSolution: ...
