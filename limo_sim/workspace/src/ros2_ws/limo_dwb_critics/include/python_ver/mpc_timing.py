"""Translation of mpc_timing.hpp: per-cycle measurements and report windows."""
from dataclasses import dataclass, field
import math
from typing import List, Protocol

TIMING_FIELDS = (
    'cycle_ms', 'thread_cpu_ms', 'prepare_ms', 'guidance_grid_ms', 'solve_ms',
    'scoring_ms', 'debug_ms', 'plan_prepare_ms', 'plan_prepare_cpu_ms',
    'costmap_lock_wait_ms', 'critics_prepare_ms', 'critics_prepare_cpu_ms',
    'debrief_ms', 'dwb_publish_ms',
)


@dataclass
class CriticPrepareTiming:
    prepare_ms: float = 0.0
    prepare_cpu_ms: float = 0.0
    called: bool = False


@dataclass
class MpcTimingSample:
    cycle_ms: float = 0.0
    thread_cpu_ms: float = 0.0
    prepare_ms: float = 0.0
    guidance_grid_ms: float = 0.0
    solve_ms: float = 0.0
    scoring_ms: float = 0.0
    debug_ms: float = 0.0
    plan_prepare_ms: float = 0.0
    plan_prepare_cpu_ms: float = 0.0
    costmap_lock_wait_ms: float = 0.0
    critics_prepare_ms: float = 0.0
    critics_prepare_cpu_ms: float = 0.0
    debrief_ms: float = 0.0
    dwb_publish_ms: float = 0.0
    critic_prepare: List[CriticPrepareTiming] = field(default_factory=list)
    generated_count: int = 0
    evaluated_count: int = 0
    rejected_count: int = 0
    success: bool = False
    detouring: bool = False
    grid_updated: bool = False

    def reset(self):
        for name in TIMING_FIELDS:
            setattr(self, name, 0.0)
        self.generated_count = self.evaluated_count = self.rejected_count = 0
        self.success = self.detouring = self.grid_updated = False
        for critic in self.critic_prepare:
            critic.prepare_ms = critic.prepare_cpu_ms = 0.0
            critic.called = False


@dataclass
class Metric:
    sum: float = 0.0
    maximum: float = 0.0
    count: int = 0

    def add(self, value):
        if math.isfinite(value) and value >= 0:
            self.sum += value
            self.maximum = max(self.maximum, value)
            self.count += 1

    def mean(self):
        return self.sum / self.count if self.count else 0.0


@dataclass
class Window:
    times: List[Metric] = field(default_factory=lambda: [Metric() for _ in TIMING_FIELDS])
    cycles: int = 0
    failures: int = 0
    deadline_misses: int = 0
    generated: int = 0
    evaluated: int = 0
    rejected: int = 0
    detour_cycles: int = 0
    grid_updates: int = 0


@dataclass
class CriticWindow:
    prepare: Metric = field(default_factory=Metric)
    prepare_cpu: Metric = field(default_factory=Metric)
    calls: int = 0


class MpcTimingPublisher(Protocol):
    def enabled(self) -> bool: ...
    def setCriticNames(self, names: List[str]) -> None: ...
    def activate(self) -> None: ...
    def deactivate(self) -> None: ...
    def finish(self, sample: MpcTimingSample) -> None: ...
