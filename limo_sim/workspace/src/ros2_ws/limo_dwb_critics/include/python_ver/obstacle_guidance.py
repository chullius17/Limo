"""Translation of obstacle_guidance.hpp: grid geometry and public interface."""
from dataclasses import dataclass
from typing import Protocol, Sequence


@dataclass
class Offset:
    x: int
    y: int


@dataclass
class Neighbor:
    x: int
    y: int
    length: float
    half_length: float
    heading: float


class ObstacleGuidance(Protocol):
    """The implementation is in src/python_ver/obstacle_guidance.py."""
    def update(self, width: int, height: int, resolution: float,
               costs: Sequence[int], clearance: float, cost_weight: float) -> None: ...
    def traversable(self, x: int, y: int) -> bool: ...
    def setTarget(self, x: int, y: int) -> bool: ...
    def distance(self, x: float, y: float) -> float: ...
    def heading(self, x: float, y: float, current_heading: float) -> float: ...
    def route(self, x: int, y: int, current_heading: float) -> list: ...
