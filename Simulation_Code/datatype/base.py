from dataclasses import dataclass
from typing import TypedDict, NotRequired, TypeAlias, Iterator

from numpy import ndarray

from datatype.condition import TrialCondition


CollisionIndex: TypeAlias = int

ConditionsGenerator: TypeAlias = Iterator[TrialCondition]
Conditions: TypeAlias = list[TrialCondition]

Path: TypeAlias = ndarray


class Collision(TypedDict):
    has_collided: bool
    position: NotRequired[tuple[float, float]]
    index: NotRequired[int]
    orientation_rad: NotRequired[float]


@dataclass
class SimMetadata:
    using_method: str
    level: int = 0
    batch: int = 0
    trial_index: int = 0
    first_gen_time: float = 0.0
    has_collision: bool = False
    second_gen_time: float = 0.0
    final_path_length: float = 0.0
    total_time: float = 0.0
    reached: bool = False


@dataclass
class PathMetadata:
    self_start: tuple[float, float] = (0.0, 0.0)
    self_target: tuple[float, float] = (0.0, 0.0)
    other_start: tuple[float, float] = (0.0, 0.0)
    other_target: tuple[float, float] = (0.0, 0.0)
