from dataclasses import dataclass
from typing import TypedDict


@dataclass
class MapParameters:
    """
    A data structure to represent the parameters of a map in the Social Locomotion Model.
    :param width: the width of the map, unit is meters
    :param height: the height of the map, unit is meters
    :param precision: the precision of the map, unit is meters
    """
    width: float = 7.0
    height: float = 7.0
    precision: float = 0.01


class PathPlan(TypedDict):
    """
    A data structure to represent the plan of a path in the Social Locomotion Model.
    :param start: the start position of the path, unit is centimeters
    :param target: the target position of the path, unit is centimeters
    """
    start: tuple[float, float]
    target: tuple[float, float]


class TrialConditionBlocker(TypedDict):
    """
    A data structure to represent a blocker in a trial condition in the Social Locomotion Model.
    :param position: the position of the blocker, unit is centimeters
    :param size: the size of the blocker (WIDTH, HEIGHT), unit is centimeters
    """
    position: tuple[float, float]
    size: tuple[float, float]


class TrialCondition(TypedDict):
    """
    A data structure to represent a trial condition in the Social Locomotion Model.
    :param start: the start position of the path, unit is centimeters
    :param target: the target position of the path, unit is centimeters
    :param blockers: the blockers in the trial condition, a tuple of TrialConditionBlocker
    """
    start: tuple[int, int]
    target: tuple[int, int]
    blockers: tuple[TrialConditionBlocker, ...]
