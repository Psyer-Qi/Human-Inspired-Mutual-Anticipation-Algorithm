from dataclasses import dataclass


@dataclass
class CompanionCondition:
    start: list
    target: list
    blocker_positions: list
    blocker_orientations: list
    blocker_sizes: list
    is_with_human: bool


