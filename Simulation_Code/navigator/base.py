from datatype.base import Collision, Path
from datatype.condition import TrialCondition


class Navigator:
    def __init__(self):
        pass

    def get_path(self, condition: TrialCondition, is_self: bool, collision_info: Collision) -> Path:
        pass
