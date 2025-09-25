from datatype.base import Collision, Path
from datatype.condition import TrialCondition
from navigator.rvo2 import RVO2PathFinder
from pipelines.base import Simulation


class RVO2(Simulation):
    def __init__(self, batch: int, with_tom: bool = False):
        super().__init__(method="rvo", batch=batch, with_tom=with_tom)

    def get_path(self, condition: TrialCondition, is_self: bool = True,
                 collision_info: Collision | None = None) -> Path:

        if is_self:
            if collision_info is not None and collision_info['has_collided']:
                start = collision_info['position']
                end = condition['target']
            else:
                start = self.path_metadata.self_start
                end = condition['target']
        else:
            if collision_info is not None and collision_info['has_collided']:
                start = collision_info['position']
                end = condition['start']
            else:
                start = self.path_metadata.other_start
                end = condition['start']

        path_solver = RVO2PathFinder()
        path_solver.set_path_plan(start, end)

        blockers = [b for b in condition['blockers']]
        if collision_info is not None and not collision_info['has_collided']:
            blockers.append(collision_info)

        path_solver.set_obstacles(blockers)

        return path_solver.get_path()
