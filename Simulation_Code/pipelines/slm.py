from datatype.base import Collision, Path
from datatype.condition import *
from datatype.sifm import SIFM
from datatype.slm import *
from navigator.slm import SlmPathFinder
from pipelines.base import Simulation


class SLM(Simulation):
    def __init__(self, batch: int, with_tom: bool = True):
        super().__init__(batch=batch, method="slm", with_tom=with_tom)

        self.path_solver = SlmPathFinder(map_params=self.map_params, use_gpu=True)
        self.sifm = SIFM()

    def get_path(self, condition: TrialCondition, is_self: bool = True,
                 collision_info: Collision | None = None) -> Path:
        self._set_condition(self.current_trial_index, condition, is_self, collision_info)
        self.path_solver.solve()
        return self.path_solver.path

    def _set_condition(self, index: int, condition: TrialCondition, is_self=True,
                       collision_info: Collision | None = None):
        self.path_solver.reset()

        if is_self:
            if collision_info is not None and collision_info['has_collided']:
                self.path_solver.path_plan = PathPlan(start=collision_info['position'], target=condition['target'])
            else:
                self.path_solver.path_plan = PathPlan(start=self.path_metadata.self_start, target=condition['target'])
        else:
            if collision_info is not None and collision_info['has_collided']:
                self.path_solver.path_plan = PathPlan(start=collision_info['position'], target=condition['start'])
            else:
                self.path_solver.path_plan = PathPlan(start=self.path_metadata.other_start, target=condition['start'])

        blockers = [
            SlmBlocker(
                is_human=0,
                position=ti.Vector(blocker['position']),
                a=blocker['size'][0],
                b=blocker['size'][1],
                c=6.76
            ) for blocker in condition['blockers']
        ]

        if collision_info is not None and not collision_info['has_collided']:
            blockers.append(SlmBlocker(
                is_human=1,
                position=ti.Vector(collision_info['position']),
                a=self.sifm.a,
                b=self.sifm.b,
                c=self.sifm.c,
                orientation_rad=collision_info['orientation_rad']
            ))
        self.path_solver.blockers = tuple(blockers)
