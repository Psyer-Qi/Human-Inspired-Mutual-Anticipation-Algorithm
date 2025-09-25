import math
from copy import deepcopy

import numpy as np

from datatype.base import Collision, Path
from datatype.condition import TrialCondition
from navigator.rvo import compute_v_des, rvo_update
from pipelines.base import Simulation


class RVO(Simulation):
    def __init__(self, batch: int, with_tom: bool = False):
        super().__init__(method="rvo", batch=batch, with_tom=with_tom)

    @staticmethod
    def __cm_to_m(point: tuple[float, float] | tuple[int, int]) -> tuple[float, float]:
        return point[0] / 100, point[1] / 100

    @staticmethod
    def __m_to_cm(point: tuple[float, float]) -> tuple[int, int]:
        return int(point[0] * 100), int(point[1] * 100)

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

        ws_model = dict()
        ws_model['robot_radius'] = 0.24
        ws_model['circular_obstacles'] = []
        for blocker in condition['blockers']:
            position = self.__cm_to_m(blocker['position'])
            # ws_model['circular_obstacles'].append([position[0], position[1], 0.5])
            # ws_model['circular_obstacles'].append([position[0] + 0.5, position[1] + 0.5, 0.5])
            if blocker['size'] == (100, 100):
                ws_model['circular_obstacles'].append([position[0], position[1], 0.5])
            #     # ws_model['circular_obstacles'].append([position[0] + 0.5, position[1] + 0.5, 0.5])
            elif blocker['size'] == (300, 100):
                ws_model['circular_obstacles'].append([position[0] - 1, position[1], 0.5])
            #     # ws_model['circular_obstacles'].append([position[0] - 0.5, position[1] + 0.5, 0.5])
            #
                ws_model['circular_obstacles'].append([position[0], position[1], 0.5])
            #     # ws_model['circular_obstacles'].append([position[0] + 0.5, position[1] + 0.5, 0.5])
            #
                ws_model['circular_obstacles'].append([position[0] + 1, position[1], 0.5])
            #     # ws_model['circular_obstacles'].append([position[0] + 1.5, position[1] + 0.5, 0.5])
            else:
                ws_model['circular_obstacles'].append([position[0], position[1] - 1, 0.5])
            #     # ws_model['circular_obstacles'].append([position[0] + 0.5, position[1] - 0.5, 0.5])
            #
                ws_model['circular_obstacles'].append([position[0], position[1], 0.5])
            #     # ws_model['circular_obstacles'].append([position[0] + 0.5, position[1] + 0.5, 0.5])
            #
                ws_model['circular_obstacles'].append([position[0], position[1] + 1, 0.5])
            #     # ws_model['circular_obstacles'].append([position[0] + 0.5, position[1] + 1.5, 0.5])

        if collision_info is not None and not collision_info['has_collided']:
            position = self.__cm_to_m(collision_info['position'])
            ws_model['circular_obstacles'].append([position[0], position[1], 0.24])

        current_position = [list(self.__cm_to_m(start))]
        current_velocity = [[0, 0]]
        v_max = [1.36]
        target_position = [list(self.__cm_to_m(end))]
        step_size = 0.01
        path = [list(self.__cm_to_m(start))]

        steps = 0

        while True:
            v_desire = compute_v_des(current_position, target_position, v_max)
            current_velocity = rvo_update(current_position, v_desire, current_velocity, ws_model)
            current_position[0][0] += step_size * current_velocity[0][0]
            current_position[0][1] += step_size * current_velocity[0][1]
            path.append(deepcopy(current_position[0]))

            if (abs(current_position[0][0] - target_position[0][0]) < 0.1 and
                    abs(current_position[0][1] - target_position[0][1]) < 0.1):
                break

            steps += 1
            if steps > 3000:
                raise ValueError("RVO failed to find a path.")

        return np.array(path) * 100
