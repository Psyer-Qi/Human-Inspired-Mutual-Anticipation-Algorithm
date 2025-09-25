from dataclasses import fields
from itertools import product, combinations
from json import dumps
from os.path import exists
from time import perf_counter
from typing import cast

import matplotlib.pyplot as plt
import numpy as np
from loguru import logger
from matplotlib.patches import Circle

from datatype.base import *
from datatype.condition import *

COLLISION_THRESHOLD = 48


class Timer:
    def __init__(self):
        self.__start_time = perf_counter()

    def tick(self):
        self.__start_time = perf_counter()

    @property
    def start_time(self):
        return self.__start_time

    @property
    def elapsed(self):
        return perf_counter() - self.__start_time


class Simulation:
    def __init__(self, batch: int, method: str, map_params=MapParameters(width=8, height=8), with_tom: bool = True):
        self.method = method
        self.meta_data = SimMetadata(using_method=method)
        self.path_metadata = PathMetadata()
        self.meta_data.batch = batch

        self.current_tom_level: int = 0

        self.map_params = map_params
        self.map_offset = 50
        self.points_per_dim = (int(map_params.height / map_params.precision),
                               int(map_params.width / map_params.precision))

        self.current_trial_index = 0

        self.conditions = self.generate_conditions(is_generator=False)

        self.with_tom = with_tom

    # Simulation abstract methods
    def get_path(self, condition: TrialCondition, is_self: bool = True,
                 collision_info: Collision | None = None) -> Path:
        pass

    def simulate(self):
        
        data_file = f'./result/sim_{self.meta_data.using_method}_{"with" if self.with_tom else "without"}_b{self.meta_data.batch}_result.csv'
        
        if exists(data_file):
            with open(data_file, 'r') as f:
                num_of_records = len(f.readlines()) - 1
            logger.info(f"Found {num_of_records} records. Resuming...\n")
        else:
            logger.info(f"Starting from new.\n")
            num_of_records = 0
            with open(
                    f'./result/sim_{self.meta_data.using_method}_{"with" if self.with_tom else "without"}_b{self.meta_data.batch}_result.csv',
                    'w') as file:
                header = ','.join([f"{item.name}" for item in fields(self.meta_data)]) + '\n'
                file.write(header)

        # self.meta_data.level = self.current_tom_level

        for index, condition in enumerate(self.conditions):
            if index < num_of_records:
                logger.debug(f"Passing trial {index}")
                continue
            self.current_trial_index = index
            self.meta_data.trial_index = index
            logger.info(
                f"Batch {self.meta_data.batch}: Running trial {index + 1} of {len(self.conditions)}, {self.meta_data.using_method} {'with' if self.with_tom else 'without'} ToM")
            timer = Timer()

            self.trial(condition)

            logger.info(f"Trial {index + 1} of {len(self.conditions)} completed in {timer.elapsed:.4f} seconds.")
            # gc.collect()

    def save_result(self):
        with open(
                f'./result/sim_{self.meta_data.using_method}_{"with" if self.with_tom else "without"}_b{self.meta_data.batch}_result.csv',
                'a') as file:
            newline = ','.join([f'{getattr(self.meta_data, item.name)}' for item in fields(self.meta_data)]) + '\n'
            file.write(newline)

        with open(
                f'./output/{self.meta_data.using_method}_{"with" if self.with_tom else "without"}_b{self.meta_data.batch}_r{self.meta_data.trial_index}.json',
                'w') as file:
            sim_metadata = self.meta_data.__dict__
            sim_metadata.update(self.path_metadata.__dict__)
            file.write(dumps(sim_metadata))

    def trial(self, condition: TrialCondition):
        timer = Timer()

        if self.with_tom:
            self.current_tom_level = np.random.choice((1, 2), p=(0.8, 0.2))
        else:
            self.current_tom_level = 0

        self.meta_data.level = int(self.current_tom_level)

        self.path_metadata.self_start = self.get_offset_point(condition['start'])
        self.path_metadata.self_target = condition['target']
        self.path_metadata.other_start = self.get_offset_point(condition['target'])
        self.path_metadata.other_target = condition['start']

        try:
            match self.current_tom_level:
                case 0:
                    self._trial_level_0(condition)
                case 1:
                    self._trial_level_1(condition)
                case 2:
                    self._trial_level_2(condition)
                case _:
                    raise ValueError("Invalid TOM level")
        except ValueError as e:
            logger.warning(e)
            self.meta_data.reached = False
        else:
            self.meta_data.reached = True

        self.meta_data.total_time = timer.elapsed
        self.save_result()

    def _trial_level_0(self, condition: TrialCondition):
        timer = Timer()
        self_path = self.get_path(condition, is_self=True)
        self.meta_data.first_gen_time = timer.elapsed
        other_path = self.get_path(condition, is_self=False)

        self._trial_post_process(condition, self_path, other_path)

    def _trial_level_1(self, condition: TrialCondition):
        timer = Timer()
        self_path = self.get_path(condition, is_self=True)
        other_path = self.get_path(condition, is_self=False)
        nearest_info = self.pending_collision(self_path, other_path, is_near=True)
        self_path_revised = self.get_path(condition, is_self=True, collision_info=nearest_info)
        self.meta_data.first_gen_time = timer.elapsed

        self._trial_post_process(condition, self_path_revised, other_path)

    def _trial_level_2(self, condition: TrialCondition):
        timer = Timer()
        other_path = self.get_path(condition, is_self=False)
        self_path = self.get_path(condition, is_self=True)
        nearest_info = self.pending_collision(other_path, self_path, is_near=True)
        other_path_revised = self.get_path(condition, is_self=False, collision_info=nearest_info)

        nearest_info = self.pending_collision(self_path, other_path_revised, is_near=True)
        self_path_revised = self.get_path(condition, is_self=True, collision_info=nearest_info)
        self.meta_data.first_gen_time = timer.elapsed

        self._trial_post_process(condition, self_path_revised, other_path)

    def _trial_post_process(self, condition: TrialCondition, self_path: Path, other_path: Path):
        collision_info = self.pending_collision(self_path, other_path)
        prefix = f'{self.meta_data.using_method}_{"with" if self.with_tom else "without"}_b{self.meta_data.batch}_r{self.meta_data.trial_index}'
        if collision_info['has_collided']:
            self.meta_data.has_collision = True
            timer = Timer()
            self_path_regenerated = self.get_path(condition, is_self=True, collision_info=collision_info)
            self.meta_data.second_gen_time = timer.elapsed
            final_path = np.concatenate((self_path[:collision_info["index"]], self_path_regenerated), axis=0)
            self.meta_data.final_path_length = self.get_path_length(final_path)

            np.savetxt(f'./output/{prefix}_self.csv', final_path - self.map_offset, delimiter=',')
        else:
            self.meta_data.has_collision = False
            self.meta_data.final_path_length = self.get_path_length(self_path)
            self.meta_data.second_gen_time = 0.0
            np.savetxt(f'./output/{prefix}_self.csv', self_path - self.map_offset, delimiter=',')

        np.savetxt(f'./output/{prefix}_other.csv', other_path - self.map_offset, delimiter=',')

    def generate_conditions(self, is_generator=False, start: int = 0) -> ConditionsGenerator | Conditions:

        if not start == 0 and is_generator:
            raise ValueError("Start index can only be non-zero when not using generator.")

        blocker_sizes = [(100, 100), (100, 300), (300, 100)]
        blocker_positions = [(150, 150), (550, 150), (350, 350), (150, 550), (550, 550)]
        start_end_pairs = [((350, 700), (350, 0)), ((700, 350), (0, 350)), ((0, 700), (700, 0)), ((700, 700), (0, 0))]

        blocker_combinations = [
            [(pos, size) for pos, size in zip(positions, sizes)]
            for num_blockers in range(1, 4)
            for positions in combinations(blocker_positions, num_blockers)
            for sizes in product(blocker_sizes, repeat=num_blockers)
        ]

        def generator() -> Iterator[TrialCondition]:
            for start_end in start_end_pairs:
                for blockers in blocker_combinations:
                    yield TrialCondition(start=(start_end[0][0] + self.map_offset, start_end[0][1] + self.map_offset),
                                         target=(start_end[1][0] + self.map_offset, start_end[1][1] + self.map_offset),
                                         blockers=tuple(TrialConditionBlocker(position=(position[0] + self.map_offset,
                                                                                        position[1] + self.map_offset),
                                                                              size=size)
                                                        for position, size in blockers))

        if is_generator:
            return generator()

        return list(generator())[start:]

    @staticmethod
    def adjust_path_steps(original_path: Path, target_steps: int) -> Path:
        """
        Adjust the number of steps in a path by interpolating between the original points.
        :param original_path: ndarray of shape (n, 2)
        :param target_steps: int
        :return: ndarray of shape (target_steps, 2)
        """
        n, d = original_path.shape
        original_steps = np.linspace(0, 1, n)
        adjusted_steps = np.linspace(0, 1, target_steps)
        normalized_path = np.zeros((target_steps, d))

        for i in range(d):
            normalized_path[:, i] = np.interp(adjusted_steps, original_steps, original_path[:, i])

        return normalized_path

    @staticmethod
    def find_closest(arr, value) -> CollisionIndex:
        """
        Find the index of the first element in a sorted numpy array that is closest to the given value.

        :param arr: numpy array of sorted integers
        :param value: the target value
        :return: index of the closest element
        """
        idx = np.searchsorted(arr, value, side='left')
        if idx > 0 and (idx == len(arr) or np.abs(value - arr[idx - 1]) <= np.abs(value - arr[idx])):
            return idx - 1
        else:
            return idx

    def pending_collision(self, path_a: Path, path_b: Path, is_near=False) -> Collision:
        collision_info: Collision = {
            "has_collided": False
        }

        steps_a, _ = path_a.shape
        steps_b, _ = path_b.shape
        steps_avg = int((steps_a + steps_b) / 2)

        path_a = self.adjust_path_steps(path_a, steps_avg)
        path_b = self.adjust_path_steps(path_b, steps_avg)

        path_distance = np.linalg.norm(path_a - path_b, axis=1)
        min_distance = np.min(path_distance)

        if min_distance > COLLISION_THRESHOLD:
            if not is_near:
                return collision_info
        else:
            if not is_near:
                collision_info["has_collided"] = True

        min_index = np.argmin(path_distance)
        collision_info["index"] = cast(int, min_index)
        collision_info["position"] = cast(tuple[float, float],
                                          tuple(path_b[min_index, :]) if is_near else tuple(path_a[min_index, :]))

        movements = np.diff(path_b, axis=0)
        orientations = np.arctan2(movements[:, 1], movements[:, 0])
        # orientations = np.rad2deg(orientations)
        orientations = np.concatenate(([0], orientations)) - np.pi / 2
        # collision_info["orientation_rad"] = np.deg2rad(orientations[min_index])
        collision_info["orientation_rad"] = orientations[min_index]

        return collision_info

    def get_offset_point(self, point: tuple[int, int]) -> tuple[int, int]:
        local_offset = int(np.clip(np.random.normal(0, 25), -25, 25))

        base_boundary = self.map_offset
        right_boundary = self.points_per_dim[1] - base_boundary
        bottom_boundary = self.points_per_dim[0] - base_boundary

        # !! INCOMPLETE, only under this start-end pair setting !!

        if point[0] == base_boundary or point[0] == bottom_boundary:
            if point[1] == base_boundary:
                return point[0], abs(local_offset)
            if point[1] == right_boundary:
                return point[0], right_boundary - abs(local_offset)
            return point[0], point[1] + local_offset

        if point[1] == base_boundary or point[1] == right_boundary:
            return point[0] + local_offset, point[1]

    @staticmethod
    def get_path_length(path: Path) -> float:
        movements = np.diff(path, axis=0)
        distances = np.linalg.norm(movements, axis=1)
        return float(np.sum(distances))

    def plot(self, self_path: Path, other_path: Path, condition: TrialCondition):
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        ax.set_xlim(0, self.points_per_dim[0])
        ax.set_ylim(0, self.points_per_dim[1])

        for blocker in condition['blockers']:
            circle = Circle((blocker['position'][0], blocker['position'][1]), 0.5, color='black')
            ax.add_patch(circle)

        ax.plot(self_path[:, 0], self_path[:, 1], color='red')
        ax.plot(other_path[:, 0], other_path[:, 1], color='blue')

        plt.show()
