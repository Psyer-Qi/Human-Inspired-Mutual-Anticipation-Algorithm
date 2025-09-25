import math
import random

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

import navigator.rvo.math as rvo_math
from datatype.base import Path, Collision
from datatype.condition import TrialConditionBlocker
from navigator.rvo.simulator import Simulator
from navigator.rvo.vector import Vector2


class RVO2PathFinder:
    def __init__(self):
        self.start: Vector2 = Vector2()
        self.goal: Vector2 = Vector2()
        self.simulator = Simulator()
        self.simulator.set_time_step(0.1)
        self.simulator.set_agent_defaults(15.0, 10, 15.0, 1.0, 24.0, 1.34, Vector2(0.0, 0.0))

    def set_obstacles(self, obstacles: list[TrialConditionBlocker | Collision]):
        for obstacle in obstacles:
            if 'has_collided' in obstacle:
                self.simulator.add_obstacle([
                    Vector2(obstacle['position'][0] + 24, obstacle['position'][1] + 24),
                    Vector2(obstacle['position'][0] - 24, obstacle['position'][1] + 24),
                    Vector2(obstacle['position'][0] - 24, obstacle['position'][1] - 24),
                    Vector2(obstacle['position'][0] + 24, obstacle['position'][1] - 24)
                ])
            else:
                self.simulator.add_obstacle([
                    Vector2(obstacle['position'][0] + obstacle['size'][0] / 2,
                            obstacle['position'][1] + obstacle['size'][1] / 2),
                    Vector2(obstacle['position'][0] - obstacle['size'][0] / 2,
                            obstacle['position'][1] + obstacle['size'][1] / 2),
                    Vector2(obstacle['position'][0] - obstacle['size'][0] / 2,
                            obstacle['position'][1] - obstacle['size'][1] / 2),
                    Vector2(obstacle['position'][0] + obstacle['size'][0] / 2,
                            obstacle['position'][1] - obstacle['size'][1] / 2)
                ])

        self.simulator.process_obstacles()

    def set_path_plan(self, start: tuple[float, float] | tuple[int, int], goal: tuple[float, float] | tuple[int, int]):
        self.simulator.agents_.clear()
        self.simulator.add_agent(Vector2.from_tuple(start))
        self.start = Vector2.from_tuple(start)
        self.goal = Vector2.from_tuple(goal)

    def set_preferred_velocities(self):
        # Set the preferred velocity to be a vector of unit magnitude (speed) in the direction of the goal.
        goal_vector = self.goal - self.simulator.agents_[0].position_

        if rvo_math.abs_sq(goal_vector) > 1.0:
            goal_vector = rvo_math.normalize(goal_vector)

        self.simulator.set_agent_pref_velocity(0, goal_vector)

        # Perturb a little to avoid deadlocks due to perfect symmetry.
        angle = random.random() * 2.0 * math.pi
        dist = random.random() * 0.0002

        self.simulator.set_agent_pref_velocity(0, self.simulator.agents_[0].pref_velocity_ +
                                               dist * Vector2(math.cos(angle), math.sin(angle)))

    @property
    def reached_goal(self) -> bool:
        return rvo_math.abs_sq(self.simulator.agents_[0].position_ - self.goal) <= 4

    def get_path(self) -> Path:
        path = [list(self.simulator.agents_[0].position_)]
        while not self.reached_goal:
            self.set_preferred_velocities()
            self.simulator.step()
            path.append(list(self.simulator.agents_[0].position_))
            if len(path) > 500000:
                break
        return np.array(path, dtype=np.float32)

    def plot(self, path: Path, obstacles: list[TrialConditionBlocker | Collision]):
        fig, ax = plt.subplots()

        # 绘制路径
        ax.plot(path[:, 0], path[:, 1])

        for obstacle in obstacles:
            if 'has_collided' in obstacle:
                rect = Rectangle(obstacle['position'], 24, 24, facecolor='black')
            else:
                rect = Rectangle((obstacle['position'][0] - obstacle['size'][0] / 2,
                                  obstacle['position'][1] - obstacle['size'][1] / 2), obstacle['size'][0],
                                 obstacle['size'][1], facecolor='black')

            # 将矩形添加到绘图区域
            ax.add_patch(rect)

        # 设置坐标轴范围
        ax.set_xlim(0, 800)
        ax.set_ylim(0, 800)

        # 显示图形
        plt.show()
