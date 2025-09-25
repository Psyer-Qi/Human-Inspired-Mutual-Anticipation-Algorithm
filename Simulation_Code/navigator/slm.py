from typing import cast

import matplotlib.pyplot as plt
import numpy as np
import pykonal
import pyekfmm as fmm
import taichi as ti
import taichi.math as tm

from datatype.base import Path
from datatype.condition import *
from datatype.sifm import SIFM
from datatype.slm import SlmBlocker


@ti.data_oriented
class SlmPathFinder:
    __taichi_inited: bool = False

    def __init__(self, map_params: MapParameters, slm_params: SIFM = SIFM(), use_gpu: bool = False):
        # Init taichi only once if not inited. Once inited, it can be used until the program ends.
        if not SlmPathFinder.__taichi_inited:
            ti.init(arch=ti.gpu if use_gpu else ti.cpu)
            SlmPathFinder.__taichi_inited = True

        self.map_params = map_params
        self.sifm_params = slm_params

        self.path_plan_inverted = PathPlan(start=(0, 0), target=(0, 0))
        self.blockers: tuple[SlmBlocker, ...] = ()

        self.points_per_dim = (int(map_params.height / map_params.precision),
                               int(map_params.width / map_params.precision))

        self.energy_map_field = ti.field(dtype=ti.f32, shape=self.points_per_dim)
        self.energy_map_np = np.empty(self.points_per_dim, dtype=np.float32)

        # For SLM path planning using fast marching method
        self.time_map: np.ndarray = np.empty(())
        self.path: Path[float] = np.empty(())

        # use for energy_map calculation
        self.target_position = ti.math.vec2([0, 0])

    @property
    def energy_map(self):
        return self.energy_map_np / self.sifm_params.th

    @property
    def velocity_map(self):
        return 1 - self.energy_map_np / self.sifm_params.th

    @property
    def path_plan(self):
        return PathPlan(start=cast(tuple[float, float], self.path_plan_inverted['start'][::-1]),
                        target=self.path_plan_inverted['target'])

    @path_plan.setter
    def path_plan(self, path_plan: PathPlan):
        self.path_plan_inverted: PathPlan = {
            'start': cast(tuple[float, float], path_plan['start'][::-1]),
            'target': path_plan['target']
        }

    def reset(self):
        self.energy_map_field.fill(0.0)

    def solve(self):
        self.solve_map()
        self.solve_path()

    def solve_map(self):
        for index, blocker in enumerate(self.blockers):
            self.__get_map(blocker)
            if index == 0:
                self.energy_map_np = self.energy_map_field.to_numpy()
            else:
                self.energy_map_np += self.energy_map_field.to_numpy()

    def solve_path(self):
        ray_tracer = pykonal.EikonalSolver()
        ray_tracer.velocity.min_coords = 0, 0, 0
        ray_tracer.velocity.node_intervals = 1, 1, 1
        ray_tracer.velocity.npts = self.points_per_dim[0], self.points_per_dim[1], 1
        time_map = (fmm.eikonal(self.velocity_map.flatten(order='C'),
                                xyz=np.array([self.path_plan_inverted['start'][0],
                                              0,
                                              self.path_plan_inverted['start'][1]]),
                                ax=[0, 1, self.points_per_dim[0]],
                                ay=[0, 1, 1],
                                az=[0, 1, self.points_per_dim[1]],
                                order=2)
                    .reshape([self.points_per_dim[0], self.points_per_dim[1]], order='C'))

        ray_tracer.traveltime.values = np.expand_dims(time_map, 2)
        self.path = ray_tracer.trace_ray(np.array((*self.path_plan_inverted['target'], 0), dtype=float))[:, :2]

    @ti.kernel
    def __get_map(self, blocker: SlmBlocker):
        blocker_boundary = ti.Vector([blocker.position[0] - 25,
                                      blocker.position[0] + 25,
                                      blocker.position[1] - 25,
                                      blocker.position[1] + 25])

        for x, y in self.energy_map_field:
            current_position = ti.Vector([x, y])

            is_blocked = (
                                 blocker.is_human == 0 and
                                 blocker_boundary[0] <= x <= blocker_boundary[1] and
                                 blocker_boundary[2] <= y <= blocker_boundary[3]
                         ) or (
                                 blocker.is_human != 0 and
                                 x == blocker.position[0] and
                                 y == blocker.position[1]
                         )

            # If blocked，pass. It looks verbose because 'continue' IS NOT SUPPORTED in ti.kernel.
            if is_blocked:
                self.energy_map_field[x, y] = np.inf
            else:
                self_field = self.get_self_field(current_position, blocker.position)

                # Fix for weird nan issue, only god knows why it happens.
                if not (self_field == 0.0 or self_field > 0 or self_field < 0):
                    self_field = self.sifm_params.n2 + self.sifm_params.c * self.sifm_params.b

                distance_to_blocker = (blocker.position - ti.Vector([x, y])).norm() * self.map_params.precision

                other_field: ti.f32 = 0.0
                if blocker.is_human == 0:
                    other_field = self.get_other_field_nonhuman(current_position, blocker, distance_to_blocker)
                else:
                    other_field = self.get_other_field_human(current_position, blocker)

                self.energy_map_field[x, y] = (self_field * other_field) / (distance_to_blocker ** 2)

    @ti.func
    def get_self_field(self, current_position: ti.math.vec2, blocker_position) -> ti.types.f32:
        self_field: ti.f32 = 0.0

        # vec2 == vec2 -> vec2[bool]
        if (current_position == self.target_position).all():
            self_field = self.sifm_params.n2
        else:
            cos_b = 0.0
            sin_b = 0.0

            diff = current_position - self.target_position

            ori = tm.acos(diff[1] * (-1) / diff.norm())
            ori += np.pi
            cos_b, sin_b = self.get_trigonometry_by_rad(blocker_position, current_position, ori)

            f_b: ti.f32 = 0.0
            if not cos_b < 0:
                f_b = cos_b

            a = self.sifm_params.a
            b = self.sifm_params.b
            c = self.sifm_params.c
            m2 = self.sifm_params.m2
            n2 = self.sifm_params.n2

            self_field = m2 * f_b + n2 + c * (a * b) / tm.sqrt((a * cos_b * a * cos_b) + (b * sin_b * b * sin_b))

        return self_field

    @ti.func
    def get_other_field_human(self, current_position, blocker: SlmBlocker) -> ti.types.f32:
        other_field: ti.f32 = 0.0

        cos_a, sin_a = self.get_trigonometry_by_rad(current_position, blocker.position, blocker.orientation_rad)

        f_a: ti.f32 = 0.0
        if not cos_a < 0:
            f_a = cos_a

        a = self.sifm_params.a
        b = self.sifm_params.b
        c = self.sifm_params.c

        other_field = self.sifm_params.m1 * f_a + self.sifm_params.n1 + c * (a * b) / tm.sqrt(
            (a * cos_a * a * cos_a) + (b * sin_a * b * sin_a))

        return other_field

    @ti.func
    def get_other_field_nonhuman(self, current_position, blocker: SlmBlocker, distance_to_blocker) -> ti.types.f32:
        other_field = 0.0
        d_inside = 0.0

        abs_diff = abs(blocker.position - current_position)
        common_multiplier = distance_to_blocker / 2

        if abs_diff[0] * blocker.a <= abs_diff[1] * blocker.b:
            d_inside = blocker.a / abs_diff[1] * common_multiplier
        else:
            d_inside = blocker.b / abs_diff[0] * common_multiplier

        other_field = blocker.c * d_inside

        return other_field

    @ti.func
    def get_trigonometry_by_rad(self, pos_a, pos_b, ori_rad: ti.f32):
        pos_c = pos_b + ti.Vector([ti.sin(ori_rad), -1 * ti.cos(ori_rad)])
        vec1 = pos_a - pos_b
        vec2 = pos_c - pos_b
        cos = ti.math.dot(vec1, vec2) / (vec1.norm() * vec2.norm())
        sin = ti.sqrt(1 - cos * cos)

        return cos, sin

    def plot(self,
             is_show=True,
             is_save=False,
             file_name: str | None = None,
             show_energy: bool = True,
             show_path: bool = True,
             fix_inf: bool = True,
             cmap: str = 'RdBu_r'
             ):

        assert is_save or is_show, "At least one of is_save and is_show should be True."
        if is_save:
            assert file_name is not None, "file_name should not be None if is_save is True."

        x = np.arange(1, self.points_per_dim[0] + 1)
        y = np.arange(1, self.points_per_dim[1] + 1)
        x_grid, y_grid = np.meshgrid(x, y)

        if show_energy:
            map_val = self.energy_map
            levels = np.arange(0, 1.02, 0.02)
        else:
            map_val = self.velocity_map
            levels = np.arange(0, 1000, 1)

        if fix_inf:
            map_val[map_val > 1] = 1

        map_val = map_val.T

        contourf_plot = plt.contourf(x_grid, y_grid, map_val, levels=levels, cmap=cmap)
        plt.axis('equal')
        plt.colorbar(contourf_plot)

        if show_path:
            plt.plot(self.path[:, 0], self.path[:, 1], 'r-', linewidth=2)

        if is_save:
            plt.savefig(file_name, dpi=600)

        if is_show:
            plt.show()
