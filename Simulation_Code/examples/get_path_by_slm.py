from time import perf_counter

import taichi as ti

from datatype.condition import MapParameters, PathPlan
from datatype.sifm import SIFM
from datatype.slm import SlmBlocker
from navigator.slm import SlmPathFinder


if __name__ == "__main__":
    # create map configuration
    test_map = MapParameters(width=7, height=7, precision=0.01)

    # create sifm parameters
    sifm = SIFM()

    # create two blockers
    blocker_nonhuman = SlmBlocker(
        is_human=0, position=ti.Vector([340, 215]), a=50, b=50, c=6.76
    )

    solver = SlmPathFinder(map_params=test_map, use_gpu=False)
    solver.blockers = (blocker_nonhuman,)
    solver.path_plan = PathPlan(start=(80, 2), target=(600, 430))

    t = perf_counter()
    solver.solve_map()
    print(f"Map time: {perf_counter()-t:4f}")

    t = perf_counter()
    solver.reset()
    blocker_nonhuman.position = ti.Vector([215, 340])
    solver.blockers = (blocker_nonhuman,)
    print(f"reset time: {perf_counter()-t:4f}")

    t = perf_counter()
    solver.solve_map()
    print(f"Second Map time: {perf_counter()-t:4f}")

    t = perf_counter()
    solver.solve_path()
    print(f"Path time: {perf_counter()-t:4f}")

    solver.plot()
