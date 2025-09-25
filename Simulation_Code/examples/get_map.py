from numpy import radians

from datatype.condition import *
from datatype.sifm import *
from datatype.slm import *
from navigator.slm import SlmPathFinder


if __name__ == '__main__':
    # create map configuration
    test_map = MapParameters(width=7, height=7, precision=0.01)

    # create sifm parameters
    sifm = SIFM()

    # create two blockers
    blocker_nonhuman = SlmBlocker(is_human=0, position=ti.Vector([340, 215]), a=50, b=50, c=6.76)

    blocker_human = SlmBlocker(is_human=1,
                               position=ti.Vector([200, 180]),
                               a=sifm.a,
                               b=sifm.b,
                               c=sifm.c,
                               orientation_rad=radians(45))

    solver = SlmPathFinder(map_params=test_map, use_gpu=False)

    solver.blockers = (blocker_nonhuman, blocker_human)
    solver.solve_map()
    solver.plot(show_path=False)

    energy_map = solver.energy_map

    print(energy_map.shape)
