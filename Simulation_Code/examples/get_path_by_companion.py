from datatype.companion import CompanionCondition
from navigator.companion import CompanionPathFinder


if __name__ == '__main__':
    solver = CompanionPathFinder()

    condition = CompanionCondition(
        start=[0, 1],
        target=[6.0, 3.5],
        blocker_positions=[[3.4, 2.15]],
        blocker_orientations=[0],
        blocker_sizes=[[0.5, 0.5]],
        is_with_human=False
    )

    solver.get_path(condition)

    print('done.')

