import matlab.engine
from loguru import logger
from numpy import degrees

from datatype.base import Collision, Path
from datatype.companion import CompanionCondition
from datatype.condition import TrialCondition
from navigator.companion import CompanionPathFinder
from pipelines.base import Simulation

M_FILES_PATH = r'C:\Users\backup\Downloads\sifm_new-rvo2\sifm_new-rvo2\navigator\companion'


class COMPANION(Simulation):
    __matlab__inited: bool = False
    matlab_instance = None

    def __init__(self, batch: int, with_tom: bool = False):
        super().__init__(method='companion', batch=batch, with_tom=with_tom)
        logger.info("Starting MATLAB Engine")
        while not COMPANION.__matlab__inited:
            try:
                COMPANION.matlab_instance = matlab.engine.start_matlab()
                COMPANION.matlab_instance.cd(M_FILES_PATH, nargout=0)
            except Exception as e:
                logger.warning("MATLAB Engine Failed to Start, Retrying...")
            else:
                logger.info("MATLAB Engine Started")
                COMPANION.__matlab__inited = True
                break
        self.solver = CompanionPathFinder(self.matlab_instance)

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

        start = list(self.__cm_to_m(start))
        end = list(self.__cm_to_m(end))

        blockers = condition['blockers']
        blocker_positions = []
        blocker_orientations = []
        blocker_sizes = []

        is_with_human = False

        for blocker in blockers:
            blocker_positions.append(list(self.__cm_to_m(blocker['position'])))
            blocker_size = list(self.__cm_to_m(blocker['size']))
            blocker_sizes.append([blocker_size[0]/2, blocker_size[1]/2])
            blocker_orientations.append(0.0)

        if collision_info is not None and not collision_info['has_collided']:
            blocker_positions.append(list(self.__cm_to_m(collision_info['position'])))
            blocker_sizes.append([0.24, 0.12])
            blocker_orientations.append(degrees(collision_info['orientation_rad']))
            is_with_human = True

        path_condition = CompanionCondition(start=start,
                                            target=end,
                                            blocker_positions=blocker_positions,
                                            blocker_sizes=blocker_sizes,
                                            blocker_orientations=blocker_orientations,
                                            is_with_human=is_with_human)

        path = self.solver.get_path(path_condition)
        path = self.adjust_path_steps(path, len(path) * 15)
        return path
