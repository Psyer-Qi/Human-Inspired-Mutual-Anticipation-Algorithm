import matlab.engine
import numpy as np
from loguru import logger

from datatype.base import Path
from datatype.condition import TrialConditionBlocker
from datatype.companion import CompanionCondition

# change this to the path of your COMPANION matlab files
M_FILES_PATH = r'C:\Users\backup\Downloads\sifm_new-rvo2\sifm_new-rvo2\navigator\companion'


class CompanionPathFinder:
    __matlab__inited: bool = False
    matlab_instance = None

    def __init__(self, instance):
        logger.info("Starting MATLAB Engine")
        # while not CompanionPathFinder.__matlab__inited:
        #     try:
        #         CompanionPathFinder.matlab_instance = matlab.engine.start_matlab()
        #         CompanionPathFinder.matlab_instance.cd(M_FILES_PATH, nargout=0)
        #     except Exception as e:
        #         logger.warning("MATLAB Engine Failed to Start, Retrying...")
        #     else:
        #         logger.info("MATLAB Engine Started")
        #         CompanionPathFinder.__matlab__inited = True
        #         break
        self.instance = instance

    def get_path(self, condition: CompanionCondition) -> Path:
        path = self.instance.get_path(8.0,
                                             condition.start,
                                             condition.target,
                                             condition.blocker_positions,
                                             condition.blocker_orientations,
                                             condition.blocker_sizes,
                                             condition.is_with_human)
        if len(path) == 1:
            raise ValueError('path not found')

        return np.asarray(path)
