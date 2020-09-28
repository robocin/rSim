import os
from ctypes import *
from typing import Dict

import numpy as np
from numpy.ctypeslib import as_ctypes

try:
    os.environ['LD_LIBRARY_PATH'] = os.environ['LD_LIBRARY_PATH']+':'+os.getcwd()
except KeyError:
    os.environ['LD_LIBRARY_PATH'] = os.getcwd()


robosim_lib = cdll.LoadLibrary(os.path.join(os.path.dirname(__file__),
                                            'librobosim_c.so'))

robosim_lib.newWorld.argtypes = None
robosim_lib.newWorld.restype = c_void_p
robosim_lib.delWorld.argtypes = [c_void_p]
robosim_lib.delWorld.restype = None
robosim_lib.step.argtypes = [c_void_p, c_void_p]
robosim_lib.step.restype = None
robosim_lib.getState.argtypes = [c_void_p, c_void_p]
robosim_lib.getState.restype = None
robosim_lib.getFieldParams.argtypes = [c_void_p, c_void_p]
robosim_lib.getFieldParams.restype = None
robosim_lib.getEpisodeTime.argtypes = [c_void_p]
robosim_lib.getEpisodeTime.restype = int
robosim_lib.getGoalsBlue.argtypes = [c_void_p]
robosim_lib.getGoalsBlue.restype = int
robosim_lib.getGoalsYellow.argtypes = [c_void_p]
robosim_lib.getGoalsYellow.restype = int


class SimulatorVSS():
    def __init__(self, field_type:int = 0, n_robots:int = 3):
        '''
        TODO: DOCUMENTAR PARAMETROS
        '''
        self.field_type:int = field_type
        self.n_robots:int = n_robots
        self.world:object = robosim_lib.newWorld(self.field_type, self.n_robots)
        self.field_params:Dict[str, np.float64] = self.get_field_params()
        self.state_size:int = 5 + (self.n_robots * 4) + (self.n_robots * 4)
    
    def __del__(self):
        robosim_lib.delWorld(self.world)

    def get_state(self):
        '''
        TODO: DOCUMENTAR O FORMATO DO STATE
        '''
        state = np.zeros(self.state_size, dtype=np.float64)
        robosim_lib.getState(self.world, as_ctypes(state))
        return state

    def step(self, action):
        '''
        TODO DOCUMENTAR FORMATO ACTION
        '''
        action = np.array(action, dtype=np.float64)
        action = action.flatten()
        robosim_lib.step(self.world, as_ctypes(action))

    def reset(self):
        robosim_lib.delWorld(self.world)
        self.world = robosim_lib.newWorld(self.field_type, self.n_robots)
        return self.get_state()

    def get_field_params(self):
        '''
        TODO DOCUMENTAR FORMATO RETURN FIELD PARAMS
        '''
        params = np.zeros(4, dtype=np.float64)
        keys = ['field_width', 'field_length', 'goal_width', 'goal_length']
        robosim_lib.getFieldParams(self.world, as_ctypes(params))
        return {key: param for key, param in zip(keys, params)}

    def get_status(self) -> Dict[str,int]:
        '''
        TODO DOCUMENTAR FORMATO STATUS
        '''
        status:Dict[str,int] = {'goals_blue': 0, 'goals_yellow': 0, 'time': 0}
        status['goals_blue'] = robosim_lib.getGoalsBlue(self.world)
        status['goals_yellow'] = robosim_lib.getGoalsYellow(self.world)
        status['time'] = robosim_lib.getEpisodeTime(self.world)
        return status
