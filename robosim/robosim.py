import os
from ctypes import *

import gym
import numpy as np
from gym.spaces import Box
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
robosim_lib.getDone.argtypes = [c_void_p]
robosim_lib.getDone.restype = bool
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


class SimEnv(gym.Env):
    def __init__(self):
        self.world = robosim_lib.newWorld()
        self.field_params = None
        self.action_space = Box(-1.0, 1.0, (6, 2), dtype=np.float64)
        self.observation_space = Box(-1.0, 1.0, (29, ), dtype=np.float64)
        self.set_field_params()

    def __del__(self):
        robosim_lib.delWorld(self.world)

    def get_state(self):
        state = np.zeros(29, dtype=np.float64)
        robosim_lib.getState(self.world, as_ctypes(state))
        return state

    def step(self, action):
        action = np.array(action, dtype=np.float64)
        action = action.flatten()
        robosim_lib.step(self.world, as_ctypes(action))
        state = self.get_state()
        done = robosim_lib.getDone(self.world)
        return state, done

    def reset(self):
        robosim_lib.delWorld(self.world)
        self.world = robosim_lib.newWorld()
        return self.get_state()

    def set_field_params(self):
        params = np.zeros(4, dtype=np.float64)
        keys = ['field_width', 'field_length', 'goal_width', 'goal_length']
        robosim_lib.getFieldParams(self.world, as_ctypes(params))
        self.field_params = {key: param for key, param in zip(keys, params)}

    def get_status(self):
        status = {'goals_blue': 0, 'goals_yellow': 0, 'time': 0}
        status['goals_blue'] = robosim_lib.getGoalsBlue(self.world)
        status['goals_yellow'] = robosim_lib.getGoalsYellow(self.world)
        status['time'] = robosim_lib.getEpisodeTime(self.world)
        return status

    def render(self):
        return
