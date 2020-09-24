from numpy.ctypeslib import as_ctypes
import numpy as np
from ctypes import *
import os

os.environ['LD_LIBRARY_PATH'] = os.environ['LD_LIBRARY_PATH']+':'+os.getcwd()


robosim_lib = cdll.LoadLibrary(os.path.join(os.path.dirname(__file__),
                                            'librobosim_c.so'))

robosim_lib.World_new.argtypes = None
robosim_lib.World_new.restype = c_void_p
robosim_lib.World_del.argtypes = [c_void_p]
robosim_lib.World_del.restype = None
robosim_lib.step.argtypes = [c_void_p, c_void_p, c_void_p]
robosim_lib.step.restype = None


class RoboSim(object):
    def __init__(self):
        self.obj = robosim_lib.World_new()
        self.state = None

    def __del__(self):
        robosim_lib.World_del(self.obj)

    def step(self, action):
        if self.state is None:
            self.state = np.zeros(29, dtype=np.float64)
        action = np.array(action, dtype=np.float64)
        action = action.flatten()
        robosim_lib.step(self.obj, as_ctypes(action), as_ctypes(self.state))
        return self.state
