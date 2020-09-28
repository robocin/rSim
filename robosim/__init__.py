from robosim import *
from gym.envs.registration import register

register(
    id='RoboSim-v0',
    entry_point='robosim.robosim:SimEnv'
)
