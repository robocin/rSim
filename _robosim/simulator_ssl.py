import os
from ctypes import *
from typing import Dict

import numpy as np
from numpy.ctypeslib import as_ctypes


robosim_lib = cdll.LoadLibrary(os.path.join(os.path.dirname(__file__),
                                            'librobosim_c_ssl.so'))

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


class SimulatorSSL():
    '''
        RoboSim SSL Simulator.

        Based on GRSim. Transfers all the graphic functions
        to handle in python, whenever the user wants to.
        The SimulatorSSL uses a C++ interface to create the simulation
        and physics of the simulator.
    '''

    def __init__(self, field_type: int = 0,
                 n_robots_blue: int = 3,
                 n_robots_yellow: int = 3,
                 time_step_ms: int = 16,
                 ball_pos: np.ndarray = None,
                 blue_robots_pos: np.ndarray = None,
                 yellow_robots_pos: np.ndarray = None) -> None:
        '''
            Creates our Simulator object.

            Parameters
            ----------
            field_type : int
                The number corresponding to the type of the field
                0 - Div A field
                1 - Div B field
                2 - HW Challenge field

            n_robots_blue : int
                Number of blue robots

            n_robots_yellow : int
                Number of blue robots

            time_step_ms : int
                Simulation timestep in milliseconds

            ball_pos : np.ndarray
                Ball position array [ballX, ballY, ballVx, ballVy]

            yellow and blue robots_pos : np.ndarray
                Array of robot position array [[robotX, robotY, robotTheta]]

            Returns
            -------
            None
        '''
        self.field_type: int = field_type
        self.n_robots_blue: int = n_robots_blue
        self.n_robots_yellow: int = n_robots_yellow
        self.time_step_ms = time_step_ms
        # convert from list to np.ndarray
        ball_pos = np.array(ball_pos).flatten()
        blue_robots_pos = np.array(blue_robots_pos).flatten()
        yellow_robots_pos = np.array(yellow_robots_pos).flatten()
        self.world: object = robosim_lib.newWorld(self.field_type,
                                                  self.n_robots_blue,
                                                  self.n_robots_yellow,
                                                  self.time_step_ms,
                                                  as_ctypes(ball_pos),
                                                  as_ctypes(blue_robots_pos),
                                                  as_ctypes(yellow_robots_pos)
                                                  )
        self.field_params: Dict[str, np.float64] = self.get_field_params()
        self.state_size: int = 5 \
            + (self.n_robots_blue * 11)\
            + (self.n_robots_yellow * 11)

    def __del__(self):
        robosim_lib.delWorld(self.world)
        del self.world

    def get_state(self) -> np.ndarray:
        '''
            Returns the state array.
            State:
                - Ball: x, y, z, v_x, v_y
                - Robots_blue: x, y, theta, v_x, v_y, v_theta, ir, v_wheel[0->3]
                - Robots_yellow: x, y, theta, v_x, v_y, v_theta, ir, v_wheel[0->3]
                Units:
                    x, y, z     -> meters
                    theta       -> degrees (0, 360)
                    v_x, v_y    -> meters/seconds
                    v_theta     -> degrees/seconds
                    ir          -> bool
                    v_wheel     -> rad/s

            State size: 5 + 11*n_robots_blue + 11*n_robots_yellow

            Parameters
            ----------
            None

            Returns
            -------
            np.ndarray
                State
        '''
        state = np.zeros(self.state_size, dtype=np.float64)
        robosim_lib.getState(self.world, as_ctypes(state))
        return state

    def step(self, action: np.ndarray) -> None:
        '''
            Steps the simulator given an action.

            Parameters
            ----------
            action: np.ndarray
            Action of shape (n_robots, 7),
                [0] wheel_speeds
                if wheel_speeds:
                    [1->4] v_wheel[0->3]
                else:
                    [1] v_x
                    [2] v_y
                    [3] v_theta
                    [4] unused
                [5] kick_v_x
                [6] kick_v_z
                [7] dribbler
                
            Returns
            -------
            None
        '''
        action = np.array(action, dtype=np.float64)
        action = action.flatten()
        robosim_lib.step(self.world, as_ctypes(action))

    def reset(self,
              ball_pos: np.ndarray,
              blue_robots_pos: np.ndarray,
              yellow_robots_pos: np.ndarray) -> np.ndarray:
        '''
            Resets the simulator and it's render view
            if it's in use.

            Parameters
            ----------
            None

            Returns
            -------
            np.ndarray
                State
        '''
        robosim_lib.delWorld(self.world)
        self.world = robosim_lib.newWorld(self.field_type,
                                          self.n_robots_blue,
                                          self.n_robots_yellow,
                                          self.time_step_ms,
                                          as_ctypes(ball_pos.flatten()),
                                          as_ctypes(blue_robots_pos.flatten()),
                                          as_ctypes(
                                              yellow_robots_pos.flatten())
                                          )
        return self.get_state()

    def get_field_params(self) -> Dict[str, float]:
        '''
            Returns the field parameters from the given
            field type.

            Parameters
            ----------
            None

            Returns
            -------
            dict
                length, width, penalty_length, penalty_width, goal_width, 
                goal_depth, ball_radius, rbt_distance_center_kicker,
                rbt_kicker_thickness, rbt_kicker_width, rbt_wheel0_angle,
                rbt_wheel1_angle, rbt_wheel2_angle, rbt_wheel3_angle,
                rbt_radius, rbt_wheel_radius, rbt_motor_max_rpm
        '''
        params = np.zeros(17, dtype=np.float64)
        keys = ['length', 'width', 'penalty_length', 'penalty_width',
                'goal_width', 'goal_depth', 'ball_radius',
                'rbt_distance_center_kicker', 'rbt_kicker_thickness',
                'rbt_kicker_width', 'rbt_wheel0_angle', 'rbt_wheel1_angle',
                'rbt_wheel2_angle', 'rbt_wheel3_angle', 'rbt_radius',
                'rbt_wheel_radius', 'rbt_motor_max_rpm']
        robosim_lib.getFieldParams(self.world, as_ctypes(params))
        return {key: param for key, param in zip(keys, params)}
