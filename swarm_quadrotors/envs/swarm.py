'''
TO DO:
1) Complete step function, define reward and done
2)  Complete render and close functions
'''

import gym,copy
from gym import error, spaces, utils
from gym.utils import seeding


from flightsim.crazyflie_params import quad_params
from flightsim.simulate import Quadrotor
from flightsim.control import SE3Control as Control
from flightsim.desired_states import *

class Swarm(gym.Env):
    def __init__(self,init):
        """
        (Perform swarm initialization)
        Inputs:
            init, a dict defining the swarm's initial conditions with keys
                x, position, m, shape=(3,N)
                v, linear velocity, m/s, shape=(3,N)
                q, quaternion [i,j,k,w], shape=(4,N)
                w, angular velocity, rad/s, shape=(3,N)
=
        """
        self.N = len(init)
        self.Quadrotors = [Quadrotor(quad_params) for i in range(self.N)]
        self.Controls = [Control(quad_params) for i in range(self.N)]
        self.T_control = 1/500 # in seconds, determines control loop frequency
        self.states = copy.deepcopy(init)
        self.init_states = copy.deepcopy(init)


    def step(self, action):
        for i, (quad,control) in enumerate(zip(self.Quadrotors,self.Controls)):
            c = control.update(self.states[i], action[i])
            self.states[i] = quad.step(self.states[i], c['cmd_motor_speeds'], self.T_control)
        return self.states

    def reset(self):
        self.states = copy.deepcopy(self.init_states)

    # def render(self, mode='human'):
    #
    #
    # def close(self):
    # ...