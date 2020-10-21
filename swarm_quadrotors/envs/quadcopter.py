import gym,copy
from gym import error, spaces, utils
from gym.utils import seeding

import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from flightsim.crazyflie_params import quad_params
from flightsim.simulate import Quadrotor
from flightsim.control import SE3Control as Control


class Quadcopter(gym.Env):
    def __init__(self,init,obst):
        """
        (Perform quadcopter initialization)
        Inputs:
            init, a dict defining the swarm's initial conditions with keys
                x, position, m, shape=(3,N)
                v, linear velocity, m/s, shape=(3,N)
                q, quaternion [i,j,k,w], shape=(4,N)
                w, angular velocity, rad/s, shape=(3,N)
            obst, a list of arrays denoting obstacles
=
        """
        self.Quadrotor = Quadrotor(quad_params)
        self.Control = Control(quad_params)
        self.T_control = 1/500 # in seconds, determines control loop frequency
        self.states = copy.deepcopy(init)
        self.init_states = copy.deepcopy(init)
        self.reward = 0
        self.done = False
        self.obstacles = obst


    def step(self, action, point, r=0.15):
        c = self.Control.update(self.states, action)
        self.states = self.Quadrotor.step(self.states, c['cmd_motor_speeds'], self.T_control)

        self.reward += np.linalg.norm(self.states['x']-point)
        if self.collision_detection(self.states['x']):
            self.done = True
            self.reward += 1000

        return self.states,self.reward,self.done


    def reset(self):
        self.states = copy.deepcopy(self.init_states)
        self.reward = 0
        self.done = False

    def render(self, mode='human'):
        pass


    def close(self):
        pass

    # --------------------------------------------

    def collision_detection(self,p):
        for obs in self.obstacles:
            if Polygon(obs).contains(Point(p)):
                return 1
        return 0

    def plot_obstacles(self):
        for obst in self.obstacles:
            plt.fill(*Polygon(obst).exterior.xy, 'grey')

    def in_bounds(self,p, space):
        return Polygon(space).contains(Point(p))