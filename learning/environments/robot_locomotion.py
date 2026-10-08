import sys, os
base_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../')
sys.path.append(base_dir)
sys.path.append(os.path.join(base_dir, 'rl'))

import numpy as np
import gymnasium as gym
from gymnasium import utils, spaces
from gymnasium.utils import seeding
from os import path
import copy

from simulation.simulation_utils import *
from common.common import *
import tasks

class RobotLocomotionEnv(gym.Env):
    def __init__(self, args):
        # init task and robot
        print("Initializing RobotLocomotion")
        print("Task: ", args.task)
        task_class = getattr(tasks, args.task)
        self.task = task_class()
        self.robot = build_robot(args)
        
        # get init pos
        self.robot_init_pos, has_self_collision = presimulate(self.robot)
        
        if has_self_collision:
            print_error('robot design has self collision')

        # init simulation
        self.sim = make_sim_fn(self.task, self.robot, self.robot_init_pos)
        self.robot_index = self.sim.find_robot_index(self.robot)

        # init objective function
        self.objective_fn = self.task.get_objective_fn()

        # init frame skip
        self.frame_skip = self.task.interval

        # define action space and observation space
        self.action_dim = self.sim.get_robot_dof_count(self.robot_index)
        self.action_range = np.array([-np.pi, np.pi])
        self.action_space = spaces.Box(low = np.full(self.action_dim, -1.0), 
            high = np.full(self.action_dim, 1.0), dtype = np.float32)

        observation = self.get_obs()
        self.observation_space = spaces.Box(low = np.full(observation.shape, -np.inf), 
            high = np.full(observation.shape, np.inf), dtype = np.float32)

        # init seed
        self.seed()

        self.viewer = None

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def set_frame_skip(self, frame_skip):
        self.frame_skip = frame_skip

    def reset(self, seed = None, options = None):

        super().reset(seed=seed)

        self.sim.remove_robot(0)
        self.sim.add_robot(self.robot, self.robot_init_pos, rd.Quaterniond(0.0, 0.0, 1.0, 0.0))
        self.robot_index = self.sim.find_robot_index(self.robot)
        assert self.robot_index == 0
    
        return self.get_obs(), self.get_info()

    def get_obs(self):
        state = get_robot_state(self.sim, self.robot_index)
        # obs = deepcopy(state)
        obs = np.hstack((state[0:9], state[10], state[12:])) # remove x, z positions from observation
        return obs

    def get_info(self):
        return {}

    def compute_reward(self):
        state = get_robot_state(self.sim, self.robot_index)
        base_R = np.reshape(state[0:9], (3, 3))
        base_pos = state[9:12]
        base_vel = state[12:18]

        base_x_axis, target_x_axis = base_R[:, 0], np.array([-1., 0., 0.])
        base_y_axis, target_y_axis = base_R[:, 1], np.array([0., 1., 0.])
        # base_z_axis, target_z_axis = base_R[:, 2], np.array([0., 0., -1.])

        # reward = base_vel[3]
        # reward = base_vel[3] + np.dot(base_x_axis, target_x_axis) * 0.1 + np.dot(base_y_axis, target_y_axis) * 0.1
        reward = base_vel[3] + np.dot(base_x_axis, target_x_axis) * 0.1 + np.dot(base_y_axis, target_y_axis) * 0.1 - np.sum(self.last_action ** 2) / self.action_dim * 0.7
        # reward = 1.0 + base_vel[3] - np.sum(self.last_action ** 2) / self.action_dim * 0.7

        return reward

    def detect_crash(self):
        state = get_robot_state(self.sim, self.robot_index)
        base_R = np.reshape(state[0:9], (3, 3))
        base_pos = state[9:12]
        
        base_x_axis, target_x_axis = base_R[:, 0], np.array([-1., 0., 0.])
        base_y_axis, target_y_axis = base_R[:, 1], np.array([0., 1., 0.])
        base_z_axis, target_z_axis = base_R[:, 2], np.array([0., 0., -1.])
        
        if np.dot(base_x_axis, target_x_axis) < 0. or np.dot(base_y_axis, target_y_axis) < 0. or \
            np.dot(base_z_axis, target_z_axis) < 0.:
            crash = True
        else:
            crash = False

        # crash = False

        return crash

    # control frequency is same as the simulation frequency
    # control observation is directly infered from state
    # control output action is the same as the action in simulation
    def step(self, action):
        clipped_action = np.clip(action, -1., 1.)
        
        self.last_action = deepcopy(clipped_action)

        act = clipped_action * np.pi / 2.

        reward = 0.0
        self.sim.set_joint_targets(self.robot_index, deepcopy(act.reshape(-1, 1)))
        for _ in range(self.frame_skip):
            self.sim.step()
            # reward += self.objective_fn(self.sim)
            reward += self.compute_reward()
            
        obs = self.get_obs()
        
        done = self.detect_crash()
        
        return obs, reward, done, False, self.get_info()
    

    def render(self, mode = "human"):
        self.render_mode = mode
        if self.render_mode != "human":
            return

        # Get robot bounds
        lower = np.zeros(3)
        upper = np.zeros(3)
        self.sim.get_robot_world_aabb(self.robot_index, lower, upper)
        time_step = self.task.time_step * self.frame_skip

        if self.viewer is None:
            self.viewer = rd.GLFWViewer()
            self.viewer.camera_params.position = 0.5 * (lower + upper)
            self.viewer.camera_params.yaw = 0.0
            self.viewer.camera_params.pitch = -np.pi / 6
            self.viewer.camera_params.distance = 2.0 * np.linalg.norm(upper - lower)

        target_pos = 0.5 * (lower + upper)
        camera_pos = self.viewer.camera_params.position.copy()
        camera_pos += 5.0 * time_step * (target_pos - camera_pos)
        self.viewer.camera_params.position = camera_pos
        self.viewer.update(time_step)
        self.viewer.render(self.sim)
            # sim_time += time_step


    def __del__(self):
        if self.viewer is not None:
            del self.viewer
        

