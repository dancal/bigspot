#!/usr/bin/env python3
#Author: lnotspotl
#Modified: Improved TrotGaitController with smoother movement parameters

import numpy as np
from . GaitController import GaitController
from . PIDController import PID_controller
from RoboticsUtilities.Transformations import rotz
import time

class TrotGaitController(GaitController):
    def __init__(self, default_stance, stance_time, swing_time, time_step, use_imu):
        contact_phases = np.array([[1, 0, 1, 0],
                                   [0, 1, 0, 1]])

        super().__init__(stance_time, swing_time, time_step, contact_phases, default_stance)

        self.use_imu = use_imu

        # ⭐ IMPROVED: Slower movement for smoother operation
        self.max_x_velocity = 0.03   # Reduced from 0.06 (50% slower)
        self.max_y_velocity = 0.02   # Reduced from 0.04 (50% slower)
        self.max_yaw_rate   = 0.2    # Reduced from 0.4 (50% slower)

        self.z_error_constant = 0.08
        z_leg_lift = 0.08

        self.swingController = TrotSwingController(self.stance_ticks, self.swing_ticks, 
                                                   self.time_step, z_leg_lift, self.default_stance)
        self.stanceController = TrotStanceController(self.stance_ticks, self.swing_ticks, 
                                                     self.time_step, self.z_error_constant)

        # ⭐ PID Controller for body stability (optional IMU compensation)
        self.pid_controller = PID_controller(1.0, 1.7, 0.0)

    def updateStateCommand(self, msg, state, command):
        command.velocity[0] = msg.axes[4] * self.max_x_velocity
        command.velocity[1] = msg.axes[3] * self.max_y_velocity
        command.yaw_rate = msg.axes[0] * self.max_yaw_rate

    def step(self, state, command):
        contact_modes = self.contacts(state.ticks)

        new_foot_locations = np.zeros((3, 4))

        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]

            if contact_mode == 1:
                new_location = self.stanceController.next_foot_location(leg_index, state, command)
            else:
                swing_proportion = float(self.subphase_ticks(state.ticks)) / float(self.swing_ticks)
                new_location = self.swingController.next_foot_location(swing_proportion, 
                                                                       leg_index, state, command)

            new_foot_locations[:, leg_index] = new_location

        return new_foot_locations

    def run(self, state, command):
        state.foot_locations = self.step(state, command)
        state.ticks += 1

        if self.use_imu:
            imu_comp = self.pid_controller.run(state.imu_roll, state.imu_pitch)
            rot_mat = np.eye(4)
            # TODO: Apply imu_comp to foot locations

        state.robot_height = command.robot_height
        return state.foot_locations


class TrotSwingController(object):
    def __init__(self, stance_ticks, swing_ticks, time_step, z_leg_lift, default_stance):
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.z_leg_lift = z_leg_lift
        self.default_stance = default_stance

    def raibert_touchdown_location(self, leg_index, command):
        delta_pos_2d = command.velocity * (self.stance_ticks + self.swing_ticks) * self.time_step
        delta_pos = np.array([delta_pos_2d[0], delta_pos_2d[1], 0])

        theta = (self.stance_ticks + self.swing_ticks) * self.time_step * command.yaw_rate
        rotation = rotz(theta)

        return np.matmul(rotation, self.default_stance[:, leg_index]) + delta_pos

    def swing_height(self, swing_phase):
        if swing_phase < 0.5:
            swing_height_ = swing_phase / 0.5 * self.z_leg_lift
        else:
            swing_height_ = self.z_leg_lift * (1.0 - (swing_phase - 0.5) / 0.5)
        return swing_height_

    def next_foot_location(self, swing_prop, leg_index, state, command):
        assert swing_prop >= 0 and swing_prop <= 1

        foot_location = state.foot_locations[:, leg_index]
        swing_height_ = self.swing_height(swing_prop)
        touchdown_location = self.raibert_touchdown_location(leg_index, command)

        time_left = self.time_step * self.swing_ticks * (1.0 - swing_prop)
        velocity = (touchdown_location - foot_location) / float(time_left) * np.array([1, 1, 0])

        delta_foot_location = velocity * self.time_step
        z_vector = np.array([0, 0, swing_height_ + command.robot_height])

        return foot_location * np.array([1, 1, 0]) + z_vector + delta_foot_location


class TrotStanceController(object):
    def __init__(self, stance_ticks, swing_ticks, time_step, z_error_constant):
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.z_error_constant = z_error_constant

    def next_foot_location(self, leg_index, state, command):
        foot_location = state.foot_locations[:, leg_index]

        time_left = self.time_step * self.stance_ticks
        
        velocity = (command.velocity / self.stance_ticks) * np.array([1, 1, 0])
        delta_foot_location = velocity * self.time_step

        z_velocity = 1.0 / self.z_error_constant * (state.robot_height - foot_location[2])
        z_delta = z_velocity * self.time_step

        delta_ori = rotz(-command.yaw_rate * self.time_step)
        
        return np.matmul(delta_ori, foot_location) + np.array([delta_foot_location[0], 
                                                               delta_foot_location[1], z_delta])
