#!/usr/bin/env python3
#Author: lnotspotl

from cgi import print_arguments
import rospy
import numpy as np
import time
import threading

from RoboticsUtilities.Transformations import rotxyz
#from .PIDController import PID_controller

class DanceController(object):
    def __init__(self, default_stance):
        self.default_stance = default_stance
        self.max_reach  = 0.03

        self.ticks      = 1
        self.FR_X       = 0.
        self.FR_Y       = 0.
        self.FL_X       = 0.
        self.FL_Y       = 0.
        self.STEP_DEF   = 0.15
        self.STEP       = self.STEP_DEF
        self.STEP_MAX   = 3
        self.TOGGLE     = False

    def updateStateCommand(self, msg, state, command):
        # local body position
        #self.index      = 1
        self.FR_X       = msg.axes[1]
        self.FR_Y       = msg.axes[0]
        self.FL_X       = msg.axes[4]
        self.FL_Y       = msg.axes[3]

    def step(self, state, command):

        #state.body_local_position[0] = -0.09
        state.body_local_position[0] = (self.STEP * self.ticks) * 0.14

        temp        = np.copy(self.default_stance)
        temp[2]     = [command.robot_height/self.ticks] * 4

        temp[1][0] += self.FR_Y * self.max_reach
        temp[0][0] += self.FR_X * self.max_reach
        temp[1][1] += self.FL_Y * self.max_reach
        temp[0][1] += self.FL_X * self.max_reach

        return temp

    def run(self, state, command):

        if self.ticks > self.STEP_MAX:
            self.ticks  = self.STEP_MAX
            self.TOGGLE = True
            self.STEP   = -self.STEP_DEF
        elif self.ticks < 1:
            self.ticks  = 1
            self.TOGGLE = False
            self.STEP   = self.STEP_DEF
        else:
            self.ticks      = self.ticks + self.STEP

        #print(self.TOGGLE, self.ticks, self.STEP )
        state.foot_locations = self.step(state, command)
        return state.foot_locations
