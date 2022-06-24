#!/usr/bin/env python3
#Author: lnotspotl

from cgi import print_arguments
import rospy
import numpy as np
import time
import threading

from RoboticsUtilities.Transformations import rotxyz
#from .PIDController import PID_controller

class ReadyController(object):
    IDX    = 1
    def __init__(self, default_stance, stance_time, swing_time, time_step):
        self.def_stance = default_stance
        self.max_reach = 0.065

        self.FR_X       = 0.
        self.FR_Y       = 0.
        self.FL_X       = 0.
        self.FL_Y       = 0.

        self.IDX        = 0
        self.STEP       = 0.01
        self.STEP_MAX   = 1

    def updateStateCommand(self, msg, state, command):

        self.IDX        = 0
        state.body_local_position[0] = -self.IDX  * 0.12
        #state.body_local_orientation[0] = -1 * 0.3

        #state.body_local_position[2] = 0.8 * 0.1
        #state.body_local_position[2] = 0.5
        #state.body_local_orientation[0] = -1 * 0.3

        #self.FR_X       = msg.axes[1]
        #self.FR_Y       = msg.axes[0]
        #self.FL_X       = msg.axes[4]
        #self.FL_Y       = msg.axes[3]   

    @property
    def default_stance(self):
        a = np.copy(self.def_stance)
        return a

    def run(self, state, command):

        temp        = self.default_stance
        temp[2]     = [command.robot_height*self.IDX] * 4

        self.IDX += self.STEP
        if self.IDX > self.STEP_MAX:
            self.IDX = 1

        temp[0][2]  += -1 * 0.05
        temp[0][3]  += -1 * 0.05
        temp[2][2]  -= 1 * 0.02
        temp[2][3]  -= 1 * 0.02

        #state.foot_locations = self.step(state, command)
        state.foot_locations = temp
        return state.foot_locations
