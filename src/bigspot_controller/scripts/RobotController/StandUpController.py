#!/usr/bin/env python3
#Author: lnotspotl

from cgi import print_arguments
import rospy
import numpy as np
import time
import threading

from RoboticsUtilities.Transformations import rotxyz
#from .PIDController import PID_controller

class StandUpController(object):
    DOWN    = False
    def __init__(self, default_stance):
        self.def_stance = default_stance
        self.max_reach = 0.065
        self.FR_X       = 0.
        self.FR_Y       = 0.
        self.FL_X       = 0.
        self.FL_Y       = 0.

        self.STEP       = 0.15
        self.STEP_MAX   = 1
        self.ISDOWN     = False

    def updateStateCommand(self, msg, state, command):

        state.body_local_position[0] =  state.ticks * 0.05
        
    @property
    def default_stance(self):
        a = np.copy(self.def_stance)
        return a

    def run(self, state, command):

        temp        = self.default_stance
        state.ticks = state.ticks + 1

        temp[0]     = [command.robot_height/state.ticks] * 4
        #temp[0][2]  = state.ticks * 0.1
        #temp[0][3]  = state.ticks * 0.1

        #temp[1][2]  = -state.ticks * 0.1
        #temp[1][3]  = -state.ticks * 0.1

        #state.foot_locations = self.step(state, command)
        state.ticks  += self.STEP
        if state.ticks > self.STEP_MAX:
            state.ticks  = self.STEP_MAX

        time.sleep(0.2)
        print(state.ticks)
        state.foot_locations = temp
        return state.foot_locations
