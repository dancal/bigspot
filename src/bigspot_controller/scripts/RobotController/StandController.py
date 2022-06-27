#!/usr/bin/env python3
#Author: lnotspotl

from cgi import print_arguments
import rospy
import numpy as np
import time
import threading

from RoboticsUtilities.Transformations import rotxyz
#from .PIDController import PID_controller

class StandController(object):
    DOWN    = False
    def __init__(self, default_stance):
        self.def_stance = default_stance
        self.max_reach = 0.065

        self.FR_X       = 0.
        self.FR_Y       = 0.
        self.FL_X       = 0.
        self.FL_Y       = 0.
        self.STEP       = 1
        
    def updateStateCommand(self, msg, state, command):

        state.body_local_position[0] = -1 * 0.15

        self.FR_X       = msg.axes[1]
        self.FR_Y       = msg.axes[0]

        self.FL_X       = msg.axes[4]
        self.FL_Y       = msg.axes[3]
        
    @property
    def default_stance(self):
        a = np.copy(self.def_stance)
        return a

    def run(self, state, command):

        temp        = self.default_stance
        temp[2]     = [command.robot_height/2] * 4
        
        temp[1][0] += self.FR_Y * self.max_reach
        temp[0][0] += self.FR_X * self.max_reach

        temp[1][1] += self.FL_Y * self.max_reach
        temp[0][1] += self.FL_X * self.max_reach
  
        #state.foot_locations = self.step(state, command)
        state.foot_locations = temp
        return state.foot_locations
