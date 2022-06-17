#!/usr/bin/env python3
#Author: lnotspotl

import rospy
import numpy as np
import time
from RoboticsUtilities.Transformations import rotxyz
from .PIDController import PID_controller
from . GaitController import GaitController

class ReadyController(GaitController):
    def __init__(self, default_stance, stance_time, swing_time, time_step):
        self.def_stance = np.copy(default_stance)

        self.index = 1
        self.rate       = rospy.Rate(3000)
        self.first      = True
        


    def updateStateCommand(self, msg, state, command):
        # local body position

        if self.first:
            for i in range(1,5):
                self.index  = i
                time.sleep(0.22)

        self.first  = False
    @property
    def default_stance(self):
        return self.def_stance

    def step(self, state, command):
        # state.robot_height = 0.1
        temp    = self.default_stance
        if command.ready_event:
            temp[2] = [command.robot_height/self.index] * 4
        
        return temp

    def run(self, state, command):
        #state.robot_height = 0.01
        state.foot_locations = self.step(state, command)
        return state.foot_locations
