#!/usr/bin/env python3
#Author: lnotspotl

import rospy
import numpy as np

from . GaitController import GaitController

class StandController(GaitController):
    def __init__(self, default_stance):
        self.def_stance = np.copy(default_stance)
        self.max_reach  = 0.065

        self.FR_X = 0.
        self.FR_Y = 0.
        self.FL_X = 0.
        self.FL_Y = 0.
        self.RR   = 1

    def updateStateCommand(self,msg,state,command):

        state.body_local_position[0] = msg.axes[7] * 0.14;
        self.FR_X = msg.axes[1]
        self.FR_Y = msg.axes[0]

        self.FL_X = msg.axes[4]
        self.FL_Y = msg.axes[3]


    @property
    def default_stance(self):
        a = np.copy(self.def_stance)
        return a

    def step(self, state, command):

        temp        = self.def_stance

        temp[2]     = [command.robot_height/self.RR] * 4
        
        temp[1][0] += self.FR_Y * self.max_reach
        temp[0][0] += self.FR_X * self.max_reach

        temp[1][1] += self.FL_Y * self.max_reach
        temp[0][1] += self.FL_X * self.max_reach

        return temp

    def run(self,state,command):

        #state.robot_height = 0.1
        state.foot_locations = self.step(state, command)
        

        #state.foot_locations = temp
        return state.foot_locations