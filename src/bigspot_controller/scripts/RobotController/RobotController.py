#!/usr/bin/evn python3
#Author: lnotspotl

import rospy
import numpy as np
import tf
import math

from std_msgs.msg import String

from . StateCommand import State, Command, BehaviorState
from . LieController import LieController
from . RestController import RestController
from . TrotGaitController import TrotGaitController
from . CrawlGaitController import CrawlGaitController
from . StandController import StandController

class Robot(object):
    def __init__(self, body, legs, imu):
        self.body = body
        self.legs = legs

        self.delta_x            = self.body[0] * 0.5
        self.delta_y            = self.body[1] * 0.5 + self.legs[1]
        self.x_shift_front      = 0.06
        self.x_shift_back       = -0.18
        self.default_height     = 0.15

        self.publisher_lcd_state    = rospy.Publisher("bigspot_lcd/state", String, queue_size = 1)

        self.trotGaitController     = TrotGaitController(self.default_stance, stance_time = 0.18, swing_time = 0.25, time_step = 0.019, use_imu = imu)
        self.crawlGaitController    = CrawlGaitController(self.default_stance, stance_time = 0.55, swing_time = 0.45, time_step = 0.02)
        self.standController        = StandController(self.default_stance)

        self.restController         = RestController(self.default_stance)
        self.lieController          = LieController(self.default_stance)

        self.currentController      = self.restController
        self.state                  = State(self.default_height)
        self.state.foot_locations   = self.default_stance
        self.command                = Command(self.default_height)

    def change_controller(self):
        
        if self.command.trot_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.TROT
                self.currentController = self.trotGaitController
                self.currentController.pid_controller.reset()
                self.state.ticks = 0
            self.command.trot_event = False

        elif self.command.crawl_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.CRAWL
                self.currentController = self.crawlGaitController
                self.currentController.first_cycle = True;
                self.state.ticks = 0
            self.command.crawl_event = False

        elif self.command.stand_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.STAND
                self.currentController = self.standController
            self.command.stand_event = False

        elif self.command.rest_event:
            self.state.behavior_state = BehaviorState.REST
            self.currentController = self.restController
            self.currentController.pid_controller.reset()
            self.command.rest_event = False

        elif self.command.lie_event:
            self.state.behavior_state = BehaviorState.LIE
            self.currentController = self.lieController
            #self.currentController.pid_controller.reset()
            self.command.lie_event = False

    def joystick_command(self,msg):
        if msg.buttons[0]: # rest
            self.command.rest_event     = True
            self.command.trot_event     = False
            self.command.crawl_event    = False
            self.command.stand_event    = False
            self.command.lie_event      = False
            self.publisher_lcd_state.publish("rest")
            print("rest")

        elif msg.buttons[1]: # trot
            self.command.rest_event     = False
            self.command.trot_event     = True
            self.command.crawl_event    = False
            self.command.stand_event    = False
            self.command.lie_event      = False
            self.publisher_lcd_state.publish("trot")
            print("trot")

        elif msg.buttons[2]: # stand
            self.command.rest_event     = False
            self.command.trot_event     = False
            self.command.crawl_event    = False
            self.command.stand_event    = True
            self.command.lie_event      = False
            self.publisher_lcd_state.publish("stand")
            print("stand")

        elif msg.buttons[3]: # crawl
            self.command.rest_event     = False
            self.command.trot_event     = False
            self.command.crawl_event    = True
            self.command.stand_event    = False
            self.command.lie_event      = False
            self.publisher_lcd_state.publish("crawl")
            print("crawl")

        elif msg.buttons[10]: # lie
            self.command.rest_event     = False
            self.command.trot_event     = False
            self.command.crawl_event    = False
            self.command.stand_event    = False
            self.command.lie_event      = True
            self.publisher_lcd_state.publish("lie")
            print("lie")

        self.currentController.updateStateCommand(msg, self.state, self.command)

    def imu_orientation(self,msg):
        q = msg.orientation
        self.state.imu_roll     = q.x
        self.state.imu_pitch    = q.y

    def run(self):
        return self.currentController.run(self.state, self.command)

    @property
    def default_stance(self):
        #                 FR,                              ,FL,                              ,RR                               ,RL
        return np.array([[self.delta_x + self.x_shift_front,-self.delta_x + self.x_shift_front,self.delta_x + self.x_shift_back,-self.delta_x + self.x_shift_back],
                         [self.delta_y                     ,self.delta_y                    ,self.delta_y                    ,self.delta_y                    ],
                         [0                                ,0                                ,0                                ,0                                ]])
    @property
    def default_stance_org(self):
        #                 FR,                              ,FL,                              ,RR                               ,RL
        return np.array([[self.delta_x + self.x_shift_front,self.delta_x + self.x_shift_front,-self.delta_x + self.x_shift_back,-self.delta_x + self.x_shift_back],
                         [-self.delta_y                    ,self.delta_y                     ,-self.delta_y                    , self.delta_y                    ],
                         [0                                ,0                                ,0                                ,0                                ]])