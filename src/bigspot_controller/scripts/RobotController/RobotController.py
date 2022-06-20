#!/usr/bin/evn python3
#Author: lnotspotl

import numpy as np
import tf
import rospy

from . StateCommand import State, Command, BehaviorState
from . ReadyController import ReadyController
from . RestController import RestController
from . TrotGaitController import TrotGaitController
from . CrawlGaitController import CrawlGaitController
from . DanceController import DanceController
from . StandController import StandController


class Robot(object):
    def __init__(self, body, legs, imu):
        self.body           = body
        self.legs           = legs

        self.delta_x        = self.body[0] * 0.5
        self.delta_y        = self.body[1] * 0.5 + self.legs[1]
        self.x_shift_front  = 0.02
        self.x_shift_back   = -0.082
        self.default_height = 0.18

        self.trotGaitController     = TrotGaitController(self.default_stance, stance_time = 0.2, swing_time = 0.24, time_step = 0.02,use_imu = imu)
        self.crawlGaitController    = CrawlGaitController(self.default_stance, stance_time = 0.55, swing_time = 0.45, time_step = 0.02)
        self.standController        = StandController(self.default_stance)
        self.danceController        = DanceController(self.default_stance)

        self.restController         = RestController(self.default_stance)
        self.readyController        = ReadyController(self.default_stance, stance_time = 0.55, swing_time = 0.45, time_step = 0.02)
        
        self.currentController      = self.restController
        self.state                  = State(self.default_height)
        self.state.foot_locations   = self.default_stance
        self.command                = Command(self.default_height)

    def change_controller(self):
        
        if self.command.rest_event:
            self.state.behavior_state = BehaviorState.REST
            self.currentController = self.restController
            self.currentController.pid_controller.reset()
            self.command.rest_event = False

        elif self.command.trot_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.TROT
                self.currentController = self.trotGaitController
                self.currentController.pid_controller.reset()
                self.state.ticks = 0
            self.command.trot_event = False

        elif self.command.dance_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.DANCE
                self.currentController = self.danceController
            self.command.dance_event = False

        #elif self.command.stand_event:
        #    if self.state.behavior_state == BehaviorState.REST:
        #        self.state.behavior_state = BehaviorState.STAND
        #        self.currentController = self.standController
        #    self.command.stand_event = False

        elif self.command.ready_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.READY
                self.currentController = self.readyController
                self.state.ticks = 0
            self.command.ready_event = False

        #elif self.command.crawl_event:
        #    if self.state.behavior_state == BehaviorState.REST:
        #        self.state.behavior_state = BehaviorState.CRAWL
        #        self.currentController = self.crawlGaitController
        #        self.currentController.first_cycle = True
        #        self.state.ticks = 0
        #    self.command.crawl_event = False


    def joystick_command(self,msg):

        if msg.buttons[0]:              # rest [PS2:A, PS3:X]
            self.command.trot_event     = False
            self.command.crawl_event    = False
            self.command.stand_event    = False
            self.command.ready_event    = False
            self.command.rest_event     = True
            rospy.loginfo(f"Rest")

        elif msg.buttons[1]:            # trot [PS2:B, PS3:O]
            self.command.trot_event     = True
            self.command.crawl_event    = False
            self.command.stand_event    = False
            self.command.ready_event    = False
            self.command.rest_event     = False
            rospy.loginfo(f"trot")

        elif msg.buttons[3]:            # dance [PS2:X, PS3:ㅁ]
            self.command.trot_event     = False
            self.command.crawl_event    = False
            self.command.stand_event    = False
            self.command.ready_event    = False
            self.command.dance_event    = True
            self.command.rest_event     = False
            rospy.loginfo(f"dance")

        #elif msg.buttons[3]: # stand
        #    self.command.trot_event     = False
        #    self.command.crawl_event    = False
        #    self.command.stand_event    = True
        #    self.command.ready_event    = False
        #    self.command.rest_event     = False
        #    print("stand")

        elif msg.buttons[4]:            # ready [PS2: Y, PS3: △]
            self.command.trot_event     = False
            self.command.crawl_event    = False
            self.command.stand_event    = False
            self.command.ready_event    = True
            self.command.rest_event     = False
            rospy.loginfo(f"ready")

        self.currentController.updateStateCommand(msg, self.state, self.command)

    def imu_orientation(self,msg):
        q = msg.orientation
        rpy_angles = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        self.state.imu_roll = rpy_angles[0]
        self.state.imu_pitch = rpy_angles[1]

    def run(self):
        return self.currentController.run(self.state, self.command)

    @property
    def default_stance(self):
        # FR, FL, RR, RL
        return np.array([[self.delta_x + self.x_shift_front,self.delta_x + self.x_shift_front,-self.delta_x + self.x_shift_back,-self.delta_x + self.x_shift_back],
                         [-self.delta_y                    ,self.delta_y                     ,-self.delta_y                    , self.delta_y                    ],
                         [0                                ,0                                ,0                                ,0                                ]])