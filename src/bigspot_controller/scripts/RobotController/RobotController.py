#!/usr/bin/evn python3
#Author: lnotspotl
#Modified: Add LayDownController and BehaviorState.LAYDOWN, start from lying down position

import numpy as np
import tf
import rospy

from sensor_msgs.msg import Range
from sensor_msgs.msg import Joy

from . StateCommand import State, Command, BehaviorState
from . RestController import RestController
from . LayDownController import LayDownController
from . TrotGaitController import TrotGaitController
from . CrawlGaitController import CrawlGaitController
from . DanceController import DanceController
from . StandController import StandController
from . StandUpController import StandUpController

from RobotSensors.RgbSensorController import RgbSensorController

class Robot(object):
    def __init__(self, body, legs, imu):
        self.body                   = body
        self.legs                   = legs

        self.delta_x                = self.body[0] * 0.5
        self.delta_y                = self.body[1] * 0.5 + self.legs[1]
        self.x_shift_front          = 0.024
        self.x_shift_back           = -0.06
        self.default_height         = 0.2

        self.rgbController          = RgbSensorController()

        # Initialize controllers
        self.layDownController      = LayDownController(self.default_stance)
        self.restController         = RestController(self.default_stance)
        self.trotGaitController     = TrotGaitController(self.default_stance, stance_time = 0.18, swing_time = 0.24, time_step = 0.02, use_imu = imu)
        self.crawlGaitController    = CrawlGaitController(self.default_stance, stance_time = 0.55, swing_time = 0.45, time_step = 0.02)
        
        #self.standController        = StandController(self.default_stance)
        self.standUpController      = StandUpController(self.default_stance)
        
        self.danceController        = DanceController(self.default_stance)

        # ⭐ START FROM LYING DOWN POSITION
        self.currentController      = self.layDownController
        self.state                  = State(self.default_height)
        self.state.foot_locations   = self.default_stance
        self.command                = Command(self.default_height)
        self.lastButtonMsg          = []

    def change_controller(self):
        """Handle state transitions between different controllers"""
        
        # Transition from LAYDOWN to REST or other states
        if self.state.behavior_state == BehaviorState.LAYDOWN:
            # Wait until lying-down animation is complete
            if self.state.ticks >= 30:  # STEP_MAX = 30
                self.state.behavior_state = BehaviorState.REST
                self.currentController = self.restController
                self.currentController.pid_controller.reset()
                self.state.ticks = 0
                rospy.loginfo("Transitioned from LAYDOWN to REST")
        
        # REST state transitions
        elif self.command.rest_event:
            if self.state.behavior_state != BehaviorState.REST:
                self.state.behavior_state = BehaviorState.REST
                self.currentController = self.restController
                self.currentController.pid_controller.reset()
            self.command.rest_event = False

        # TROT gait transition
        elif self.command.trot_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.TROT
                self.currentController = self.trotGaitController
                self.currentController.pid_controller.reset()
                self.state.ticks = 0
                rospy.loginfo("Transitioned to TROT")
            self.command.trot_event = False

        # CRAWL gait transition
        elif self.command.crawl_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.CRAWL
                self.currentController = self.crawlGaitController
                self.currentController.first_cycle = True
                self.state.ticks = 0
                rospy.loginfo("Transitioned to CRAWL")
            self.command.crawl_event = False


    def ultrasonic_command(self, msg):
        """Handle ultrasonic sensor input for obstacle avoidance"""

        distance        = 100
        try:
            distance1   = msg.range * 100
            distance2   = msg.range * 100
            distance    = np.min([distance1, distance2])
        except:
            pass

        # Color feedback based on distance
        sColor          = 'FFFFFF'
        if distance <= 20:
            sColor      = 'FF0000'  # Red - too close
        elif distance <= 30:
            sColor      = '0000FF'  # Blue - caution
        elif distance <= 40:
            sColor      = '00FF00'  # Green - safe
        elif distance <= 50:
            sColor      = '66CC00'  # Yellow-green
        elif distance <= 80:
            sColor      = '00FF00'  # Green - very safe

        self.rgbController.set_color_rgb(sColor)

    def joystick_command(self, msg):
        """Handle joystick input for robot control"""

        if len(msg.buttons) > 0:
            
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

            elif msg.buttons[4]:            # crawl
                self.command.trot_event     = False
                self.command.crawl_event    = True
                self.command.ready_event    = False
                self.command.dance_event    = False
                self.command.rest_event     = False
                self.command.standup_event  = False
                self.command.stand_event    = True
                print("crawl")

            self.currentController.updateStateCommand(msg, self.state, self.command)
            self.lastButtonMsg  = msg.buttons

    def rgb_controller(self):
        print('rgb')

    def imu_orientation(self, msg):
        """Process IMU orientation data"""
        q = msg.orientation
        rpy_angles = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.state.imu_roll = rpy_angles[0]
        self.state.imu_pitch = rpy_angles[1]

    def run(self):
        """Main control loop - run current controller"""
        return self.currentController.run(self.state, self.command)

    @property
    def default_stance(self):
        # FR, FL, RR, RL
        return np.array([[self.delta_x + self.x_shift_front, self.delta_x + self.x_shift_front, -self.delta_x + self.x_shift_back, -self.delta_x + self.x_shift_back],
                         [-self.delta_y,                      self.delta_y,                      -self.delta_y,                     self.delta_y                    ],
                         [0,                                  0,                                 0,                                 0                                ]])

    @property
    def default_stance_org(self):
        # FR, FL, RR, RL
        return np.array([[self.delta_x + self.x_shift_front, self.delta_x + self.x_shift_front, -self.delta_x + self.x_shift_back, -self.delta_x + self.x_shift_back],
                         [-self.delta_y,                      self.delta_y,                      -self.delta_y,                      self.delta_y                    ],
                         [0,                                  0,                                 0,                                 0                                ]])
