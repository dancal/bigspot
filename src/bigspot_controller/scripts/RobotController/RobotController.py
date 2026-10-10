#!/usr/bin/evn python3
#Author: lnotspotl
#Modified: Add LayDownController and BehaviorState.LAYDOWN, start from lying down position

import numpy as np
import tf
import rospy
import time

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
from RoboticsUtilities.Transformations import rotxyz

from RobotSensors.RgbSensorController import RgbSensorController
from RobotHardwares.calibration import STANDING_HEIGHT_M

class Robot(object):
    def __init__(self, body, legs, imu):
        self.body                   = body
        self.legs                   = legs

        self.delta_x                = self.body[0] * 0.5
        self.delta_y                = self.body[1] * 0.5 + self.legs[1]
        self.x_shift_front          = 0.024
        self.x_shift_back           = -0.06
        self.default_height         = STANDING_HEIGHT_M

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
        self.state.behavior_state   = BehaviorState.LAYDOWN
        self.restController.use_imu = imu
        self.last_joy_time = self.last_imu_time = float('-inf')
        self.imu_required = imu
        self.stable_since = None
        self.ready = False
        self.lastButtonMsg          = []
        self.sonar_received=float('-inf')
        self.sonar_distance=None
        self.require_ultrasonic=False  # Hardware entry point explicitly enables it.

    def change_controller(self):
        """Handle state transitions between different controllers"""
        
        # Transition from LAYDOWN to REST or other states
        if self.state.behavior_state == BehaviorState.LAYDOWN:
            if self.command.standup_event and time.monotonic()-self.last_imu_time < .3:
                self.state.behavior_state = BehaviorState.STANDUP
                self.currentController = self.standUpController
                self.standUpController.reset()
            self.command.standup_event = False
        elif self.state.behavior_state == BehaviorState.STANDUP:
            if self.standUpController.done:
                self.state.behavior_state = BehaviorState.REST
                self.currentController = self.restController
                self.restController.pid_controller.reset()
                self.ready = False
                self.stable_since = None
        
        # REST state transitions
        elif self.command.rest_event:
            if self.state.behavior_state != BehaviorState.REST:
                self.state.behavior_state = BehaviorState.REST
                self.currentController = self.restController
                self.currentController.pid_controller.reset()
            self.command.rest_event = False

        # TROT gait transition
        elif self.command.trot_event:
            if self.state.behavior_state == BehaviorState.REST and self.ready:
                self.state.behavior_state = BehaviorState.TROT
                self.currentController = self.trotGaitController
                self.currentController.pid_controller.reset()
                self.state.ticks = 0
                rospy.loginfo("Transitioned to TROT")
            self.command.trot_event = False

        # CRAWL gait transition
        elif self.command.crawl_event:
            if self.state.behavior_state == BehaviorState.REST and self.ready:
                self.state.behavior_state = BehaviorState.CRAWL
                self.currentController = self.crawlGaitController
                self.currentController.first_cycle = True
                self.state.ticks = 0
                rospy.loginfo("Transitioned to CRAWL")
            self.command.crawl_event = False


    def ultrasonic_command(self, msg):
        """Handle ultrasonic sensor input for obstacle avoidance"""

        value=float(msg.range)
        self.sonar_received=time.monotonic()
        self.sonar_distance=value if np.isfinite(value) and .02<=value<=4. else None
        distance=100*value if self.sonar_distance is not None else 0.

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
        if len(msg.buttons) < 8 or len(msg.axes) < 8 or not np.isfinite(msg.axes).all():
            return
        self.last_joy_time = time.monotonic()
        if self.state.behavior_state == BehaviorState.LAYDOWN:
            if msg.buttons[0]:
                self.command.standup_event = True
            return
        if self.state.behavior_state == BehaviorState.STANDUP:
            return
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
        values = np.array([q.x,q.y,q.z,q.w])
        norm = np.linalg.norm(values)
        if not np.isfinite(values).all() or abs(norm-1.) > .05:
            return
        rpy_angles = tf.transformations.euler_from_quaternion(values/norm)
        self.state.imu_roll = rpy_angles[0]
        self.state.imu_pitch = rpy_angles[1]
        self.last_imu_time = time.monotonic()

    def run(self):
        """Main control loop - run current controller"""
        now = time.monotonic()
        fresh = now-self.last_imu_time < .3
        stable = fresh and max(abs(self.state.imu_roll),abs(self.state.imu_pitch)) < np.deg2rad(3)
        if self.state.behavior_state == BehaviorState.REST:
            self.stable_since = (self.stable_since or now) if stable else None
            self.ready = self.stable_since is not None and now-self.stable_since >= 1.
        if now-self.last_joy_time > .5 or (self.imu_required and not fresh):
            self.command.velocity[:] = 0.
            self.command.yaw_rate = 0.
            self.command.trot_event = self.command.crawl_event = False
            if self.state.behavior_state in (BehaviorState.CRAWL,BehaviorState.TROT):
                self.state.behavior_state = BehaviorState.REST
                self.currentController = self.restController
                self.restController.pid_controller.reset()
        if self.imu_required and not fresh and self.state.behavior_state != BehaviorState.LAYDOWN:
            return self.state.foot_locations.copy()
        saved_velocity=self.command.velocity.copy()
        saved_yaw=self.command.yaw_rate
        if self.require_ultrasonic and (now-self.sonar_received>.35 or self.sonar_distance is None):
            self.command.velocity[0]=min(0.,self.command.velocity[0])
        elif self.sonar_distance is not None:
            cap=.10*np.clip((self.sonar_distance-.22)/.38,0.,1.)
            self.command.velocity[0]=min(self.command.velocity[0],cap)
            if self.sonar_distance<=.22:
                self.command.yaw_rate=0.
        try:
            feet = self.currentController.run(self.state, self.command)
        finally:
            self.command.velocity[:]=saved_velocity
            self.command.yaw_rate=saved_yaw
        if self.imu_required and fresh and self.state.behavior_state == BehaviorState.CRAWL:
            correction = np.clip(-self.restController.pid_controller.run(self.state.imu_roll,self.state.imu_pitch),-.1,.1)
            feet = rotxyz(correction[0],correction[1],0) @ feet
        return feet

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
