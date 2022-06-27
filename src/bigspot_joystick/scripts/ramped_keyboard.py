#! /usr/bin/env python3
# -*- coding: utf-8 -*-
#
# This file presents an interface for interacting with the Playstation 4 Controller
# in Python. Simply plug your PS4 controller into your computer using USB and run this
# script!
#
# NOTE: I assume in this script that the only joystick plugged in is the PS4 controller.
#       if this is not the case, you will need to change the class accordingly.
#
# Copyright © 2015 Clay L. McLeod <clay.l.mcleod@gmail.com>
#
# Distributed under terms of the MIT license.

import os
import pprint
import pygame
import math

import rospy
from math import fabs
from numpy import array_equal
from time import sleep, time
from sensor_msgs.msg import Joy

class PS3Controller(object):
    """Class representing the PS4 controller. Pretty straightforward functionality."""

    controller      = None
    axis_data       = None
    button_data     = None
    axis_data_pre   = None
    button_data_pre = None
    hat_data        = None
    clock           = pygame.time.Clock()
    is_activated    = False
    def init(self, rate):
        """Initialize the joystick components"""

        rospy.init_node("Joystick_ramped")
        #rospy.Subscriber("joy", Joy, self.callback)
        self.publisher = rospy.Publisher("bigspot_joy/joy_ramped", Joy, queue_size = 10)
        self.rate = rospy.Rate(rate)

        self.speed_index = 0
        self.available_speeds = [3.0, 1.1, 1.2, 1.3, 1.4, 1.5]

    def listen(self):
        
        pygame.init()
        pygame.joystick.init()

        clock           = pygame.time.Clock()
        joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
        for joy in joysticks:
            print(joy.get_name(), joy.get_id(), joy.get_guid(), joy.get_instance_id())

        joystickCount   = pygame.joystick.get_count()
        if joystickCount > 0:
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()

            if not self.hat_data:
                self.hat_data = {}
                for i in range(self.controller.get_numhats()):
                    self.hat_data[i] = (0, 0)

        else:
            pygame.display.set_caption("SPOTMICRO")
            self.screen      = pygame.display.set_mode((600, 600))

        self.axis_data          = [0.,0.,1.,0.,0.,1.,0.,0.]
        self.button_data        = [0,0,0,0,0,0,0,0,0,0,0]

        self.is_activated       = False
        while not rospy.is_shutdown():
            logMessage      = ""
            if joystickCount <= 0:
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:
                        #self.axis_data      = [0.,0.,1.,0.,0.,1.,0.,0.]
                        #self.button_data    = [0,1,0,0,0,0,0,0,0,0,0]      
                        # 6 : LL
                        if event.key == pygame.K_LEFT:
                            self.axis_data[3]  = round(1.0 * self.available_speeds[self.speed_index], 2)
                        if event.key == pygame.K_RIGHT:
                            self.axis_data[3]  = round(-1.0 * self.available_speeds[self.speed_index], 2)
                        if event.key == pygame.K_UP:
                            self.axis_data[4]  = round(1.0 * self.available_speeds[self.speed_index], 3)
                        if event.key == pygame.K_DOWN:
                            self.axis_data[4]  = round(-1.0 * self.available_speeds[self.speed_index], 3)
                        if event.key == pygame.K_a:      # A
                            self.button_data    = [1,0,0,0,0,0,0,0,0,0,0]
                            logMessage          = "rest"
                        elif event.key == pygame.K_b:      # B
                            self.button_data    = [0,1,0,0,0,0,0,0,0,0,0]                     
                            logMessage          = "trot"
                        elif event.key == pygame.K_x:      # X
                            self.button_data    = [0,0,0,1,0,0,0,0,0,0,0]                   
                            logMessage          = "crawl"
                        elif event.key == pygame.K_y:      # Y
                            self.button_data    = [0,0,0,0,1,0,0,0,0,0,0]  
                            self.axis_data      = [0.,0.,1.,0.,0.,1.,0.,-0.75]               
                            logMessage          = "stand"

                    elif event.type == pygame.KEYUP:
                        #self.button_data    = [1,0,0,0,0,0,0,0,0,0,0]      
                        self.axis_data      = [0.,0.,1.,0.,0.,1.,0.,0.]

                    joy                 = Joy()
                    joy.header.stamp    = rospy.Time.now()
                    joy.axes            = self.axis_data
                    joy.buttons         = self.button_data
                    print(joy)
                    self.publisher.publish(joy)

            else:
                # JoyStick
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN and event.button == 11:
                        # Press START/OPTIONS to enable the servos
                        self.axis_data          = [0.,0.,1.,0.,0.,1.,0.,0.]
                        if self.is_activated:
                            rospy.loginfo('STOP')
                            self.is_activated   = False
                        else:
                            rospy.loginfo('START')
                            self.is_activated   = True
                            #self.axis_data      = [0.,0.,1.,0.,0.,1.,0.,0.]
                            self.button_data    = [1,0,0,0,0,0,0,0,0,0,0]
                            logMessage          = 'rest'

                    if not self.is_activated:
                        rospy.loginfo('Press START/OPTIONS to enable the servos')
                        continue
                    
                    if event.type == pygame.JOYAXISMOTION:
                        self.axis_data[event.axis]  = round(event.value,2) * -1
                        # * self.available_speeds[self.speed_index])
                        #self.button_data    = [1,0,0,0,0,0,0,0,0,0,0]
                    elif event.type == pygame.JOYBUTTONUP:

                        if event.button == 8:   
                            self.use_button = False
                            self.button_data    = [0,0,0,0,0,0,0,0,0,0,0]
                        if event.button == 9:   
                            self.use_button = True
                            self.button_data    = [0,0,0,0,0,0,0,1,0,0,0]
                        elif event.button == 6:        # return
                            self.speed_index += 1
                            if self.speed_index >= len(self.available_speeds):
                                self.speed_index = 0
                            logMessage          = "Joystick speed: " + str(self.available_speeds[self.speed_index])
                        elif event.button == 7:      # list
                            self.speed_index -= 1
                            if self.speed_index < 0:
                                self.speed_index = 0
                            logMessage          = "Joystick speed: " + str(self.available_speeds[self.speed_index])

                    elif event.type == pygame.JOYBUTTONDOWN:
                        if event.button == 0:      # A
                            self.button_data    = [1,0,0,0,0,0,0,0,0,0,0]
                            logMessage          = "rest"
                        elif event.button == 1:      # B
                            self.button_data    = [0,1,0,0,0,0,0,0,0,0,0]                     
                            logMessage          = "trot"
                        elif event.button == 3:      # X
                            self.button_data    = [0,0,0,1,0,0,0,0,0,0,0]                   
                            logMessage          = "crawl"
                        elif event.button == 4:      # Y
                            self.axis_data      = [0.,0.,1.,0.,0.,1.,0.,-0.70]
                            self.button_data    = [0,0,0,0,1,0,0,0,0,0,0]                 
                            logMessage          = "stand"
                        #elif event.button == 8:      # Home
                        #    self.button_data    = [0,0,0,0,0,0,0,1,0,0,0]                 
                        #    logMessage          = "home"

                    elif event.type == pygame.JOYHATMOTION:
                        self.hat_data[event.hat] = event.value

                    joy                 = Joy()
                    joy.header.stamp    = rospy.Time.now()
                    joy.axes            = self.axis_data
                    joy.buttons         = self.button_data
                    self.publisher.publish(joy)
                    print(self.button_data)
                    if logMessage:
                        rospy.loginfo(logMessage)
                        logMessage              = ''

            #self.rate.sleep()
            self.clock.tick(60)

if __name__ == "__main__":
    ps4 = PS3Controller()
    ps4.init(rate = 60)
    ps4.listen()