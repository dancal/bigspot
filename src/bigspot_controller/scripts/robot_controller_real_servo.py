#!/usr/bin/env python3
#Author: lnotspotl

from xmlrpc.client import FastMarshaller
import rospy
import time

from sensor_msgs.msg import Joy,Imu
from RobotController import RobotController
from InverseKinematics import robot_IK
from RobotHardwares import RobotHardwares 
from std_msgs.msg import Float64
#from time import sleep, time
import numpy as np
#import pygame

USE_IMU = False
RATE = 60

rospy.init_node("Robot_Controller")

# Robot geometry
body = [0.4, 0.13]
legs = [0.0, 0.04, 0.183, 0.2] 

bigspot_robot       = RobotController.Robot(body, legs, USE_IMU)
inverseKinematics   = robot_IK.InverseKinematics(body, legs)
servoControllers    = RobotHardwares.ServoController()

command_topics = ["/bigspot_controller/FRS_Joint/command",
                  "/bigspot_controller/FRL_Joint/command",
                  "/bigspot_controller/FRF_Joint/command",
                  "/bigspot_controller/FLS_Joint/command",
                  "/bigspot_controller/FLL_Joint/command",
                  "/bigspot_controller/FLF_Joint/command",
                  "/bigspot_controller/RRS_Joint/command",
                  "/bigspot_controller/RRL_Joint/command",
                  "/bigspot_controller/RRF_Joint/command",
                  "/bigspot_controller/RLS_Joint/command",
                  "/bigspot_controller/RLL_Joint/command",
                  "/bigspot_controller/RLF_Joint/command"]

publishers = []
for i in range(len(command_topics)):
    publishers.append(rospy.Publisher(command_topics[i], Float64, queue_size = 10))

if USE_IMU:
    rospy.Subscriber("bigspot_imu/base_link_orientation",Imu,bigspot_robot.imu_orientation)
rospy.Subscriber("bigspot_joy/joy_ramped",Joy,bigspot_robot.joystick_command)

rate = rospy.Rate(RATE)

del body
del legs
del command_topics
del USE_IMU
#del RATE

#deg2rad         = pi/180
#rad2deg         = 180/pi
#clock           = pygame.time.Clock()    
while not rospy.is_shutdown():

    leg_positions = bigspot_robot.run()
    bigspot_robot.change_controller()

    dx      = bigspot_robot.state.body_local_position[0]
    dy      = bigspot_robot.state.body_local_position[1]
    dz      = bigspot_robot.state.body_local_position[2]
    
    roll    = bigspot_robot.state.body_local_orientation[0]
    pitch   = bigspot_robot.state.body_local_orientation[1]
    yaw     = bigspot_robot.state.body_local_orientation[2]

    try:
        # FR, FL, RR, RL
        joint_angles    = inverseKinematics.inverse_kinematics(leg_positions, dx, dy, dz, roll, pitch, yaw)
        #print(joint_angles)
    except Exception as e:
        print('inverseKinematics = ', e)
        pass

    try:
        for i in range(len(joint_angles)):
            publishers[i].publish(joint_angles[i])
    except Exception as e:
        print('publishers = ', e)
        pass

    try:
        servoControllers.move(joint_angles, bigspot_robot.command)
    except Exception as e:
        print('servoControllers = ', e)
        pass

    #clock.tick(RATE)
    #time.sleep(0.05)
    rate.sleep()
