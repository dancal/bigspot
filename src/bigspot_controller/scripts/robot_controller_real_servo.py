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
import pygame

USE_IMU = True
RATE = 60

rospy.init_node("Robot_Controller")

# Robot geometry
body = [0.1908, 0.080]
legs = [0.0, 0.04, 0.100, 0.094333] 

notspot_robot       = RobotController.Robot(body, legs, USE_IMU)
inverseKinematics   = robot_IK.InverseKinematics(body, legs)
servoControllers    = RobotHardwares.ServoController()

command_topics = ["/notspot_controller/FR1_joint/command",
                  "/notspot_controller/FR2_joint/command",
                  "/notspot_controller/FR3_joint/command",
                  "/notspot_controller/FL1_joint/command",
                  "/notspot_controller/FL2_joint/command",
                  "/notspot_controller/FL3_joint/command",
                  "/notspot_controller/RR1_joint/command",
                  "/notspot_controller/RR2_joint/command",
                  "/notspot_controller/RR3_joint/command",
                  "/notspot_controller/RL1_joint/command",
                  "/notspot_controller/RL2_joint/command",
                  "/notspot_controller/RL3_joint/command"]

publishers = []
for i in range(len(command_topics)):
    publishers.append(rospy.Publisher(command_topics[i], Float64, queue_size = 10))

if USE_IMU:
    rospy.Subscriber("notspot_imu/base_link_orientation", Imu, notspot_robot.imu_orientation)
rospy.Subscriber("notspot_joy/joy_ramped", Joy, notspot_robot.joystick_command)

rate = rospy.Rate(RATE)

del body
del legs
del command_topics
del USE_IMU
#del RATE

#deg2rad         = pi/180
#rad2deg         = 180/pi
clock           = pygame.time.Clock()    
while not rospy.is_shutdown():

    leg_positions = notspot_robot.run()
    notspot_robot.change_controller()

    dx      = notspot_robot.state.body_local_position[0]
    dy      = notspot_robot.state.body_local_position[1]
    dz      = notspot_robot.state.body_local_position[2]
    
    roll    = notspot_robot.state.body_local_orientation[0]
    pitch   = notspot_robot.state.body_local_orientation[1]
    yaw     = notspot_robot.state.body_local_orientation[2]

    try:
        # FR, FL, RR, RL
        joint_angles    = inverseKinematics.inverse_kinematics(leg_positions, dx, dy, dz, roll, pitch, yaw)
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
        servoControllers.move(joint_angles, notspot_robot.command)
    except Exception as e:
        print('servoControllers = ', e)
        pass

    #clock.tick(RATE)
    rate.sleep()
