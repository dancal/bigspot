#!/usr/bin/env python3
#Author: lnotspotl

from xmlrpc.client import FastMarshaller
import rospy

from sensor_msgs.msg import Joy,Imu
from RobotController import RobotController
from InverseKinematics import robot_IK
from RobotHardwares import RobotHardwares 
from std_msgs.msg import Float64
from time import sleep, time
import pygame
from numpy import array_equal, sin, cos, pi
import numpy as np

USE_IMU = True
RATE = 60

rospy.init_node("Robot_Controller")

# Robot geometry
body = [0.1908, 0.080]
#legs = [0.0, 0.04, 0.100, 0.094333] 
legs = [0.0, 1.47, 1.570, 1.574333] 

notspot_robot       = RobotController.Robot(body, legs, USE_IMU)
inverseKinematics   = robot_IK.InverseKinematics(body, legs)
servoControllers    = RobotHardwares.ServoController()

#command_topics = ["/notspot_controller/FR1_joint/command",
#                  "/notspot_controller/FR2_joint/command",
#                  "/notspot_controller/FR3_joint/command",
#                  "/notspot_controller/FL1_joint/command",
#                  "/notspot_controller/FL2_joint/command",
#                  "/notspot_controller/FL3_joint/command",
#                  "/notspot_controller/RR1_joint/command",
#                  "/notspot_controller/RR2_joint/command",
#                  "/notspot_controller/RR3_joint/command",
#                  "/notspot_controller/RL1_joint/command",
#                  "/notspot_controller/RL2_joint/command",
#                  "/notspot_controller/RL3_joint/command"]
  

#command_topics = ["/notspot_controller/front_right_shoulder/command",
#                  "/notspot_controller/front_right_foot/command",
#                  "/notspot_controller/front_right_leg/command",
#                  "/notspot_controller/front_left_shoulder/command",
#                  "/notspot_controller/front_left_foot/command",
#                  "/notspot_controller/front_left_leg/command",
#                  "/notspot_controller/rear_right_shoulder/command",
#                  "/notspot_controller/rear_right_foot/command",
#                  "/notspot_controller/rear_right_leg/command",
#                  "/notspot_controller/rear_left_shoulder/command",
#                  "/notspot_controller/rear_left_foot/command",
#                  "/notspot_controller/rear_left_leg/command"]

command_topics = ["/notspot_controller/front_right_shoulder/command",
                  "/notspot_controller/front_right_leg/command",
                  "/notspot_controller/front_right_foot/command",

                  "/notspot_controller/front_left_shoulder/command",
                  "/notspot_controller/front_left_leg/command",
                  "/notspot_controller/front_left_foot/command",

                  "/notspot_controller/rear_right_shoulder/command",
                  "/notspot_controller/rear_right_leg/command",
                  "/notspot_controller/rear_right_foot/command",

                  "/notspot_controller/rear_left_shoulder/command",
                  "/notspot_controller/rear_left_leg/command",
                  "/notspot_controller/rear_left_foot/command"]

publishers = []
for i in range(len(command_topics)):
    publishers.append(rospy.Publisher(command_topics[i], Float64, queue_size = 0))

if USE_IMU:
    rospy.Subscriber("notspot_imu/base_link_orientation",Imu,notspot_robot.imu_orientation)
rospy.Subscriber("notspot_joy/joy_ramped",Joy,notspot_robot.joystick_command)

rate = rospy.Rate(RATE)

del body
del legs
del command_topics
del USE_IMU
#del RATE

#deg2rad         = pi/180
#rad2deg         = 180/pi
clock           = pygame.time.Clock()
servo_angles_   = []
while not rospy.is_shutdown():
    leg_positions = notspot_robot.run()
    notspot_robot.change_controller()

    dx = notspot_robot.state.body_local_position[0]
    dy = notspot_robot.state.body_local_position[1]
    dz = notspot_robot.state.body_local_position[2]
    
    roll = notspot_robot.state.body_local_orientation[0]
    pitch = notspot_robot.state.body_local_orientation[1]
    yaw = notspot_robot.state.body_local_orientation[2]
    try:
        # self.servo_rear_shoulder_left = servo.Servo(self.pca9685_1.channels[self.servo_rear_shoulder_left_channel])
        # FR, FL, RR, RL
        joint_angles = inverseKinematics.inverse_kinematics(leg_positions, dx, dy, dz, roll, pitch, yaw)

        servo_angles    = []
        for i in range(len(joint_angles)):
            servo_angles.append( int(np.rad2deg(joint_angles[i])) )

        bEqual          = array_equal(servo_angles_, servo_angles)
        if not bEqual:
            servoControllers.move(servo_angles)

            for i in range(len(joint_angles)):
                publishers[i].publish(joint_angles[i])
            
        servo_angles_    = servo_angles
    except:
        pass

    clock.tick(RATE)
    #rate.sleep()
