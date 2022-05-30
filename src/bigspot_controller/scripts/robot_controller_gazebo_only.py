#!/usr/bin/env python3
#Author: lnotspotl

import rospy

from sensor_msgs.msg import Joy,Imu
from RobotController import RobotController
from InverseKinematics import robot_IK
from std_msgs.msg import Float64

USE_IMU = False
RATE = 60

rospy.init_node("Robot_Controller")
 
# Robot geometry
body = [0.399853, 0.135999]
#legs = [0.0, 0.07, 0.15, 0.03] 
#legs = [0.0, 0.11, 0.15, 0.03] 
legs = [0.0, 0.078, 0.183, 0.197] # 0.360
#legs = [0.21594267, 0.0, 0.0, 0.0] # 0.360
notspot_robot = RobotController.Robot(body, legs, USE_IMU)
inverseKinematics = robot_IK.InverseKinematics(body, legs)

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
    rospy.Subscriber("bigspot_imu/base_link_orientation",Imu,notspot_robot.imu_orientation)
rospy.Subscriber("bigspot_joy/joy_ramped",Joy,notspot_robot.joystick_command)

rate = rospy.Rate(RATE)

del body
del legs
del command_topics
del USE_IMU
del RATE

print("loop")
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
        joint_angles = inverseKinematics.inverse_kinematics(leg_positions, dx, dy, dz, roll, pitch, yaw)
        for i in range(len(joint_angles)):
            #                 FR,                              ,FL,                              ,RR                               ,RL
            #if i == 4:
            #    joint_angles[i] = joint_angles[i] * -1
            #if i == 5:
            #    joint_angles[i] = joint_angles[i] * -1
#
            #if i == 5:
            #    joint_angles[i] = joint_angles[i] * -1
#
            #if i == 10:
            #    joint_angles[i] = joint_angles[i] * -1
            #if i == 11:
            #    joint_angles[i] = joint_angles[i] * -1

            publishers[i].publish(joint_angles[i])
        #print(joint_angles)
    except Exception as e:
        print(e)
        pass

    rate.sleep()
