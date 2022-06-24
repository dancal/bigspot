#!/usr/bin/env python3
#Author: dancal

from board import SCL, SDA

#from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit

from numpy import array_equal, sin, cos, pi
import numpy as np
import math
import time

# roslaunch bigspot run_robot_gazebo_only.launch
# roslaunch bigspot run_robot_gazebo_real.launch
# roslaunch bigspot run_robot_real.launch

# roslaunch bigspot_joystick ramped_joystick.launch
# roslaunch bigspot_joystick ramped_keyboard.launch

# roslaunch bigspot_mpu6050 mpu.launch
# roslaunch bigspot_mpu9250 mpu.launch
# roslaunch bigspot_lcd lcd.launch
# roslaunch bigspot_rgb rgb.launch
# roslaunch bigspot_ultrasonic ultrasonic.launch




# apt-get install ros-noetic-joy
# apt-get install ros-noetic-imu-tools
# apt-get install libgpiod2

# pip3 install pygame
# pip3 install adafruit-circuitpython-rgbled
# pip3 install adafruit-circuitpython-motor
# pip3 install adafruit-circuitpython-servokit
# pip3 install adafruit-circuitpython-rgbled / https://github.com/adafruit/Adafruit_CircuitPython_RGBLED

class ServoItem:
    posName     = None
    pca9685     = None
    servoPin    = 0
    defAngle    = 0
    direction   = 1

    beforePos   = 0
    currentPos  = 0
    def __init__(self, posName, pca9685, servoPin, defAngle, direction, restPos):
        self.posName    = posName
        self.pca9685    = pca9685
        self.servoPin   = servoPin
        self.defAngle   = defAngle
        self.direction  = direction
        self.restPos    = restPos

        self.pca9685.servo[self.servoPin].actuation_range = 180
        self.pca9685.servo[self.servoPin].set_pulse_width_range(500, 2500)

    def deg2rad(self, deg):
        return deg * np.pi / 180.0

    def rad2deg(self, rad):
        deg = rad * 180.0 / np.pi
        if deg > 90:
            deg = 90
        elif deg < -90:
            deg = -90
        return deg

    def posChange(self, currentPos):
        if self.beforePos == currentPos:
            return False
        
        return True

    def moveLieAngle(self):
        curPos                  = int(math.ceil(((self.rad2deg(self.restPos) * self.direction) + self.defAngle)))
        #if self.servoPin == 0 or self.servoPin == 3 or self.servoPin == 6 or self.servoPin == 9:    # SHOULDER
        #    angle               = 0.000001
        #    curPos              = int(math.ceil(((self.rad2deg(angle) * self.direction) + self.defAngle)))
        #    self.currentPos     = curPos + self.restPos
        #elif self.servoPin == 1 or self.servoPin == 4: # F LEG
        #    angle               = 0.9
        #    curPos              = int(math.ceil(((self.rad2deg(angle) * self.direction) + self.defAngle)))
        #    self.currentPos     = curPos
        #elif self.servoPin == 7 or self.servoPin == 10: # R LEG
        #    angle               = 1.4
        #    curPos              = int(math.ceil(((self.rad2deg(angle) * self.direction) + self.defAngle)))
        #    self.currentPos     = curPos
        #elif self.servoPin == 2 or self.servoPin == 5 or self.servoPin == 8 or self.servoPin == 11: # FOOT
        #    angle               = -1.5
        #    curPos              = int(math.ceil(((self.rad2deg(angle) * self.direction) + self.defAngle)))
        #    self.currentPos     = curPos
        #else:
        #    return
        #
        #print('self.posName = ', self.posName, ', pos = ', self.currentPos)
        #if self.direction < 0 and self.currentPos < 0:
        #    self.currentPos     = 0

        if ( self.currentPos > 180 ):
            self.currentPos     = 180
        if ( self.currentPos < -180 ):
            self.currentPos     = -180

        self.pca9685.servo[self.servoPin].angle         = self.currentPos

    def moveAngle(self, angle):
        curPos                  = int(math.ceil(((self.rad2deg(angle) * self.direction) + self.defAngle)))
        
        self.currentPos         = curPos + self.restPos
        #if self.direction < 0 and self.currentPos < 0:
        #    self.currentPos     = 0

        if ( self.currentPos > 180 ):
            self.currentPos     = 180
        if ( self.currentPos < -180 ):
            self.currentPos     = -180

        if self.posChange(self.currentPos) == True:
            try:
                self.pca9685.servo[self.servoPin].angle = self.currentPos
                #print(self.posName, 'curPos =', curPos, 'Angle = ', self.currentPos, 'beforePos =', self.beforePos, ' == servopin == ', self.servoPin, ', angle = ', angle, 'rad2deg = ', self.rad2deg(angle))
            except Exception as ex:
                print('self.posName = ', self.posName, ', pca9685 = ', ex, ', self.currentPos = ', self.currentPos)
            
        self.beforePos      =  self.currentPos

# FR, FL, RR, RL
class ServoController:
    i2c                 = None
    FirstMove           = True
    ServoKitF           = None
    ServoKitB           = None

    servoDefAngle       = None

    servoAngle          = []
    servoAnglePre       = []

    # FR, FL, RR, RL
    servoMoters         = []

    def __init__(self):
        print("servo Moter init")
        
        self.ServoKitB          = ServoKit(channels=16, address=0x40)
        self.ServoKitF          = ServoKit(channels=16, address=0x41)
        #self.ServoKitB          = None
        #self.ServoKitF          = None

        # FRONT
        ##self.servoMoters.append( ServoItem('FLS', self.ServoKitF, 0, 90,   1, 5))     # 0
        ##self.servoMoters.append( ServoItem('FLL', self.ServoKitF, 1, 60,   1, 50))     # 1
        ##self.servoMoters.append( ServoItem('FLF', self.ServoKitF, 2, 160,  1, 20))     # 2
##
        ##self.servoMoters.append( ServoItem('FRS', self.ServoKitF, 3, 90,  -1, -10))     # 3
        ##self.servoMoters.append( ServoItem('FRL', self.ServoKitF, 4, 122, -1, -45))     # 4
        ##self.servoMoters.append( ServoItem('FRF', self.ServoKitF, 5, 22,  -1, -35))     # 5
##
        ### REAR
        ##self.servoMoters.append( ServoItem('RLS', self.ServoKitB, 0, 90,  -1, -5))     # 6
        ##self.servoMoters.append( ServoItem('RLL', self.ServoKitB, 1, 48,  1, 45))     # 7
        ##self.servoMoters.append( ServoItem('RLF', self.ServoKitB, 2, 158,  1, 10))     # 8
##
        ##self.servoMoters.append( ServoItem('RRS', self.ServoKitB, 3, 90,  -1, -3))     # 9
        ##self.servoMoters.append( ServoItem('RRL', self.ServoKitB, 4, 48,  1, -32))     # 10
        ##self.servoMoters.append( ServoItem('RRF', self.ServoKitB, 5, 22,  -1, -30))     # 11

        # FRONT
        self.servoMoters.append( ServoItem('FLS', self.ServoKitF, 0, 90,   1, 5))     # 0
        self.servoMoters.append( ServoItem('FLL', self.ServoKitF, 1, 60,   1, 40))     # 1
        self.servoMoters.append( ServoItem('FLF', self.ServoKitF, 2, 160,  1, 20))     # 2

        self.servoMoters.append( ServoItem('FRS', self.ServoKitF, 3, 90,  -1, -10))     # 3
        self.servoMoters.append( ServoItem('FRL', self.ServoKitF, 4, 122, -1, -35))     # 4
        self.servoMoters.append( ServoItem('FRF', self.ServoKitF, 5, 22,  1, -35))     # 5

        # REAR
        self.servoMoters.append( ServoItem('RLS', self.ServoKitB, 0, 90,  -1, -5))     # 6
        self.servoMoters.append( ServoItem('RLL', self.ServoKitB, 1, 48,  -1, 35))     # 7
        self.servoMoters.append( ServoItem('RLF', self.ServoKitB, 2, 158,  1, 10))     # 8

        self.servoMoters.append( ServoItem('RRS', self.ServoKitB, 3, 90,  -1, -3))     # 9
        self.servoMoters.append( ServoItem('RRL', self.ServoKitB, 4, 136,  1, -12))     # 10
        self.servoMoters.append( ServoItem('RRF', self.ServoKitB, 5, 22,  1, -30))     # 11

        #self.moveFirst()
        
    def moveFirst(self):

        for i in range(len(self.servoMoters)):
            self.servoMoters[i].moveAngle(joint_angles[i])
            time.sleep(0.05)

        time.sleep(2)

    def move(self, joint_angles, state):
        for i in range(len(self.servoMoters)):
            self.servoMoters[i].moveAngle(joint_angles[i])
            #time.sleep(0.1)
        
if __name__ == "__main__":
    ps4 = ServoController()
