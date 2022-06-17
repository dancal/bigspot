#!/usr/bin/env python3

# sudo pip3 install adafruit-circuitpython-rgbled
# https://github.com/mmabey/CircuitPython_HCSR04
import RPi.GPIO as GPIO
import time
import sys
import signal
import rospy
import board
import numpy as np

import adafruit_hcsr04
#from hcsr04 import HCSR04

from sensor_msgs.msg import Joy
from std_msgs.msg import String

class UltraSonic:

    sonar   = None
    trig = 27 # 7th
    echo = 17 # 6th
    
    rate    = 10
    distance_publisher  = None

    rgb_led     = None
    setdistance = 0 

    def led_on(self, pin):
        gpio.setmode(gpio.BOARD)
        gpio.setup(pin, gpio.OUT)

        gpio.output(pin, True)

    def led_off(pin):
        gpio.setmode(gpio.BOARD)
        gpio.setup(pin, gpio.OUT)

        gpio.cleanup(pin)

    def __init__(self, rate):

        rospy.init_node('ultrasonic_node', anonymous=True)
        rospy.loginfo(f"UltraSonic Sensor Init")

        self.sonarL             = adafruit_hcsr04.HCSR04(trigger_pin=board.D16, echo_pin=board.D19)
        self.sonarR             = adafruit_hcsr04.HCSR04(trigger_pin=board.D20, echo_pin=board.D21)

        self.rgb_publisher      = rospy.Publisher('notspot_rgb/rgb_dist', String, queue_size=1)
        self.distance_publisher = rospy.Publisher('notspot_ultrasonic/sonic_dist', Joy, queue_size=1)
        
        self.rate               = rospy.Rate(rate)

    def hex_to_rgb(self, hex):
        return tuple(int(hex[i:i+2], 16) for i in (0, 2, 4))

    def run(self):

        # LEFT SONA
        try:
            distance            = 100
            Mode                = False
            while not rospy.is_shutdown():

                try:
                    distance1   = self.sonarL.distance
                    distance2   = self.sonarR.distance
                    distance    = np.min([distance1, distance2])
                except:
                    pass
                    
                if distance <= 10:
                    Mode        = True
                    for i in range(3):
                        joy         = Joy()
                        back_step   = -1
                        #joy.buttons = [0,0,1,0,0,0,0,0,0,0,0]
                        joy.axes    = [0.,0,0.,0.,0,0.,0.,back_step]
                        self.distance_publisher.publish(joy)
                        time.sleep(0.2)

                if distance > 50 and Mode == True:
                    joy         = Joy()
                    #joy.buttons = [1,0,0,0,0,0,0,0,0,0,0]
                    joy.axes    = [0.,0,0.,0.,0,0.,0.,0.]
                    self.distance_publisher.publish(joy)
                    Mode    = False

                #print(distance)

                if distance <= 10:
                    self.rgb_publisher.publish('FF0000')
                elif distance <= 20:
                    self.rgb_publisher.publish('0000FF')
                elif distance <= 30:
                    self.rgb_publisher.publish('00FF00')
                elif distance <= 40:
                    self.rgb_publisher.publish('FFFF00')
                elif distance <= 50:
                    self.rgb_publisher.publish('00BFFF')
                elif distance <= 60:
                    self.rgb_publisher.publish('FF9900')
                else:
                    self.rgb_publisher.publish('FFFFFF')

                self.rate.sleep()
                time.sleep(0.1)

        except KeyboardInterrupt:
            pass

if __name__ == "__main__":
    sonic = UltraSonic(rate = 30)
    sonic.run()