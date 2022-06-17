#!/usr/bin/env python3

import time
import board
import sys
import signal
import rospy

from std_msgs.msg import String

import RPi.GPIO as GPIO

WHITE = [30, 30, 30]
OFF = [0, 0, 0]
RED = [100, 0, 0]
GREEN = [0, 100, 0]
BLUE = [0, 0, 100]
YELLOW = [50, 50, 0]
PURPLE = [50, 0, 50]
CYAN = [0, 50, 50]

class Squid:
	
    RED_PIN = 0
    GREEN_PIN = 0
    BLUE_PIN = 0

    red_pwm = 0
    green_pwm = 0
    blue_pwm = 0

    def __init__(self, red_pin, green_pin, blue_pin):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.RED_PIN, self.GREEN_PIN, self.BLUE_PIN = red_pin, green_pin, blue_pin

        GPIO.setup(self.RED_PIN, GPIO.OUT)
        self.red_pwm = GPIO.PWM(self.RED_PIN, 500)
        self.red_pwm.start(0)
        
        GPIO.setup(self.GREEN_PIN, GPIO.OUT)
        self.green_pwm = GPIO.PWM(self.GREEN_PIN, 500)
        self.green_pwm.start(0)
        
        GPIO.setup(self.BLUE_PIN, GPIO.OUT)
        self.blue_pwm = GPIO.PWM(self.BLUE_PIN, 500)
        self.blue_pwm.start(0)
 
    def set_red(self, brightness):
        self.red_pwm.ChangeDutyCycle(brightness)
         
    def set_green(self, brightness):
        self.green_pwm.ChangeDutyCycle(brightness)
              
    def set_blue(self, brightness):
        self.blue_pwm.ChangeDutyCycle(brightness)
        
    def set_color(self, rgb, brightness = 100):
        self.set_red(rgb[0] * brightness / 100)
        self.set_green(rgb[1] * brightness / 100)
        self.set_blue(rgb[2] * brightness / 100)
        
    def set_color_rgb(self, rgb_string):
        self.set_red(int(rgb_string[1:3], 16) / 255.0)
        self.set_green(int(rgb_string[3:5], 16) / 255.0)
        self.set_blue(int(rgb_string[5:7], 16) / 255.0)


class RgbSensor:
    # Create the RGB LED object
    rgb         = None
    color       = [255, 255, 255]
    def __init__(self, rate):

        rospy.init_node('rgb_node', anonymous=True)
        rospy.Subscriber("notspot_rgb/rgb_dist", String, self.callback_rgb)
        rospy.loginfo(f"Rgb Sensor Init")

        self.rgb    = Squid(17, 27, 22)
        self.rgb.set_color_rgb('FFFFFF')
        self.rate   = rospy.Rate(rate)

    def hex_to_rgb(self, hex):
        return tuple(int(hex[i:i+2], 16) for i in (0, 2, 4))
        
    def callback_rgb(self, msg):
        self.color  = self.hex_to_rgb(msg.data)
        self.rgb.set_color_rgb( msg.data  )
        #print('msg = ', msg, ', color = ', self.color)

    def run(self):
        while not rospy.is_shutdown():
            time.sleep(0.15)
            self.rate.sleep()

if __name__ == "__main__":
    rgbLed = RgbSensor(rate = 30)
    rgbLed.run()