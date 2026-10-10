#!/usr/bin/env python3
"""ROS 1 dual HC-SR04P acquisition, metres, alternating pings, explicit faults."""
import math
import time
import rospy
import board
import adafruit_hcsr04
from sensor_msgs.msg import Range


class UltraSonic:
    def __init__(self,rate=8):
        rospy.init_node('ultrasonic_node')
        self.sensors=[adafruit_hcsr04.HCSR04(trigger_pin=board.D16,echo_pin=board.D19,timeout=.05),
                      adafruit_hcsr04.HCSR04(trigger_pin=board.D20,echo_pin=board.D21,timeout=.05)]
        self.publishers=[rospy.Publisher('bigspot_ultrasonic/'+name,Range,queue_size=1)
                         for name in ('left','right')]
        self.combined=rospy.Publisher('bigspot_ultrasonic/sonic_dist',Range,queue_size=1)
        self.values=[float('nan')]*2
        self.times=[float('-inf')]*2
        self.rate=rospy.Rate(min(16.,2*rate))

    def message(self,side,distance):
        result=Range()
        result.header.stamp=rospy.Time.now()
        result.header.frame_id='ultrasonic_'+side
        result.radiation_type=Range.ULTRASOUND
        result.field_of_view=.26
        result.min_range,result.max_range=.02,4.
        result.range=distance
        return result

    def run(self):
        side=0
        last_ping=float('-inf')
        try:
            while not rospy.is_shutdown():
                wait=.06-(time.monotonic()-last_ping)
                if wait>0:
                    time.sleep(wait)
                last_ping=time.monotonic()
                try:
                    distance=float(self.sensors[side].distance)/100.
                    if not math.isfinite(distance) or not .02<=distance<=4.:
                        distance=float('nan')
                except (RuntimeError,OSError):
                    distance=float('nan')
                self.values[side],self.times[side]=distance,time.monotonic()
                self.publishers[side].publish(self.message('left' if side==0 else 'right',distance))
                valid=all(math.isfinite(v) for v in self.values) and min(self.times)>time.monotonic()-.35
                self.combined.publish(self.message('front',min(self.values) if valid else float('nan')))
                side=1-side
                self.rate.sleep()
        finally:
            for sensor in self.sensors:
                sensor.deinit()


if __name__=='__main__':
    UltraSonic().run()
