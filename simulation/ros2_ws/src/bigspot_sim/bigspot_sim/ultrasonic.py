"""Optional Raspberry Pi HC-SR04P driver. No GPIO access until explicitly run."""
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


def centimetres_to_metres(value):
    distance=float(value)/100.
    return distance if math.isfinite(distance) and .02<=distance<=4. else float('nan')


class Ultrasonic(Node):
    def __init__(self):
        super().__init__('bigspot_ultrasonic')
        # Imports require Blinka and the physical GPIO board, not a desktop PC.
        import board
        import adafruit_hcsr04
        self.sensors=[adafruit_hcsr04.HCSR04(trigger_pin=board.D16,echo_pin=board.D19,timeout=.05),
                      adafruit_hcsr04.HCSR04(trigger_pin=board.D20,echo_pin=board.D21,timeout=.05)]
        self.publishers=[self.create_publisher(Range,'/bigspot/ultrasonic/'+side,10) for side in ('left','right')]
        self.side=0
        self.last_ping=float('-inf')
        self.create_timer(.065,self.sample)

    def sample(self):
        if time.monotonic()-self.last_ping<.06:
            return
        i=self.side
        self.side=1-i
        self.last_ping=time.monotonic()
        message=Range()
        message.header.stamp=self.get_clock().now().to_msg()
        message.header.frame_id='ultrasonic_'+('left' if i==0 else 'right')
        message.radiation_type=Range.ULTRASOUND
        message.field_of_view=.26
        message.min_range,message.max_range=.02,4.
        try:
            message.range=centimetres_to_metres(self.sensors[i].distance)
        except (RuntimeError,OSError):
            # A timeout may be a missing echo, bad wiring or angled surface.
            # It cannot justify declaring the path clear.
            message.range=float('nan')
        self.publishers[i].publish(message)

    def destroy_node(self):
        for sensor in self.sensors:
            sensor.deinit()
        return super().destroy_node()


def main():
    rclpy.init()
    node=Ultrasonic()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
