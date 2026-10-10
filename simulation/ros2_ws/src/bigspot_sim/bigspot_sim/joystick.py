"""Remappable Joy indices; start, stop, deadman, boost, run, poses and actions.

Button indices depend on the device/driver. Physical gamepad mapping is unverified.
"""
import time
import math
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from std_msgs.msg import String
from .joy_mapping import JoyMapping, DEFAULTS


class Joystick(Node):
    def __init__(self,**kwargs):
        super().__init__('bigspot_joystick',**kwargs)
        for key,value in DEFAULTS.items():
            self.declare_parameter(key,value)
        self.mapping=JoyMapping({k:self.get_parameter(k).value for k in DEFAULTS})
        self.message = None
        self.received = float('-inf')
        self.previous_start = False
        self.publisher = self.create_publisher(Twist,'/cmd_vel',10)
        self.action_publisher=self.create_publisher(String,'/bigspot/action',10)
        self.selection_publisher=self.create_publisher(String,'/bigspot/joystick_state',10)
        self.client = self.create_client(SetBool,'/bigspot/enable')
        self.create_subscription(Joy,'/joy',self.joy,10)
        self.create_timer(.05,self.tick)

    def joy(self, msg):
        if len(msg.axes)<4 or len(msg.buttons)<6 or not all(math.isfinite(v) for v in msg.axes):
            return
        actions,enable=self.mapping.events(msg.axes,msg.buttons)
        if enable is not None:
            if self.client.service_is_ready():
                request=SetBool.Request()
                request.data=enable
                self.client.call_async(request)
        for name in actions:
            action=String()
            action.data=name
            self.action_publisher.publish(action)
        self.message, self.received=msg,time.monotonic()

    def tick(self):
        command=Twist()
        msg=self.message
        if msg is not None:
            fresh=time.monotonic()-self.received<.3
            if self.mapping.cancel_dance_if_unsafe(msg.axes,msg.buttons,fresh):
                action=String()
                action.data='stop'
                self.action_publisher.publish(action)
            velocity=self.mapping.velocity(msg.axes,msg.buttons,fresh=fresh)
            command.linear.x,command.linear.y,command.angular.z=map(float,velocity)
        self.publisher.publish(command)
        selection=String()
        selection.data=json.dumps(self.mapping.status())
        self.selection_publisher.publish(selection)


def main():
    rclpy.init()
    node=Joystick()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
