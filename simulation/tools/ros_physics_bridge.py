"""Real ROS 2 transport between simulated sensors and posture controller."""
import time
import json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, Joy, Range
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import SetBool
from bigspot_sim.posture_node import Controller


class Bridge(Node):
    def __init__(self,names,parameters,enable_experiments=False,use_joystick=False,joystick_settings=None):
        rclpy.init()
        super().__init__('bigspot_cpu_physics')
        self.controller=Controller(parameter_overrides=[Parameter('use_sim_time',value=True),
                                  Parameter('policy_file',value=str(Path(__file__).resolve().parents[1]/'ros2_ws/src/bigspot_sim/config/control_policy_adaptive.json')),
                                  Parameter('gait',value=parameters.get('gait_mode','crawl')),
                                  Parameter('enable_experiments',value=enable_experiments)]+
                                  [Parameter(k,value=v) for k,v in parameters.items()])
        self.transport_executor=SingleThreadedExecutor()
        self.transport_executor.add_node(self)
        self.transport_executor.add_node(self.controller)
        self.joystick_node=None
        if use_joystick:
            from bigspot_sim.joystick import Joystick
            self.joystick_node=Joystick(parameter_overrides=[Parameter(k,value=v) for k,v in (joystick_settings or {}).items()])
            self.transport_executor.add_node(self.joystick_node)
        self.joy_publisher=self.create_publisher(Joy,'/joy',10)
        self.joy_count=0
        self.names=names
        self.clock=self.create_publisher(Clock,'/clock',10)
        self.imu=self.create_publisher(Imu,'/bigspot/imu',10)
        self.joints=self.create_publisher(JointState,'/bigspot/joint_states',10)
        self.twist=self.create_publisher(Twist,'/cmd_vel',10)
        self.contact=self.create_publisher(Float32MultiArray,'/bigspot/contact_forces',10)
        self.seat_contact=self.create_publisher(Float32MultiArray,'/bigspot/seat_feedback',10)
        self.ground_contact=self.create_publisher(Float32MultiArray,'/bigspot/ground_rest_feedback',10)
        self.ranges=[self.create_publisher(Range,'/bigspot/ultrasonic/'+side,10) for side in ('left','right')]
        self.range_count=0
        self.action=self.create_publisher(String,'/bigspot/action',10)
        self.action_count=0
        self.observed_actions=[]
        self.create_subscription(String,'/bigspot/action',lambda m:self.observed_actions.append(m.data),10)
        self.latest_status={}
        self.command=None
        self.state=self.controller.posture.state
        self.fault=''
        self.count=0
        self.command_time=float('-inf')
        self.received_states=set()
        self.create_subscription(JointState,'/bigspot/joint_commands',self.commands,10)
        self.create_subscription(String,'/bigspot/state',self.status,10)
        self.enable_client=self.create_client(SetBool,'/bigspot/enable')
        self.start_sent=False
        self.start_future=None
        self.last_velocity=None
        print('ROS graph started: /bigspot_cpu_physics and /bigspot_sim_controller',flush=True)
        # Discovery is asynchronous; seed clock/sensors before requesting start.
        until=time.monotonic()+.3
        while time.monotonic()<until:
            self.transport_executor.spin_once(timeout_sec=.01)

    def commands(self,msg):
        measured=dict(zip(msg.name,msg.position))
        if all(n in measured for n in self.names):
            self.command=np.array([measured[n] for n in self.names])
            self.command_time=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            self.count+=1

    def status(self,msg):
        value=json.loads(msg.data)
        self.state=value['state']
        self.fault=value.get('fault','')
        self.received_states.add(self.state)
        self.latest_status=value

    def update(self,t,quaternion_wxyz,gyro,q,velocity,start,send_command,accel=None,forces=None,action=None,joy=None,
               ranges=None,orientation_available=True,send_imu=True,send_joints=True,seat_feedback=None,ground_feedback=None):
        clock=Clock()
        # Truncating an accumulated floating-point time can put a sample one
        # nanosecond before a timer deadline and skip that entire control tick.
        nanoseconds=round(t*1e9)
        clock.clock.sec=nanoseconds//1_000_000_000
        clock.clock.nanosec=nanoseconds%1_000_000_000
        imu=Imu()
        imu.header.stamp=clock.clock
        imu.header.frame_id='base_link'
        w,x,y,z=quaternion_wxyz
        imu.orientation.w,imu.orientation.x,imu.orientation.y,imu.orientation.z=map(float,[w,x,y,z])
        if not orientation_available:
            imu.orientation_covariance[0]=-1.
        imu.angular_velocity.x,imu.angular_velocity.y=map(float,gyro[:2])
        imu.angular_velocity.z=float(gyro[2]) if len(gyro)>2 else 0.
        if accel is not None:
            imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z=map(float,accel)
        else:
            imu.linear_acceleration_covariance[0]=-1.
        joints=JointState()
        joints.header.stamp=clock.clock
        joints.name=self.names
        joints.position=[float(v) for v in q]
        if send_imu:
            self.imu.publish(imu)
        if send_joints:
            self.joints.publish(joints)
        self.clock.publish(clock)
        for i,distance in enumerate(ranges if ranges is not None else []):
            if distance is None:
                continue
            message=Range()
            message.header.stamp=clock.clock
            message.header.frame_id='ultrasonic_'+('left' if i==0 else 'right')
            message.radiation_type=Range.ULTRASOUND
            message.field_of_view=.26
            message.min_range,message.max_range=.02,4.
            message.range=float(distance)
            self.ranges[i].publish(message)
            self.range_count+=1
        if joy is not None:
            raw=Joy()
            raw.header.stamp=clock.clock
            raw.axes=[float(v) for v in joy[0]]
            raw.buttons=[int(v) for v in joy[1]]
            self.joy_publisher.publish(raw)
            self.joy_count+=1
        if forces is not None:
            contact=Float32MultiArray()
            contact.data=[float(v) for v in forces]
            self.contact.publish(contact)
        if seat_feedback is not None:
            seat_message=Float32MultiArray();seat_message.data=[float(v) for v in seat_feedback]
            self.seat_contact.publish(seat_message)
        if ground_feedback is not None:
            ground_message=Float32MultiArray();ground_message.data=[float(v) for v in ground_feedback]
            self.ground_contact.publish(ground_message)
        if send_command:
            twist=Twist()
            twist.linear.x,twist.linear.y,twist.angular.z=map(float,velocity)
            self.twist.publish(twist)
        if action:
            event=String()
            event.data=action
            self.action.publish(event)
            self.action_count+=1
        if start and not self.start_sent and self.enable_client.service_is_ready():
            request=SetBool.Request()
            request.data=True
            self.start_future=self.enable_client.call_async(request)
            self.start_sent=True
        # Drain actual middleware callbacks, including the simulation-clock timer.
        for _ in range(50):
            self.transport_executor.spin_once(timeout_sec=0.)
        # Wait for the actual ROS clock/timer publications for this physics
        # sample. Without this barrier DDS delivery can repeat the old sample
        # and make the following acceleration sample appear to jump twice.
        deadline=time.monotonic()+.02
        if t>.02:
            while (self.command_time<t-.004 or self.latest_status.get('control_time_s',0.)<t-.004) and time.monotonic()<deadline:
                self.transport_executor.spin_once(timeout_sec=.001)
            if self.command_time<t-.05:
                raise RuntimeError('ROS controller did not publish a current joint command')
        if self.start_future is not None and self.start_future.done():
            response=self.start_future.result()
            if not response.success:
                self.start_sent=False
            self.start_future=None
        return self.command,self.state

    def close(self):
        result={'ros_executed':True,'ros_distribution':'humble',
                'joint_command_messages_received':self.count,'ros_states_received':sorted(self.received_states),
                'ros_nodes':['bigspot_cpu_physics','bigspot_sim_controller'],
                'action_messages_sent':self.action_count,'final_status':self.latest_status}
        result['joy_messages_sent']=self.joy_count
        result['ultrasonic_messages_sent']=self.range_count
        result['observed_action_messages']=self.observed_actions
        if self.joystick_node:
            result['ros_nodes'].append('bigspot_joystick')
            result['joystick_selection']=self.joystick_node.mapping.status()
            self.joystick_node.destroy_node()
        self.transport_executor.shutdown()
        self.controller.destroy_node()
        self.destroy_node()
        rclpy.shutdown()
        if self.count<10:
            raise RuntimeError('ROS joint command transport was not established')
        return result
