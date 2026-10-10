"""DDS-driven CPU plant: external targets drive the emulated local motor PD."""
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock as TimerClock, ClockType
from sensor_msgs.msg import JointState, Imu, Image, CameraInfo
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from .ros_nodes import design_for, stamp, set_quaternion
from .sim_backend import CpuPlant
from .control_core import JOINT_NAMES
from .navigation import DepthCamera


class CpuNode(Node):
    def __init__(self):
        super().__init__('bigspot_cpu_plant')
        self.design = design_for(self)
        self.plant = CpuPlant(self.design)
        self.camera = DepthCamera()
        self.duration = float(self.declare_parameter('duration',35.).value)
        self.next_camera = self.next_sensor = 0.
        self.publishers = {n:self.create_publisher(t,n,10) for n,t in (
            ('/clock',Clock),('/joint_states',JointState),('/imu/data',Imu),
            ('/odom',Odometry),('/camera/depth/image_rect_raw',Image),('/camera/depth/camera_info',CameraInfo))}
        self.tf, self.static = TransformBroadcaster(self), StaticTransformBroadcaster(self)
        self.sent_static = False
        self.create_subscription(JointState,'/joint_targets',self.target,10)
        self.create_timer(self.plant.dt,self.tick,clock=TimerClock(clock_type=ClockType.STEADY_TIME))

    def target(self,m):
        t = m.header.stamp.sec+m.header.stamp.nanosec/1e9
        self.plant.watchdog.receive_target(m.name,m.position,m.effort,t,float(self.plant.data.time),time.monotonic())

    def tick(self):
        p=self.plant
        if float(p.data.time)>=self.duration:
            p.data.ctrl[:]=0.
            return
        p.step(time.monotonic())
        t=float(p.data.time);s=stamp(t)
        self.publishers['/clock'].publish(Clock(clock=s))
        if t<self.next_sensor:return
        self.next_sensor=t+.009
        q,v,xyzw,gyro=p.measurements()
        joints=JointState();joints.header.stamp=s;joints.name=list(JOINT_NAMES);joints.position=q.tolist();joints.velocity=v.tolist();joints.effort=p.data.ctrl.tolist()
        self.publishers['/joint_states'].publish(joints)
        imu=Imu();imu.header.stamp=s;imu.header.frame_id='body_imu_link'
        imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w=map(float,xyzw)
        imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z=map(float,gyro)
        imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z=map(float,p.data.sensordata[3:6])
        self.publishers['/imu/data'].publish(imu)
        odom=Odometry();odom.header.stamp=s;odom.header.frame_id='odom';odom.child_frame_id='base_link'
        odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=map(float,p.data.qpos[:3])
        set_quaternion(odom.pose.pose.orientation,p.data.qpos[3:7]);self.publishers['/odom'].publish(odom)
        tf=TransformStamped();tf.header=odom.header;tf.child_frame_id='base_link';tf.transform.rotation=odom.pose.pose.orientation
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,p.data.qpos[:3]);self.tf.sendTransform(tf)
        if t<self.next_camera:return
        self.next_camera=t+.2;self.camera.capture(p.model,p.data)
        image=Image();image.header.stamp=s;image.header.frame_id='sim_depth_optical';image.width=self.camera.width;image.height=self.camera.height
        image.encoding='32FC1';image.step=image.width*4;image.data=np.asarray(self.camera.depth,dtype='<f4').tobytes()
        self.publishers['/camera/depth/image_rect_raw'].publish(image)
        info=CameraInfo();info.header=image.header;info.width=image.width;info.height=image.height;f=self.camera.fx;cx=(image.width-1)/2;cy=(image.height-1)/2
        info.k=[f,0.,cx,0.,f,cy,0.,0.,1.];info.p=[f,0.,cx,0.,0.,f,cy,0.,0.,0.,1.,0.];self.publishers['/camera/depth/camera_info'].publish(info)
        if not self.sent_static:
            mj=p.mj;cid=mj.mj_name2id(p.model,mj.mjtObj.mjOBJ_CAMERA,'front_depth');bid=mj.mj_name2id(p.model,mj.mjtObj.mjOBJ_BODY,'base_link')
            rotation=p.data.xmat[bid].reshape(3,3).T@p.data.cam_xmat[cid].reshape(3,3)@np.diag([1,-1,-1])
            quat=np.zeros(4);mj.mju_mat2Quat(quat,rotation.ravel())
            tf=TransformStamped();tf.header.stamp=s;tf.header.frame_id='base_link';tf.child_frame_id='sim_depth_optical'
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,self.design['camera_xyz_m']);set_quaternion(tf.transform.rotation,quat)
            self.static.sendTransform(tf);self.sent_static=True


def main():
    rclpy.init();node=CpuNode()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
