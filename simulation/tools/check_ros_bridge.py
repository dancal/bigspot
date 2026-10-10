"""Run in the Isaac ROS 5.0 container to validate actual DDS topic flow."""
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, Image, CameraInfo, Imu


def main():
    rclpy.init()
    node = Node('bigspot_bridge_check')
    received = {}
    errors = []
    def clock(msg):
        received['clock'] = msg.clock.sec+msg.clock.nanosec/1e9
    def joints(msg):
        if len(msg.name)!=12 or len(msg.position)!=12 or not all(map(math.isfinite,msg.position)):
            errors.append('joint state must contain 12 finite positions')
        received['joints'] = time.monotonic()
    def image(msg):
        if msg.width!=640 or msg.height!=480 or not msg.data:
            errors.append('invalid image dimensions or empty data')
        received['image'] = time.monotonic()
    def info(msg):
        if msg.k[0]<=0 or msg.k[4]<=0 or msg.header.frame_id!='camera_optical_frame':
            errors.append('invalid camera calibration/frame')
        received['info'] = time.monotonic()
    def imu(msg):
        q = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
        gyro = [msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]
        if not all(map(math.isfinite,q+gyro)) or abs(sum(v*v for v in q)-1)>0.05:
            errors.append('IMU requires a finite unit quaternion and angular velocity')
        received['imu'] = time.monotonic()
    node.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data)
    node.create_subscription(JointState,'/bigspot/joint_states',joints,qos_profile_sensor_data)
    node.create_subscription(Image,'/bigspot/camera/image_rect',image,qos_profile_sensor_data)
    node.create_subscription(CameraInfo,'/bigspot/camera/camera_info',info,qos_profile_sensor_data)
    node.create_subscription(Imu,'/bigspot/imu',imu,qos_profile_sensor_data)
    deadline = time.monotonic()+20
    initial_clock = None
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if initial_clock is None and 'clock' in received:
                initial_clock = received['clock']
            if len(received)==5 and initial_clock is not None and received['clock']>initial_clock+.2:
                if errors:
                    raise RuntimeError('; '.join(sorted(set(errors))))
                print('PASS: advancing simulation clock, 12 DOFs, IMU, RGB, camera_info received')
                return
        raise RuntimeError(f'Bridge timeout: received {list(received)}. Check ROS_DOMAIN_ID, Fast DDS, container network, and Sim/Jazzy to Lyrical interoperability.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
