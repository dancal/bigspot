import json
import time
import inspect
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu, Range
from std_msgs.msg import String, Float32MultiArray, Bool
from std_srvs.srv import SetBool
from .posture import PostureController, quaternion_rpy
from .motion import MotionController,PERFORMANCES
from .dance import DANCES
from .state_estimator import StateEstimator, stamp_seconds
from .awareness import RangePair
from .hardware_reference import pulse_angles


class Controller(Node):
    def __init__(self, **kwargs):
        super().__init__('bigspot_sim_controller', **kwargs)
        self.declare_parameter('gait', 'auto')
        folder=Path(get_package_share_directory('bigspot_sim'))/'config'
        self.declare_parameter('policy_file',str(folder/'control_policy_ground_startup.json'))
        defaults=json.loads(Path(self.get_parameter('policy_file').value).read_text())['parameters']
        constructor_defaults={k:p.default for k,p in inspect.signature(PostureController).parameters.items()
                              if p.default is not inspect.Parameter.empty}
        constructor_defaults.update({k:p.default for k,p in inspect.signature(MotionController).parameters.items()
                              if p.default is not inspect.Parameter.empty and k!='enable_experiments'})
        defaults={**constructor_defaults,**defaults}
        for k,v in defaults.items():
            self.declare_parameter(k,v)
        model = json.loads((Path(get_package_share_directory('bigspot_sim'))/'config/model.json').read_text())
        parameters = {k:self.get_parameter(k).value for k in defaults}
        self.declare_parameter('enable_experiments',False)
        self.posture = MotionController(model,enable_experiments=self.get_parameter('enable_experiments').value, **parameters)
        self.posture.prepare_dances()
        self.estimator=StateEstimator()
        self.ranges=RangePair()
        requested_mode=self.get_parameter('gait').value
        if requested_mode!='auto':
            if requested_mode not in ('crawl','trot'):
                raise ValueError('Invalid gait mode')
            self.posture.gait.mode=requested_mode
        self.gait = self.posture.gait
        self.names = [n for leg in self.gait.legs for n in leg.names]
        self.publisher = self.create_publisher(JointState, '/bigspot/joint_commands', 10)
        self.status = self.create_publisher(String, '/bigspot/state', 10)
        self.servo_enable = self.create_publisher(Bool, '/bigspot/sim_servo_enable', 10)
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.create_subscription(Imu, '/bigspot/imu', self.imu, 10)
        self.create_subscription(JointState, '/bigspot/joint_states', self.joints, 10)
        self.create_subscription(Float32MultiArray,'/bigspot/contact_forces',self.contacts,10)
        self.create_subscription(Float32MultiArray,'/bigspot/seat_feedback',self.seat_contact,10)
        self.create_subscription(Float32MultiArray,'/bigspot/ground_rest_feedback',self.ground_contact,10)
        self.create_subscription(String,'/bigspot/action',self.action,10)
        self.create_subscription(Range,'/bigspot/ultrasonic/left',lambda m:self.range(m,0),10)
        self.create_subscription(Range,'/bigspot/ultrasonic/right',lambda m:self.range(m,1),10)
        self.create_service(SetBool, '/bigspot/enable', self.enable)
        self.enabled = False
        self.velocity = np.zeros(3)
        self.received = self.imu_received = float('-inf')
        self.rp, self.gyro = np.zeros(2), np.zeros(2)
        self.error = float('inf')
        self.joint_received=float('-inf')
        self.seat_received=float('-inf');self.seat_values={}
        self.ground_received=float('-inf');self.ground_values={}
        self.last_sim = None
        self.create_timer(.016, self.tick)

    def command(self, msg):
        values = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        if np.isfinite(values).all():
            if self.gait.static is not None and abs(values[2]) > 1e-4:
                values[2] = 0.
                if not getattr(self,'yaw_warning',False):
                    self.get_logger().warning('Slow static crawl has no validated yaw control; yaw input ignored')
                    self.yaw_warning=True
            self.velocity, self.received = values, time.monotonic()

    def imu(self, msg):
        try:
            q = msg.orientation
            gyro=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]
            accel=[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
            t=stamp_seconds(msg.header.stamp.sec,msg.header.stamp.nanosec)
            rpy=self.estimator.update([q.x,q.y,q.z,q.w],gyro,accel,t,
                 orientation_available=msg.orientation_covariance[0]!=-1,
                 accel_available=msg.linear_acceleration_covariance[0]!=-1,
                 gyro_available=msg.angular_velocity_covariance[0]!=-1)
            self.rp, self.gyro, self.imu_received = rpy[:2], np.array(gyro[:2]), time.monotonic()
        except ValueError:
            return

    def contacts(self,msg):
        try:
            self.estimator.contacts(msg.data,self.get_clock().now().nanoseconds/1e9)
        except ValueError:
            return

    def range(self,msg,side):
        stamp=stamp_seconds(msg.header.stamp.sec,msg.header.stamp.nanosec)
        now=self.get_clock().now().nanoseconds/1e9
        if stamp-now>.05 or msg.radiation_type!=Range.ULTRASOUND:
            return
        try:
            self.ranges.update(side,msg.range,stamp,max(now,stamp),msg.min_range,msg.max_range)
        except ValueError:
            return

    def seat_contact(self,msg):
        values=np.array(msg.data)
        if values.shape!=(9,) or not np.isfinite(values).all() or min(values[:3])<0:return
        self.seat_values={'rear_support_force_n':float(values[0]),'paw_contact_force_n':float(values[1]),
            'paw_gap_m':float(values[2]),'paw_contact_tip_offset_m':float(values[3]),
            'rear_support_margin_m':float(values[4]),'front_paw_height_m':values[5:7].tolist(),
            'other_self_contact_force_n':float(values[7]),'rear_body_force_n':float(values[8])}
        self.seat_received=self.get_clock().now().nanoseconds/1e9

    def observation(self):
        value=self.estimator.snapshot(self.get_clock().now().nanoseconds/1e9)
        value['joint_feedback_fresh']=time.monotonic()-self.joint_received<.25
        value['measured_joint_angles_rad']=getattr(self,'measured_q',None)
        value['ultrasonic']=self.ranges.snapshot(self.get_clock().now().nanoseconds/1e9)
        value['seat_feedback']={**self.seat_values,'fresh':0<=self.get_clock().now().nanoseconds/1e9-self.seat_received<.20}
        value['ground_rest_feedback']={**self.ground_values,'fresh':0<=self.get_clock().now().nanoseconds/1e9-self.ground_received<.20}
        if value['ground_rest_feedback']['fresh'] and value['ground_rest_feedback'].get('body_force_n',0.)>1.:
            value['motion_state']='BODY_SUPPORTED';value['airborne_seconds']=0.
        self.posture.observe(value)
        return value

    def ground_contact(self,msg):
        values=np.asarray(msg.data,dtype=float)
        if values.shape!=(3,) or not np.isfinite(values).all() or values[0]<0. or values[2]<=0.:return
        self.ground_values={'body_force_n':float(values[0]),'body_clearance_m':float(values[1]),'base_height_m':float(values[2])}
        self.ground_received=self.get_clock().now().nanoseconds/1e9

    def action(self,msg):
        self.observation()
        if msg.data.strip()=='stop':
            self.velocity[:]=0.
            self.posture.request('stop')
            return
        if not self.enabled:
            self.posture.action_result='Rejected: enable and stand first'
            return
        if not self.estimator.gyro_available:
            self.posture.action_result='Rejected: angular velocity sensor unavailable'
            return
        self.posture.request(msg.data.strip())

    def joints(self, msg):
        measured = dict(zip(msg.name, msg.position))
        if all(n in measured for n in self.names):
            q = np.array([measured[n] for n in self.names])
            if np.isfinite(q).all():
                self.measured_q=q.tolist()
                self.error = float(np.max(np.abs(q-self.gait.q.ravel())))
                self.joint_received=time.monotonic()

    def enable(self, request, response):
        if request.data and time.monotonic()-self.imu_received > .25:
            response.success, response.message = False, 'Fresh valid IMU required before standing'
            return response
        if self.posture.state == 'FAULT':
            response.success, response.message = False, self.posture.fault+'; restart after inspection'
            return response
        self.enabled = request.data
        if self.enabled:
            self.posture.start()
        else:
            self.posture.request('stop')
        self.velocity[:] = 0.
        self.received = float('-inf')
        response.success = True
        response.message = 'Standing sequence requested' if request.data else 'Movement stopping; posture held'
        return response

    def tick(self):
        stamp = self.get_clock().now()
        sim = stamp.nanoseconds/1e9
        if sim <= 0:
            return
        dt = .016 if self.last_sim is None else sim-self.last_sim
        self.last_sim = sim
        # ROS simulation-clock timers can catch up at an identical timestamp.
        # That is a duplicate callback, not a disconnected joystick.
        if dt == 0:
            return
        if dt < 0 or dt > .1:
            self.enabled = False
            self.get_logger().warning(f'Simulation clock discontinuity: {dt:.3f}s; movement disabled')
            return
        now = time.monotonic()
        observation=self.observation()
        previous=self.gait.q.copy()
        try:
            positions = self.posture.step(dt, self.velocity, self.rp, self.gyro,
                imu_fresh=now-self.imu_received < .25,
                command_fresh=self.enabled and now-self.received < .5, joint_error=self.error)
            if self.posture.hardware_reference:
                pulse_angles(positions,self.posture.hardware_reference)
        except ValueError as error:
            self.posture.fault, self.posture.state = str(error), 'FAULT'
            self.enabled = False
            self.gait.q=previous
            positions = self.gait.q.ravel().tolist()
        msg = JointState()
        msg.header.stamp, msg.name, msg.position = stamp.to_msg(), self.names, positions
        self.publisher.publish(msg)
        ground=self.posture.ground_startup
        drive=ground.snapshot() if ground else {'servo_enabled':True,'strength':1.}
        self.servo_enable.publish(Bool(data=drive['servo_enabled']))
        status = String()
        status.data = json.dumps({'control_time_s':sim,'state':self.posture.state,'fault':self.posture.fault,
                                 'action':self.posture.action,'action_result':self.posture.action_result,
                                 'mode':self.posture.mode,'estimator':observation,'startup_drive':drive,
                                 'action_events':self.posture.action_events,
                                 'clap':{'count':getattr(self.posture,'clap_count',0),
                                    'elapsed_s':self.posture.action_time,'cancelled':getattr(self.posture,'clap_cancelled',False)}
                                    if self.posture.action=='seated_clap' else None,
                                 'dance':self.posture.dance_status(),
                                 'sensor_constraints':self.posture.constraints.status,
                                 'velocity_profile':{'velocity':self.gait.speed.tolist(),
                                    'acceleration':self.gait.speed_acceleration.tolist(),
                                    'jerk':self.gait.speed_jerk.tolist()},
                                 'pose_target_rpy_rad':getattr(self.posture,'pose_last_rpy',np.zeros(3)).tolist()})
        self.status.publish(status)


def main():
    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
