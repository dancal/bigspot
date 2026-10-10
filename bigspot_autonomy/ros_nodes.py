"""ROS interfaces: live CPU physics, depth experiment, hardware-inhibited controller."""
import json,math,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState,Imu,Image,CameraInfo
from geometry_msgs.msg import TwistStamped,TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String,Bool
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster,Buffer,TransformListener
from ament_index_python.packages import get_package_share_directory
from .design import load_design,LEGS,motor_names
from .gait import Gait
from .safety import Supervisor
from .can_protocol import control

def design_for(node):
    default=Path(get_package_share_directory('bigspot_autonomy'))/'config/selected_design.json'
    return load_design(node.declare_parameter('design_file',str(default)).value)

def quat_matrix(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def set_quaternion(msg,q):msg.w,msg.x,msg.y,msg.z=map(float,q)
def stamp(t):
    from builtin_interfaces.msg import Time
    return Time(sec=int(t),nanosec=int((t-int(t))*1e9))

class PhysicsNode(Node):
    def __init__(self):
        super().__init__('bigspot_physics');self.design=design_for(self)
        self.seconds=self.declare_parameter('duration',35.).value
        self.realtime=self.declare_parameter('realtime',True).value
        self.obstacles=self.declare_parameter('obstacles',True).value
        self.startup=self.declare_parameter('startup',True).value
        self.velocity=np.zeros(3);self.last_cmd=-math.inf;self.next_pub=0.;self.next_camera=0.
        self.create_subscription(TwistStamped,'/cmd_vel',self.command,10)
        self.pubs={name:self.create_publisher(typ,name,10) for name,typ in [('/clock',Clock),('/joint_states',JointState),('/imu/data',Imu),('/odom',Odometry),('/camera/depth/image_rect_raw',Image),('/camera/depth/camera_info',CameraInfo)]}
        self.tf=TransformBroadcaster(self);self.static_tf=StaticTransformBroadcaster(self);self.sent_static=False
        from .navigation import DepthCamera
        self.camera=DepthCamera();self.wall=time.monotonic();self.counts={'imu':0,'joints':0,'camera':0,'commands':0}
    def command(self,msg):
        self.velocity[:]=[msg.twist.linear.x,msg.twist.linear.y,msg.twist.angular.z];self.last_cmd=time.monotonic();self.counts['commands']+=1
    def command_source(self,t):return self.velocity.copy() if time.monotonic()-self.last_cmd<.3 else np.zeros(3)
    def callback(self,model,data,desired,torque,estimate):
        import mujoco
        t=float(data.time)
        if t<self.next_pub:return
        self.next_pub=t+.019
        rclpy.spin_once(self,timeout_sec=0)
        if self.realtime:
            delay=self.wall+t-time.monotonic()
            if delay>0:time.sleep(delay)
        s=stamp(t);clock=Clock();clock.clock=s;self.pubs['/clock'].publish(clock)
        joints=JointState();joints.header.stamp=s;joints.name=[f'{leg}_{axis}' for leg,_,_ in LEGS for axis in ('abad','hip','knee')];joints.position=data.qpos[7:].tolist();joints.velocity=data.qvel[6:].tolist();joints.effort=torque.tolist();self.pubs['/joint_states'].publish(joints);self.counts['joints']+=1
        imu=Imu();imu.header.stamp=s;imu.header.frame_id='body_imu_link'
        # Complementary attitude estimate generated from noisy gyro/accelerometer.
        cr,sr,cp,sp,cy,sy=math.cos(estimate[0]/2),math.sin(estimate[0]/2),math.cos(estimate[1]/2),math.sin(estimate[1]/2),math.cos(estimate[2]/2),math.sin(estimate[2]/2)
        set_quaternion(imu.orientation,(cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy))
        imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z=map(float,data.sensordata[:3]);imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z=map(float,data.sensordata[3:6]);self.pubs['/imu/data'].publish(imu);self.counts['imu']+=1
        odom=Odometry();odom.header.stamp=s;odom.header.frame_id='odom';odom.child_frame_id='base_link';odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=map(float,data.qpos[:3]);set_quaternion(odom.pose.pose.orientation,data.qpos[3:7]);self.pubs['/odom'].publish(odom)
        tf=TransformStamped();tf.header=odom.header;tf.child_frame_id='base_link';tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,data.qpos[:3]);tf.transform.rotation=odom.pose.pose.orientation;self.tf.sendTransform(tf)
        if t>=self.next_camera:
            self.camera.capture(model,data);self.next_camera=t+.2;self.counts['camera']+=1
            image=Image();image.header.stamp=s;image.header.frame_id='sim_depth_optical';image.width=self.camera.width;image.height=self.camera.height;image.encoding='32FC1';image.step=image.width*4;image.data=np.asarray(self.camera.depth,dtype='<f4').tobytes();self.pubs['/camera/depth/image_rect_raw'].publish(image)
            info=CameraInfo();info.header=image.header;info.width=image.width;info.height=image.height;fx=self.camera.fx;cx=(info.width-1)/2;cy=(info.height-1)/2;info.k=[fx,0.,cx,0.,fx,cy,0.,0.,1.];info.p=[fx,0.,cx,0.,0.,fx,cy,0.,0.,0.,1.,0.];self.pubs['/camera/depth/camera_info'].publish(info)
            if not self.sent_static:
                cid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_CAMERA,'front_depth');bid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'base_link')
                rb=data.xmat[bid].reshape(3,3);rc=data.cam_xmat[cid].reshape(3,3)@np.diag([1,-1,-1]);local=rb.T@rc;q=np.zeros(4);mujoco.mju_mat2Quat(q,local.ravel())
                tf=TransformStamped();tf.header.stamp=s;tf.header.frame_id='base_link';tf.child_frame_id='sim_depth_optical';tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,self.design['camera_xyz_m']);set_quaternion(tf.transform.rotation,q);self.static_tf.sendTransform(tf);self.sent_static=True

def physics_main():
    from .physics import rollout
    rclpy.init();node=PhysicsNode()
    try:
        result=rollout(node.design,speed=0,seconds=node.seconds,startup=node.startup,on_step=node.callback,command_source=node.command_source,test_obstacles=node.obstacles)
        result['ros_topic_counts']=node.counts;node.get_logger().info(json.dumps(result))
        path=Path(node.declare_parameter('report_file','/tmp/bigspot_ros_report.json').value);path.write_text(json.dumps(result,indent=2)+'\n')
    finally:node.destroy_node();rclpy.shutdown()

class DepthNavNode(Node):
    def __init__(self):
        super().__init__('bigspot_depth_navigation')
        from .navigation import GridPlanner
        self.planner=GridPlanner();self.goal=np.asarray(self.declare_parameter('goal_xy',[3.,0.]).value);self.xy=None;self.yaw=0.;self.info=None;self.last_image=-math.inf;self.last_odom=-math.inf;self.route=[]
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.pub=self.create_publisher(TwistStamped,'/cmd_vel',10)
        self.create_subscription(CameraInfo,'/camera/depth/camera_info',lambda m:setattr(self,'info',m),qos_profile_sensor_data)
        self.create_subscription(Image,'/camera/depth/image_rect_raw',self.image,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.odom,10);self.create_timer(.05,self.tick)
    def odom(self,m):
        self.xy=np.array([m.pose.pose.position.x,m.pose.pose.position.y]);r=quat_matrix(m.pose.pose.orientation);self.yaw=math.atan2(r[1,0],r[0,0]);self.last_odom=time.monotonic()
    def image(self,m):
        if self.info is None or self.xy is None:return
        try:tf=self.tf.lookup_transform('odom',m.header.frame_id,rclpy.time.Time.from_msg(m.header.stamp))
        except Exception:return
        if m.encoding not in ('32FC1','16UC1'):return
        depth=np.frombuffer(bytes(m.data),dtype=('>f4' if m.is_bigendian else '<f4') if m.encoding=='32FC1' else ('>u2' if m.is_bigendian else '<u2')).reshape(m.height,-1)[:,:m.width].astype(float)
        if m.encoding=='16UC1':depth*=.001
        y,x=np.mgrid[0:m.height:4,0:m.width:4];z=depth[::4,::4];k=self.info.k
        points=np.stack(((x-k[2])*z/k[0],(y-k[5])*z/k[4],z),axis=-1).reshape(-1,3);points=points[np.isfinite(points).all(axis=1)&(points[:,2]>.1)&(points[:,2]<4)]
        tr=tf.transform.translation;points=points@quat_matrix(tf.transform.rotation).T+np.array([tr.x,tr.y,tr.z]);points=points[(points[:,2]>.06)&(points[:,2]<.9)]
        self.planner.integrate(points);self.route=self.planner.path(self.xy,self.goal);self.last_image=time.monotonic()
    def tick(self):
        msg=TwistStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='base_link'
        if self.xy is not None and self.route and time.monotonic()-max(self.last_image,self.last_odom)<.3 and time.monotonic()-min(self.last_image,self.last_odom)<.3:
            distance=np.linalg.norm(self.goal-self.xy)
            if distance>.16:
                target=self.route[-1]
                for p in self.route:
                    if np.linalg.norm(p-self.xy)>.30:target=p;break
                delta=target-self.xy;v=delta/max(np.linalg.norm(delta),1e-9)*min(.24,.7*distance)
                msg.twist.linear.x=float(math.cos(self.yaw)*v[0]+math.sin(self.yaw)*v[1]);msg.twist.linear.y=float(-math.sin(self.yaw)*v[0]+math.cos(self.yaw)*v[1]);msg.twist.angular.z=float(np.clip(-1.5*self.yaw,-.3,.3))
        self.pub.publish(msg)

def navigation_main():
    rclpy.init();node=DepthNavNode()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()

class ControllerNode(Node):
    def __init__(self):
        super().__init__('bigspot_controller');self.design=design_for(self);self.gait=Gait(self.design);self.safety=Supervisor();self.velocity=np.zeros(3);self.tilt=np.zeros(2);self.positions=None;self.temperatures=25.
        self.targets=self.create_publisher(JointState,'/joint_targets',10);self.status=self.create_publisher(String,'/controller/status',10);self.frames=self.create_publisher(String,'/can/dry_run_frames',10)
        self.create_subscription(TwistStamped,'/cmd_vel',self.cmd,10);self.create_subscription(Imu,'/imu/data',self.imu,qos_profile_sensor_data);self.create_subscription(JointState,'/joint_states',self.joints,10);self.create_subscription(Bool,'/estop',lambda m:setattr(self.safety,'estop',m.data),10)
        self.output_q=None;self.create_timer(.005,self.tick)
    def now(self):return time.monotonic()
    def cmd(self,m):self.velocity[:]=[m.twist.linear.x,m.twist.linear.y,m.twist.angular.z];self.safety.update('command',self.now())
    def imu(self,m):
        r=quat_matrix(m.orientation);self.tilt[:]=[math.atan2(r[2,1],r[2,2]),math.atan2(-r[2,0],math.hypot(r[2,1],r[2,2]))];self.safety.update('imu',self.now())
    def joints(self,m):
        expected=[f'{leg}_{a}' for leg,_,_ in LEGS for a in ('abad','hip','knee')]
        if len(m.name)!=12 or set(m.name)!=set(expected) or len(m.position)!=12:return
        self.positions=[m.position[m.name.index(name)] for name in expected]
        if np.isfinite(self.positions).all():self.safety.update('joints',self.now())
    def tick(self):
        result=self.safety.evaluate(self.now(),tilt=float(max(abs(self.tilt))))
        velocity=self.velocity if result.motion_enabled else np.zeros(3)
        q=self.gait.targets(.005,velocity,roll=self.tilt[0]*.5,pitch=self.tilt[1]*.5)
        if self.output_q is None and self.positions is not None:self.output_q=np.asarray(self.positions).copy()
        if self.output_q is not None:
            self.output_q+=np.clip(q-self.output_q,-.004,.004);q=self.output_q.copy()
        msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=[f'{leg}_{a}' for leg,_,_ in LEGS for a in ('abad','hip','knee')];msg.position=q.tolist();self.targets.publish(msg)
        packets=[]
        if result.torque_enabled:
            for i,(model,p) in enumerate(zip(motor_names(self.design),q)):
                f=control(model,i%6+1,float(p),0.,self.design['control']['kp'],self.design['control']['kd']);packets.append({'bus':i//6,'id':f.arbitration_id,'data':f.data.hex()})
        self.frames.publish(String(data=json.dumps(packets)));self.status.publish(String(data=json.dumps({'reason':result.reason,'torque_enabled':result.torque_enabled,'hardware_output':False})))

def controller_main():
    rclpy.init();node=ControllerNode()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
