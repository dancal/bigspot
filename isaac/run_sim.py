"""Isaac Sim 6.0.1 free-base QDD + ROS 2 bridge. GPU execution pending.

Run using Isaac Sim's python.sh after sourcing ROS 2 Lyrical. Physics uses CPU;
RTX cameras require NVIDIA graphics. Body attitude is simulated ground truth.
Only initialization writes poses; all subsequent motion uses bounded efforts.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from bigspot_autonomy.design import load_design
from bigspot_autonomy.control_core import JOINT_NAMES
from bigspot_autonomy.sim_backend import EffortWatchdog
from bigspot_autonomy.asset_paths import resolve_meshes
from bigspot_autonomy.camera_geometry import (WIDTH,HEIGHT,BASELINE_M,FOCAL_MM,
    HORIZONTAL_APERTURE_MM,camera_matrices,depth_points)


def array(value):
    return value.numpy() if hasattr(value,'numpy') else np.asarray(value)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--seconds',type=float,default=0.,help='0 runs until closed')
    parser.add_argument('--design',type=Path,default=ROOT/'config/pi4_design.json')
    parser.add_argument('--urdf',type=Path,default=ROOT/'design/pi4/bigspot.urdf')
    parser.add_argument('--report',type=Path,default=ROOT/'generated/isaac/session.json')
    args=parser.parse_args();design=load_design(args.design)
    if not args.urdf.is_file():raise SystemExit('Generate the matching model with tools/export_sim_model.py')
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':args.headless})
    node=None;callbacks=[]
    try:
        import isaacsim.core.experimental.utils.app as app_utils
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.simulation_manager import SimulationManager,SimulationEvent
        from isaacsim.core.experimental.prims import Articulation,RigidPrim
        import omni.usd
        from pxr import UsdGeom,UsdPhysics,UsdLux,Gf,UsdShade
        app_utils.enable_extension('isaacsim.asset.importer.urdf')
        app_utils.enable_extension('isaacsim.ros2.bridge')
        app_utils.enable_extension('isaacsim.sensors.experimental.rtx')
        app.update()
        from isaacsim.asset.importer.urdf import URDFImporter,URDFImporterConfig
        from isaacsim.sensors.experimental.rtx import RtxCamera,CameraSensor
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState,Imu,Image,CameraInfo,PointCloud2,PointField
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import TransformStamped
        from rosgraph_msgs.msg import Clock
        from tf2_ros import StaticTransformBroadcaster
        from builtin_interfaces.msg import Time

        generated=ROOT/'generated/isaac';generated.mkdir(parents=True,exist_ok=True)
        resolved=resolve_meshes(args.urdf,ROOT,generated/'resolved_robot.urdf')
        usd=URDFImporter(URDFImporterConfig(urdf_path=str(resolved),usd_path=str(generated/'usd'),
            fix_base=False,merge_fixed_joints=False,merge_mesh=False,allow_self_collision=True,
            joint_drive_type='force',joint_target_type='position',override_joint_stiffness=0.,
            override_joint_damping=0.)).import_urdf()
        stage_utils.set_stage_units(meters_per_unit=1.)
        stage_utils.add_reference_to_stage(usd,'/World/BigSpot')
        stage=omni.usd.get_context().get_stage()
        initial_height=design.get('startup_base_height_m',.067)
        UsdGeom.XformCommonAPI(stage.GetPrimAtPath('/World/BigSpot')).SetTranslate(Gf.Vec3d(0,0,initial_height))
        scene=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene')
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0,0,-1));scene.CreateGravityMagnitudeAttr().Set(9.81)
        material=UsdShade.Material.Define(stage,'/World/FloorMaterial')
        physics_material=UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_material.CreateStaticFrictionAttr().Set(.8);physics_material.CreateDynamicFrictionAttr().Set(.8)
        physics_material.CreateRestitutionAttr().Set(0.)

        def box(path,position,size,color,collision=True):
            cube=UsdGeom.Cube.Define(stage,path);cube.GetSizeAttr().Set(1.)
            xform=UsdGeom.XformCommonAPI(cube);xform.SetTranslate(Gf.Vec3d(*position));xform.SetScale(Gf.Vec3f(*size))
            cube.CreateDisplayColorAttr().Set([Gf.Vec3f(*color)])
            if collision:
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
                UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material,materialPurpose='physics')
            return cube
        box('/World/Floor',(0,0,-.05),(16,16,.1),(.22,.25,.28))
        # Fixed visual landmarks improve observability; no localization truth TF.
        rng=np.random.default_rng(17)
        for i in range(100):
            x,y=float(rng.uniform(-1,5)),float(rng.uniform(-2,2))
            box(f'/World/Mark_{i}',(x,y,.0005),(.10,.10,.001),tuple(rng.uniform(.1,.9,3)),False)
        box('/World/Obstacle_0',(1.1,0,.25),(.36,.5,.5),(.9,.4,.15))
        box('/World/Obstacle_1',(2.1,.65,.25),(.4,.36,.5),(.2,.7,.35))
        light=UsdLux.DomeLight.Define(stage,'/World/Light');light.CreateIntensityAttr().Set(1500.)
        app.update()
        prims=list(stage.Traverse())
        roots=[p for p in prims if str(p.GetPath()).startswith('/World/BigSpot') and p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        if len(roots)!=1:raise RuntimeError('Expected exactly one articulation root')
        def named(name):
            paths=[str(p.GetPath()) for p in prims if str(p.GetPath()).startswith('/World/BigSpot/') and p.GetName()==name]
            if len(paths)!=1:raise RuntimeError('Cannot resolve robot frame '+name)
            return paths[0]
        base_path,camera_path=named('base_link'),named('camera_link')
        cameras=[]
        for name,y in (('left',BASELINE_M/2),('right',-BASELINE_M/2),('color',0.)):
            path=camera_path+'/'+name
            camera=UsdGeom.Camera.Define(stage,path)
            transform=UsdGeom.XformCommonAPI(camera)
            transform.SetTranslate(Gf.Vec3d(0,y,0));transform.SetRotate((90,0,-90))
            camera.GetHorizontalApertureAttr().Set(HORIZONTAL_APERTURE_MM)
            camera.GetVerticalApertureAttr().Set(HORIZONTAL_APERTURE_MM*HEIGHT/WIDTH)
            camera.GetFocalLengthAttr().Set(FOCAL_MM);camera.GetClippingRangeAttr().Set(Gf.Vec2f(.05,20.))
            author=RtxCamera(path,tick_rate=0.,reset_xform_op_properties=False)
            cameras.append(CameraSensor(author,resolution=(HEIGHT,WIDTH),annotators=['rgb','distance_to_image_plane']))

        class Bridge(Node):
            def __init__(self):
                super().__init__('bigspot_isaac_bridge');self.watchdog=EffortWatchdog(design);self.t=0.;self.frames=0
                self.counts={'joints':0,'imu':0,'camera':0,'targets':0};self.latest_q=self.latest_v=None
                topics=[('/clock',Clock),('/joint_states',JointState),('/imu/data',Imu),('/sim/ground_truth/odom',Odometry),
                    ('/camera/infra1/image_rect_raw',Image),('/camera/infra1/camera_info',CameraInfo),
                    ('/camera/infra2/image_rect_raw',Image),('/camera/infra2/camera_info',CameraInfo),
                    ('/camera/color/image_raw',Image),('/camera/color/camera_info',CameraInfo),
                    ('/camera/aligned_depth_to_color/image_raw',Image),('/camera/aligned_depth_to_color/camera_info',CameraInfo),
                    ('/camera/depth/color/points',PointCloud2)]
                self.pubs={n:self.create_publisher(typ,n,10) for n,typ in topics}
                self.create_subscription(JointState,'/joint_targets',self.target,10)
                self.static=StaticTransformBroadcaster(self)
                transforms=[]
                for frame,y in (('camera_infra1_optical_frame',BASELINE_M/2),('camera_infra2_optical_frame',-BASELINE_M/2),('camera_color_optical_frame',0.)):
                    tf=TransformStamped();tf.header.frame_id='camera_link';tf.child_frame_id=frame
                    tf.transform.translation.y=y
                    tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=-.5,.5,-.5,.5
                    transforms.append(tf)
                self.static.sendTransform(transforms)
            def timestamp(self):
                ns=round(self.t*1e9);return Time(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)
            def target(self,m):
                stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
                if self.watchdog.receive_target(m.name,m.position,m.effort,stamp,self.t,time.monotonic()):self.counts['targets']+=1
            def image(self,topic,frame,value,encoding):
                msg=Image();msg.header.stamp=self.timestamp();msg.header.frame_id=frame
                msg.height,msg.width=value.shape[:2];msg.encoding=encoding;msg.step=int(value.strides[0]);msg.data=value.tobytes();self.pubs[topic].publish(msg)
            def info(self,topic,frame,right=False):
                msg=CameraInfo();msg.header.stamp=self.timestamp();msg.header.frame_id=frame;msg.width=WIDTH;msg.height=HEIGHT
                msg.distortion_model='plumb_bob';msg.d=[0.]*5;msg.k,msg.p=camera_matrices(right);msg.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.];self.pubs[topic].publish(msg)
            def publish_cameras(self):
                values=[c.get_data('rgb')[0] for c in cameras];depth=cameras[2].get_data('distance_to_image_plane')[0]
                if depth is None or any(x is None for x in values):return
                rgb=[array(x)[...,:3].astype(np.uint8) for x in values];depth=np.asarray(array(depth),dtype='<f4').reshape(HEIGHT,WIDTH)
                if any(x.shape!=(HEIGHT,WIDTH,3) for x in rgb):return
                for i in range(2):
                    mono=np.asarray(np.clip(rgb[i].astype(float)@np.array([.299,.587,.114]),0,255),dtype=np.uint8)
                    frame=f'camera_infra{i+1}_optical_frame'
                    self.image(f'/camera/infra{i+1}/image_rect_raw',frame,mono,'mono8');self.info(f'/camera/infra{i+1}/camera_info',frame,i==1)
                frame='camera_color_optical_frame'
                self.image('/camera/color/image_raw',frame,np.ascontiguousarray(rgb[2]),'rgb8');self.info('/camera/color/camera_info',frame)
                self.image('/camera/aligned_depth_to_color/image_raw',frame,np.ascontiguousarray(depth),'32FC1');self.info('/camera/aligned_depth_to_color/camera_info',frame)
                xyz=depth_points(depth)
                msg=PointCloud2();msg.header.stamp=self.timestamp();msg.header.frame_id=frame;msg.height=1;msg.width=len(xyz)
                msg.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
                msg.point_step=12;msg.row_step=12*len(xyz);msg.is_dense=True;msg.data=xyz.tobytes();self.pubs['/camera/depth/color/points'].publish(msg)
                self.counts['camera']+=1

        rclpy.init();node=Bridge()
        SimulationManager.setup_simulation(dt=.002,device='cpu')
        app_utils.play();app.update()
        articulation=Articulation(str(roots[0].GetPath()));indices=articulation.get_dof_indices(list(JOINT_NAMES))
        articulation.set_dof_gains(stiffnesses=0.,dampings=0.)
        articulation.set_dof_positions(np.tile(design.get('startup_joint_angles_rad',[0.,-1.43,2.86]),4),dof_indices=indices)
        articulation.set_world_poses(positions=np.array([[0.,0.,initial_height]]),orientations=np.array([[1.,0.,0.,0.]]))
        base=RigidPrim(base_path)
        def before(dt,context):
            node.t=SimulationManager.get_simulation_time();rclpy.spin_once(node,timeout_sec=0)
            q=array(articulation.get_dof_positions(dof_indices=indices)).reshape(12)
            v=array(articulation.get_dof_velocities(dof_indices=indices)).reshape(12)
            tau=node.watchdog.motor_efforts(node.t,time.monotonic(),q,v)
            articulation.set_dof_efforts(tau,dof_indices=indices)
        def after(dt,context):
            node.t=SimulationManager.get_simulation_time();node.pubs['/clock'].publish(Clock(clock=node.timestamp()))
            if node.frames%5==0:
                q=array(articulation.get_dof_positions(dof_indices=indices)).reshape(12);v=array(articulation.get_dof_velocities(dof_indices=indices)).reshape(12)
                msg=JointState();msg.header.stamp=node.timestamp();msg.name=list(JOINT_NAMES);msg.position=q.tolist();msg.velocity=v.tolist();node.pubs['/joint_states'].publish(msg);node.counts['joints']+=1
                pos,quat=base.get_world_poses();position=array(pos).reshape(3);w,x,y,z=array(quat).reshape(4)
                _,angular=base.get_velocities();world_gyro=array(angular).reshape(3)
                r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
                imu=Imu();imu.header.stamp=node.timestamp();imu.header.frame_id='body_imu_link'
                imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w=map(float,(x,y,z,w))
                imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z=map(float,r.T@world_gyro)
                imu.linear_acceleration_covariance[0]=-1.;node.pubs['/imu/data'].publish(imu);node.counts['imu']+=1
                truth=Odometry();truth.header.stamp=node.timestamp();truth.header.frame_id='sim_world';truth.child_frame_id='base_link'
                truth.pose.pose.position.x,truth.pose.pose.position.y,truth.pose.pose.position.z=map(float,position);truth.pose.pose.orientation=imu.orientation
                node.pubs['/sim/ground_truth/odom'].publish(truth)
            node.frames+=1
        callbacks=[SimulationManager.register_callback(before,SimulationEvent.PHYSICS_PRE_STEP),
                   SimulationManager.register_callback(after,SimulationEvent.PHYSICS_POST_STEP)]
        next_camera=0.
        while app.is_running() and (args.seconds==0 or node.t<args.seconds):
            app.update()
            if node.t>=next_camera:
                node.publish_cameras();next_camera=node.t+1/30
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps({'engine':'Isaac Sim 6.0.1','counts':node.counts,
            'sim_seconds':node.t,'watchdog_latched':node.watchdog.expired,'hardware_tested':False,
            'navigation_success_verified':False,'ground_truth_used_for_navigation':False,
            'self_collision_enabled':True},indent=2)+'\n')
    finally:
        if node is not None:node.destroy_node();rclpy.shutdown()
        if callbacks:
            for callback in callbacks:SimulationManager.deregister_callback(callback)
        app.close()


if __name__=='__main__':main()
