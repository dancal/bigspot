"""Live DDS test: planner -> controller -> targets -> external CPU plant.

Requires an installed/sourced ROS workspace; never calls physics.rollout.
The CPU planner uses simulator ground-truth odometry, not Isaac Visual SLAM.
"""
import json
import math
import os
import sys
import time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from bigspot_autonomy.cpu_ros import CpuNode
from bigspot_autonomy.ros_nodes import DepthNavNode, ControllerNode


def main():
    rclpy.init(args=['--ros-args','-p','design_file:='+str(ROOT/'config/pi4_design.json'),
        '-p','duration:=50.0','-p','use_sim_time:=true','-p','simulation_auto_arm:=true','-p','autonomous:=true',
        '-r','/camera/aligned_depth_to_color/image_raw:=/camera/depth/image_rect_raw',
        '-r','/visual_slam/tracking/odometry:=/odom'])
    physics=CpuNode()
    # The simulator drives /clock; its physics timer must use wall time.
    from rclpy.parameter import Parameter
    physics.set_parameters([Parameter('use_sim_time',value=False)])
    nav=DepthNavNode();controller=ControllerNode();observer=Node('bigspot_smoke_observer')
    counts={'targets':0,'odom':0,'status':0};states=set()
    def target(m):counts['targets']+=1
    def odom(m):counts['odom']+=1
    def status(m):counts['status']+=1;states.add(json.loads(m.data)['state'])
    observer.create_subscription(JointState,'/joint_targets',target,10)
    observer.create_subscription(Odometry,'/odom',odom,10)
    observer.create_subscription(String,'/controller/status',status,10)
    executor=SingleThreadedExecutor();nodes=(physics,nav,controller,observer)
    for node in nodes:executor.add_node(node)
    started=time.monotonic();max_tilt=collision=0.;last_physics_time=-1.
    try:
        while physics.plant.data.time<45 and time.monotonic()-started<180:
            executor.spin_once(timeout_sec=.01)
            p=physics.plant
            if p.data.time==last_physics_time:continue
            last_physics_time=float(p.data.time)
            if p.data.time>6:max_tilt=max(max_tilt,float(np.rad2deg(max(abs(controller.core.rp)))))
            for i,c in enumerate(p.data.contact):
                names=[p.mj.mj_id2name(p.model,p.mj.mjtObj.mjOBJ_GEOM,int(g)) or '' for g in (c.geom1,c.geom2)]
                if any(n.startswith('obstacle_') for n in names):
                    force=np.zeros(6);p.mj.mj_contactForce(p.model,p.data,i,force);collision+=max(0.,force[0])*p.dt
        p=physics.plant;distance=float(math.hypot(p.data.qpos[0]-3,p.data.qpos[1]))
        passed=bool(p.data.time>=45 and 'READY' in states and 'FAULT' not in states and
            not p.watchdog.expired and max_tilt<35 and distance<.25 and collision<.5 and counts['targets']>100)
        report={'closed_loop_passed':passed,'ros_executed':True,'isaac_executed':False,
            'hardware_tested':False,'ground_truth_odometry':True,'topic_counts':counts,
            'states':sorted(states),'sim_seconds':float(p.data.time),'goal_distance_m':distance,
            'max_tilt_deg':max_tilt,'obstacle_collision_impulse_ns':collision,'ros_distro':os.environ.get('ROS_DISTRO')}
        path=ROOT/'generated/ros_smoke_report.json';path.parent.mkdir(exist_ok=True);path.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2));return 0 if passed else 2
    finally:
        executor.shutdown()
        for node in nodes:node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
