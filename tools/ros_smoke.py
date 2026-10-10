"""Exercise real DDS topics and camera->planner->cmd_vel->free-base physics."""
import json,os,sys,threading,time,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu,JointState
from bigspot_autonomy.ros_nodes import PhysicsNode,DepthNavNode,ControllerNode
from bigspot_autonomy.physics import rollout

if __name__=='__main__':
    rclpy.init(args=['--ros-args','-p','design_file:='+str(ROOT/'config/selected_design.json')])
    physics=PhysicsNode();nav=DepthNavNode();controller=ControllerNode()
    executor=MultiThreadedExecutor();executor.add_node(nav);executor.add_node(controller)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    physics.wall=time.monotonic();physics.realtime=True
    try:
        r=rollout(physics.design,speed=0.,seconds=35.,startup=True,on_step=physics.callback,command_source=physics.command_source,test_obstacles=True,output=ROOT/'generated/ros_smoke')
        r['topic_counts']=physics.counts;r['goal_distance_m']=float(math.hypot(r['net_x_m']-3,r['net_y_m']))
        r['ros_version']=os.environ.get('ROS_DISTRO');r['hardware_output']=False;r['isaac_tested']=False
        r['closed_loop_passed']=not r['fallen'] and r['obstacle_collision_impulse_ns']<.5 and r['goal_distance_m']<.25 and physics.counts['commands']>100
        (ROOT/'generated/ros_smoke_report.json').write_text(json.dumps(r,indent=2)+'\n')
        print(json.dumps({k:r[k] for k in ('closed_loop_passed','topic_counts','goal_distance_m','ros_version','thermal_margin_ratio','obstacle_collision_impulse_ns')}),flush=True)
    finally:
        executor.shutdown();thread.join(2)
        for node in (physics,nav,controller):node.destroy_node()
        rclpy.shutdown()
