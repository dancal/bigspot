"""Scoped Nav2 stack without unrelated route/docking/following servers."""
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    params=str(Path(get_package_share_directory('bigspot_autonomy'))/'config/nav2.yaml')
    specs=[('nav2_controller','controller_server'),('nav2_planner','planner_server'),('nav2_smoother','smoother_server'),
           ('nav2_behaviors','behavior_server'),('nav2_bt_navigator','bt_navigator'),('nav2_velocity_smoother','velocity_smoother'),('nav2_collision_monitor','collision_monitor')]
    actions=[]
    for package,name in specs:
        remaps=[('cmd_vel','cmd_vel_nav')] if name in ('controller_server','behavior_server','velocity_smoother') else []
        actions.append(Node(package=package,executable=name,name=name,parameters=[params],remappings=remaps,output='screen'))
    actions.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='bigspot_navigation_lifecycle',parameters=[{'autostart':True,'node_names':[name for _,name in specs]}]))
    return LaunchDescription(actions)
