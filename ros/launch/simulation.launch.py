from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share=Path(get_package_share_directory('bigspot_autonomy'))
    return LaunchDescription([
        DeclareLaunchArgument('duration',default_value='35.0'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':(share/'design/bigspot.urdf').read_text(),'use_sim_time':True}]),
        Node(package='bigspot_autonomy',executable='physics_node',parameters=[{'duration':ParameterValue(LaunchConfiguration('duration'),value_type=float),'realtime':True,'startup':True,'obstacles':True}],output='screen'),
        Node(package='bigspot_autonomy',executable='depth_nav_node',parameters=[{'use_sim_time':True}],output='screen'),
        Node(package='bigspot_autonomy',executable='controller_node',parameters=[{'use_sim_time':True}],output='screen')])
