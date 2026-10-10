from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    folder=Path(get_package_share_directory('bigspot_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('forward_speed',default_value='0.015'),
        DeclareLaunchArgument('dance_button',default_value='12'),
        DeclareLaunchArgument('cute_button',default_value='13'),
        DeclareLaunchArgument('dance_action',default_value='dance_show'),
        DeclareLaunchArgument('cute_action',default_value='cute'),
        DeclareLaunchArgument('joystick_config',default_value=str(folder/'config/joystick.yaml')),
        Node(package='joy',executable='joy_node',parameters=[{'autorepeat_rate':20.0,'deadzone':.1}]),
        Node(package='bigspot_sim',executable='joystick',output='screen',
             parameters=[LaunchConfiguration('joystick_config'),{'forward_speed':ParameterValue(LaunchConfiguration('forward_speed'),value_type=float),
                          'dance_button':ParameterValue(LaunchConfiguration('dance_button'),value_type=int),
                          'cute_button':ParameterValue(LaunchConfiguration('cute_button'),value_type=int),
                          'dance_action':ParameterValue(LaunchConfiguration('dance_action'),value_type=str),
                          'cute_action':ParameterValue(LaunchConfiguration('cute_action'),value_type=str)}]),
    ])
