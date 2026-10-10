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
        DeclareLaunchArgument('enable_experiments',default_value='false'),
        DeclareLaunchArgument('dance_bpm',default_value='60.0'),
        DeclareLaunchArgument('rhythm_bpm',default_value='90.0'),
        DeclareLaunchArgument('dance_button',default_value='12'),
        DeclareLaunchArgument('cute_button',default_value='13'),
        DeclareLaunchArgument('dance_action',default_value='dance_show'),
        DeclareLaunchArgument('cute_action',default_value='cute'),
        DeclareLaunchArgument('joystick_config',default_value=str(folder/'config/joystick.yaml')),
        DeclareLaunchArgument('policy',default_value=str(folder/'config/control_policy_ground_startup.json')),
        Node(package='robot_state_publisher',executable='robot_state_publisher',
             parameters=[{'robot_description':(folder/'urdf/bigspot.urdf').read_text(),'use_sim_time':True}],
             remappings=[('joint_states','/bigspot/joint_states')]),
        Node(package='bigspot_sim',executable='controller',output='screen',
             parameters=[{'use_sim_time':True,'gait':'auto','policy_file':LaunchConfiguration('policy'),
                 'dance_bpm':ParameterValue(LaunchConfiguration('dance_bpm'),value_type=float),
                 'rhythm_bpm':ParameterValue(LaunchConfiguration('rhythm_bpm'),value_type=float),
                 'enable_experiments':ParameterValue(LaunchConfiguration('enable_experiments'),value_type=bool)}]),
        Node(package='joy',executable='joy_node',parameters=[{'autorepeat_rate':20.0,'deadzone':.1}]),
        Node(package='bigspot_sim',executable='joystick',output='screen',
             parameters=[LaunchConfiguration('joystick_config'),{'dance_button':ParameterValue(LaunchConfiguration('dance_button'),value_type=int),
                          'cute_button':ParameterValue(LaunchConfiguration('cute_button'),value_type=int),
                          'dance_action':ParameterValue(LaunchConfiguration('dance_action'),value_type=str),
                          'cute_action':ParameterValue(LaunchConfiguration('cute_action'),value_type=str)}]),
    ])
