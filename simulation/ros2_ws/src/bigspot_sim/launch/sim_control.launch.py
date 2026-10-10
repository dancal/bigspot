from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = (Path(get_package_share_directory('bigspot_sim'))/'urdf/bigspot.urdf').read_text()
    return LaunchDescription([
        DeclareLaunchArgument('gait', default_value='auto'),
        DeclareLaunchArgument('enable_experiments',default_value='false'),
        DeclareLaunchArgument('policy',default_value=str(Path(get_package_share_directory('bigspot_sim'))/'config/control_policy_ground_startup.json')),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}],
             remappings=[('joint_states', '/bigspot/joint_states')]),
        Node(package='bigspot_sim', executable='controller', output='screen',
             parameters=[{'use_sim_time': True, 'gait': LaunchConfiguration('gait'),
                          'policy_file':LaunchConfiguration('policy'),
                          'enable_experiments':ParameterValue(LaunchConfiguration('enable_experiments'),value_type=bool)}]),
    ])
