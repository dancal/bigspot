"""Isaac ROS 5.0 GPU AprilTag component, using the simulated mono camera."""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([ComposableNodeContainer(
        name='bigspot_perception', namespace='', package='rclcpp_components',
        executable='component_container_mt', output='screen',
        composable_node_descriptions=[ComposableNode(
            package='isaac_ros_apriltag', plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
            name='apriltag', parameters=[{'use_sim_time': True, 'size': .22, 'max_tags': 64}],
            remappings=[('image', '/bigspot/camera/image_rect'),
                        ('camera_info', '/bigspot/camera/camera_info')])])])
