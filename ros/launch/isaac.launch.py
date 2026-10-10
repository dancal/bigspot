"""Isaac ROS 5.0 deployment profile. GPU/camera execution has NOT been tested."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node,ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share=Path(get_package_share_directory('bigspot_autonomy'));params=str(share/'config/isaac.yaml')
    camera=Node(package='realsense2_camera',executable='realsense2_camera_node',name='camera',parameters=[{
        'enable_infra1':True,'enable_infra2':True,'enable_color':True,'enable_depth':True,
        'enable_gyro':True,'enable_accel':True,'unite_imu_method':2,'gyro_fps':200,'accel_fps':250,
        'depth_module.depth_profile':'640,480,30','depth_module.infra_profile':'640,480,30',
        'rgb_camera.color_profile':'640,480,30','depth_module.emitter_enabled':0,
        'align_depth.enable':True,'pointcloud.enable':True}],output='screen')
    vslam=ComposableNode(package='isaac_ros_cuvslam',plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',name='visual_slam_node',parameters=[params],remappings=[
        ('visual_slam/image_0','/camera/infra1/image_rect_raw'),('visual_slam/image_1','/camera/infra2/image_rect_raw'),
        ('visual_slam/camera_info_0','/camera/infra1/camera_info'),('visual_slam/camera_info_1','/camera/infra2/camera_info'),('visual_slam/imu','/camera/imu')])
    nvblox=ComposableNode(package='nvblox_ros',plugin='nvblox::NvbloxNode',name='nvblox_node',parameters=[params],remappings=[
        ('camera_0/depth/image','/camera/aligned_depth_to_color/image_raw'),('camera_0/depth/camera_info','/camera/aligned_depth_to_color/camera_info'),
        ('camera_0/color/image','/camera/color/image_raw'),('camera_0/color/camera_info','/camera/color/camera_info')])
    container=ComposableNodeContainer(name='bigspot_perception',namespace='',package='rclcpp_components',executable='component_container_mt',composable_node_descriptions=[vslam,nvblox],output='screen')
    navigation=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/nav2.launch.py')))
    robot=Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':(share/'design/bigspot.urdf').read_text()}])
    return LaunchDescription([camera,robot,container,TimerAction(period=8.,actions=[navigation]),Node(package='bigspot_autonomy',executable='controller_node',output='screen')])
