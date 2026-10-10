"""Isaac ROS 5: select one camera source and one odometry TF authority."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def setup(context):
    share=Path(get_package_share_directory('bigspot_autonomy'))
    source=LaunchConfiguration('camera_source').perform(context)
    if source not in ('sim','realsense','external'):
        raise ValueError('camera_source must be sim, realsense, or external')
    simulated=source=='sim';params=str(share/'config/isaac.yaml')
    actions=[]
    if source=='realsense':
        actions.append(Node(package='realsense2_camera',executable='realsense2_camera_node',name='camera',namespace='',
            parameters=[{'camera_name':'camera','enable_infra1':True,'enable_infra2':True,'enable_color':True,
                'enable_depth':True,'enable_gyro':True,'enable_accel':True,'unite_imu_method':2,
                'gyro_fps':200,'accel_fps':250,'depth_module.depth_profile':'640,480,30',
                'depth_module.infra_profile':'640,480,30','rgb_camera.color_profile':'640,480,30',
                'depth_module.emitter_enabled':0,'align_depth.enable':True,'pointcloud.enable':True}],output='screen'))
    vslam=ComposableNode(package='isaac_ros_cuvslam',plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node',parameters=[params,{'use_sim_time':simulated,'tracking_mode':0 if simulated else 1}],
        remappings=[('visual_slam/image_0','/camera/infra1/image_rect_raw'),
            ('visual_slam/image_1','/camera/infra2/image_rect_raw'),
            ('visual_slam/camera_info_0','/camera/infra1/camera_info'),
            ('visual_slam/camera_info_1','/camera/infra2/camera_info'),('visual_slam/imu','/camera/imu')])
    nvblox=ComposableNode(package='nvblox_ros',plugin='nvblox::NvbloxNode',name='nvblox_node',
        parameters=[params,{'use_sim_time':simulated}],remappings=[
            ('camera_0/depth/image','/camera/aligned_depth_to_color/image_raw'),
            ('camera_0/depth/camera_info','/camera/aligned_depth_to_color/camera_info'),
            ('camera_0/color/image','/camera/color/image_raw'),('camera_0/color/camera_info','/camera/color/camera_info')])
    actions.append(ComposableNodeContainer(name='bigspot_perception',namespace='',package='rclcpp_components',
        executable='component_container_mt',composable_node_descriptions=[vslam,nvblox],output='screen'))
    actions.append(Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{
        'robot_description':(share/'design/pi4/bigspot.urdf').read_text(),'use_sim_time':simulated}]))
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/nav2.launch.py')),
        launch_arguments={'use_sim_time':str(simulated).lower()}.items()))
    actions.append(Node(package='bigspot_autonomy',executable='controller_node',parameters=[{
        'use_sim_time':simulated,'design_file':str(share/'config/pi4_design.json'),'autonomous':True,
        'simulation_auto_arm':False}],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('camera_source',default_value='sim'),OpaqueFunction(function=setup)])
