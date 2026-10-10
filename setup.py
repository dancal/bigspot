from setuptools import setup
from glob import glob
setup(name='bigspot_autonomy',version='0.1.0',packages=['bigspot_autonomy'],
      data_files=[('share/ament_index/resource_index/packages',['resource/bigspot_autonomy']),
                  ('share/bigspot_autonomy',['package.xml']),('share/bigspot_autonomy/config',glob('config/*.json')+glob('ros/config/*.yaml')),
                  ('share/bigspot_autonomy/design',glob('design/*.urdf')),('share/bigspot_autonomy/launch',glob('ros/launch/*.py'))],
      install_requires=['setuptools','numpy'],zip_safe=False,
      entry_points={'console_scripts':['physics_node=bigspot_autonomy.ros_nodes:physics_main',
                                      'depth_nav_node=bigspot_autonomy.ros_nodes:navigation_main',
                                      'controller_node=bigspot_autonomy.ros_nodes:controller_main']})
