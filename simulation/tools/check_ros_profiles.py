"""Validate installed policy selection in actual rclpy, without commanding motors."""
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_share_directory
from bigspot_sim.posture_node import Controller

rclpy.init()
folder=Path(get_package_share_directory('bigspot_sim'))/'config'
try:
    for filename,expected in [('control_policy.json','crawl'),('control_policy_fast.json','trot')]:
        node=Controller(parameter_overrides=[Parameter('policy_file',value=str(folder/filename))])
        try:
            if node.gait.mode!=expected:
                raise RuntimeError(f'{filename}: unexpected mode {node.gait.mode}')
            print(f'{filename}: mode={node.gait.mode}, max_forward={node.gait.speed_limits[0]:.3f} m/s, period={node.gait.period:.3f} s')
        finally:
            node.destroy_node()
finally:
    rclpy.shutdown()
