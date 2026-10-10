#!/usr/bin/env bash
set -eo pipefail
repo="$(cd "$(dirname "$0")/../.." && pwd)"
export PYTHONNOUSERSITE=1
. /opt/ros/humble/setup.bash
cd "$repo/simulation/ros2_ws"
colcon build --symlink-install --packages-select bigspot_sim
. "$repo/simulation/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=47
python3 - <<'PY'
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import yaml
import rclpy
from rclpy.parameter import Parameter
from bigspot_sim.joystick import Joystick
from bigspot_sim.joy_mapping import DEFAULTS
file=Path(get_package_share_directory('bigspot_sim'))/'config/joystick.yaml'
assert file.is_file(), 'Installed joystick configuration missing'
settings=yaml.safe_load(file.read_text())['bigspot_joystick']['ros__parameters']
assert all(k in DEFAULTS and settings[k]==DEFAULTS[k] for k in settings)
rclpy.init()
node=Joystick(parameter_overrides=[Parameter(k,value=v) for k,v in settings.items()])
assert node.mapping.settings['dance_action']=='dance_rhythm'
assert len(node.mapping.catalog)==26
node.destroy_node()
rclpy.shutdown()
print('INSTALLED_JOYSTICK_YAML_AND_ROS_PARAMETERS_OK')
PY
ros2 launch bigspot_sim motion.launch.py --show-args
if [ -d /dev/input ]; then ls /dev/input; else echo 'NO_WSL_INPUT_DEVICE'; fi
