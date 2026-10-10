#!/usr/bin/env bash
set -euo pipefail
[[ "${ROS_DISTRO:-}" == lyrical ]] || { echo 'Activate Isaac ROS 5.0 (ROS 2 Lyrical) first.' >&2; exit 1; }
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/../.." && pwd)"
sudo apt-get update
sudo apt-get install -y python3-numpy python3-colcon-common-extensions ros-lyrical-robot-state-publisher ros-lyrical-isaac-ros-apriltag
python3 "$script_dir/build_model.py"
cd "$repo_dir/simulation/ros2_ws"
colcon build --symlink-install --packages-select bigspot_sim
printf '%s\n' 'source install/setup.bash' 'ros2 launch bigspot_sim sim_control.launch.py' 'ros2 launch bigspot_sim perception.launch.py'
