#!/usr/bin/env bash
set -euo pipefail
repo="$(cd "$(dirname "$0")/../.." && pwd)"
set +u
. /opt/ros/humble/setup.bash
set -u
runtime="$HOME/.cache/bigspot-sim/venv"
python3 -m venv --system-site-packages "$runtime"
"$runtime/bin/python" -m pip install 'mujoco==3.15.0' 'numpy==1.26.4' 'pillow==12.3.0'
cd "$repo/simulation/ros2_ws"
colcon build --packages-select bigspot_sim --symlink-install
