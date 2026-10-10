#!/usr/bin/env bash
set -eo pipefail
repo="$(cd "$(dirname "$0")/../.." && pwd)"
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=42
export MUJOCO_GL=osmesa
. /opt/ros/humble/setup.bash
. "$repo/simulation/ros2_ws/install/setup.bash"
exec "$HOME/.cache/bigspot-sim/venv/bin/python" "$repo/simulation/tools/posture_simulation.py" --ros --render --seconds 24 --policy "$repo/simulation/ros2_ws/src/bigspot_sim/config/control_policy.json" --case ros_wave_validation "$@"
