#!/usr/bin/env bash
set -eo pipefail
repo="$(cd "$(dirname "$0")/../.." && pwd)"
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=42 MUJOCO_GL=osmesa
. /opt/ros/humble/setup.bash
. "$repo/simulation/ros2_ws/install/setup.bash"
exec "$HOME/.cache/bigspot-sim/venv/bin/python" "$repo/simulation/tools/posture_simulation.py" \
  --ros --render --seconds 24 --speed .1 \
  --policy "$repo/simulation/ros2_ws/src/bigspot_sim/config/control_policy_fast.json" \
  --case ros_fast_validation "$@"
