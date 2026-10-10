#!/usr/bin/env bash
set -eo pipefail
repo="$(cd "$(dirname "$0")/../.." && pwd)"
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}" MUJOCO_GL=osmesa
. /opt/ros/humble/setup.bash
. "$repo/simulation/ros2_ws/install/setup.bash"
exec "$HOME/.cache/bigspot-sim/venv/bin/python" "$repo/simulation/tools/show_suite.py" "$@"
