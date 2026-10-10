#!/usr/bin/env bash
# Build only; OS, ROS, CUDA, drivers and Isaac packages must already be installed.
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="${1:?Usage: build_ros_workspace.sh /absolute/ros_workspace}"
if [[ "$workspace_dir" != /* ]]; then echo 'An absolute workspace path is required.' >&2; exit 2; fi
if [[ "${ROS_DISTRO:-}" != lyrical ]]; then echo 'Source ROS 2 Lyrical before building the Isaac ROS 5 workspace.' >&2; exit 2; fi
for package in isaac_ros_cuvslam nvblox_ros nvblox_nav2 nav2_bringup robot_state_publisher; do
    ros2 pkg prefix "$package" >/dev/null || { echo "Missing ROS package: $package" >&2; exit 2; }
done
python3 -c 'import numpy, mujoco, rclpy'
mkdir -p "$workspace_dir/src"
target="$workspace_dir/src/bigspot_autonomy"
if [[ ! -e "$target" && ! -L "$target" ]]; then ln -s "$repo_dir" "$target"; fi
if [[ "$(readlink -f "$target")" != "$repo_dir" ]]; then echo 'Workspace already contains a different bigspot_autonomy checkout.' >&2; exit 2; fi
cd "$workspace_dir"
colcon build --symlink-install --packages-select bigspot_autonomy
