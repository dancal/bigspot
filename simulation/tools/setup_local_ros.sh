#!/usr/bin/env bash
set -euo pipefail
# Local Ubuntu 22.04 transport/CPU simulation; not Isaac ROS 5.0.
. /etc/os-release
test "$VERSION_ID" = 22.04
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y ca-certificates curl gnupg python3-pip python3-venv
curl --fail --location https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/bigspot-ros-archive-keyring.gpg
printf 'deb [arch=%s signed-by=/usr/share/keyrings/bigspot-ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main\n' "$(dpkg --print-architecture)" > /etc/apt/sources.list.d/bigspot-ros2.list
apt-get update
# Inspect apt's proposed transactions; do not remove existing packages.
apt-get --no-remove install -y ros-humble-ros-base ros-humble-robot-state-publisher ros-humble-joy ros-humble-teleop-twist-joy python3-colcon-common-extensions libgl1-mesa-dri libosmesa6
