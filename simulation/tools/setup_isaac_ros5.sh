#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 "$script_dir/preflight.py"
command -v docker >/dev/null || { echo 'Install Docker Engine and NVIDIA Container Toolkit using NVIDIA Getting Started first.' >&2; exit 1; }
docker info >/dev/null
docker run --rm --gpus all ubuntu:24.04 nvidia-smi >/dev/null
sudo apt-get update
sudo apt-get install -y curl gnupg
key_tmp="$(mktemp)"
trap 'rm -f -- "$key_tmp"' EXIT
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key -o "$key_tmp"
gpg --batch --yes --dearmor --output "$key_tmp.gpg" "$key_tmp"
sudo install -m 0644 "$key_tmp.gpg" /usr/share/keyrings/nvidia-isaac-ros.gpg
rm -f -- "$key_tmp.gpg"
printf '%s\n' 'deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-5.0 noble main' | sudo tee /etc/apt/sources.list.d/nvidia-isaac-ros.list >/dev/null
sudo apt-get update
sudo apt-get install -y isaac-ros-cli
sudo isaac-ros init docker
printf '%s\n' 'Next: from repository root, export ISAAC_ROS_WS="$PWD"; isaac-ros activate' 'Inside the container: bash simulation/tools/build_workspace.sh'
