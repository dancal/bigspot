#!/usr/bin/env bash
set -eo pipefail
task_root=$(cd -- "$(dirname -- "$0")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$task_root/install/setup.bash"
source "$task_root/.venv-linux/bin/activate"
export PYTHONPATH="$task_root:${PYTHONPATH:-}"
python "$task_root/tools/ros_smoke.py"
