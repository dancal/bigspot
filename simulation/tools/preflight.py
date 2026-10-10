"""Read-only requirements check for the target Ubuntu GPU workstation."""
import json
import platform
import re
import shutil
import subprocess
import sys

errors = []
report = {'platform': platform.platform(), 'required': {'ubuntu': '24.04', 'isaac_ros': '5.0.0', 'ros': 'lyrical', 'isaac_sim': '6.0.1', 'nvidia_driver_min': 595, 'sim_vram_min_mib': 16384}}
if platform.system() != 'Linux':
    errors.append('Target must be Ubuntu 24.04; this checker does not install software.')
else:
    from pathlib import Path
    os_release = Path('/etc/os-release').read_text()
    if 'ID=ubuntu' not in os_release or 'VERSION_ID="24.04"' not in os_release:
        errors.append('Isaac ROS 5.0 target requires Ubuntu 24.04.')
    if 'microsoft' in platform.release().lower():
        errors.append('WSL is not the selected supported native Ubuntu GPU target.')
if not shutil.which('nvidia-smi'):
    errors.append('NVIDIA GPU/driver not found; AMD GPU cannot run this stack.')
else:
    result = subprocess.run(['nvidia-smi', '--query-gpu=name,memory.total,driver_version', '--format=csv,noheader,nounits'], capture_output=True, text=True)
    report['gpus'] = result.stdout.strip()
    viable = False
    for line in result.stdout.splitlines():
        name, memory, driver = map(str.strip, line.split(','))
        # Conservative allow-list: Turing RTX 20xx and non-RTX compute GPUs do not pass.
        ampere_or_newer_rtx = re.search(r'\bRTX\s+(?:30\d\d|40\d\d|50\d\d|A\d+|PRO\b)', name, re.IGNORECASE)
        if ampere_or_newer_rtx and float(memory) >= 16384 and int(driver.split('.')[0]) >= 595:
            viable = True
    if not viable:
        errors.append('Need an Ampere+ RTX GPU, at least 16 GiB VRAM and driver 595+. Other server GPU names require manual RTX capability verification.')
    # Do not install/upgrade a GPU driver automatically.
report['errors'] = errors
report['ready'] = not errors
print(json.dumps(report, indent=2))
sys.exit(bool(errors))
