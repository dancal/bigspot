"""Read-only preflight. Does not install drivers or enable motor power."""
import json,os,platform,subprocess,shutil
from pathlib import Path
report={'host':platform.system(),'target':'Isaac ROS 5.0 / ROS 2 Lyrical','passed':False,'hardware_output':False}
if platform.system()=='Linux':
    os_info=Path('/etc/os-release').read_text();report['ubuntu_24_04']='VERSION_ID="24.04"' in os_info
    report['ros_distro']=os.environ.get('ROS_DISTRO')
    if shutil.which('nvidia-smi'):
        p=subprocess.run(['nvidia-smi','--query-gpu=name,driver_version,memory.total,compute_cap','--format=csv,noheader,nounits'],capture_output=True,text=True)
        report['gpu']=p.stdout.strip();report['error']=p.stderr.strip()
        for row in p.stdout.splitlines():
            fields=[x.strip() for x in row.split(',')]
            if len(fields)==4:
                report['gpu_supported']=int(fields[1].split('.')[0])>=595 and float(fields[2])>=8192 and float(fields[3])>=8.0
    report['passed']=bool(report.get('gpu_supported') and report['ubuntu_24_04'] and report['ros_distro']=='lyrical')
else:report['reason']='Local Windows/AMD host cannot run Isaac ROS CUDA stack. CPU physics remains available.'
report['jetson_note']='Jetson Orin/Thor uses JetPack 7.2. Verify on the actual Jetson separately; x86 nvidia-smi check is not a Jetson certification.'
print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 2)
