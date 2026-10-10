"""Read-only host inventory; never installs drivers or enables motors."""
import argparse
import json
import os
import platform
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path

def inventory():
    host=platform.system()
    report={'checked_at':datetime.now(timezone.utc).isoformat(),'host':host,
            'target':'Isaac ROS 5.0 / ROS 2 Lyrical / Isaac Sim 6.0.1',
            'hardware_output':False,'gpu':[],'ros_distro':os.environ.get('ROS_DISTRO'),
            'ubuntu_24_04':False,'ros_ready':False,'sim_requirements_met':False}
    release=Path('/etc/os-release')
    if host=='Linux' and release.exists():
        fields=dict(line.split('=',1) for line in release.read_text().splitlines() if '=' in line)
        report['ubuntu_24_04']=fields.get('ID','').strip('"')=='ubuntu' and fields.get('VERSION_ID','').strip('"')=='24.04'
    executable=shutil.which('nvidia-smi')
    if executable:
        result=subprocess.run([executable,'--query-gpu=name,driver_version,memory.total,compute_cap','--format=csv,noheader,nounits'],capture_output=True,text=True,timeout=15)
        report['gpu_query_error']=result.stderr.strip()
        for row in result.stdout.splitlines():
            fields=[x.strip() for x in row.split(',')]
            if len(fields)!=4:continue
            try:
                driver=tuple(int(x) for x in fields[1].split('.'))
                memory=int(float(fields[2]));cap=float(fields[3])
            except ValueError:continue
            sim_driver=(595,97) if host=='Windows' else (595,58,3)
            report['gpu'].append({'name':fields[0],'driver':fields[1],'memory_mib':memory,'compute_capability':cap,
                'ros_gpu_requirements_met':driver[0]>=595 and memory>=8192 and cap>=8.,
                'sim_memory_driver_requirements_met':memory>=16384 and driver>=sim_driver})
    report['ros_ready']=bool(host=='Linux' and report['ubuntu_24_04'] and report['ros_distro']=='lyrical' and any(g['ros_gpu_requirements_met'] for g in report['gpu']))
    report['sim_requirements_met']=any(g['sim_memory_driver_requirements_met'] for g in report['gpu'])
    report['passed']=report['ros_ready'] and report['sim_requirements_met']
    report['scope']='Host checks only; CUDA toolkit, RT cores, RAM, Isaac packages, ROS DDS and runtime are not certified by this inventory.'
    report['memory_note']='Reported usable VRAM may be below nominal capacity. Below-threshold results require the official compatibility checker; low-memory scenes may run but are unverified.'
    report['sources']=['https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html','https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/requirements.html']
    return report

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path);args=parser.parse_args()
    report=inventory();content=json.dumps(report,indent=2)+'\n'
    if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(content)
    print(content,end='');raise SystemExit(0 if report['passed'] else 2)
