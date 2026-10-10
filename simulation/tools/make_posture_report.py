"""Render actual ROS/physics traces and a compact replayable video."""
from pathlib import Path
import sys
import json
import subprocess

ROOT=Path(__file__).resolve().parents[2]
if sys.platform=='win32':
    sys.path.insert(0,str(ROOT/'simulation/generated/python_deps'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

folder=ROOT/'simulation/generated/posture'
# Add a separately named final displacement from recorded physics traces.
# The original reward uses the commanded walking window; retain that metric.
for trace in folder.glob('*.csv'):
    report_path=trace.with_suffix('.json')
    if not report_path.exists():
        continue
    summary=json.loads(report_path.read_text())
    recorded=np.genfromtxt(trace,delimiter=',',names=True,dtype=None,encoding='utf-8')
    net=float(recorded['base_x_m'][-1]-recorded['base_x_m'][0])
    summary['net_distance_after_stop_m']=net
    target=summary.get('walk_target_m')
    summary['net_distance_error_fraction']=abs(net-target)/max(target,.01) if target is not None else None
    summary['net_distance_source']='Final minus initial base_x from recorded CSV'
    report_path.write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True,constrained_layout=True)
for name,label,color in [('startup_baseline','Before learning','#b44639'),
                          ('learned_validation','After learning / direct physics','#168a82'),
                          ('ros_learned_validation','After learning / real ROS 2','#315dcc')]:
    d=np.genfromtxt(folder/(name+'.csv'),delimiter=',',names=True,dtype=None,encoding='utf-8')
    axes[0].plot(d['time_s'],d['base_z_m'],label=label,color=color)
    axes[1].plot(d['time_s'],d['base_x_m']-d['base_x_m'][0],color=color)
    tilt=np.maximum(np.abs(d['roll_rad']),np.abs(d['pitch_rad']))*180/np.pi
    axes[2].plot(d['time_s'],tilt,color=color)
axes[0].set_ylabel('Body height (m)')
axes[1].set_ylabel('Forward position (m)')
axes[2].set_ylabel('Max |roll|, |pitch| (degrees)')
axes[2].set_xlabel('Simulation time (s)')
axes[0].legend(loc='upper left',fontsize=9)
for ax in axes:
    ax.grid(alpha=.25)
    ax.axvspan(1,5,color='#999999',alpha=.10)
    ax.axvspan(8,17,color='#999999',alpha=.06)
fig.suptitle('BigSpot / crouch - rise - balance - crawl - command loss - stop\nSame nominal model, 6 Nm assumed motor cap; training learns controller parameters')
fig.savefig(folder/'posture_validation.png',dpi=130)
plt.close(fig)

import imageio_ffmpeg
ffmpeg=imageio_ffmpeg.get_ffmpeg_exe()
subprocess.run([ffmpeg,'-y','-i',str(folder/'ros_learned_validation.gif'),'-an','-c:v','libx264',
                '-pix_fmt','yuv420p','-movflags','+faststart',str(folder/'ros_learned_validation.mp4')],
               check=True,capture_output=True)
print('Saved posture_validation.png and ros_learned_validation.mp4')
