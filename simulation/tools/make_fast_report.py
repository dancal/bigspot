"""Compare measured slow/fast ROS motion; export normal-time fast replay."""
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
import imageio_ffmpeg
from PIL import Image,ImageSequence

folder=ROOT/'simulation/generated/posture'
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True,constrained_layout=True)
speeds={}
for case,label,color in [('ros_wave_validation','Slow wave crawl','#888888'),
                          ('ros_fast_validation','Fast diagonal trot','#286bbe')]:
    d=np.genfromtxt(folder/(case+'.csv'),delimiter=',',names=True,dtype=None,encoding='utf-8')
    walking=d[(d['time_s']>=9)&(d['time_s']<21)]
    speeds[case]=float(np.mean(np.diff(walking['base_x_m'])/np.diff(walking['time_s'])))
    axes[0].plot(d['time_s'],d['base_x_m']-d['base_x_m'][0],color=color,label=label)
    velocity=np.diff(d['base_x_m'])/np.diff(d['time_s'])
    axes[1].plot(d['time_s'][1:],velocity,color=color,alpha=.18)
    smoothed=np.convolve(velocity,np.ones(13)/13,mode='valid')
    axes[1].plot(d['time_s'][7:-6],smoothed,color=color)
    tilt=np.maximum(np.abs(d['roll_rad']),np.abs(d['pitch_rad']))*180/np.pi
    axes[2].plot(d['time_s'],tilt,color=color)
axes[0].set_ylabel('Forward distance (m)')
axes[0].legend()
axes[1].set_ylabel('Speed (m/s)')
axes[1].axhline(.1,color='#286bbe',ls='--',lw=1,label='Fast target 0.10 m/s')
axes[1].set_title('Raw velocity faint; moving average over 0.21 s solid')
axes[2].set_ylabel('Max |roll|, |pitch| (degrees)')
axes[2].set_xlabel('Simulation time (s)')
for ax in axes:
    ax.grid(alpha=.25)
    ax.axvspan(8,21,color='#cccccc',alpha=.15)
fig.suptitle('BigSpot / actual ROS 2 slow and fast gait comparison\nSame 6 Nm assumed motor cap; gravity and ground contact; normal-time replay')
fig.savefig(folder/'fast_speed_validation.png',dpi=130)
plt.close(fig)
comparison={'window_s':[9,21],'slow_mean_m_s':speeds['ros_wave_validation'],
            'fast_mean_m_s':speeds['ros_fast_validation'],
            'measured_ratio':speeds['ros_fast_validation']/speeds['ros_wave_validation'],
            'source':'Mean consecutive base_x differences divided by actual CSV time differences'}
(folder/'fast_speed_comparison.json').write_text(json.dumps(comparison,indent=2)+'\n',encoding='utf-8')

ffmpeg=imageio_ffmpeg.get_ffmpeg_exe()
subprocess.run([ffmpeg,'-y','-i',str(folder/'ros_fast_validation.gif'),'-an','-c:v','libx264',
                '-pix_fmt','yuv420p','-movflags','+faststart',str(folder/'ros_fast_validation.mp4')],check=True,capture_output=True)
subprocess.run([ffmpeg,'-y','-ss','8','-i',str(folder/'ros_fast_validation.mp4'),'-t','12',
                '-an','-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',
                str(folder/'ros_fast_walk.mp4')],check=True,capture_output=True)
recording=Image.open(folder/'ros_fast_validation.gif')
frames=[]
durations=[]
elapsed=0
for frame in ImageSequence.Iterator(recording):
    duration=frame.info.get('duration',50)
    if 8000<=elapsed<20000:
        frames.append(frame.convert('RGB').copy())
        durations.append(duration)
    elapsed+=duration
frames[0].save(folder/'ros_fast_walk.gif',save_all=True,append_images=frames[1:],duration=durations,loop=0)
print(json.dumps(comparison,indent=2))
print(f'Recording duration {elapsed/1000:.1f} s; exported normal-time videos and GIF')
