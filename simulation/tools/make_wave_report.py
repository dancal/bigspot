"""Compare recorded foot/contact traces; export the actual new ROS recording."""
from pathlib import Path
import sys
import subprocess

ROOT=Path(__file__).resolve().parents[2]
if sys.platform=='win32':
    sys.path.insert(0,str(ROOT/'simulation/generated/python_deps'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import imageio_ffmpeg

folder=ROOT/'simulation/generated/posture'
before=np.genfromtxt(folder/'gait_before_24s_feet.csv',delimiter=',',names=True,dtype=None,encoding='utf-8')
after=np.genfromtxt(folder/'ros_wave_validation_feet.csv',delimiter=',',names=True,dtype=None,encoding='utf-8')
fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True,constrained_layout=True)
for i,(leg,color) in enumerate(zip(['FR','FL','RR','RL'],['#c4692c','#298ec1','#b69c17','#39924c'])):
    ax=axes.flat[i]
    ax.plot(before['time_s'],before[leg+'_z_m']*1000,color='#999999',label='Earlier crawl')
    ax.plot(after['time_s'],after[leg+'_z_m']*1000,color=color,label='New ROS wave crawl')
    ax.set_title(leg+' / measured sole center height')
    ax.set_ylabel('Height (mm)')
    ax.set_ylim(-1,32)
    if i==0:
        ax.legend(fontsize=9)
for case,label,color in [('gait_before_24s','Earlier crawl','#999999'),('ros_wave_validation','New ROS wave crawl','#265da4')]:
    body=np.genfromtxt(folder/(case+'.csv'),delimiter=',',names=True,dtype=None,encoding='utf-8')
    foot=before if case=='gait_before_24s' else after
    axes[2,0].plot(body['time_s'][1:],np.diff(body['base_x_m'])/np.diff(body['time_s']),color=color,label=label)
    count=sum(foot[leg+'_contact'] for leg in ['FR','FL','RR','RL'])
    axes[2,1].plot(foot['time_s'],count,color=color,alpha=.8,label=label)
axes[2,0].set_title('Body forward speed / target 0.01 m/s')
axes[2,0].set_ylabel('m/s')
axes[2,0].set_ylim(-.25,.25)
axes[2,1].set_title('Measured ground contacts / three desired during swing')
axes[2,1].set_ylabel('Feet with collision contact')
for ax in axes.flat:
    ax.set_xlim(8,24)
    ax.grid(alpha=.2)
axes[2,0].set_xlabel('Time (s)')
axes[2,1].set_xlabel('Time (s)')
fig.suptitle('BigSpot / four-foot gait review and correction\nActual recorded MuJoCo physics; new run exchanges real ROS 2 messages; motor cap 6 Nm assumed')
fig.savefig(folder/'wave_gait_validation.png',dpi=130)
plt.close(fig)

ffmpeg=imageio_ffmpeg.get_ffmpeg_exe()
subprocess.run([ffmpeg,'-y','-i',str(folder/'ros_wave_validation.gif'),'-an','-c:v','libx264',
                '-pix_fmt','yuv420p','-movflags','+faststart',str(folder/'ros_wave_validation.mp4')],check=True,capture_output=True)
subprocess.run([ffmpeg,'-y','-ss','8','-i',str(folder/'ros_wave_validation.mp4'),'-t','12',
                '-an','-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',
                str(folder/'ros_wave_walk.mp4')],check=True,capture_output=True)
from PIL import Image,ImageSequence
recording=Image.open(folder/'ros_wave_validation.gif')
walk_frames=[]
walk_durations=[]
elapsed=0
for frame in ImageSequence.Iterator(recording):
    duration=frame.info.get('duration',50)
    if 8000 <= elapsed < 20000:
        walk_frames.append(frame.convert('RGB').copy())
        walk_durations.append(duration)
    elapsed+=duration
walk_frames[0].save(folder/'ros_wave_walk.gif',save_all=True,append_images=walk_frames[1:],duration=walk_durations,loop=0)
print('Saved wave_gait_validation.png, ros_wave_validation.mp4 and ros_wave_walk.mp4')
