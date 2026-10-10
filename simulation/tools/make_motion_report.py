"""Export measured ROS motion comparisons and normal-time replay clips."""
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

folder=ROOT/'simulation/generated/motion'
normal=['forward','backward','left','right','turn_left_90','turn_right_90','run','accelerate','buttons','bow','crouch']
experiments=['jump_left','jump_right','jump_back','rear_stand_experimental','handstand_experimental']
reports={s:json.loads((folder/('ros_'+s+'.json')).read_text()) for s in normal+experiments}
for s in normal:
    r=reports[s]
    if not r['ros_executed'] or r['fallen'] or r['fault'] or r['end_state']!='READY':
        raise RuntimeError('Normal motion validation did not pass: '+s)
for s in ['turn_left_90','turn_right_90']:
    if abs(abs(reports[s]['yaw_change_deg'])-90)>2:
        raise RuntimeError('Relative turn angle outside two-degree tolerance: '+s)
    if 'Completed relative' not in reports[s]['action_result']:
        raise RuntimeError('Relative turn did not complete: '+s)
buttons=reports['buttons']
if buttons['observed_action_messages'].count('run')!=1 or buttons['observed_action_messages'].count('stop')!=1:
    raise RuntimeError('Joystick edge events were not delivered exactly once')

def read(s):
    return np.genfromtxt(folder/('ros_'+s+'.csv'),delimiter=',',names=True,dtype=None,encoding='utf-8')

fig,axes=plt.subplots(3,2,figsize=(12,10),constrained_layout=True)
for s in ['forward','backward','left','right','run']:
    a=read(s);a=a[a['time_s']>=8]
    axes[0,0].plot(a['x_m']-a['x_m'][0],a['y_m']-a['y_m'][0],label=s)
axes[0,0].set(xlabel='World x displacement (m)',ylabel='World y displacement (m)',title='Measured base motion; lateral drift visible')
axes[0,0].axis('equal');axes[0,0].legend()
a=read('buttons')
v=np.diff(a['x_m'])/np.diff(a['time_s'])
axes[0,1].plot(a['time_s'][1:],v,alpha=.15)
axes[0,1].plot(a['time_s'][7:-6],np.convolve(v,np.ones(13)/13,'valid'),label='Measured speed')
axes[0,1].plot(a['time_s'],a['cmd_x'],ls='--',label='Twist received by controller')
for start,stop,label in [(8,10,'Move'),(10,12,'Boost'),(12,14,'Run'),(14,18,'Stop')]:
    axes[0,1].axvspan(start,stop,alpha=.08)
    axes[0,1].text((start+stop)/2,.22,label,ha='center')
axes[0,1].set(xlabel='Time (s)',ylabel='Forward speed (m/s)',title='Actual ROS Joy messages -> Twist / action')
axes[0,1].legend()
for s in ['turn_left_90','turn_right_90']:
    a=read(s);a=a[a['time_s']>=8]
    yaw=(np.unwrap(a['yaw_rad'])-a['yaw_rad'][0])*180/np.pi
    axes[1,0].plot(a['time_s'],yaw,label=s)
axes[1,0].axhline(90,ls='--',color='grey');axes[1,0].axhline(-90,ls='--',color='grey')
axes[1,0].set(xlabel='Time (s)',ylabel='Relative yaw (deg)',title='Closed-loop 90-degree turns')
axes[1,0].legend()
for s in ['jump_left','jump_right','jump_back']:
    a=read(s)
    sole=np.min(np.column_stack([a[name+'_sole_z_m'] for name in ['FR','FL','RR','RL']]),axis=1)
    axes[1,1].plot(a['time_s'],sole*1000,label=s)
axes[1,1].axhline(10,ls='--',color='grey',label='Minimum clearance criterion')
axes[1,1].set(xlim=(8.5,12),xlabel='Time (s)',ylabel='Lowest of four soles (mm)',title='Jump experiments: success criteria NOT met')
axes[1,1].legend()
for s in ['rear_stand_experimental','handstand_experimental']:
    a=read(s)
    axes[2,0].plot(a['time_s'],a['pitch_rad']*180/np.pi,label=s)
axes[2,0].axhline(65,ls='--',color='grey');axes[2,0].axhline(-65,ls='--',color='grey')
axes[2,0].set(xlabel='Time (s)',ylabel='Measured pitch (deg)',title='Vertical acrobatics: rejected / unsuccessful')
axes[2,0].legend()
for i,name in enumerate(['FR','FL','RR','RL']):
    a=read('jump_left')
    axes[2,1].plot(a['time_s'],a[name+'_force_N'],label=name)
axes[2,1].set(xlim=(8.5,12),xlabel='Time (s)',ylabel='Normal contact force (N)',title='Jump-left landing feedback from physics')
axes[2,1].legend()
for ax in axes.ravel():
    ax.grid(alpha=.25)
fig.suptitle('BigSpot / RDS5160 x12 / nominal 7.4 V, 30 A limit\nActual ROS 2 Humble + MuJoCo CPU; motor curve / mass / collision assumptions unmeasured')
fig.savefig(folder/'motion_validation.png',dpi=125)
plt.close(fig)

ffmpeg=imageio_ffmpeg.get_ffmpeg_exe()
def ff(args):
    subprocess.run([ffmpeg,'-y',*map(str,args)],check=True,capture_output=True)

clips=[]
for s,start,seconds in [('backward',8,4),('left',8,4),('right',8,4),
                       ('turn_left_90',8.5,7),('turn_right_90',8.5,7),('buttons',8,7),
                       ('bow',8.5,4),('crouch',8.5,4)]:
    clip=folder/('clip_'+s+'.mp4')
    ff(['-ss',start,'-i',folder/('ros_'+s+'.gif'),'-t',seconds,'-an','-r','20',
        '-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',clip])
    clips.append(clip)
concat=folder/'motion_clips.txt'
concat.write_text(''.join("file '"+p.as_posix()+"'\n" for p in clips),encoding='utf-8')
ff(['-f','concat','-safe','0','-i',concat,'-c','copy','-movflags','+faststart',folder/'motion_demo.mp4'])
failed=[]
for s in ['jump_left','jump_right','jump_back','handstand_experimental']:
    clip=folder/('clip_'+s+'.mp4')
    ff(['-ss','8.5','-i',folder/('ros_'+s+'.gif'),'-t','6' if not s.endswith('_experimental') else '16',
        '-an','-r','20','-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',clip])
    failed.append(clip)
concat.write_text(''.join("file '"+p.as_posix()+"'\n" for p in failed),encoding='utf-8')
ff(['-f','concat','-safe','0','-i',concat,'-c','copy','-movflags','+faststart',folder/'experimental_attempts.mp4'])

# Compact preview samples actual replay at 10fps, preserving the time of each clip.
preview=[]
for s,start,end in [('left',8,11),('right',8,11),('backward',8,11),('buttons',8,15)]:
    recording=Image.open(folder/('ros_'+s+'.gif'))
    elapsed=0.
    sample_times=np.arange(round(start*10),round(end*10))/10.
    sample_index=0
    for frame in ImageSequence.Iterator(recording):
        duration=frame.info.get('duration',50)/1000.
        while sample_index<len(sample_times) and elapsed-1e-9<=sample_times[sample_index]<elapsed+duration-1e-9:
            preview.append(frame.convert('RGB').copy())
            sample_index+=1
        elapsed+=duration
preview[0].save(folder/'motion_preview.gif',save_all=True,append_images=preview[1:],duration=100,loop=0)
preview[len(preview)//2].save(folder/'motion_preview.png')
summary={'runtime':'Actual ROS 2 Humble / MuJoCo 3.15 CPU','servo':'RDS5160 x12',
         'user_confirmed_supply':{'nominal_voltage_v':7.4,'maximum_current_a':30},
         'normal_motion_cases':normal,'normal_motion_passed':True,
         'experimental_cases':experiments,'experimental_success':False,
         'regression_tests_passed':30,'reports':reports,
         'motor_torque_speed_and_current_model_measured':False,
         'imu_orientation_ideal_in_simulation':True,'physical_joystick_tested':False,
         'physical_robot_tested':False,'isaac_ros_5_executed':False,
         'video_realtime':True,'preview_duration_s':len(preview)*.1}
(folder/'motion_validation.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
print(json.dumps({s:{k:reports[s].get(k) for k in ['mean_velocity_m_s','yaw_change_deg','max_tilt_deg',
       'jump_success','acrobatics_success','action_result']} for s in normal+experiments},indent=2))
print('Normal-time replay: motion_demo.mp4; unsuccessful experiments: experimental_attempts.mp4')
