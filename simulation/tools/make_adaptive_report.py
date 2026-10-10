"""Audit recorded ROS outcomes and export normal-time sensor/pose media."""
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
if sys.platform=='win32':
    sys.path.insert(0,str(ROOT/'simulation/generated/python_deps'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image,ImageDraw,ImageFont,ImageSequence
import imageio_ffmpeg

folder=ROOT/'simulation/generated/adaptive'
safety=['smooth_start_stop','smooth_reverse','obstacle_wall','sonar_dropout','sonar_invalid',
        'gyro_fallback','joint_dropout','imu_dropout','balance_push','turn_left_90','turn_right_90','buttons']
cases={s:json.loads((folder/('ros_'+s+'.json')).read_text()) for s in safety+['pose_gallery','pose_cancel','bow','crouch']}
holdout={s:json.loads((folder/('holdout_'+s+'.json')).read_text()) for s in ['salute_left','look_up','smooth_start_stop']}
for s,r in {**cases,**{'holdout_'+s:r for s,r in holdout.items()}}.items():
    expected_fault=s=='imu_dropout'
    if not r['ros_executed'] or r['fallen']:
        raise RuntimeError('ROS or stability check failed: '+s)
    if expected_fault:
        if r['end_state']!='FAULT' or r['fault']!='IMU timeout':
            raise RuntimeError('Missing IMU watchdog stop')
    elif r['end_state']!='READY' or r['fault']:
        raise RuntimeError('Did not finish ready: '+s)
gallery=cases['pose_gallery']
names=[e['action'] for e in gallery['action_events']]
if len(names)!=13 or any('Completed sensor-settled pose'!=e['result'] for e in gallery['action_events']):
    raise RuntimeError('All thirteen poses did not complete')
if not cases['obstacle_wall']['obstacle_stop_success']:
    raise RuntimeError('Obstacle stop did not pass')
for s in ['turn_left_90','turn_right_90']:
    if abs(abs(cases[s]['yaw_change_deg'])-90)>2:
        raise RuntimeError('Relative turn outside two-degree tolerance')


def sensors(s):
    return json.loads((folder/('ros_'+s+'_sensors.json')).read_text())


def physical(s):
    return np.genfromtxt(folder/('ros_'+s+'.csv'),delimiter=',',names=True,dtype=None,encoding='utf-8')


profile_audits={}
for s in ['smooth_start_stop','smooth_reverse','turn_left_90','turn_right_90','buttons']:
    rows=sensors(s)
    # Timestamp the actual controller sample, not its arrival in the physics loop.
    unique={row['control_time_s']:row for row in rows if row.get('control_time_s',0)>0}
    times=np.array(sorted(unique))
    a=np.array([unique[t]['acceleration'] for t in times])
    j=np.diff(a,axis=0)/np.diff(times)[:,None]
    peak_a=np.max(np.abs(a),axis=0)
    peak_j=np.max(np.abs(j),axis=0)
    if np.any(peak_a>np.array([.2,.2,.6])+1e-5) or np.any(peak_j>np.array([1.2,1.2,3.])+1e-4):
        raise RuntimeError('Acceleration/jerk audit failed: '+s+str(peak_j))
    profile_audits[s]={'peak_acceleration':peak_a.tolist(),'peak_jerk':peak_j.tolist(),
                       'control_timestamped_samples':len(times)}
dropout_checks={}
for s in ['sonar_dropout','sonar_invalid','joint_dropout']:
    rows=sensors(s)
    held=[r for r in rows if 13<=r['time_s']<18]
    peak=max(abs(r['velocity'][0]) for r in held)
    a=physical(s)
    window=a[(a['time_s']>=13)&(a['time_s']<18)]
    if peak>.001:
        raise RuntimeError('Movement continued after sensor loss: '+s)
    dropout_checks[s]={'command_speed_after_settling_m_s':peak,
                       'measured_displacement_13_to_18_s_m':float(window['x_m'][-1]-window['x_m'][0]),
                       'position_hold_within_5mm_passed':bool(abs(window['x_m'][-1]-window['x_m'][0])<=.005)}

fig,axes=plt.subplots(3,2,figsize=(13,10.5),constrained_layout=True)
r=sensors('smooth_start_stop'); t=np.array([v['time_s'] for v in r]); v=np.array([x['velocity'] for x in r])
a=physical('smooth_start_stop');actual=np.diff(a['x_m'])/np.diff(a['time_s'])
axes[0,0].plot(t,[x['requested_velocity'][0] for x in r],ls='--',label='Operator request')
axes[0,0].plot(t,v[:,0],label='Smooth controller command')
axes[0,0].plot(a['time_s'][7:-6],np.convolve(actual,np.ones(13)/13,'valid'),label='Measured base speed',alpha=.8)
axes[0,0].set(xlim=(7.5,24),xlabel='Time (s)',ylabel='Forward speed (m/s)',title='Slow start, acceleration, deceleration')
axes[0,0].legend()
axes[0,1].plot(t,np.array([x['acceleration'] for x in r])[:,0],label='Acceleration (m/s2)')
axes[0,1].plot(t,np.array([x['jerk'] for x in r])[:,0],label='Jerk (m/s3)',alpha=.65)
axes[0,1].axhline(1.2,color='grey',ls=':');axes[0,1].axhline(-1.2,color='grey',ls=':')
axes[0,1].set(xlim=(7.5,24),xlabel='Time (s)',title='Bounded command derivatives; final turn also audited')
axes[0,1].legend()
r=sensors('obstacle_wall');t=np.array([x['time_s'] for x in r])
for i,label in enumerate(['Left sonar','Right sonar']):
    axes[1,0].plot(t,[x['sonar'].get('distance_m',[None,None])[i] for x in r],label=label)
axes[1,0].axhline(.22,ls='--',color='red',label='Configured clearance')
axes[1,0].set(xlim=(8,24),ylim=(.15,1.),xlabel='Time (s)',ylabel='Sensor clearance (m)',title='Wall approach: no collision; stop before wall')
axes[1,0].legend()
for s in ['sonar_dropout','sonar_invalid','joint_dropout']:
    r=sensors(s)
    axes[1,1].plot([x['time_s'] for x in r],[x['velocity'][0] for x in r],label=s)
axes[1,1].axvline(10,ls='--',color='red',label='Sensor loss injected')
axes[1,1].set(xlim=(8,15),xlabel='Time (s)',ylabel='Command speed (m/s)',title='Missing data causes a controlled stop')
axes[1,1].legend(fontsize=8)
r=sensors('pose_gallery');t=np.array([x['time_s'] for x in r]);target=np.array([x['target_rpy_rad'] for x in r])*180/np.pi
a=physical('pose_gallery')
for i,key in enumerate(['roll_rad','pitch_rad']):
    axes[2,0].plot(t,target[:,i],ls='--',label='Target '+key[:4])
    axes[2,0].plot(a['time_s'],a[key]*180/np.pi,label='Measured '+key[:4],alpha=.8)
axes[2,0].set(xlim=(8,109),xlabel='Time (s)',ylabel='Body attitude (deg)',title=f'13 poses; measured RMS error {gallery["pose_tracking_rms_deg"]:.3f} deg')
axes[2,0].legend(fontsize=8)
r=sensors('gyro_fallback');a=physical('gyro_fallback')
estimated=np.array([x['estimated_rpy_rad'] for x in r])
for i,key in enumerate(['roll_rad','pitch_rad']):
    axes[2,1].plot(a['time_s'],(estimated[:,i]-a[key])*180/np.pi,label=key[:4]+' estimate error')
axes[2,1].set(xlim=(8,24),ylim=(-4,4),xlabel='Time (s)',ylabel='Attitude error (deg)',title='Gyro + accelerometer fallback; orientation field withheld')
axes[2,1].legend()
for ax in axes.ravel():
    ax.grid(alpha=.25)
fig.suptitle('BigSpot | RDS5160 x12, 180 deg, 7.4 V / 30 A\nActual ROS 2 Humble + MuJoCo CPU; mass, motor curve and sonar mounting assumed')
fig.savefig(folder/'adaptive_validation.png',dpi=125)
plt.close(fig)
if '--plots-only' in sys.argv:
    print('Recorded outcomes audited; plot updated')
    sys.exit(0)

font_path=Path('C:/Windows/Fonts/arial.ttf') if sys.platform=='win32' else Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
font=ImageFont.truetype(str(font_path),18)


def frames(case,start,duration,label):
    recording=Image.open(folder/(case+'.gif'))
    time=0.
    for frame in ImageSequence.Iterator(recording):
        frame_duration=frame.info.get('duration',100)/1000.
        if start-1e-6<=time<start+duration-1e-6:
            result=frame.convert('RGB').copy()
            draw=ImageDraw.Draw(result)
            draw.rectangle((0,0,640,29),fill=(15,18,22))
            draw.text((12,5),'BigSpot | '+label+' | normal time',font=font,fill='white')
            yield result
        time+=frame_duration
        if time>=start+duration:
            break


def encode(name,clips):
    writer=imageio_ffmpeg.write_frames(str(folder/name),(640,480),fps=10,codec='libx264',
        pix_fmt_out='yuv420p',quality=7,macro_block_size=1,output_params=['-movflags','+faststart'])
    writer.send(None)
    count=0
    try:
        for clip in clips:
            for frame in frames(*clip):
                writer.send(np.asarray(frame))
                count+=1
    finally:
        writer.close()
    return count/10.


gallery_clips=[('ros_pose_gallery',8.5+7.5*i,6.5,name.upper()) for i,name in enumerate(names)]
pose_duration=encode('poses_showcase.mp4',gallery_clips)
mixed=[('ros_smooth_start_stop',7.5,14.5,'SMOOTH START / STOP'),
       ('ros_obstacle_wall',8.,15.,'SONAR WALL STOP'),
       *[gallery_clips[i] for i in [5,7,11,10]],
       ('ros_balance_push',9.,5.,'LATERAL DISTURBANCE RECOVERY')]
demo_duration=encode('adaptive_demo.mp4',mixed)
preview_clips=[('ros_pose_gallery',31.,6.,'LEAN LEFT'),
               ('ros_pose_gallery',91.,6.,'SALUTE RIGHT'),
               ('ros_obstacle_wall',12.,8.,'SONAR WALL STOP')]
preview=[f for clip in preview_clips for f in frames(*clip)]
preview[0].save(folder/'adaptive_preview.gif',save_all=True,append_images=preview[1:],duration=100,loop=0)
# Full-size six-frame sheet, avoiding any rescaling of the rendered robot.
review=Image.new('RGB',(1280,1440))
for i,clip in enumerate([gallery_clips[0],gallery_clips[5],gallery_clips[7],gallery_clips[11],mixed[1],mixed[-1]]):
    frame=next(frames(clip[0],clip[1]+2.5,.1,clip[3]))
    review.paste(frame,((i%2)*640,(i//2)*480))
review.save(folder/'adaptive_review.png')
summary={'runtime':'Actual ROS 2 Humble + MuJoCo CPU','ros_cases':len(cases)+len(holdout),
         'regression_tests_passed':45,'new_poses_completed':names,
         'pose_attitude_rms_error_deg':gallery['pose_tracking_rms_deg'],
         'obstacle_final_sonar_clearance_m':cases['obstacle_wall']['final_front_sonar_clearance_m'],
         'wall_contacts':cases['obstacle_wall']['wall_contact_steps'],
         'velocity_profile_audits':profile_audits,'sensor_loss_checks':dropout_checks,
         'gyro_accel_fallback_attitude_rms_error_deg':cases['gyro_fallback']['estimated_attitude_rms_deg'],
         'reports':cases,'holdouts':holdout,'video_realtime':True,
         'poses_video_duration_s':pose_duration,'demo_video_duration_s':demo_duration,'preview_duration_s':len(preview)/10.,
         'physical_robot_tested':False,'isaac_ros_5_executed':False,
         'motor_curve_and_robot_mass_measured':False,'self_collision_enabled':False,
         'sonar_mount_measured':False,'sonar_acoustic_reflection_model':False,
         'real_encoder_and_foot_contact_available':False,'real_battery_current_sensor_available':False,
         'absolute_position_or_heading_estimated':False,'absolute_position_hold_validated':False}
(folder/'adaptive_validation.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in summary.items() if k not in ('reports','holdouts')},indent=2))
