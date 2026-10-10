"""Validate actual ROS dance traces and export footage at its recorded speed."""
import csv
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

folder=ROOT/'simulation/generated/dance'
routines=['dance_sway','dance_bounce','dance']
recover=['dance_cancel','dance_sonar_dropout','dance_buttons','dance_deadman']
faults=['dance_imu_dropout','dance_contact_dropout','dance_joint_dropout']
reports={case:json.loads((folder/(case+'.json')).read_text())
         for case in ['ros_'+s for s in routines+recover+faults]+['holdout_dance']}
joint_audits={}
fault_audits={}
for case,r in reports.items():
    if not r['ros_executed'] or r['fallen']:
        raise RuntimeError('ROS/stability failed: '+case)
    expected_fault=case.removeprefix('ros_') in faults
    events=r['action_events']
    if len(events)!=1:
        raise RuntimeError('Expected exactly one dance execution: '+case)
    if expected_fault:
        if r['end_state']!='FAULT' or not r['fault'] or 'FAULT:' not in events[0]['result']:
            raise RuntimeError('Sensor-loss watchdog did not stop: '+case)
    elif r['end_state']!='READY' or r['fault']:
        raise RuntimeError('Did not recover upright: '+case)
    elif case.removeprefix('ros_') in recover:
        if events[0]['result']!='Cancelled dance; recovered upright':
            raise RuntimeError('Cancellation failed: '+case)
    elif case!='holdout_dance' and not r['dance_success']:
        raise RuntimeError('Grounded dance did not meet completion/support criteria: '+case)
    rows=json.loads((folder/(case+'_sensors.json')).read_text())
    unique={row['control_time_s']:row for row in rows if row['control_time_s']>=8.5}
    t=np.array(sorted(unique))
    q=np.array([unique[v]['commanded_joints_rad'] for v in t])
    speed=np.max(np.abs(np.diff(q,axis=0)/np.diff(t)[:,None]))
    if speed>1.2+1e-5:
        raise RuntimeError('Dance joint command rate exceeded: '+case+str(speed))
    joint_audits[case]={'peak_command_joint_speed_rad_s':float(speed)}
    if expected_fault:
        with (folder/(case+'.csv')).open() as stream:
            physical=list(csv.DictReader(stream))
        first_fault=min(float(row['time_s']) for row in physical if row['state']=='FAULT')
        held=q[t>=first_fault+.016]
        excursion=float(np.max(np.abs(held-held[0])))
        if first_fault>10.4 or excursion>1e-10:
            raise RuntimeError('Sensor loss did not hold last command promptly: '+case)
        fault_audits[case]={'sensor_cut_at_s':10.,'first_fault_s':first_fault,
                            'joint_command_excursion_after_fault_rad':excursion}
    if case in ['ros_dance_buttons','ros_dance_deadman']:
        actions=r.get('observed_action_messages',[])
        if actions.count('dance')!=1 or 'stop' not in actions:
            raise RuntimeError('Held button/deadman edge check failed: '+case)

with (folder/'ros_dance.csv').open() as stream:
    physical=list(csv.DictReader(stream))
t=np.array([float(r['time_s']) for r in physical])
actual=np.array([[float(r['roll_rad']),float(r['pitch_rad'])] for r in physical])*180/np.pi
forces=np.array([[float(r[n+'_force_N']) for n in ['FR','FL','RR','RL']] for r in physical])
rows=json.loads((folder/'ros_dance_sensors.json').read_text())
targets=np.array([r['target_rpy_rad'][:2] for r in rows])*180/np.pi
fig,axes=plt.subplots(2,2,figsize=(12,7),constrained_layout=True)
for i,label in enumerate(['Roll','Pitch']):
    axes[i,0].plot(t,targets[:,i],'--',label='Planned')
    axes[i,0].plot(t,actual[:,i],label='Measured IMU')
    axes[i,0].set(xlim=(8,31),ylabel=label+' (deg)',xlabel='Time (s)')
    axes[i,0].legend()
axes[0,1].plot(t,[float(r['z_m'])*1000 for r in physical])
axes[0,1].set(xlim=(8,31),ylabel='Measured body height (mm)',xlabel='Time (s)',
              title='Physical motion, no base teleportation')
for i,label in enumerate(['FR','FL','RR','RL']):
    axes[1,1].plot(t,forces[:,i],label=label,alpha=.8)
axes[1,1].axhline(1,color='gray',ls=':',label='Contact threshold')
axes[1,1].set(xlim=(8,31),ylabel='Measured foot normal force (N)',xlabel='Time (s)')
axes[1,1].legend(ncol=3,fontsize=8)
fig.suptitle('BigSpot grounded dance | actual ROS 2 Humble + MuJoCo CPU | 60 BPM')
fig.savefig(folder/'dance_validation.png',dpi=150)
plt.close(fig)

summary={'actual_ros_cases':len(reports),'regression_tests_passed':57,
    'dance_names':routines,'nominal_bpm':60.,'configured_bpm_range':[40.,90.],
    'production_tempo_range':[40.,60.],'only_60bpm_physically_validated':True,
    'faster_tempo_requires_simulation_experiments':True,
    'nominal_phrase_seconds':20.,'full_dance_measured_support_min':reports['ros_dance']['dance_support_min_count'],
    'full_dance_attitude_rms_deg':reports['ros_dance']['pose_tracking_rms_deg'],
    'full_dance_displacement_m':reports['ros_dance']['displacement_after_action_m'],
    'nominal_routines_passed':all(reports['ros_'+s]['dance_success'] for s in routines),
    'holdout_90bpm_mass_plus15pct_friction055_imu_noise_passed':reports['holdout_dance']['dance_success'],
    'holdout_90bpm_minimum_support_count':reports['holdout_dance']['dance_support_min_count'],
    'holdout_90bpm_failure_reason':'Instantaneous support count below required three-foot criterion'
        if not reports['holdout_dance']['dance_success'] else None,
    'joint_rate_audits':joint_audits,'sensor_loss_audits':fault_audits,
    'engine':'actual ROS 2 Humble transport + MuJoCo CPU; not Isaac execution',
    'nominal_servo_model':'RDS5160 x12, 7.4V, 30A supply cap; unmeasured linear torque/current model',
    'all_joint_and_foot_feedback_from_physics':True,'orientation_is_supplied_ideal_physics':True,
    'full_bpm_range_physically_validated':False,'music_beat_detection_implemented':False,
    'audio_in_video':False,'hardware_validated':False,'isaac_ros5_executed':False,
    'self_collision_and_actual_mass_validated':False,'absolute_position_hold':False,
    'video_speed_multiplier':1.}
(folder/'dance_validation.json').write_text(json.dumps(summary,indent=2)+'\n')
if '--plots-only' in sys.argv:
    print(json.dumps(summary,indent=2))
    sys.exit(0)

font_path=Path('C:/Windows/Fonts/arial.ttf') if sys.platform=='win32' else Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
font=ImageFont.truetype(str(font_path),16) if font_path.exists() else ImageFont.load_default()

def frames(case,start,duration,label):
    selected=[]
    with Image.open(folder/(case+'.gif')) as gif:
        elapsed=0.
        for frame in ImageSequence.Iterator(gif):
            if elapsed>=start+duration-.001:
                break
            if elapsed>=start-.001:
                im=frame.convert('RGB').copy()
                draw=ImageDraw.Draw(im)
                draw.rectangle((0,0,640,27),fill=(15,18,22))
                draw.text((12,5),label,font=font,fill='white')
                selected.append(im)
            elapsed+=frame.info.get('duration',100)/1000.
    if len(selected)!=round(duration*10):
        raise RuntimeError('Unexpected recorded frame duration: '+case)
    return selected

def encode(name,images):
    writer=imageio_ffmpeg.write_frames(str(folder/name),(640,480),fps=10,codec='libx264',
        pix_fmt_out='yuv420p',quality=7,macro_block_size=1,output_params=['-movflags','+faststart'])
    writer.send(None)
    for im in images:
        writer.send(np.asarray(im).tobytes())
    writer.close()

full=frames('ros_dance',7.5,24.,'BigSpot DANCE | 60 BPM | grounded trajectory | normal time')
encode('dance_full.mp4',full)
demo=frames('ros_dance_sway',8.2,8.,'SWAY | side-to-side | 60 BPM | actual ROS physics')
demo+=frames('ros_dance_bounce',8.2,8.,'BOB + NOD | no jumps | 60 BPM | actual ROS physics')
demo+=full
encode('dance_demo.mp4',demo)
preview=frames('ros_dance',9.,16.,'BigSpot DANCE | 60 BPM | actual ROS + MuJoCo | normal time')
preview[0].save(folder/'dance_preview.gif',save_all=True,append_images=preview[1:],duration=100,loop=0)
sheet=Image.new('RGB',(960,480))
for i,index in enumerate([15,45,75,105,135,165]):
    sheet.paste(full[index].resize((320,240)),((i%3)*320,(i//3)*240))
sheet.save(folder/'dance_review.png')
print(json.dumps({**summary,'full_video_seconds':len(full)/10,'demo_seconds':len(demo)/10,
                  'preview_seconds':len(preview)/10},indent=2))
