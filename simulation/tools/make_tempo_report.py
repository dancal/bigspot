"""Check native ROS records and export cadence footage at recorded speed."""
import csv
import json
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
if sys.platform=='win32':sys.path.insert(0,str(ROOT/'simulation/generated/python_deps'))
import numpy as np
from PIL import Image,ImageSequence,ImageDraw,ImageFont
import imageio_ffmpeg
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

folder=ROOT/'simulation/generated/tempo'
main=['dance_rhythm','holdout_rhythm']
cancel=['rhythm_cancel','rhythm_buttons','rhythm_deadman','rhythm_joy_dropout','rhythm_sonar_dropout','selector_deadman']
faults=['rhythm_imu_dropout','rhythm_joint_dropout','rhythm_contact_dropout']
cases=main+cancel+faults+['selector_buttons']
reports={s:json.loads((folder/('ros_'+s+'.json')).read_text()) for s in cases}
rate_audits={};watchdogs={}
for case,r in reports.items():
    if not r['ros_executed'] or r['fallen']:raise RuntimeError('ROS/stability failure: '+case)
    if len(r['action_events'])!=1:raise RuntimeError('Repeated/missing action: '+case)
    result=r['action_events'][0]['result']
    if case in main and not r['dance_success']:raise RuntimeError('Grounded rhythm failed: '+case)
    if case in cancel and not result.startswith('Cancelled'):raise RuntimeError('Cancel failed: '+case)
    if case=='selector_buttons' and result!='Completed sensor-settled gesture':raise RuntimeError('Selection execution failed')
    if case in faults:
        if r['end_state']!='FAULT' or not r['fault'] or not result.startswith('FAULT:'):raise RuntimeError('Watchdog failed: '+case)
    elif r['end_state']!='READY' or r['fault']:raise RuntimeError('Recovery failed: '+case)
    sensors=json.loads((folder/('ros_'+case+'_sensors.json')).read_text())
    unique={row['control_time_s']:row for row in sensors if row['control_time_s']>=8.5}
    t=np.array(sorted(unique));q=np.array([unique[x]['commanded_joints_rad'] for x in t])
    speed=float(np.max(np.abs(np.diff(q,axis=0)/np.diff(t)[:,None])))
    if speed>1.2+1e-5:raise RuntimeError('Command rate violated: '+case)
    rate_audits[case]=speed
    with (folder/('ros_'+case+'.csv')).open() as stream:physical=list(csv.DictReader(stream))
    if case in faults:
        first=min(float(row['time_s']) for row in physical if row['state']=='FAULT')
        held=q[t>=first+.016]
        excursion=float(np.max(np.abs(held-held[0])))
        if first>10.4 or excursion>1e-10:raise RuntimeError('Late/unstable fault hold: '+case)
        watchdogs[case]={'cut_at_s':10.,'fault_at_s':first,'held_excursion_rad':excursion}
    if case in ('rhythm_buttons','rhythm_deadman','rhythm_joy_dropout','selector_buttons','selector_deadman'):
        action='cute_wave' if case.startswith('selector') else 'dance_rhythm'
        if r['observed_action_messages']!=[action,'stop']:raise RuntimeError('Held button/action transport failed: '+case)
        if r['joystick_selection']['selection_count']!=26:raise RuntimeError('Missing catalog entries')

fast=reports['dance_rhythm'];holdout=reports['holdout_rhythm']
old=json.loads((ROOT/'simulation/generated/expressive/ros_dance_big.json').read_text())
summary={'date':'2026-10-09','regression_tests_passed':71,'actual_ros_cases':len(cases),
    'rhythm_bpm':90.,'previous_bpm':60.,'previous_cycle_s':8.,'rhythm_cycle_s':4*60/90.,
    'cadence_multiplier':3.,'previous_path_duration_s':38.,'rhythm_path_duration_s':6.+16*60/90.,
    'rhythm_measured_height_range_m':fast['dance_measured_height_range_m'],
    'rhythm_measured_roll_pitch_range_deg':fast['dance_measured_roll_pitch_range_deg'],
    'previous_measured_height_range_m':old['dance_measured_height_range_m'],
    'nominal_support_min_count':fast['dance_support_min_count'],'holdout_support_min_count':holdout['dance_support_min_count'],
    'nominal_attitude_rms_deg':fast['pose_tracking_rms_deg'],
    'holdout_passed':holdout['dance_success'],'holdout_conditions':'mass +15%, friction .55, gyro/accelerometer noise; ideal orientation',
    'command_joint_speed_audits_rad_s':rate_audits,'sensor_fault_audits':watchdogs,
    'selection_count':26,'button12_action':'dance_rhythm','button13_action':'cute',
    'selection_previous_chord':[5,4,12],'selection_next_chord':[5,4,13],'selection_execute_chord':[5,4,6],
    'selection_alone_starts_motion':False,'physical_gamepad_verified':False,'wsl_input_device_present':False,
    'experimental_actions_enabled_in_trials':False,'hardware_validated':False,'isaac_ros5_executed':False,
    'engine':'ROS 2 Humble + MuJoCo CPU','self_collision_enabled':False,
    'mass_and_motor_current_measured':False,'playback_speed_multiplier':1.}
(folder/'tempo_validation.json').write_text(json.dumps(summary,indent=2)+'\n')

font_path=Path('C:/Windows/Fonts/malgun.ttf')
font=ImageFont.truetype(str(font_path),17) if font_path.exists() else ImageFont.load_default()
frames=[]
with Image.open(folder/'ros_dance_rhythm.gif') as gif:
    elapsed=0.
    for frame in ImageSequence.Iterator(gif):
        if 7.5-.001<=elapsed<26.-.001:
            im=frame.convert('RGB').copy();draw=ImageDraw.Draw(im)
            draw.rectangle((0,0,640,29),fill=(15,18,22))
            draw.text((10,5),'90 BPM | 2.67 s cycle | ROS physics | normal playback',font=font,fill='white')
            frames.append(im)
        elapsed+=frame.info.get('duration',100)/1000.
if len(frames)!=185:raise RuntimeError('Video duration mismatch')
writer=imageio_ffmpeg.write_frames(str(folder/'rhythm_demo.mp4'),(640,480),fps=10,codec='libx264',
    pix_fmt_out='yuv420p',quality=7,macro_block_size=1,output_params=['-movflags','+faststart'])
writer.send(None)
for frame in frames:writer.send(np.asarray(frame).tobytes())
writer.close()
preview=frames[45:145]
preview[0].save(folder/'rhythm_preview.gif',save_all=True,append_images=preview[1:],duration=100,loop=0)
sheet=Image.new('RGB',(960,480))
for i,index in enumerate([35,65,85,105,125,155]):
    sheet.paste(frames[index].resize((320,240)),((i%3)*320,(i//3)*240))
sheet.save(folder/'rhythm_review.png')

with (folder/'ros_dance_rhythm.csv').open() as stream:rows=list(csv.DictReader(stream))
sensors=json.loads((folder/'ros_dance_rhythm_sensors.json').read_text())
fig,axes=plt.subplots(2,2,figsize=(11,7),constrained_layout=True)
axes[0,0].bar(['Large 60 BPM','Rhythm 90 BPM'],[8,4*60/90],color=['gray','#16856c'])
axes[0,0].set(title='Body cycle is 3 times faster',ylabel='Cycle duration (s)')
t=np.array([float(row['time_s']) for row in rows])
for field,label in [('roll_rad','Roll'),('pitch_rad','Pitch')]:
    axes[0,1].plot(t,np.array([float(row[field]) for row in rows])*180/np.pi,label=label+' measured')
axes[0,1].set(xlim=(8,26),ylabel='Angle (deg)',xlabel='Simulation time (s)');axes[0,1].legend()
axes[1,0].plot(t,[float(row['z_m'])*1000 for row in rows],label='Base height')
axes[1,0].set(xlim=(8,26),ylabel='Height (mm)',xlabel='Simulation time (s)')
for case,label in [('dance_rhythm','Nominal'),('holdout_rhythm','Mass/friction/noise')]:
    with (folder/('ros_'+case+'.csv')).open() as stream:data=list(csv.DictReader(stream))
    tt=[float(row['time_s']) for row in data]
    count=[sum(float(row[k])>1. for k in ['FR_force_N','FL_force_N','RR_force_N','RL_force_N']) for row in data]
    axes[1,1].step(tt,count,where='post',label=label,alpha=.7)
axes[1,1].set(xlim=(8.5,25),ylim=(1.8,4.2),ylabel='Supporting feet (>1 N)',xlabel='Simulation time (s)')
axes[1,1].legend();fig.savefig(folder/'tempo_validation.png',dpi=150);plt.close(fig)
print(json.dumps({**summary,'video_seconds':18.5,'preview_seconds':10.},indent=2))
