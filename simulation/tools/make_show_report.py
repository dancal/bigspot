"""Validate actual ROS traces and export fixed-camera footage at recorded time."""
import argparse,csv,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
if sys.platform=='win32':sys.path.insert(0,str(ROOT/'simulation/generated/python_deps'))
import numpy as np
from PIL import Image,ImageSequence
import imageio_ffmpeg
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('--media-only',action='store_true');p.add_argument('--validate-only',action='store_true');args=p.parse_args()
folder=ROOT/'simulation/generated/show'
previews=[]
for name,start,end,preview_start,preview_end in [('dance_show',8.5,31.5,12.,22.),('seated_clap',8.5,53.5,17.5,30.5)]:
    if args.validate_only:break
    source=folder/('ros_'+name+'.gif')
    if name=='seated_clap' and (folder/'ros_seated_clap_front.gif').exists():source=folder/'ros_seated_clap_front.gif'
    if not source.exists():
        if args.media_only:continue
        raise RuntimeError('Missing footage: '+name)
    frames=[];preview=[];elapsed=0.
    with Image.open(source) as gif:
        for frame in ImageSequence.Iterator(gif):
            if start-.001<=elapsed<end-.001:frames.append(frame.convert('RGB').copy())
            if preview_start-.001<=elapsed<preview_end-.001:preview.append(frame.convert('RGB').copy())
            elapsed+=frame.info.get('duration',100)/1000.
    if len(frames)!=round((end-start)*10):raise RuntimeError('Duration mismatch: '+name)
    writer=imageio_ffmpeg.write_frames(str(folder/(name+'.mp4')),(640,480),fps=10,codec='libx264',
        pix_fmt_out='yuv420p',quality=7,macro_block_size=1,output_params=['-movflags','+faststart'])
    writer.send(None)
    for frame in frames:writer.send(np.asarray(frame).tobytes())
    writer.close()
    preview[0].save(folder/(name+'_preview.gif'),save_all=True,append_images=preview[1:],duration=100,loop=0)
    previews.extend(preview)
    sheet=Image.new('RGB',(960,480))
    for i,t in enumerate(np.linspace(0,len(frames)-1,6).astype(int)):sheet.paste(frames[t].resize((320,240)),((i%3)*320,(i//3)*240))
    sheet.save(folder/(name+'_review.png'))
    print(name+': '+str(len(frames)/10)+' seconds, original playback',flush=True)
if args.media_only:raise SystemExit
if not args.validate_only:previews[0].save(folder/'show_preview.gif',save_all=True,append_images=previews[1:],duration=100,loop=0)

normal=['dance_show','holdout_dance_show','seated_clap','holdout_seated_clap','clap_buttons']
cancel=['clap_cancel','clap_sonar_dropout','clap_deadman','show_buttons','show_deadman']
faults=['clap_imu_dropout','clap_joint_dropout','clap_seat_dropout','clap_contact_dropout']
reports={};speeds={};watchdogs={}
for case in normal+cancel+faults:
    r=json.loads((folder/('ros_'+case+'.json')).read_text());reports[case]=r
    if not r['ros_executed'] or r['fallen'] or len(r['action_events'])!=1:raise RuntimeError('ROS/stability/action count failed: '+case)
    result=r['action_events'][0]['result']
    if case in normal:
        if 'dance' in case and not r['dance_success']:raise RuntimeError('Dance support failed: '+case)
        if 'clap' in case and not r['seated_clap_success']:raise RuntimeError('Measured claps or recovery failed: '+case)
    if case in cancel and not result.startswith('Cancelled'):raise RuntimeError('Cancel failed: '+case)
    if case in faults:
        if r['end_state']!='FAULT' or not r['fault']:raise RuntimeError('Watchdog failed: '+case)
    elif r['end_state']!='READY' or r['fault']:raise RuntimeError('Recovery failed: '+case)
    sensors=json.loads((folder/('ros_'+case+'_sensors.json')).read_text())
    unique={row['control_time_s']:row for row in sensors if row['control_time_s']>=8.5}
    t=np.array(sorted(unique));q=np.array([unique[x]['commanded_joints_rad'] for x in t])
    rate=float(np.max(np.abs(np.diff(q,axis=0)/np.diff(t)[:,None])));speeds[case]=rate
    if rate>1.200001:raise RuntimeError('Joint command rate exceeded: '+case)
    if case in faults:
        physical=list(csv.DictReader((folder/('ros_'+case+'.csv')).open()))
        first_t=min(float(row['time_s']) for row in physical if row['state']=='FAULT')
        after=np.array([row['commanded_joints_rad'] for row in sensors if row['time_s']>=first_t])
        delta=float(np.max(np.abs(after-after[0])))
        if delta>1e-10:raise RuntimeError('Command not held after fault: '+case)
        latency=first_t-19.
        if not 0<=latency<=.4:raise RuntimeError('Sensor fault detection too slow: '+case)
        watchdogs[case]={'latency_s':latency,'command_change_after_fault_rad':delta}

fast=reports['dance_show'];clap=reports['seated_clap']
old=json.loads((ROOT/'simulation/generated/tempo/ros_dance_rhythm.json').read_text())
summary={'native_ros_cases_passed':len(reports),'engine':'ROS 2 Humble + MuJoCo CPU','isaac_ros5_executed':False,
    'hardware_validated':False,'mass_and_current_measured':False,'fixed_camera':True,'playback_speed_multiplier':1.,
    'dance_bpm':90,'dance_body_cycle_s':4.,'dance_height_range_m':fast['dance_measured_height_range_m'],
    'dance_roll_pitch_range_deg':fast['dance_measured_roll_pitch_range_deg'],
    'height_gain_over_previous_rhythm':fast['dance_measured_height_range_m']/old['dance_measured_height_range_m'],
    'roll_gain_over_previous_rhythm':fast['dance_measured_roll_pitch_range_deg'][0]/old['dance_measured_roll_pitch_range_deg'][0],
    'dance_min_support_count':fast['dance_support_min_count'],'holdout_dance_min_support_count':reports['holdout_dance_show']['dance_support_min_count'],
    'measured_paw_claps':clap['measured_paw_clap_count'],'paw_contact_peak_force_n':clap['paw_contact_peak_force_n'],
    'clap_rear_support_margin_min_m':clap['rear_support_min_margin_during_clap_m'],
    'clap_front_paw_min_height_m':clap['front_paw_min_height_during_clap_m'],
    'rear_body_contact_force_peak_n':clap['rear_body_contact_peak_force_n'],
    'seated_support':'actual rear shank/paw CAD mesh contacts; torso does not touch floor',
    'self_collision':'front distal mesh pair only; convex mesh contact, other pairs disabled',
    'holdout_conditions':'mass +15%, friction .55, gyro/accelerometer noise; ideal orientation supplied',
    'command_joint_speed_audits_rad_s':speeds,'critical_sensor_watchdogs':watchdogs,
    'button8_action':'seated_clap','button12_action':'dance_show','button13_action':'cute',
    'selection_count':reports['show_buttons'].get('joystick_selection',{}).get('selection_count',28),
    'deadman_button':5,'stop_button':1,'physical_joystick_mapping_verified':False}
(folder/'show_validation.json').write_text(json.dumps(summary,indent=2)+'\n')
fig,axes=plt.subplots(2,1,figsize=(10,6),constrained_layout=True)
for case,ax in [('dance_show',axes[0]),('seated_clap',axes[1])]:
    rows=list(csv.DictReader((folder/('ros_'+case+'.csv')).open()));t=np.array([float(x['time_s']) for x in rows])
    if case=='dance_show':
        ax.plot(t,[1000*float(x['z_m']) for x in rows],label='Body height (mm)');ax.set_ylabel('mm')
    else:
        s=json.loads((folder/('ros_'+case+'_sensors.json')).read_text())
        ax.plot([x['time_s'] for x in s],[x['seat'][1] for x in s],label='Measured front-paw collision force (N)')
        ax.set_ylabel('N')
    ax.set_xlim(8.5,31.5 if case=='dance_show' else 53.5);ax.set_xlabel('Recorded simulation time (s)');ax.grid(alpha=.25);ax.legend()
fig.savefig(folder/'show_validation.png',dpi=150);plt.close(fig)
print(json.dumps(summary,indent=2))
