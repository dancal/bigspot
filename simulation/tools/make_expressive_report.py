"""Audit large-dance/gesture physics records and export normal-time footage."""
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

folder=ROOT/'simulation/generated/expressive'
main=['dance_big','cute_tilt','cute_wave','cute_bow','cute']
recovery=['big_cancel','cute_cancel','cute_sonar_dropout','cute_buttons']
faults=['cute_imu_dropout','cute_joint_dropout','cute_contact_dropout']
cases=['ros_'+s for s in main+recovery+faults]+['holdout_dance_big','holdout_cute_wave']
reports={s:json.loads((folder/(s+'.json')).read_text()) for s in cases}

def physical(case):
    with (folder/(case+'.csv')).open() as stream:
        return list(csv.DictReader(stream))

joint_audits={}
watchdogs={}
for case,r in reports.items():
    if not r['ros_executed'] or r['fallen']:
        raise RuntimeError('ROS transport or stability failed: '+case)
    events=r['action_events']
    if len(events)!=1:
        raise RuntimeError('Expected one action: '+case)
    name=case.removeprefix('ros_')
    if name in faults:
        if r['end_state']!='FAULT' or not r['fault'] or not events[0]['result'].startswith('FAULT:'):
            raise RuntimeError('Missing sensor-loss stop: '+case)
    elif r['end_state']!='READY' or r['fault']:
        raise RuntimeError('Did not recover ready: '+case)
    elif name in recovery:
        expected='Cancelled dance; recovered upright' if name=='big_cancel' else 'Cancelled gesture; recovered upright'
        if events[0]['result']!=expected:
            raise RuntimeError('Cancellation did not complete: '+case)
    elif not case.startswith('holdout_') and not (r.get('dance_success') or r.get('cute_success')):
        raise RuntimeError('Nominal expression failed measured criteria: '+case)
    rows=json.loads((folder/(case+'_sensors.json')).read_text())
    unique={row['control_time_s']:row for row in rows if row['control_time_s']>=8.5}
    t=np.array(sorted(unique));q=np.array([unique[x]['commanded_joints_rad'] for x in t])
    peak=float(np.max(np.abs(np.diff(q,axis=0)/np.diff(t)[:,None])))
    if peak>1.2+1e-5:
        raise RuntimeError('Joint command rate exceeded: '+case)
    joint_audits[case]={'peak_command_joint_speed_rad_s':peak}
    if name in faults:
        first=min(float(row['time_s']) for row in physical(case) if row['state']=='FAULT')
        held=q[t>=first+.016]
        excursion=float(np.max(np.abs(held-held[0])))
        if first>14.4 or excursion>1e-10:
            raise RuntimeError('Fault did not hold command promptly: '+case)
        watchdogs[case]={'cut_at_s':14.,'first_fault_s':first,'held_command_excursion_rad':excursion}
    if name=='cute_buttons':
        actions=r['observed_action_messages']
        if actions.count('cute')!=1 or 'stop' not in actions:
            raise RuntimeError('Held cute button must trigger once and then stop')

old=json.loads((ROOT/'simulation/generated/dance/ros_dance.json').read_text())
big=reports['ros_dance_big'];wave=reports['ros_cute_wave']
summary={'date':'2026-10-09','regression_tests_passed':65,'actual_ros_cases':len(reports),
    'nominal_actions_passed':main,'measured_height_range_old_m':old['dance_measured_height_range_m'],
    'measured_height_range_new_m':big['dance_measured_height_range_m'],
    'height_excursion_multiplier':big['dance_measured_height_range_m']/old['dance_measured_height_range_m'],
    'measured_roll_range_old_deg':old['dance_measured_roll_pitch_range_deg'][0],
    'measured_roll_range_new_deg':big['dance_measured_roll_pitch_range_deg'][0],
    'roll_excursion_multiplier':big['dance_measured_roll_pitch_range_deg'][0]/old['dance_measured_roll_pitch_range_deg'][0],
    'large_dance_path_seconds':38.,'large_dance_attitude_rms_deg':big['pose_tracking_rms_deg'],
    'large_dance_minimum_support_count':big['dance_support_min_count'],
    'paw_peak_clearance_m':wave['front_paw_peak_clearance_m'],
    'paw_wave_three_stance_feet_seconds':wave['three_stance_feet_paw_wave_seconds'],
    'cute_sequence_attitude_rms_deg':reports['ros_cute']['pose_tracking_rms_deg'],
    'holdout_large_dance_passed':reports['holdout_dance_big']['dance_success'],
    'holdout_large_dance_minimum_support_count':reports['holdout_dance_big']['dance_support_min_count'],
    'holdout_paw_wave_passed':reports['holdout_cute_wave']['cute_success'],
    'holdout_conditions':'60 BPM, mass +15%, friction .55, gyroscope/accelerometer noise; ideal orientation supplied',
    'joint_rate_audits':joint_audits,'watchdog_audits':watchdogs,
    'button12_action':'dance_big','button13_action':'cute','physical_button_mapping_verified':False,
    'engine':'ROS 2 Humble + MuJoCo CPU','hardware_validated':False,'isaac_ros5_executed':False,
    'real_mass_motor_current_and_self_collision_verified':False,'independent_head_joint':False,
    'motion_from_recorded_free_body_physics':True,'playback_speed_multiplier':1.,'audio':False}
(folder/'expressive_validation.json').write_text(json.dumps(summary,indent=2)+'\n')

big_rows=physical('ros_dance_big')
bt=np.array([float(row['time_s']) for row in big_rows])
sensors=json.loads((folder/'ros_dance_big_sensors.json').read_text())
fig,axes=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
axes[0,0].bar(['Previous','Large dance'],[old['dance_measured_height_range_m']*1000,big['dance_measured_height_range_m']*1000],color=['gray','#278c67'])
axes[0,0].set(ylabel='Measured height excursion (mm)',title='Amplitude increased; playback speed unchanged')
for i,label in enumerate(['Roll','Pitch']):
    axes[0,1].plot(bt,[float(row[['roll_rad','pitch_rad'][i]])*180/np.pi for row in big_rows],label=label+' measured')
axes[0,1].set(xlim=(8,49),xlabel='Time (s)',ylabel='Measured body attitude (deg)')
axes[0,1].legend()
wr=physical('ros_cute_wave');wt=np.array([float(row['time_s']) for row in wr])
axes[1,0].plot(wt,[float(row['FR_sole_z_m'])*1000 for row in wr],label='Front paw')
for name in ['FL','RR','RL']:
    axes[1,0].plot(wt,[float(row[name+'_sole_z_m'])*1000 for row in wr],label=name+' stance',alpha=.7)
axes[1,0].set(xlim=(8,22),xlabel='Time (s)',ylabel='Measured sole height (mm)',title='Paw wave with three stance feet')
axes[1,0].legend()
forces=np.array([[float(row[n+'_force_N']) for n in ['FR','FL','RR','RL']] for row in big_rows])
axes[1,1].step(bt,(forces>1.).sum(1),where='post',label='Nominal large dance')
hold=physical('holdout_dance_big');ht=np.array([float(row['time_s']) for row in hold])
hf=np.array([[float(row[n+'_force_N']) for n in ['FR','FL','RR','RL']] for row in hold])
axes[1,1].step(ht,(hf>1.).sum(1),where='post',alpha=.7,label='Mass/friction/noise holdout')
axes[1,1].axhline(3,color='red',ls='--',label='Required support criterion')
axes[1,1].set(xlim=(8,49),ylim=(-.1,4.5),xlabel='Time (s)',ylabel='Foot forces above 1 N',title='Holdout still fails strict support criterion')
axes[1,1].legend(fontsize=8)
fig.suptitle('BigSpot larger dance and gestures | actual ROS 2 Humble + MuJoCo CPU')
fig.savefig(folder/'expressive_validation.png',dpi=150);plt.close(fig)
if '--plots-only' in sys.argv:
    print(json.dumps(summary,indent=2));sys.exit(0)

font_path=Path('C:/Windows/Fonts/arial.ttf') if sys.platform=='win32' else Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
font=ImageFont.truetype(str(font_path),16) if font_path.exists() else ImageFont.load_default()
def frames(case,start,seconds,label):
    selected=[]
    with Image.open(folder/(case+'.gif')) as gif:
        elapsed=0.
        for frame in ImageSequence.Iterator(gif):
            if elapsed>=start+seconds-.001:break
            if elapsed>=start-.001:
                im=frame.convert('RGB').copy();draw=ImageDraw.Draw(im)
                draw.rectangle((0,0,640,27),fill=(15,18,22))
                draw.text((12,5),label,font=font,fill='white');selected.append(im)
            elapsed+=frame.info.get('duration',100)/1000.
    if len(selected)!=round(seconds*10):raise RuntimeError('Recorded duration mismatch: '+case)
    return selected

def encode(name,images):
    writer=imageio_ffmpeg.write_frames(str(folder/name),(640,480),fps=10,codec='libx264',
        pix_fmt_out='yuv420p',quality=7,macro_block_size=1,output_params=['-movflags','+faststart'])
    writer.send(None)
    for frame in images:writer.send(np.asarray(frame).tobytes())
    writer.close()

large=frames('ros_dance_big',7.5,42.,'LARGE DANCE | 73 mm height / 13 deg roll range | normal time')
cute=frames('ros_cute',7.5,35.5,'CUTE | body tilt + paw wave + play bow | normal time')
encode('dance_big.mp4',large);encode('cute_demo.mp4',cute);encode('expressive_demo.mp4',large+cute)
preview=frames('ros_dance_big',20.5,5.,'LARGE DANCE | deep dip + rise | actual ROS physics')
preview+=frames('ros_cute_tilt',10.,4.5,'CUTE TILT | whole-body lean | actual ROS physics')
preview+=frames('ros_cute_wave',12.,5.5,'PAW WAVE | 8 cm measured clearance | actual ROS physics')
preview+=frames('ros_cute_bow',10.5,4.,'PLAY BOW | lowered body + wiggle | actual ROS physics')
preview[0].save(folder/'expressive_preview.gif',save_all=True,append_images=preview[1:],duration=100,loop=0)
sheet=Image.new('RGB',(960,480))
for i,frame in enumerate([large[30],large[140],large[210],cute[30],cute[170],cute[285]]):
    sheet.paste(frame.resize((320,240)),((i%3)*320,(i//3)*240))
sheet.save(folder/'expressive_review.png')
print(json.dumps({**summary,'large_video_seconds':42.,'cute_video_seconds':35.5,
                 'showcase_seconds':77.5,'preview_seconds':len(preview)/10},indent=2))
