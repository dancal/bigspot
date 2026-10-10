"""Validate cadence, actual phrase motion, ROS transport, recoveries and holds."""
import argparse,csv,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
if sys.platform=='win32':sys.path.insert(0,str(ROOT/'simulation/generated/python_deps'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from funk_metrics import measured_funk

p=argparse.ArgumentParser();p.add_argument('--output-folder',default='groove');a=p.parse_args()
if not a.output_folder.replace('_','').replace('-','').isalnum():raise ValueError('Single result directory required')
f=ROOT/'simulation/generated'/a.output_folder
normal=['dance_funk','holdout_dance_funk','funk_buttons']
cancel=['funk_cancel','funk_deadman','funk_sonar_dropout']
faults=['funk_imu_dropout','funk_joint_dropout','funk_contact_dropout']
reports={};metrics={};rates={};rates_all={};watchdogs={}
for case in normal+cancel+faults:
    r=json.loads((f/('ros_'+case+'.json')).read_text());reports[case]=r
    rows=list(csv.reader((f/('ros_'+case+'.csv')).open()))[1:]
    sensors=json.loads((f/('ros_'+case+'_sensors.json')).read_text());metrics[case]=measured_funk(sensors,rows)
    if not r['ros_executed'] or r['fallen'] or len(r['action_events'])!=1:raise RuntimeError('ROS/action/stability failed: '+case)
    event=r['action_events'][0]['result']
    if case in normal:
        if not r['funk_dance_success']:raise RuntimeError('Cadence/dynamic stability failed: '+case)
        phrases=metrics[case]['measured_phrase_motion']
        if (phrases['side_shuffle']['xy_range_m'][1]<.05 or phrases['forward_back']['xy_range_m'][0]<.08
            or phrases['twist']['yaw_range_deg']<15.):raise RuntimeError('Actual shuffle/travel/turn absent: '+case)
        hip=phrases['hip_wiggle'];head=phrases['head_shake'];nod=phrases['head_bob']
        if (hip['rear_body_xy_range_m'][1]<.04 or hip['rear_body_xy_range_m'][1]<hip['front_body_xy_range_m'][1]+.015
            or head['front_body_xy_range_m'][1]<.045 or head['front_body_xy_range_m'][1]<head['rear_body_xy_range_m'][1]+.02
            or nod['front_body_z_range_m']<.03 or nod['front_body_z_range_m']<nod['rear_body_z_range_m']+.02):
            raise RuntimeError('Actual hip/front emphasis absent: '+case)
    if case in cancel and not event.startswith('Cancelled'):raise RuntimeError('Cancellation failed: '+case)
    if case in faults:
        if r['end_state']!='FAULT' or not r['fault']:raise RuntimeError('Watchdog failed: '+case)
    elif r['end_state']!='READY' or r['fault']:raise RuntimeError('Settle failed: '+case)
    unique={s['control_time_s']:s for s in sensors if s['control_time_s']>=8.5}
    t=np.array(sorted(unique));q=np.array([unique[x]['commanded_joints_rad'] for x in t])
    speed=np.max(abs(np.diff(q,axis=0)/np.diff(t)[:,None]),axis=1)
    active=np.array([bool(unique[x].get('dance')) for x in t])
    rate=float(np.max(speed[active[:-1]|active[1:]]));rates[case]=rate
    rates_all[case]=float(speed.max())
    if rate>3.800001:raise RuntimeError('Dance command rate exceeded: '+case)
    if rates_all[case]>4.000001:raise RuntimeError('Existing normal-standing/walking ceiling exceeded: '+case)
    if r['peak_current_a_estimated']>30.000001:raise RuntimeError('Modeled current cap exceeded: '+case)
    if case in faults:
        first=min(float(row[0]) for row in rows if row[30]=='FAULT')
        held=np.array([s['commanded_joints_rad'] for s in sensors if s['time_s']>=first])
        delta=float(np.max(abs(held-held[0])));latency=first-r['funk_interrupt_at_s']
        if delta>1e-10 or not 0<=latency<=.4:raise RuntimeError('Fault latency/command hold failed: '+case)
        watchdogs[case]={'latency_s':latency,'command_change_after_fault_rad':delta}
r=reports['dance_funk'];m=metrics['dance_funk'];h=reports['holdout_dance_funk']
summary={'native_ros_cases_passed':9,'engine':'ROS 2 Humble + MuJoCo CPU','isaac_ros5_executed':False,
    'hardware_validated':False,'self_collision_enabled':False,'bpm':120,
    'previous_single_paw_step_period_s':8.5*60/90,'planned_diagonal_pair_step_period_s':.5,
    'measured_pair_step_period_s':m['median_pair_step_interval_s'],
    'main_dance_seconds':32.,'measured_diagonal_pair_steps':m['measured_diagonal_pair_steps'],
    'measured_clear_paw_landings_in_main':len(m['measured_clear_paw_landings']),
    'measured_phrase_motion':m['measured_phrase_motion'],
    'holdout_measured_phrase_motion':metrics['holdout_dance_funk']['measured_phrase_motion'],
    'body_reference_points_local_m':m['body_reference_points_local_m'],
    'head_articulation':'No independent neck joint; front/rear rigid-body emphasis by CAD hip pivots',
    'per_paw_peak_clearance_m':m['per_paw_peak_clearance_m'],
    'measured_body_xy_range_m':r['measured_body_xy_range_m'],
    'measured_roll_pitch_range_deg':r['dance_measured_roll_pitch_range_deg'],
    'dynamic_support_model':'Alternating diagonal pairs; dynamic support, not static tripod support',
    'nominal_two_or_more_support_fraction':m['dynamic_two_or_more_support_fraction'],
    'holdout_two_or_more_support_fraction':metrics['holdout_dance_funk']['dynamic_two_or_more_support_fraction'],
    'nominal_max_zero_ground_force_interval_s':r['max_zero_ground_force_interval_s'],
    'holdout_max_zero_ground_force_interval_s':h['max_zero_ground_force_interval_s'],
    'peak_current_a_estimated':r['peak_current_a_estimated'],'current_limit_active_fraction':r['current_limit_active_fraction'],
    'mass_voltage_current_model_measured':False,'ideal_orientation_supplied':True,
    'holdout_conditions':'Mass +15%, friction .55, gyro/accelerometer noise',
    'command_speed_rad_s':rates,'all_states_command_speed_rad_s':rates_all,
    'dance_command_limit_rad_s':3.8,'normal_standing_walking_command_limit_rad_s':4.,
    'sensor_watchdogs':watchdogs,'button12_action':'dance_funk','selection_count':30,
    'physical_gamepad_mapping_verified':False,'playback_multiplier':1.,
    'references':['https://dev.bostondynamics.com/docs/concepts/choreography/move_reference.html',
        'https://bostondynamics.com/blog/in-step-with-spot/','https://docs.petoi.com/applications/skill-creation']}
(f/'funk_validation.json').write_text(json.dumps(summary,indent=2)+'\n')
rows=list(csv.DictReader((f/'ros_dance_funk.csv').open()));t=np.array([float(s['time_s']) for s in rows])
fig,axes=plt.subplots(3,1,figsize=(11,8),constrained_layout=True)
for name in ['FR','FL','RR','RL']:axes[0].plot(t,[1000*float(s[name+'_sole_z_m']) for s in rows],label=name)
axes[0].set_ylabel('Actual sole height (mm)')
for name in ['roll','pitch','yaw']:axes[1].plot(t,[float(s[name+'_rad'])*180/np.pi for s in rows],label=name)
axes[1].set_ylabel('Actual body angles (deg)')
for name in ['x','y']:axes[2].plot(t,[1000*float(s[name+'_m']) for s in rows],label=name)
axes[2].set_ylabel('Actual translation (mm)')
for ax in axes:ax.set_xlim(8.5,44);ax.grid(alpha=.25);ax.legend(ncol=4);ax.set_xlabel('Recorded simulation time (s)')
fig.savefig(f/'funk_validation.png',dpi=150);plt.close(fig)
print(json.dumps({k:summary[k] for k in ['native_ros_cases_passed','bpm','measured_pair_step_period_s',
    'measured_diagonal_pair_steps','nominal_two_or_more_support_fraction','holdout_two_or_more_support_fraction',
    'command_speed_rad_s','sensor_watchdogs']},indent=2))
