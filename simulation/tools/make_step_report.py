"""Validate ROS step traces; derive charts from measured foot-ground states."""
import csv,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
if sys.platform=='win32':sys.path.insert(0,str(ROOT/'simulation/generated/python_deps'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

folder=ROOT/'simulation/generated/steps'
normal=['dance_steps','holdout_dance_steps','steps_buttons']
cancel=['steps_cancel','steps_deadman','steps_sonar_dropout']
faults=['steps_imu_dropout','steps_joint_dropout','steps_contact_dropout']
reports={};speeds={};watchdogs={};interruptions={}
for case in normal+cancel+faults:
    r=json.loads((folder/('ros_'+case+'.json')).read_text());reports[case]=r
    if not r['ros_executed'] or r['fallen'] or len(r['action_events'])!=1:
        raise RuntimeError('ROS/stability/action count failed: '+case)
    result=r['action_events'][0]['result']
    if case in normal and not r['step_dance_success']:
        raise RuntimeError('Four measured swings/landings and support failed: '+case)
    if case in cancel and not result.startswith('Cancelled'):
        raise RuntimeError('Recovery failed: '+case)
    if case in faults:
        if r['end_state']!='FAULT' or not r['fault']:raise RuntimeError('Watchdog failed: '+case)
    elif r['end_state']!='READY' or r['fault']:raise RuntimeError('Not upright: '+case)
    sensors=json.loads((folder/('ros_'+case+'_sensors.json')).read_text())
    physical=list(csv.DictReader((folder/('ros_'+case+'.csv')).open()))
    if case in cancel:
        recovered=[s for s in physical if s['state']=='DANCE_RECOVERY']
        if not recovered or min(sum(float(s[n+'_force_N'])>1 for n in ['FR','FL','RR','RL']) for s in recovered)<3:
            raise RuntimeError('Recovery lost tripod support: '+case)
    unique={s['control_time_s']:s for s in sensors if s['control_time_s']>=8.5}
    t=np.array(sorted(unique));q=np.array([unique[x]['commanded_joints_rad'] for x in t])
    rate=float(np.max(np.abs(np.diff(q,axis=0)/np.diff(t)[:,None])));speeds[case]=rate
    if rate>1.200001:raise RuntimeError('Joint command rate exceeded: '+case)
    if case not in normal:
        sample=min(physical,key=lambda s:abs(float(s['time_s'])-13.4))
        h=float(sample['FR_sole_z_m']);f=float(sample['FR_force_N'])
        if h<.025 or f>.2:raise RuntimeError('Test did not interrupt an airborne paw: '+case)
        interruptions[case]={'cut_time_s':13.4,'front_right_height_m_at_cut':h,'front_right_force_n_at_cut':f}
    if case in faults:
        first_t=min(float(s['time_s']) for s in physical if s['state']=='FAULT')
        after=np.array([s['commanded_joints_rad'] for s in sensors if s['time_s']>=first_t])
        delta=float(np.max(abs(after-after[0])))
        latency=first_t-13.4
        if delta>1e-10 or not 0<=latency<=.4:raise RuntimeError('Fault command hold/latency failed: '+case)
        watchdogs[case]={'latency_s':latency,'command_change_after_fault_rad':delta}

r=reports['dance_steps'];holdout=reports['holdout_dance_steps']
summary={'native_ros_cases_passed':len(reports),'engine':'ROS 2 Humble + MuJoCo CPU',
    'isaac_ros5_executed':False,'hardware_validated':False,'self_collision_enabled':False,
    'mass_and_current_measured':False,'playback_speed_multiplier':1.,'dance_bpm':90,
    'planned_duration_s':32.,'one_paw_step_period_s':8.5*60/90,
    'step_order':['FR','RL','FL','RR'],'measured_dance_steps':r['measured_dance_steps'],
    'measured_step_landings':r['measured_step_landings'],
    'dance_min_support_count':r['dance_support_min_count'],
    'holdout_dance_min_support_count':holdout['dance_support_min_count'],
    'holdout_conditions':'mass +15%, friction .55, gyro/accelerometer noise; ideal orientation supplied',
    'dance_height_range_m':r['dance_measured_height_range_m'],
    'command_joint_speed_audits_rad_s':speeds,'critical_sensor_watchdogs':watchdogs,
    'airborne_interruptions':interruptions,'button12_action':'dance_steps','selection_count':29,
    'physical_joystick_mapping_verified':False}
(folder/'step_validation.json').write_text(json.dumps(summary,indent=2)+'\n')
physical=list(csv.DictReader((folder/'ros_dance_steps.csv').open()))
t=np.array([float(s['time_s']) for s in physical])
fig,axes=plt.subplots(2,1,figsize=(10,6),constrained_layout=True)
for name in ['FR','FL','RR','RL']:
    axes[0].plot(t,[1000*float(s[name+'_sole_z_m']) for s in physical],label=name)
    axes[1].plot(t,[float(s[name+'_force_N']) for s in physical],label=name)
axes[0].set_ylabel('Measured sole height (mm)');axes[1].set_ylabel('Measured floor force (N)')
for ax in axes:ax.set_xlim(8.5,42);ax.grid(alpha=.25);ax.legend(ncol=4);ax.set_xlabel('Recorded simulation time (s)')
fig.savefig(folder/'step_validation.png',dpi=150);plt.close(fig)
print(json.dumps(summary,indent=2))
