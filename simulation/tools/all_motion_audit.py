"""Replay the entire selectable catalog in free-base physics, with measured metrics."""
import argparse
import csv
import json
from pathlib import Path

from motion_simulation import episode
from bigspot_sim.joy_mapping import SELECTABLE
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
DURATIONS={'dance_funk':49.,'dance_steps':49.,'dance_show':36.,'dance_rhythm':30.,
    'dance_big':52.,'dance':36.,'dance_sway':36.,'dance_bounce':36.,
    'cute':46.,'cute_wave':28.,'cute_tilt':26.,'cute_bow':26.,'seated_clap':58.}
MOVEMENT=('forward','backward','left','right','run','accelerate','smooth_reverse','smooth_start_stop')


def metrics(report,folder,case):
    trace=json.loads((folder/(case+'_sensors.json')).read_text(encoding='utf-8'))
    t=np.array([s['time_s'] for s in trace]);q=np.array([s['commanded_joints_rad'] for s in trace])
    measured=np.array([s['measured_joints_rad'] for s in trace])
    rate=np.diff(q,axis=0)/np.diff(t)[:,None]
    acceleration=np.diff(rate,axis=0)/np.diff(t[1:])[:,None]
    mask=t[1:]>=8.5;amask=t[2:]>=8.5
    events=report['action_events']
    complete=bool(events and events[-1]['result'].startswith('Completed'))
    cancelled=bool(events and events[-1]['result'].startswith('Cancelled'))
    with (folder/(case+'.csv')).open(encoding='utf-8',newline='') as stream:
        physical=list(csv.DictReader(stream))
    xyz=np.array([[float(r[k]) for k in ('x_m','y_m','z_m')] for r in physical])
    final=t>=t[-1]-1.
    final_speed=float(np.linalg.norm((xyz[final][-1,:2]-xyz[final][0,:2])/(t[final][-1]-t[final][0])))
    fault_indices=[i for i,r in enumerate(physical) if r['state']=='FAULT']
    fault_command_delta=(float(np.abs(q[fault_indices[0]:]-q[fault_indices[0]]).max())
                         if fault_indices else None)
    name=report['scenario']
    success=not report['fallen'] and not report['fault'] and report['end_state']=='READY'
    success=success and (cancelled if name.endswith('_cancel') or name.endswith('_sonar_dropout') else
                         complete if name not in MOVEMENT else not events)
    success=success and final_speed<.005
    if name.endswith(('_imu_dropout','_joint_dropout','_contact_dropout')):
        success=bool(report['fault'] and report['end_state']=='FAULT' and
                     not report['fallen'] and fault_command_delta==0.)
    direction_limits={'forward':(0,1,.04),'backward':(0,-1,.04),'left':(1,1,.02),
        'right':(1,-1,.02),'run':(0,1,.12),'accelerate':(0,1,.08),'smooth_start_stop':(0,1,.08)}
    if name in direction_limits:
        axis,sign,minimum=direction_limits[name]
        success=success and report['mean_velocity_m_s'][axis]*sign>minimum
    reverse_speed=None
    if name=='smooth_reverse':
        idx=np.flatnonzero((t>=12.5)&(t<14.5))
        reverse_speed=float((xyz[idx[-1],0]-xyz[idx[0],0])/(t[idx[-1]]-t[idx[0]]))
        success=success and reverse_speed<-.04
    turn_error=abs(abs(report['yaw_change_deg'])-90.) if name in ('turn_left_90','turn_right_90') else None
    if turn_error is not None:success=success and turn_error<2.
    if report['scenario']=='dance_funk':success=success and report['funk_dance_success']
    if report['scenario']=='dance_steps':success=success and report['step_dance_success']
    if report['scenario']=='seated_clap':success=success and report['seated_clap_success']
    return {'scenario':report['scenario'],'passed':bool(success),'ros_executed':report['ros_executed'],
        'end_state':report['end_state'],'fault':report['fault'],'fallen':report['fallen'],
        'events':events,'max_tilt_deg':report['max_tilt_deg'],
        'pose_tracking_rms_deg':report.get('pose_tracking_rms_deg'),
        'pose_max_tracking_error_deg':report.get('pose_max_tracking_error_deg'),
        'yaw_change_deg':report['yaw_change_deg'],
        'final_mean_xy_speed_m_s':final_speed,'reverse_window_mean_x_speed_m_s':reverse_speed,
        'turn_final_error_deg':turn_error,'fault_command_delta_rad':fault_command_delta,
        'peak_command_rate_after_start_rad_s':float(np.abs(rate[mask]).max()),
        'p99_command_acceleration_after_start_rad_s2':float(np.quantile(np.abs(acceleration[amask]),.99)),
        'peak_command_acceleration_after_start_rad_s2':float(np.abs(acceleration[amask]).max()),
        'measured_joint_tracking_rms_rad':float(np.sqrt(np.mean((measured[t>=8.5]-q[t>=8.5])**2))),
        'peak_current_a_estimated':report['peak_current_a_estimated'],
        'current_limit_active_fraction':report['current_limit_active_fraction'],
        'mean_velocity_m_s':report['mean_velocity_m_s'],
        'joint_acceleration_note':'Finite differences at 16 ms; a diagnostic, not an actuator limit.',
        'hardware_validated':False}


def main():
    p=argparse.ArgumentParser();p.add_argument('--folder',default='audit_baseline')
    p.add_argument('--scenarios',nargs='+');p.add_argument('--ros',action='store_true')
    p.add_argument('--holdout',action='store_true');p.add_argument('--include-movement',action='store_true')
    a=p.parse_args();names=a.scenarios or [*SELECTABLE,*(MOVEMENT if a.include_movement else ())]
    folder=ROOT/'simulation/generated'/a.folder;folder.mkdir(parents=True,exist_ok=True)
    summary={}
    for name in names:
        scenario=name+'_90' if name in ('turn_left','turn_right') else name
        case=('ros_' if a.ros else 'local_')+('holdout_' if a.holdout else '')+name
        print('BEGIN '+case,flush=True)
        report=episode(scenario,seconds=DURATIONS.get(name,24. if name in MOVEMENT else 22.),
            profile='adaptive',case=case,output_folder=a.folder,ros=a.ros,render=False,
            enable_experiments=False,**({'mass_scale':1.15,'friction':.55,'imu_noise':True} if a.holdout else {}))
        result=metrics(report,folder,case);summary[name]=result
        print(json.dumps(result,ensure_ascii=False),flush=True)
        (folder/('holdout_summary.json' if a.holdout else 'summary.json')).write_text(
            json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':main()
