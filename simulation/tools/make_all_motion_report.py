"""Fail closed on incomplete catalog, transport, holdout or command-hold evidence."""
import json
import re
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'simulation/generated/python_deps'))
sys.path.insert(0,str(ROOT/'simulation/ros2_ws/src/bigspot_sim'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from bigspot_sim.joy_mapping import SELECTABLE
from all_motion_audit import MOVEMENT

OUT=ROOT/'simulation/generated/quality'
ROS_NAMES=('dance_funk','dance_rhythm','cute_wave','salute_left','salute_cancel','seated_clap',
    'turn_left','turn_right','run','smooth_reverse','funk_cancel','funk_joint_dropout',
    'funk_imu_dropout','funk_contact_dropout','rhythm_sonar_dropout')
HOLDOUT_NAMES=('dance_funk','dance_rhythm','cute_wave','run')
COMPARISON=('dance_rhythm','dance_big','dance','dance_sway','dance_bounce',
    'cute','cute_tilt','cute_wave','cute_bow')


def read(folder,name):return json.loads((ROOT/'simulation/generated'/folder/name).read_text(encoding='utf-8'))


def main():
    before=read('audit_baseline','summary.json');current=read('audit_current','summary.json')
    retest=read('audit_final','summary.json');local={**current,**retest}
    ros=read('audit_ros','summary.json');holdout=read('audit_ros','holdout_summary.json')
    if set(local)!=set(SELECTABLE)|set(MOVEMENT)|{'salute_cancel'}:
        raise RuntimeError('Missing local catalog/locomotion/cancel cases')
    if set(ros)!=set(ROS_NAMES) or set(holdout)!=set(HOLDOUT_NAMES):
        raise RuntimeError('ROS or holdout coverage incomplete')
    for group,results in [('local',local),('ros',ros),('holdout',holdout)]:
        for name,m in results.items():
            if not m['passed']:raise RuntimeError(group+' failed: '+name)
            if group!='local' and not m['ros_executed']:raise RuntimeError('Actual ROS required')
            cap=3.8 if name=='dance_funk' or name.startswith('funk_') else 4.
            if m['peak_command_rate_after_start_rad_s']>cap+1e-6:
                raise RuntimeError('Joint command ceiling exceeded: '+name)
            if m['peak_current_a_estimated']>30.000001:raise RuntimeError('Modeled supply cap exceeded')
    log=(ROOT/'simulation/generated/audit_unit_tests.log').read_text(encoding='utf-8')
    tests=re.search(r'Ran (\d+) tests',log)
    if not tests or not log.rstrip().endswith('OK'):raise RuntimeError('Complete passing unit run required')
    comparisons={k:{'baseline_p99_command_acceleration_rad_s2':before[k]['p99_command_acceleration_after_start_rad_s2'],
        'after_p99_command_acceleration_rad_s2':local[k]['p99_command_acceleration_after_start_rad_s2'],
        'baseline_attitude_rms_deg':before[k]['pose_tracking_rms_deg'],
        'after_attitude_rms_deg':local[k]['pose_tracking_rms_deg'],
        'reduction_percent':100*(1-local[k]['p99_command_acceleration_after_start_rad_s2']/
                               before[k]['p99_command_acceleration_after_start_rad_s2'])} for k in COMPARISON}
    manifest={name:{'folder':'audit_final' if name in retest else 'audit_current','case':'local_'+name}
              for name in local}
    summary={'passed':True,'date':'2026-10-10','engine':'ROS 2 Humble + MuJoCo CPU',
        'catalog_actions_passed':len(SELECTABLE),'locomotion_cases_passed':len(MOVEMENT),
        'additional_lifted_pose_cancel_cases_passed':1,'native_ros_nominal_and_fault_cases_passed':len(ros),
        'native_ros_holdout_cases_passed':len(holdout),'unit_tests_passed':int(tests[1]),
        'local_final_cases':manifest,'local_results':local,'ros_results':ros,'holdout_results':holdout,
        'comparisons':comparisons,
        'median_p99_command_acceleration_reduction_percent':float(np.median([v['reduction_percent'] for v in comparisons.values()])),
        'metric_scope':'Nine older dances/gestures, command finite differences at 16 ms. Not perceived naturalness or measured servo jerk.',
        'filter_tradeoff':'40 ms position filter keeps the beat but slightly increases attitude tracking lag in some routines.',
        'stop_verification':'4 s physical settling observation after locomotion command becomes zero; READY requires filtered gyro/tilt/joint settling and four contacts on transition.',
        'found_and_corrected_trial':'Intermediate run returned READY but last-second mean motion was 6.36 mm/s; final run is below 5 mm/s and passes sensor settling.',
        'holdout_assumptions':{'mass_scale':1.15,'friction':.55,'gyro_and_accelerometer_noise':True,'ideal_orientation':True},
        'hardware_validated':False,'isaac_ros5_executed':False,'full_self_collision_validated':False,
        'jumps_and_vertical_acrobatics_validated':False,
        'limits':['Unmeasured mass, torque-speed/current model, drive damping, friction and sonar mounts.',
                  'Ideal IMU orientation is supplied. This does not establish real IMU accuracy.',
                  'Joint/contact feedback is simulated; new ROS 2 actions are not connected to real ROS 1 PWM.',
                  'Most self-collisions disabled; paw-paw pair enabled only for clap.',
                  'Noisy stationary commands retain numerical variation; complete joint acceleration/jerk limits are not established.',
                  'Fault handling holds commanded angles; it does not remove motor power.']}
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/'all_motion_validation.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    fig,axes=plt.subplots(2,1,figsize=(12,9),layout='constrained')
    x=np.arange(len(COMPARISON));w=.38
    axes[0].bar(x-w/2,[comparisons[k]['baseline_p99_command_acceleration_rad_s2'] for k in COMPARISON],w,label='Before',color='#e19647')
    axes[0].bar(x+w/2,[comparisons[k]['after_p99_command_acceleration_rad_s2'] for k in COMPARISON],w,label='After',color='#2d8eae')
    axes[0].set_xticks(x,[k.removeprefix('dance_') for k in COMPARISON],rotation=15)
    axes[0].set_ylabel('P99 |command acceleration| (rad/s²)');axes[0].legend();axes[0].grid(axis='y',alpha=.2)
    axes[0].set_title('Nine older routines: fewer angle stair steps; unchanged BPM')
    for folder,label,color in [('audit_baseline','Before','#e19647'),('audit_current','After','#2d8eae')]:
        s=read(folder,'local_dance_rhythm_sensors.json');t=np.array([r['time_s'] for r in s]);q=np.array([r['commanded_joints_rad'] for r in s])
        v=np.diff(q[:,8])/np.diff(t);mask=(t[1:]>=14.)&(t[1:]<15.5)
        axes[1].plot(t[1:][mask]-8.5,v[mask],label=label,color=color,lw=1.4)
    axes[1].set_xlabel('Dance elapsed time (s)');axes[1].set_ylabel('Rear knee command velocity (rad/s)')
    axes[1].set_title('Measured command stream: rhythm dance at 90 BPM');axes[1].grid(alpha=.2);axes[1].legend()
    fig.suptitle(f'BigSpot: {len(SELECTABLE)} catalog actions + {len(MOVEMENT)} locomotion + lifted-paw cancel\n'
                 f'{len(ros)+len(holdout)} actual ROS trials; {tests[1]} code checks; hardware / Isaac GPU unverified')
    fig.savefig(OUT/'all_motion_validation.png',dpi=140);plt.close(fig)
    print(json.dumps({k:summary[k] for k in ['passed','catalog_actions_passed','locomotion_cases_passed',
        'native_ros_nominal_and_fault_cases_passed','native_ros_holdout_cases_passed','unit_tests_passed',
        'median_p99_command_acceleration_reduction_percent']},ensure_ascii=False))


if __name__=='__main__':main()
