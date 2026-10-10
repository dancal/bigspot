"""Actual ROS transport validation for smooth movement and sensor-aware poses."""
import argparse
import json
from pathlib import Path
from motion_simulation import episode

ROOT=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser()
parser.add_argument('--group',choices=['poses','safety','holdout','aliases'],default='safety')
parser.add_argument('--scenarios',nargs='+')
args=parser.parse_args()
if args.group=='poses':
    scenarios=[('pose_gallery',109.,True),('pose_cancel',20.,False)]
elif args.group=='holdout':
    scenarios=[('salute_left',20.,False),('look_up',20.,False),('smooth_start_stop',24.,False)]
elif args.group=='aliases':
    scenarios=[('bow',20.,False),('crouch',20.,False)]
else:
    scenarios=[(s,24.,s in ('smooth_start_stop','obstacle_wall','balance_push')) for s in
               ['smooth_start_stop','smooth_reverse','obstacle_wall','sonar_dropout','sonar_invalid',
                'gyro_fallback','joint_dropout','imu_dropout','balance_push','turn_left_90','turn_right_90','buttons']]
if args.scenarios:
    selected=set(args.scenarios)
    scenarios=[v for v in scenarios if v[0] in selected]
    if len(scenarios)!=len(selected):
        raise ValueError('Requested scenario is not in the selected group')
reports={}
for scenario,seconds,render in scenarios:
    print(f'BEGIN {args.group}: {scenario}',flush=True)
    holdout={'friction':.55,'mass_scale':1.15,'imu_noise':True} if args.group=='holdout' else {}
    case=('holdout_' if args.group=='holdout' else 'ros_')+scenario
    report=episode(scenario,seconds=seconds,ros=True,render=render,profile='adaptive',
                   output_folder='adaptive',case=case,**holdout)
    reports[scenario]=report
    print(json.dumps({k:report.get(k) for k in ['scenario','end_state','fault','fallen','action_result',
        'max_tilt_deg','pose_tracking_rms_deg','estimated_attitude_rms_deg','obstacle_stop_success',
        'sensor_constraint_reasons','ultrasonic_messages_sent']},ensure_ascii=False),flush=True)
output=ROOT/'simulation/generated/adaptive'
(output/(args.group+'_suite.json')).write_text(json.dumps(reports,indent=2)+'\n',encoding='utf-8')
