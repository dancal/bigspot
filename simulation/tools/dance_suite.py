"""Reproducible actual ROS dance trials, cancellation and sensor loss checks."""
import argparse
import json
from pathlib import Path
from motion_simulation import episode

ROOT=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser()
parser.add_argument('--group',choices=['routines','checks','holdout'],default='routines')
parser.add_argument('--scenarios',nargs='+')
args=parser.parse_args()
if args.group=='routines':
    scenarios=[('dance_sway',36.,True),('dance_bounce',36.,True),('dance',36.,True)]
elif args.group=='holdout':
    scenarios=[('dance',32.,False)]
else:
    scenarios=[(s,22.,False) for s in ['dance_cancel','dance_sonar_dropout','dance_imu_dropout',
        'dance_contact_dropout','dance_joint_dropout','dance_buttons','dance_deadman']]
if args.scenarios:
    wanted=set(args.scenarios)
    scenarios=[s for s in scenarios if s[0] in wanted]
    if len(scenarios)!=len(wanted):
        raise ValueError('Requested scenario is not in selected group')
parameters=json.loads((ROOT/'simulation/ros2_ws/src/bigspot_sim/config/control_policy_adaptive.json').read_text())['parameters']
if args.group=='holdout':
    parameters={**parameters,'dance_bpm':90.}
reports={}
for scenario,seconds,render in scenarios:
    print(f'BEGIN {args.group}: {scenario}',flush=True)
    case=('holdout_' if args.group=='holdout' else 'ros_')+scenario
    extra={'friction':.55,'mass_scale':1.15,'imu_noise':True} if args.group=='holdout' else {}
    report=episode(scenario,parameters=parameters,seconds=seconds,ros=True,render=render,
        profile='adaptive',output_folder='dance',case=case,**extra)
    reports[scenario]=report
    print(json.dumps({k:report.get(k) for k in ['scenario','ros_executed','end_state','fault','fallen',
        'action_result','action_events','dance_success','dance_support_min_count','pose_tracking_rms_deg',
        'dance_measured_height_range_m','dance_measured_roll_pitch_range_deg']},ensure_ascii=False),flush=True)
(ROOT/'simulation/generated/dance'/f'{args.group}_suite.json').write_text(json.dumps(reports,indent=2)+'\n')
