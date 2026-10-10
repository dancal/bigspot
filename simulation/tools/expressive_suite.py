"""Large dance and dog-like gestures, with reproducible physics/sensor trials."""
import argparse
import json
from pathlib import Path
from motion_simulation import episode

ROOT=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser()
parser.add_argument('--group',choices=['main','checks','holdout'],default='main')
parser.add_argument('--scenarios',nargs='+')
parser.add_argument('--local',action='store_true')
parser.add_argument('--no-render',action='store_true')
args=parser.parse_args()
if args.group=='main':
    scenarios=[('dance_big',52.),('cute_tilt',26.),('cute_wave',28.),('cute_bow',26.),('cute',46.)]
elif args.group=='holdout':
    scenarios=[('dance_big',52.),('cute_wave',28.)]
else:
    scenarios=[(s,24.) for s in ['big_cancel','cute_cancel','cute_sonar_dropout','cute_joint_dropout',
        'cute_imu_dropout','cute_contact_dropout','cute_buttons']]
if args.scenarios:
    selected=set(args.scenarios)
    scenarios=[s for s in scenarios if s[0] in selected]
    if len(scenarios)!=len(selected):
        raise ValueError('Scenario not in selected group')
reports={}
for scenario,seconds in scenarios:
    print(f'BEGIN {args.group}: {scenario}',flush=True)
    prefix='holdout_' if args.group=='holdout' else 'local_' if args.local else 'ros_'
    extra={'friction':.55,'mass_scale':1.15,'imu_noise':True} if args.group=='holdout' else {}
    report=episode(scenario,seconds=seconds,ros=not args.local,render=args.group=='main' and not args.no_render,
        profile='adaptive',case=prefix+scenario,output_folder='expressive',**extra)
    reports[scenario]=report
    print(json.dumps({k:report.get(k) for k in ['scenario','ros_executed','end_state','fault','fallen',
        'action_result','dance_success','cute_success','dance_support_min_count',
        'dance_measured_height_range_m','dance_measured_roll_pitch_range_deg',
        'front_paw_peak_clearance_m','three_stance_feet_paw_wave_seconds','pose_tracking_rms_deg']},ensure_ascii=False),flush=True)
folder=ROOT/'simulation/generated/expressive'
(folder/(('local_' if args.local else '')+args.group+'_suite.json')).write_text(json.dumps(reports,indent=2)+'\n')
