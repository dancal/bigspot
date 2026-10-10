"""90 BPM grounded rhythm and real ROS joystick selection/stop regressions."""
import argparse
import json
import os
from motion_simulation import episode

parser=argparse.ArgumentParser()
parser.add_argument('--group',choices=['main','checks'],default='main')
parser.add_argument('--scenarios',nargs='+')
parser.add_argument('--domain',type=int,default=43)
args=parser.parse_args()
if not 0<=args.domain<=101:raise ValueError('ROS domain must be 0..101')
os.environ['ROS_DOMAIN_ID']=str(args.domain)
scenarios=([('dance_rhythm',30.),('holdout_rhythm',30.),('selector_buttons',28.),('selector_deadman',24.)]
    if args.group=='main' else [(s,18.) for s in ['rhythm_cancel','rhythm_buttons','rhythm_deadman','rhythm_joy_dropout',
        'rhythm_sonar_dropout','rhythm_joint_dropout','rhythm_imu_dropout','rhythm_contact_dropout']])
if args.scenarios:
    if not set(args.scenarios)<=set(s[0] for s in scenarios):raise ValueError('Scenario not in selected group')
    scenarios=[s for s in scenarios if s[0] in args.scenarios]
for scenario,seconds in scenarios:
    print('BEGIN '+scenario,flush=True)
    holdout=scenario=='holdout_rhythm'
    r=episode('dance_rhythm' if holdout else scenario,seconds=seconds,ros=True,
        render=scenario=='dance_rhythm',case='ros_'+scenario,profile='adaptive',output_folder='tempo',
        enable_experiments=False,**({'mass_scale':1.15,'friction':.55,'imu_noise':True} if holdout else {}))
    print(json.dumps({k:r.get(k) for k in ['scenario','ros_executed','end_state','fault','fallen',
        'action_result','dance_success','dance_support_min_count','dance_bpm',
        'dance_measured_height_range_m','dance_measured_roll_pitch_range_deg','observed_action_messages',
        'joystick_selection']},ensure_ascii=False),flush=True)
