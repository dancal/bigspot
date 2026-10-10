"""Offline cadence candidate checks; trial scaling never changes runtime config."""
import argparse
import json
from pathlib import Path
from motion_simulation import episode
from bigspot_sim.dance import DanceTrajectory

parser=argparse.ArgumentParser()
parser.add_argument('--scales',nargs='+',type=float,default=[.9,.8,.7])
parser.add_argument('--holdout',action='store_true')
parser.add_argument('--pitch-scale',type=float,default=1.)
parser.add_argument('--xy-scale',type=float,default=1.)
parser.add_argument('--bpm',type=float,default=90.)
args=parser.parse_args()
parameters=json.loads((Path(__file__).resolve().parents[2]/'simulation/ros2_ws/src/bigspot_sim/config/control_policy_adaptive.json').read_text())['parameters']
parameters['rhythm_bpm']=args.bpm
original=DanceTrajectory.body
for scale in args.scales:
    def trial(self,t):
        shift,rpy=original(self,t)
        if self.name=='dance_rhythm':
            rpy[1]*=args.pitch_scale
            shift[:2]*=args.xy_scale
        return (shift*scale,rpy*scale) if self.name=='dance_rhythm' else (shift,rpy)
    DanceTrajectory.body=trial
    report=episode('dance_rhythm',seconds=34.,parameters=parameters,profile='adaptive',output_folder='tempo',
        case=('trial_holdout_' if args.holdout else 'trial_')+str(round(scale*100))+'_pitch_'+str(round(args.pitch_scale*100))+'_xy_'+str(round(args.xy_scale*100))+'_bpm_'+str(round(args.bpm)),
        **({'mass_scale':1.15,'friction':.55,'imu_noise':True} if args.holdout else {}))
    print(json.dumps({'trial_shape_scale':scale,'trial_pitch_scale':args.pitch_scale,**{k:report[k] for k in ['dance_success',
        'dance_support_min_count','dance_measured_height_range_m','dance_measured_roll_pitch_range_deg',
        'action_result','fault']} }),flush=True)
DanceTrajectory.body=original
