"""Local free-body tuning; failed trials are kept as evidence."""
import argparse,json
from motion_simulation import episode,MotionController
from bigspot_sim.dance import DanceTrajectory
from bigspot_sim.clap import SeatedClapTrajectory
from motion_simulation import ROOT
import numpy as np

p=argparse.ArgumentParser();p.add_argument('--clap',action='store_true');p.add_argument('--scale',type=float,default=.9)
p.add_argument('--holdout',action='store_true');p.add_argument('--audit',action='store_true');a=p.parse_args()
if a.audit:
    f=ROOT/'simulation/ros2_ws/src/bigspot_sim/config'
    c=MotionController(json.loads((f/'model.json').read_text()),**json.loads((f/'control_policy_adaptive.json').read_text())['parameters'])
    q=np.array([leg.ik(pos,q) for leg,pos,q in zip(c.gait.legs,c.standing,c.gait.q)])
    plan=SeatedClapTrajectory(c.standing,c.standing,c.gait.legs)
    print(json.dumps(plan.audit(c.gait.legs,q)));raise SystemExit
original=DanceTrajectory.body
def scaled(self,t):
    shift,rpy=original(self,t)
    return (a.scale*shift,a.scale*rpy) if self.name=='dance_show' else (shift,rpy)
DanceTrajectory.body=scaled
r=episode('seated_clap' if a.clap else 'dance_show',seconds=60 if a.clap else 36,profile='adaptive',output_folder='show',
    case=('local_clap_q' if a.clap else 'local_show_'+str(round(a.scale*100)))+('_holdout' if a.holdout else ''),
    **({'mass_scale':1.15,'friction':.55,'imu_noise':True} if a.holdout else {}))
print(json.dumps({k:r.get(k) for k in ['end_state','fault','action_result','dance_success','dance_support_min_count',
    'dance_measured_height_range_m','dance_measured_roll_pitch_range_deg','measured_paw_clap_count','seated_clap_success',
    'rear_support_min_margin_during_clap_m','front_paw_min_gap_m','paw_contact_peak_n']}))
