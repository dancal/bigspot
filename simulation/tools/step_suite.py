"""ROS transport, measured foot swings and touchdown gates in free-body physics."""
import argparse,json,os
from motion_simulation import episode
p=argparse.ArgumentParser();p.add_argument('--scenarios',nargs='+',default=['dance_steps','holdout_dance_steps','steps_buttons',
    'steps_cancel','steps_deadman','steps_sonar_dropout','steps_imu_dropout','steps_joint_dropout','steps_contact_dropout'])
p.add_argument('--domain',type=int,default=46);a=p.parse_args();os.environ['ROS_DOMAIN_ID']=str(a.domain)
for name in a.scenarios:
    holdout=name.startswith('holdout_');scenario=name.removeprefix('holdout_')
    seconds=46 if scenario in ('dance_steps','steps_buttons') else 21
    print('BEGIN '+name,flush=True)
    r=episode(scenario,seconds=seconds,ros=True,render=False,case='ros_'+name,
        profile='adaptive',output_folder='steps',enable_experiments=False,
        **({'mass_scale':1.15,'friction':.55,'imu_noise':True} if holdout else {}))
    print(json.dumps({k:r.get(k) for k in ['scenario','ros_executed','end_state','fault','fallen','action_result',
        'step_dance_success','dance_support_min_count','measured_dance_steps','measured_step_landings','observed_action_messages']},ensure_ascii=False),flush=True)
