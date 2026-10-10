"""Native ROS graph, physical cadence and fault/cancel verification."""
import argparse,json,os
from motion_simulation import episode
p=argparse.ArgumentParser();p.add_argument('--scenarios',nargs='+',default=['dance_funk','holdout_dance_funk','funk_buttons',
    'funk_cancel','funk_deadman','funk_sonar_dropout','funk_imu_dropout','funk_joint_dropout','funk_contact_dropout'])
p.add_argument('--domain',type=int,default=49);p.add_argument('--output-folder',default='groove')
a=p.parse_args();os.environ['ROS_DOMAIN_ID']=str(a.domain)
for name in a.scenarios:
    holdout=name.startswith('holdout_');scenario=name.removeprefix('holdout_')
    seconds=49 if scenario in ('dance_funk','funk_buttons') else 25
    print('BEGIN '+name,flush=True)
    r=episode(scenario,seconds=seconds,ros=True,render=False,case='ros_'+name,
        profile='adaptive',output_folder=a.output_folder,enable_experiments=False,
        **({'mass_scale':1.15,'friction':.55,'imu_noise':True} if holdout else {}))
    print(json.dumps({k:r.get(k) for k in ['scenario','ros_executed','end_state','fault','fallen','action_result',
        'funk_dance_success','measured_diagonal_pair_steps','median_pair_step_interval_s',
        'dynamic_two_or_more_support_fraction','max_zero_ground_force_interval_s','observed_action_messages']},ensure_ascii=False),flush=True)
