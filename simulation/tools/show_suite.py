"""Native ROS transport and free-body validation for larger dance and claps."""
import argparse,json,os
from motion_simulation import episode
p=argparse.ArgumentParser();p.add_argument('--scenarios',nargs='+',default=['dance_show','seated_clap'])
p.add_argument('--domain',type=int,default=43);a=p.parse_args();os.environ['ROS_DOMAIN_ID']=str(a.domain)
for name in a.scenarios:
    holdout=name.startswith('holdout_');scenario=name.removeprefix('holdout_')
    seconds=60 if 'clap' in name else 36
    if 'dropout' in name and 'sonar' not in name:seconds=25
    print('BEGIN '+name,flush=True)
    r=episode(scenario,seconds=seconds,ros=True,render=name in ('dance_show','seated_clap'),
        case='ros_'+name,profile='adaptive',output_folder='show',enable_experiments=False,
        **({'mass_scale':1.15,'friction':.55,'imu_noise':True} if holdout else {}))
    print(json.dumps({k:r.get(k) for k in ['scenario','ros_executed','end_state','fault','fallen','action_result',
        'dance_success','dance_support_min_count','measured_paw_clap_count','seated_clap_success',
        'paw_contact_peak_force_n','front_paw_min_height_during_clap_m','rear_support_min_margin_during_clap_m',
        'observed_action_messages']},ensure_ascii=False),flush=True)
