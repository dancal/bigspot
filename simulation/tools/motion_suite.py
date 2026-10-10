"""Run independent reproducible multi-action scenarios, not a scripted base animation."""
import argparse
import json
from pathlib import Path
from motion_simulation import episode,ROOT


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--ros',action='store_true')
    parser.add_argument('--render',action='store_true')
    parser.add_argument('--scenarios',nargs='+',default=['forward','backward','left','right','turn_left_90',
          'turn_right_90','run','accelerate','bow','crouch','jump_left','jump_right','jump_back',
          'rear_stand_probe','handstand_probe'])
    args=parser.parse_args()
    reports=[]
    for scenario in args.scenarios:
        report=episode(scenario,ros=args.ros,render=args.render,
                       seconds=30. if scenario.endswith('_experimental') else 18.,
                       case=('ros_' if args.ros else 'physics_')+scenario)
        reports.append(report)
        print(json.dumps({k:report.get(k) for k in ['scenario','mean_velocity_m_s','yaw_change_deg',
             'max_tilt_deg','jump_success','action_result','fault','fallen','joint_command_messages_received']}),flush=True)
    path=ROOT/'simulation/generated/motion'/('ros_suite.json' if args.ros else 'physics_suite.json')
    path.write_text(json.dumps(reports,indent=2)+'\n',encoding='utf-8')
