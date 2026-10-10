"""Export original driver constants and corrected legacy IK without motor imports."""
import hashlib
import json
import sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/bigspot_controller/scripts'))
from RobotHardwares.calibration import SERVO_SPECS,IK_NAMES,STANDING_HEIGHT_M,CROUCH_HEIGHT_M
from InverseKinematics.robot_IK import InverseKinematics


def build():
    feet=np.array([[.224,.224,-.26,-.26],[-.105,.105,-.105,.105],[-STANDING_HEIGHT_M]*4])
    ik=InverseKinematics([.4,.13],[0.,.04,.183,.2])
    q=ik.inverse_kinematics(feet,0.,0.,0.,0.,0.,0.)
    payload={'standing_height_m':STANDING_HEIGHT_M,'legacy_crouch_height_m':CROUCH_HEIGHT_M,
        'legacy_body_m':[.4,.13],'legacy_leg_m':[0.,.04,.183,.2],
        'legacy_stance_m':feet.T.tolist(),'legacy_standing_ik_rad':np.array(q).reshape(4,3).tolist(),
        'servo_margin_deg':5.,'servo_input_correspondence':'original_array_slots',
        'actuation_range_deg':180,'pulse_width_us':[500,2500],
        'cad_axis_to_physical_motor_verified':False,'physical_power_off_pose_measured':False,
        'ik_domain_formula':'corrected cosine law; original source had +l4^2',
        'source_files':{},'servos':[]}
    for path in ['src/bigspot_controller/scripts/RobotHardwares/calibration.py',
                 'src/bigspot_controller/scripts/InverseKinematics/robot_IK.py']:
        payload['source_files'][path]=hashlib.sha256((ROOT/path).read_bytes()).hexdigest()
    for input_name,(name,address,pin,zero,direction,trim) in zip(IK_NAMES,SERVO_SPECS):
        payload['servos'].append({'input_joint':input_name+'_Joint','wired_servo':name,
            'address':address,'channel':pin,'def_angle_deg':zero,'direction':direction,'trim_deg':trim})
    target=ROOT/'simulation/ros2_ws/src/bigspot_sim/config/hardware_reference.json'
    target.write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('Hardware reference exported:',target)
    return payload


if __name__=='__main__':build()
