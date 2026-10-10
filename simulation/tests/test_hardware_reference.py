"""Original calibration compatibility and crouch-to-stand numeric boundaries."""
import copy
import json
import sys
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/bigspot_controller/scripts'))
sys.path.insert(0,str(ROOT/'simulation/ros2_ws/src/bigspot_sim'))
from RobotHardwares.calibration import pulse_angles as legacy_pulses,SERVO_SPECS
from InverseKinematics.robot_IK import InverseKinematics
from bigspot_sim.hardware_reference import load_reference,reference_model,pulse_angles
from bigspot_sim.motion import MotionController


class HardwareReferenceTests(unittest.TestCase):
    def setUp(self):
        self.reference=load_reference()
        cfg=ROOT/'simulation/ros2_ws/src/bigspot_sim/config'
        self.model=json.loads((cfg/'model.json').read_text())
        self.params=json.loads((cfg/'control_policy_ground_startup.json').read_text())['parameters']

    def legacy_q(self,height):
        feet=np.array([[.224,.224,-.26,-.26],[-.105,.105,-.105,.105],[-height]*4])
        return InverseKinematics([.4,.13],[0.,.04,.183,.2]).inverse_kinematics(feet,0,0,0,0,0,0)

    def test_original_standing_pulses_are_restored(self):
        q=self.legacy_q(.2)
        self.assertEqual(legacy_pulses(q),[95,156,59,80,32,104,85,161,54,87,47,102])
        np.testing.assert_array_equal(pulse_angles(q,self.reference),legacy_pulses(q))

    def test_unsafe_old_crouch_is_rejected_and_entire_new_rise_fits(self):
        with self.assertRaises(ValueError):legacy_pulses(self.legacy_q(.12))
        for height in np.linspace(.16,.2,101):
            angles=legacy_pulses(self.legacy_q(height))
            self.assertGreaterEqual(min(angles),5);self.assertLessEqual(max(angles),175)

    def test_exported_board_channel_and_trims_equal_original_constants(self):
        for s,values in zip(self.reference['servos'],SERVO_SPECS):
            actual=tuple(s[k] for k in ['wired_servo','address','channel','def_angle_deg','direction','trim_deg'])
            self.assertEqual(actual,values)

    def test_reference_does_not_modify_baseline_cad_model(self):
        baseline=copy.deepcopy(self.model)
        ref=reference_model(self.model,self.reference)
        self.assertEqual(self.model,baseline)
        self.assertLess(ref['legs'][3]['joints'][1]['upper'],np.deg2rad(56))

    def test_ground_fold_and_every_startup_command_fit_original_servo_bounds(self):
        c=MotionController(self.model,**self.params)
        self.assertAlmostEqual(c.height,.20)
        self.assertEqual(c.state,'GROUND_OFF')
        obs={'imu_valid':True,'gyro_available':True,'joint_feedback_fresh':True,
            'contact_known':True,'contact_forces_n':[10.]*4,
            'ground_rest_feedback':{'fresh':True,'body_force_n':40.,'body_clearance_m':0.,'base_height_m':.2}}
        for i in range(1100):
            obs['measured_joint_angles_rad']=c.gait.q.ravel().tolist();c.observe(obs)
            if i==100:c.start()
            q=c.step(.016,[0.,0.,0.],[0.,0.],[0.,0.],joint_error=0.)
            pulses=pulse_angles(q,self.reference)
            self.assertGreaterEqual(pulses.min(),0);self.assertLessEqual(pulses.max(),180)
        self.assertEqual(c.state,'READY')


if __name__=='__main__':unittest.main()
