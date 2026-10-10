import json
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'simulation/ros2_ws/src/bigspot_sim'))
from bigspot_sim.motion import MotionController
from bigspot_sim.state_estimator import StateEstimator, stamp_seconds


class RosTimestampTests(unittest.TestCase):
    def test_identical_ros_stamps_stay_fresh_at_fractional_seconds(self):
        estimator=StateEstimator()
        # 3 + 264000000/1e9 rounds above 3264000000/1e9 and used to fault startup.
        for sec,ns in [(3,264000000),(3,280000000),(3,296000000)]:
            estimator.update([0.,0.,0.,1.],[0.,0.,0.],[0.,0.,9.81],stamp_seconds(sec,ns))
            clock=(sec*1_000_000_000+ns)/1e9
            self.assertTrue(estimator.snapshot(clock)['imu_valid'])
        self.assertFalse(estimator.snapshot(clock-.001)['imu_valid'])
        self.assertFalse(estimator.snapshot(clock+.251)['imu_valid'])


class GroundStartupTests(unittest.TestCase):
    def setUp(self):
        cfg=ROOT/'simulation/ros2_ws/src/bigspot_sim/config'
        model=json.loads((cfg/'model.json').read_text())
        params=json.loads((cfg/'control_policy_adaptive.json').read_text())['parameters']
        self.c=MotionController(model,ground_rest=True,**params)
        self.obs={'imu_valid':True,'gyro_available':True,'joint_feedback_fresh':True,
            'measured_joint_angles_rad':self.c.gait.q.ravel().tolist(),
            'contact_known':True,'contact_forces_n':[0.,0.,0.,0.],
            'ground_rest_feedback':{'fresh':True,'body_force_n':40.,'body_clearance_m':0.,'base_height_m':.053}}

    def tick(self,count=1):
        for _ in range(count):
            self.c.observe(self.obs)
            self.c.step(.016,[.2,.1,.3],[0.,0.],[0.,0.])

    def test_off_stays_unpowered_without_start_even_with_velocity_input(self):
        self.tick(100)
        self.assertEqual(self.c.state,'GROUND_OFF')
        self.assertFalse(self.c.ground_startup.servo_enabled)
        self.assertEqual(self.c.ground_startup.strength,0.)
        np.testing.assert_array_equal(self.c.gait.speed,0.)

    def test_capture_actual_angles_before_enabling_without_command_jump(self):
        actual=self.c.gait.q.ravel().copy();actual[[0,3,6,9]]=[-.15,.15,-.1,.1]
        self.obs['measured_joint_angles_rad']=actual.tolist()
        self.c.start();self.tick(25)
        self.assertEqual(self.c.state,'SERVO_ARMING')
        np.testing.assert_array_equal(self.c.gait.q.ravel(),actual)
        self.assertTrue(self.c.ground_startup.servo_enabled)
        self.assertEqual(self.c.ground_startup.strength,0.)

    def test_missing_joint_or_body_support_never_enables(self):
        self.c.start();self.obs['joint_feedback_fresh']=False;self.tick(50)
        self.assertEqual(self.c.state,'GROUND_OFF')
        self.obs['joint_feedback_fresh']=True;self.obs['ground_rest_feedback']['body_force_n']=0.
        self.tick(50)
        self.assertFalse(self.c.ground_startup.servo_enabled)

    def test_sensor_loss_after_arming_freezes_command_and_holds_servo(self):
        self.c.start();self.tick(80)
        before=self.c.gait.q.ravel().copy()
        self.obs['joint_feedback_fresh']=False;self.tick(10)
        self.assertEqual(self.c.state,'FAULT')
        np.testing.assert_array_equal(self.c.gait.q.ravel(),before)
        self.assertTrue(self.c.ground_startup.servo_enabled)

    def test_no_four_foot_confirmation_does_not_advance_to_rise(self):
        self.c.start()
        for _ in range(580):
            self.obs['measured_joint_angles_rad']=self.c.gait.q.ravel().tolist();self.tick()
        self.assertEqual(self.c.state,'LEG_PREPARING')
        self.obs['contact_forces_n']=[10.,10.,10.,10.]
        for _ in range(40):
            self.obs['measured_joint_angles_rad']=self.c.gait.q.ravel().tolist();self.tick()
        self.assertEqual(self.c.state,'RISING')


if __name__=='__main__':unittest.main()
