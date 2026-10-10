import json
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'simulation/ros2_ws/src/bigspot_sim'))
sys.path.insert(0,str(ROOT/'src/bigspot_mpu6050/scripts'))
from bigspot_sim.motion import MotionController
from bigspot_sim.state_estimator import StateEstimator
from bigspot_sim.joy_mapping import JoyMapping
from imu_units import raw_to_si, remap_mount


class MotionTests(unittest.TestCase):
    def controller(self,experiments=False):
        folder=ROOT/'simulation/ros2_ws/src/bigspot_sim/config'
        c=MotionController(json.loads((folder/'model.json').read_text()),
             enable_experiments=experiments,
             **json.loads((folder/'control_policy_motion.json').read_text())['parameters'])
        c.transition('READY')
        c.observe({'imu_valid':True,'rpy_rad':[0,0,0],'gyro_available':True,
                   'contact_known':True,'support_count':4,'joint_feedback_fresh':True})
        return c

    def test_jump_requires_opt_in_contacts_and_actual_joint_feedback(self):
        c=self.controller()
        self.assertFalse(c.request('jump_left'))
        c.enable_experiments=True
        c.observation['contact_known']=False
        self.assertFalse(c.request('jump_left'))
        c.observation['contact_known']=True
        c.observation['joint_feedback_fresh']=False
        self.assertFalse(c.request('jump_left'))
        c.observation['joint_feedback_fresh']=True
        self.assertTrue(c.request('jump_left'))
        self.assertFalse(c.request('jump_right'))

    def test_vertical_acrobatics_are_not_mislabelled_as_supported(self):
        c=self.controller(True)
        self.assertFalse(c.request('rear_stand'))
        self.assertFalse(c.request('handstand'))
        self.assertTrue(c.request('rear_stand_probe'))

    def test_sensor_loss_during_jump_holds_last_command(self):
        c=self.controller(True)
        c.request('jump_left')
        before=c.gait.q.ravel().copy()
        c.observation['contact_known']=False
        actual=c.step(.016,[0,0,0],[0,0],[0,0])
        self.assertEqual(c.state,'FAULT')
        np.testing.assert_array_equal(actual,before)

    def test_run_cannot_bypass_startup_or_stale_command_stop(self):
        c=self.controller()
        c.transition('CROUCH')
        self.assertFalse(c.request('run'))
        c.transition('READY')
        self.assertTrue(c.request('run'))
        for _ in range(40):
            c.step(.016,[.18,0,0],[0,0],[0,0])
        for _ in range(120):
            c.step(.016,[.18,0,0],[0,0],[0,0],command_fresh=False)
        self.assertEqual(c.state,'READY')

    def test_joy_edge_deadman_stop_and_heartbeat(self):
        mapper=JoyMapping()
        axes=[1.,0.,0.,0.]
        buttons=[0,0,1,0,0,0,0,0,0,0]
        self.assertEqual(mapper.events(axes,buttons)[0],[])
        buttons[2]=0
        mapper.events(axes,buttons)
        buttons[2]=buttons[5]=1
        self.assertEqual(mapper.events(axes,buttons)[0],['jump_left'])
        self.assertEqual(mapper.events(axes,buttons)[0],[])
        np.testing.assert_array_equal(mapper.velocity(axes,buttons,fresh=False),np.zeros(3))
        buttons[0]=buttons[1]=1
        self.assertEqual(mapper.events(axes,buttons),(['stop'],False))
        np.testing.assert_array_equal(mapper.velocity(axes,buttons),np.zeros(3))
        buttons[0]=buttons[1]=0
        buttons.extend([0,1])
        self.assertEqual(mapper.events(axes,buttons)[0],['turn_right'])

    def test_joy_boost_and_run_are_separate_and_bounded(self):
        mapper=JoyMapping()
        b=[0]*10
        b[5]=1
        self.assertAlmostEqual(mapper.velocity([0,1,0,0],b)[0],.06)
        b[4]=1
        self.assertAlmostEqual(mapper.velocity([0,1,0,0],b)[0],.15)
        b[3]=1
        self.assertEqual(mapper.events([0,1,0,0],b)[0],['run'])
        self.assertAlmostEqual(mapper.velocity([0,1,0,0],b)[0],.18)

    def test_estimator_gravity_specific_force_and_airborne_contacts(self):
        e=StateEstimator()
        e.contacts([10]*4,0.)
        e.update([0,0,0,1],[0,0,0],[0,0,9.81],0.)
        np.testing.assert_allclose(e.linear_accel,0,atol=1e-12)
        self.assertEqual(e.motion_state,'SUPPORTED')
        for t in [.016,.032,.048]:
            e.contacts([0]*4,t)
            e.update([0,0,0,1],[0,0,0],[0,0,0],t)
        self.assertEqual(e.motion_state,'AIRBORNE')
        self.assertFalse(e.snapshot(.3)['contact_known'])
        self.assertFalse(e.snapshot(.3)['imu_valid'])

    def test_estimator_relative_yaw_and_invalid_sensor_samples(self):
        e=StateEstimator()
        for i in range(101):
            e.update(None,[0,0,1],[0,0,9.81],i*.01,orientation_available=False)
        self.assertAlmostEqual(e.rpy[2],1.016,places=3)
        self.assertFalse(e.snapshot(1.)['absolute_yaw_observable'])
        with self.assertRaises(ValueError):
            e.update([0,0,0,0],[0,0,0],[0,0,9.81],1.01)
        with self.assertRaises(ValueError):
            e.update([0,0,0,1],[float('nan'),0,0],[0,0,9.81],1.01)
        tilted=StateEstimator()
        rpy=tilted.update(None,[0,0,0],[0,9.81*np.sin(.3),9.81*np.cos(.3)],0.,orientation_available=False)
        np.testing.assert_allclose(rpy,[.3,0,0],atol=1e-12)
        with self.assertRaises(ValueError):
            StateEstimator().update(None,[0,0,0],[0,0,0],0.,orientation_available=False)

    def test_actual_register_scales_produce_si_units(self):
        gyro,accel=raw_to_si([1640,0,-1640],[0,0,16384],3,0)
        np.testing.assert_allclose(gyro,[np.deg2rad(100),0,-np.deg2rad(100)])
        np.testing.assert_allclose(accel,[0,0,9.80665])
        with self.assertRaises(ValueError):
            raw_to_si([0,0,0],[0,0,0],4,0)

    def test_mount_rotation_changes_gyro_axes_and_orientation_consistently(self):
        mount=[0.,0.,np.sqrt(.5),np.sqrt(.5)]
        q,gyro,accel=remap_mount(mount,[1,0,0],[0,0,9.80665],mount)
        np.testing.assert_allclose(q,[0,0,0,1],atol=1e-12)
        np.testing.assert_allclose(gyro,[0,1,0],atol=1e-12)
        np.testing.assert_allclose(accel,[0,0,9.80665],atol=1e-12)

    def test_unreachable_vertical_path_is_rejected_before_motion(self):
        c=self.controller(True)
        before=c.gait.q.copy()
        self.assertFalse(c.request('rear_stand_experimental'))
        self.assertIn('verify CAD joint limits',c.action_result)
        np.testing.assert_array_equal(c.gait.q,before)


if __name__=='__main__':
    unittest.main()
