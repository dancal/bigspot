import json
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'simulation/ros2_ws/src/bigspot_sim'))
from bigspot_sim.awareness import RangePair,SensorConstraints
from bigspot_sim.motion import MotionController
from bigspot_sim.poses import BODY_POSES,PoseTrajectory


class AdaptiveTests(unittest.TestCase):
    def controller(self):
        folder=ROOT/'simulation/ros2_ws/src/bigspot_sim/config'
        c=MotionController(json.loads((folder/'model.json').read_text()),
            **json.loads((folder/'control_policy_adaptive.json').read_text())['parameters'])
        c.transition('READY')
        c.gait.q=np.array([leg.ik(p,q) for leg,p,q in zip(c.gait.legs,c.standing,c.gait.q)])
        c.observe({'imu_valid':True,'rpy_rad':[0,0,0],'angular_velocity_rad_s':[0,0,0],
                   'contact_known':True,'support_count':4,'joint_feedback_fresh':True,
                   'ultrasonic':{'both_fresh':True,'distance_m':[4.,4.]}})
        return c

    def test_close_echo_immediate_and_far_outlier_cannot_clear_obstacle(self):
        r=RangePair()
        for t,d in [(0.,1.),(.1,1.),(.2,.18)]:
            self.assertTrue(r.update(0,d,t,t))
        self.assertEqual(r.snapshot(.2)['distance_m'][0],.18)
        r.update(0,.18,.3,.3)
        r.update(0,4.,.4,.4)
        self.assertEqual(r.snapshot(.4)['distance_m'][0],.18)
        r.update(0,4.,.5,.5)
        self.assertEqual(r.snapshot(.5)['distance_m'][0],4.)

    def test_timeout_invalid_units_and_stale_timestamp_are_not_clear_path(self):
        r=RangePair()
        r.update(0,.5,0.,0.)
        self.assertFalse(r.update(0,float('nan'),.1,.1))
        self.assertIsNone(r.snapshot(.1)['distance_m'][0])
        self.assertFalse(r.update(1,100.,.1,.1))
        self.assertFalse(r.update(1,.5,0.,1.))
        r.update(1,float('inf'),1.,1.)
        self.assertEqual(r.snapshot(1.)['distance_m'][1],4.)
        self.assertFalse(r.snapshot(1.5)['both_fresh'])

    def test_front_obstacle_caps_forward_but_does_not_claim_rear_coverage(self):
        guard=SensorConstraints([.22,.08,.6],True)
        o={'ultrasonic':{'both_fresh':True,'distance_m':[.18,.8]}}
        np.testing.assert_allclose(guard.limit([.2,0,.4],.016,o),0.)
        self.assertLess(guard.limit([-.08,0,0],.016,o)[0],0.)
        self.assertFalse(guard.status['rear_lateral_obstacle_coverage'])
        o['ultrasonic']['both_fresh']=False
        self.assertEqual(guard.limit([.2,0,0],.016,o)[0],0.)

    def test_braking_cap_monotonic_and_accounts_for_latency_and_jerk(self):
        caps=[]
        for distance in np.linspace(.22,.8,20):
            guard=SensorConstraints([.22,.08,.6],True)
            o={'ultrasonic':{'both_fresh':True,'distance_m':[distance,distance]}}
            caps.append(guard.limit([.22,0,0],.016,o)[0])
        self.assertTrue(np.all(np.diff(caps)>=0))
        for distance,cap in zip(np.linspace(.22,.8,20),caps):
            self.assertLessEqual(cap**2/(2*.2)+cap*(.35+.2/1.2),distance-.22+1e-9)

    def test_velocity_acceleration_and_jerk_bounded_on_start_stop_and_reverse(self):
        c=self.controller()
        g=c.gait
        samples=[]
        for target in ([.18,.04,.4],[0,0,0],[-.12,-.04,-.3],[0,0,0]):
            for _ in range(220):
                g.targets(.016,target)
                samples.append((g.speed.copy(),g.speed_acceleration.copy(),g.speed_jerk.copy()))
        v,a,j=np.array(samples).transpose(1,0,2)
        self.assertTrue(np.all(np.abs(a)<=g.acceleration+1e-10))
        self.assertTrue(np.all(np.abs(j)<=g.jerk_limits+1e-10))
        np.testing.assert_allclose(np.diff(v,axis=0)/.016,a[1:],atol=1e-10)
        self.assertLess(np.linalg.norm(g.speed),1e-4)
        self.assertLess(v[0,0],.001)
        self.assertGreater(v[100,0],.17)

    def test_sensor_combination_reduces_speed_for_tilt_lag_and_support_loss(self):
        c=self.controller()
        o=c.observation.copy()
        o['rpy_rad']=[np.deg2rad(15),0,0]
        self.assertLess(c.constraints.limit([.18,0,0],.016,o)[0],.1)
        o['rpy_rad']=[0,0,0]
        for _ in range(30):
            result=c.constraints.limit([.18,0,0],.016,o,joint_error=.6)
        self.assertEqual(result[0],0.)
        o['support_count']=0
        for _ in range(8):
            result=c.constraints.limit([.18,0,0],.016,o)
        self.assertEqual(result[0],0.)

    def test_all_pose_paths_reachable_with_support_transfer_before_lift(self):
        c=self.controller()
        for name in BODY_POSES:
            plan=PoseTrajectory(name,c.standing,c.standing)
            self.assertTrue(plan.audit(c.gait.legs,c.gait.q)['reachable'],name)
            if plan.lift is not None:
                self.assertGreaterEqual(plan.support_margin_m,.012)
                self.assertAlmostEqual(plan.sample(1.4)[0][plan.lift,2],c.standing[plan.lift,2])

    def test_poses_require_stopped_state_and_lifts_require_measured_contacts(self):
        c=self.controller()
        c.state='WALKING'
        self.assertFalse(c.request('look_up'))
        c.state='READY'
        c.observation['joint_feedback_fresh']=False
        self.assertFalse(c.request('salute_left'))
        self.assertFalse(c.request('look_up'))
        c.observation['joint_feedback_fresh']=True
        self.assertTrue(c.request('look_up'))

    def test_finishing_turn_keeps_filter_acceleration_for_smooth_deceleration(self):
        c=self.controller()
        c.action='turn_left'
        c.gait.speed[2]=.06
        c.gait.speed_acceleration[2]=-.12
        c.finish('Completed relative 90 degree turn')
        self.assertEqual(c.state,'STOPPING')
        self.assertAlmostEqual(c.gait.speed[2],.06)
        self.assertAlmostEqual(c.gait.speed_acceleration[2],-.12)
        old=c.gait.speed_acceleration.copy()
        c.step(.016,[0,0,0],[0,0],[0,0])
        self.assertTrue(np.all(np.abs(c.gait.speed_acceleration-old)<=c.gait.jerk_limits*.016+1e-10))

    def test_missing_accelerometer_is_not_reported_as_freefall(self):
        from bigspot_sim.state_estimator import StateEstimator
        e=StateEstimator()
        e.update([0,0,0,1],[0,0,0],[0,0,0],0.,accel_available=False)
        self.assertEqual(e.snapshot(0.)['motion_state'],'CONTACT_UNKNOWN')

    def test_pose_cancel_returns_smoothly_and_sensor_loss_holds_last_command(self):
        c=self.controller()
        self.assertTrue(c.request('look_down'))
        for _ in range(100):
            c.step(.016,[0,0,0],[0,.05],[0,0])
        before=c.gait.q.ravel().copy()
        c.request('stop')
        after=np.array(c.step(.016,[0,0,0],[0,.05],[0,0]))
        self.assertLess(np.max(np.abs(after-before)),1.2*.016+1e-10)
        before=after.copy()
        after=c.step(.016,[0,0,0],[0,.05],[0,0],imu_fresh=False)
        self.assertEqual(c.state,'FAULT')
        np.testing.assert_array_equal(after,before)

    def test_relative_heading_hold_corrects_yaw_without_absolute_heading_claim(self):
        c=self.controller()
        c.constraints.limit([.12,0,0],.016,c.observation)
        c.observation['rpy_rad'][2]=.1
        result=c.constraints.limit([.12,0,0],.016,c.observation)
        self.assertLess(result[2],0.)
        self.assertTrue(c.constraints.status['heading_hold_relative_only'])

    def test_blocked_translation_does_not_keep_marching_for_heading_hold(self):
        c=self.controller()
        c.constraints.limit([.12,0,0],.016,c.observation)
        c.observation['rpy_rad'][2]=.15
        c.observation['ultrasonic']['both_fresh']=False
        np.testing.assert_allclose(c.constraints.limit([.12,0,0],.016,c.observation),0.)
        self.assertFalse(c.constraints.status['heading_hold_relative_only'])

    def test_obstacle_stop_hysteresis_prevents_restart_from_noisy_echo(self):
        c=self.controller()
        for distance in [.249,.27,.28,.30]:
            c.observation['ultrasonic']['distance_m']=[distance,distance]
            self.assertEqual(c.constraints.limit([.12,0,0],.016,c.observation)[0],0.)
        c.observation['ultrasonic']['distance_m']=[.4,.4]
        self.assertGreater(c.constraints.limit([.12,0,0],.016,c.observation)[0],0.)

    def test_sonar_clearance_gates_pose_and_triggers_smooth_recovery(self):
        c=self.controller()
        c.observation['ultrasonic']['distance_m']=[.27,.4]
        self.assertFalse(c.request('look_down'))
        c.observation['ultrasonic']['distance_m']=[4.,4.]
        self.assertTrue(c.request('look_down'))
        c.step(.016,[0,0,0],[0,0],[0,0])
        c.observation['ultrasonic']['both_fresh']=False
        before=c.gait.q.ravel().copy()
        after=np.array(c.step(.016,[0,0,0],[0,0],[0,0]))
        self.assertTrue(c.pose_cancelled)
        self.assertLess(np.max(np.abs(after-before)),1.2*.016+1e-10)


if __name__=='__main__':
    unittest.main()
