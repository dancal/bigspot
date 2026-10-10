import json
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'simulation/ros2_ws/src/bigspot_sim'))
from bigspot_sim.motion import MotionController
from bigspot_sim.expressions import CUTES,CuteTrajectory
from bigspot_sim.dance import DanceTrajectory
from bigspot_sim.joy_mapping import JoyMapping


class ExpressionTests(unittest.TestCase):
    def controller(self):
        folder=ROOT/'simulation/ros2_ws/src/bigspot_sim/config'
        c=MotionController(json.loads((folder/'model.json').read_text()),
            **json.loads((folder/'control_policy_adaptive.json').read_text())['parameters'])
        c.transition('READY')
        c.gait.q=np.array([leg.ik(p,q) for leg,p,q in zip(c.gait.legs,c.standing,c.gait.q)])
        c.observe({'imu_valid':True,'rpy_rad':[0,0,0],'contact_known':True,'support_count':4,
            'joint_feedback_fresh':True,'contact_forces_n':[10.]*4,
            'ultrasonic':{'both_fresh':True,'distance_m':[4.,4.]}})
        return c

    def test_large_dance_has_larger_excursions_without_increasing_beat_rate(self):
        c=self.controller()
        small,big=[DanceTrajectory(n,c.standing,c.standing,60.) for n in ['dance','dance_big']]
        a=np.array([np.r_[*small.body(t)] for t in np.linspace(0,20,501)])
        b=np.array([np.r_[*big.body(t)] for t in np.linspace(0,big.duration,751)])
        self.assertGreater(np.ptp(b[:,2]),4*np.ptp(a[:,2]))
        self.assertGreater(np.ptp(b[:,3]),3*np.ptp(a[:,3]))
        self.assertEqual(big.bpm,small.bpm)
        self.assertGreater(big.duration,small.duration)

    def test_cute_paths_reachable_with_planned_tripod_support(self):
        c=self.controller()
        for name in sorted(CUTES):
            plan=CuteTrajectory(name,c.standing,c.standing)
            result=plan.audit(c.gait.legs,c.gait.q)
            self.assertTrue(result['reachable'],(name,result))
            self.assertGreaterEqual(result['support_margin_m'],.012)
            self.assertLessEqual(result['peak_planned_joint_speed_rad_s'],1.)

    def test_paw_weight_transfer_precedes_lift_and_landing_precedes_return(self):
        c=self.controller()
        plan=CuteTrajectory('cute_wave',c.standing,c.standing)
        np.testing.assert_allclose(plan.body(1.8)[0][:2],plan.transfer,atol=1e-12)
        self.assertIsNone(plan.lift_at(2.))
        self.assertEqual(plan.lift_at(4.),0)
        self.assertGreater(plan.sample(4.)[0][0,2]-c.standing[0,2],.06)
        self.assertIsNone(plan.lift_at(9.))
        np.testing.assert_allclose(plan.body(9.)[0][:2],plan.transfer,atol=1e-12)

    def test_all_gestures_return_to_standing_with_zero_endpoint_velocity(self):
        c=self.controller()
        for name in CUTES:
            plan=CuteTrajectory(name,c.standing,c.standing)
            for t,sign in [(0.,1.),(plan.duration,-1.)]:
                p=plan.sample(t)[0]
                np.testing.assert_allclose(p,c.standing,atol=1e-12)
                self.assertLess(np.max(np.abs((plan.sample(t+sign*.001)[0]-p)/.001)),1e-6)

    def test_paw_lift_waits_for_measured_support_on_other_three_feet(self):
        c=self.controller()
        self.assertTrue(c.request('cute_wave'))
        for _ in range(125):
            c.step(.016,[0,0,0],[0,0],[0,0])
        c.observation['contact_forces_n']=[15.,1.5,10.,10.]
        before=c.action_time
        for _ in range(8):
            c.step(.016,[0,0,0],[0,0],[0,0])
        self.assertLess(c.action_time,before+.08)
        self.assertGreater(c.pose_wait_support_s,0.)
        c.observation['contact_forces_n']=[10.]*4
        old=c.action_time
        c.step(.016,[0,0,0],[0,0],[0,0])
        self.assertGreater(c.action_time,old)

    def test_cancel_raised_paw_lands_smoothly_before_ready(self):
        c=self.controller()
        c.request('cute_wave')
        for _ in range(300):
            c.step(.016,[0,0,0],[0,0],[0,0])
        self.assertEqual(c.pose_trajectory.lift_at(c.action_time),0)
        c.observation.update({'support_count':3,'contact_forces_n':[0.,10.,10.,10.]})
        c.request('stop')
        for i in range(220):
            previous=c.gait.q.copy()
            if i>70:
                c.observation.update({'support_count':4,'contact_forces_n':[10.]*4})
            c.step(.016,[0,0,0],[0,0],[0,0])
            self.assertLessEqual(np.max(np.abs(c.gait.q-previous)),1.2*.016+1e-10)
        self.assertEqual(c.state,'READY')
        self.assertEqual(c.action_events[-1]['result'],'Cancelled gesture; recovered upright')

    def test_cancel_keeps_weight_transferred_until_measured_paw_landing(self):
        c=self.controller()
        c.request('cute_wave')
        for _ in range(300):
            c.step(.016,[0,0,0],[0,0],[0,0])
        c.observation.update({'support_count':3,'contact_forces_n':[0.,10.,10.,10.]})
        c.request('stop')
        seed=c.pose_recovery_seed.copy()
        for _ in range(100):
            c.step(.016,[0,0,0],[0,0],[0,0])
        self.assertFalse(c.pose_recovery_landed)
        self.assertEqual(c.pose_recovery_recenter_time,0.)
        actual=np.array([leg.fk(q) for leg,q in zip(c.gait.legs,c.gait.q)])
        np.testing.assert_allclose(actual[1:,:2],seed[1:,:2],atol=.0005)
        c.observation.update({'support_count':4,'contact_forces_n':[10.]*4})
        c.step(.016,[0,0,0],[0,0],[0,0])
        self.assertTrue(c.pose_recovery_landed)

    def test_cute_button_is_edge_triggered_and_joy_loss_requests_stop(self):
        mapper=JoyMapping()
        b=[0]*14
        b[5]=b[13]=1
        self.assertEqual(mapper.events([0]*4,b)[0],['cute'])
        self.assertEqual(mapper.events([0]*4,b)[0],[])
        self.assertTrue(mapper.cancel_dance_if_unsafe([0]*4,b,fresh=False))
        self.assertFalse(mapper.cancel_dance_if_unsafe([0]*4,b,fresh=False))


if __name__=='__main__':
    unittest.main()
