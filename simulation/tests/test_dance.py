import json
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'simulation/ros2_ws/src/bigspot_sim'))
from bigspot_sim.dance import GROUNDED_DANCES,DanceTrajectory
from bigspot_sim.motion import MotionController
from bigspot_sim.joy_mapping import JoyMapping


class DanceTests(unittest.TestCase):
    def controller(self,**extra):
        folder=ROOT/'simulation/ros2_ws/src/bigspot_sim/config'
        parameters=json.loads((folder/'control_policy_adaptive.json').read_text())['parameters']
        c=MotionController(json.loads((folder/'model.json').read_text()),**{**parameters,**extra})
        c.transition('READY')
        c.gait.q=np.array([leg.ik(p,q) for leg,p,q in zip(c.gait.legs,c.standing,c.gait.q)])
        c.observe({'imu_valid':True,'rpy_rad':[0,0,0],'angular_velocity_rad_s':[0,0,0],
                   'contact_known':True,'support_count':4,'joint_feedback_fresh':True,
                   'ultrasonic':{'both_fresh':True,'distance_m':[4.,4.]}})
        return c

    def test_all_routines_reachable_at_tempo_extremes_with_speed_and_support_reserve(self):
        c=self.controller()
        for name in sorted(GROUNDED_DANCES):
            for bpm in [40.,60.,90.]:
                plan=DanceTrajectory(name,c.standing,c.standing,bpm)
                result=plan.audit(c.gait.legs,c.gait.q)
                self.assertTrue(result['reachable'],(name,bpm,result))
                self.assertGreater(result['support_margin_m'],.025)
                self.assertLessEqual(result['peak_planned_joint_speed_rad_s'],1.)
                self.assertIsNone(plan.lift)

    def test_ramps_finish_at_standing_with_zero_endpoint_speed_and_acceleration(self):
        c=self.controller()
        h=.001
        for name in GROUNDED_DANCES:
            plan=DanceTrajectory(name,c.standing,c.standing)
            for endpoint,sign in [(0.,1.),(plan.duration,-1.)]:
                samples=[plan.sample(endpoint+sign*i*h)[0] for i in range(3)]
                np.testing.assert_allclose(samples[0],c.standing,atol=1e-12)
                self.assertLess(np.max(np.abs((samples[1]-samples[0])/h)),1e-6)
                self.assertLess(np.max(np.abs((samples[2]-2*samples[1]+samples[0])/h**2)),.002)

    def test_phrase_boundaries_are_continuous_in_position_and_velocity(self):
        c=self.controller()
        plan=DanceTrajectory('dance',c.standing,c.standing)
        h=.0001
        for beat in [3.,4.,7.,8.,11.,12.]:
            t=plan.ramp_seconds+beat*60/plan.bpm
            a,b,d=[plan.sample(t+offset)[0] for offset in [-h,0,h]]
            self.assertLess(np.max(np.abs((b-a)/h-(d-b)/h)),.0002)

    def test_invalid_tempo_is_rejected(self):
        for bpm in [0.,39.,91.,float('nan'),float('inf')]:
            with self.assertRaises(ValueError):
                self.controller(dance_bpm=bpm)

    def test_faster_than_validated_tempo_requires_explicit_experiments(self):
        c=self.controller(dance_bpm=90.)
        self.assertFalse(c.request('dance'))
        self.assertIn('above 60 BPM',c.action_result)
        c.enable_experiments=True
        self.assertTrue(c.request('dance'))

    def test_dance_requires_stopped_state_four_contacts_and_actual_joint_feedback(self):
        c=self.controller(require_joint_feedback=False)
        for key,value in [('contact_known',False),('support_count',3),('joint_feedback_fresh',False)]:
            old=c.observation[key]
            c.observation[key]=value
            self.assertFalse(c.request('dance'),key)
            c.observation[key]=old
        c.state='WALKING'
        self.assertFalse(c.request('dance'))
        c.state='READY'
        c.observation['ultrasonic']['distance_m']=[.27,4.]
        self.assertFalse(c.request('dance'))
        c.observation['ultrasonic']['distance_m']=[4.,4.]
        self.assertTrue(c.request('dance'))
        self.assertFalse(c.request('dance_bounce'))

    def test_stop_has_bounded_joint_change_and_finishes_upright(self):
        c=self.controller()
        self.assertTrue(c.request('dance'))
        for _ in range(180):
            c.step(.016,[0,0,0],[0,0],[0,0])
        c.request('stop')
        for _ in range(180):
            previous=c.gait.q.copy()
            c.step(.016,[0,0,0],[0,0],[0,0])
            self.assertLessEqual(np.max(np.abs(c.gait.q-previous)),1.2*.016+1e-10)
        self.assertEqual(c.state,'READY')
        self.assertEqual(c.action_events[-1]['result'],'Cancelled dance; recovered upright')
        self.assertFalse(c.action)

    def test_missing_imu_contact_or_joint_data_holds_last_command(self):
        for sensor in ['imu','contact','joint']:
            c=self.controller(require_joint_feedback=False)
            c.request('dance')
            c.step(.016,[0,0,0],[0,0],[0,0])
            old=c.gait.q.ravel().copy()
            if sensor=='contact':
                c.observation['contact_known']=False
            if sensor=='joint':
                c.observation['joint_feedback_fresh']=False
            command=c.step(.016,[0,0,0],[0,0],[0,0],imu_fresh=sensor!='imu')
            self.assertEqual(c.state,'FAULT',sensor)
            np.testing.assert_array_equal(command,old)

    def test_sonar_loss_and_sustained_support_loss_request_smooth_recovery(self):
        for sensor in ['sonar','support']:
            c=self.controller()
            c.request('dance')
            c.step(.016,[0,0,0],[0,0],[0,0])
            if sensor=='sonar':
                c.observation['ultrasonic']['both_fresh']=False
            else:
                c.observation['support_count']=2
            for _ in range(8):
                c.step(.016,[0,0,0],[0,0],[0,0])
            self.assertTrue(c.pose_cancelled,sensor)
            self.assertFalse(c.fault)

    def test_dance_button_is_remappable_deadman_gated_edge_and_stop_wins(self):
        mapper=JoyMapping({'dance_button':7,'crouch_button':-1})
        b=[0]*8
        b[7]=1
        self.assertEqual(mapper.events([0]*4,b)[0],[])
        b[7]=0
        mapper.events([0]*4,b)
        b[5]=b[7]=1
        self.assertEqual(mapper.events([0]*4,b)[0],['dance_funk'])
        self.assertEqual(mapper.events([0]*4,b)[0],[])
        b[1]=1
        self.assertEqual(mapper.events([0]*4,b),(['stop'],False))

    def test_deadman_or_joy_loss_cancels_dance_once(self):
        for stale in [False,True]:
            mapper=JoyMapping()
            b=[0]*13
            b[5]=b[12]=1
            self.assertEqual(mapper.events([0]*4,b)[0],['dance_funk'])
            if not stale:
                b[5]=0
            self.assertTrue(mapper.cancel_dance_if_unsafe([0]*4,b,fresh=not stale))
            self.assertFalse(mapper.cancel_dance_if_unsafe([0]*4,b,fresh=not stale))

    def test_prepared_nominal_path_still_checks_actual_entry_seed(self):
        c=self.controller()
        c.prepare_dances()
        self.assertTrue(c.request('dance'))
        self.assertTrue(c.pose_audit['full_nominal_path_prepared'])
        seed=c.standing.copy()
        seed[:,2]+=.015
        plan=DanceTrajectory('dance',c.standing,seed)
        self.assertFalse(plan.audit(c.gait.legs,c.gait.q,c.dance_audits['dance'])['reachable'])


if __name__=='__main__':
    unittest.main()
