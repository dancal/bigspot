import unittest
import numpy as np
import test_dance
from bigspot_sim.clap import SeatedClapTrajectory
from bigspot_sim.joy_mapping import JoyMapping,SELECTABLE


class SeatedClapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        c=test_dance.DanceTests().controller()
        cls.audit=SeatedClapTrajectory(c.standing,c.standing,c.gait.legs).audit(c.gait.legs,c.gait.q)

    def control(self):
        c=test_dance.DanceTests().controller();c.clap_audit=self.audit
        c.observation['seat_feedback']={'fresh':True,'rear_support_force_n':40.,'rear_support_margin_m':.03,
            'paw_contact_force_n':0.,'paw_gap_m':.20,'paw_contact_tip_offset_m':1.,'front_paw_height_m':[.08,.08]}
        c.observation['contact_forces_n']=[10.]*4
        return c

    def at_clap(self):
        c=self.control();self.assertTrue(c.request('seated_clap'),c.action_result)
        c.action_time=10.
        targets,rpy=c.clap_trajectory.sample(c.action_time);c.pose_last_rpy=rpy.copy()
        c.gait.q=np.array([leg.ik(p,q) for leg,p,q in zip(c.gait.legs,targets,c.gait.q)])
        c.observation['contact_forces_n']=[0.,0.,20.,20.];c.observation['support_count']=2
        return c

    def step(self,c,**kwargs):
        return c.step(.016,[0,0,0],[0,c.clap_trajectory.pitch],[0,0],**kwargs)

    def test_path_reachable_without_expanding_limits_and_with_joint_speed_reserve(self):
        self.assertTrue(self.audit['reachable'],self.audit)
        self.assertLess(self.audit['peak_planned_joint_speed_rad_s'],1.)
        c=self.control();p=SeatedClapTrajectory(c.standing,c.standing,c.gait.legs)
        for t in np.linspace(p.landing_end,p.duration,100):
            shift,rpy,_,q=p.return_body(t-p.landing_end)
            for leg,angles in zip(c.gait.legs[2:],q):
                self.assertTrue(np.all(angles>=leg.lower) and np.all(angles<=leg.upper))
            self.assertGreater(.28+shift[2],.12)
        self.assertFalse(self.audit['mechanical_collision_verified'])

    def test_start_needs_contact_telemetry_and_lift_waits_for_rear_support(self):
        c=self.control();c.observation['seat_feedback']['fresh']=False
        self.assertFalse(c.request('seated_clap'))
        c.observation['seat_feedback']['fresh']=True
        self.assertTrue(c.request('seated_clap'))
        c.action_time=5.99;c.observation['seat_feedback']['rear_support_margin_m']=-.02
        self.step(c);self.assertEqual(c.action_time,5.99)
        self.assertEqual(c.state,'SEATED_LOWER')
        c.observation['seat_feedback']['rear_support_margin_m']=.03
        self.step(c);self.assertGreater(c.action_time,6.)
        self.assertEqual(c.state,'SEATED_PAWS_UP')

    def test_claps_need_actual_tip_contact_and_separation_between_contacts(self):
        c=self.at_clap();s=c.observation['seat_feedback'];s['paw_gap_m']=.025
        self.step(c);self.assertEqual(c.clap_count,0)
        s['paw_contact_force_n']=1.;s['paw_contact_tip_offset_m']=.12
        self.step(c);self.assertEqual(c.clap_count,0)
        s['paw_contact_tip_offset_m']=.01;s['front_paw_height_m']=[.01,.01]
        self.step(c);self.assertEqual(c.clap_count,0)
        s['front_paw_height_m']=[.05,.05]
        self.step(c);self.assertEqual(c.clap_count,1)
        self.step(c);self.assertEqual(c.clap_count,1)
        s['paw_gap_m']=.20;s['paw_contact_force_n']=0.
        self.step(c);s['paw_gap_m']=.025;s['paw_contact_force_n']=1.
        self.step(c);self.assertEqual(c.clap_count,2)

    def test_critical_sensor_loss_holds_last_command_and_preserves_first_fault(self):
        for sensor in ['imu','joint','contact','seat']:
            c=self.at_clap();old=c.gait.q.ravel().copy()
            if sensor=='joint':c.observation['joint_feedback_fresh']=False
            if sensor=='contact':c.observation['contact_known']=False
            if sensor=='seat':c.observation['seat_feedback']['fresh']=False
            command=self.step(c,imu_fresh=sensor!='imu')
            self.assertEqual(c.state,'FAULT',sensor);np.testing.assert_array_equal(command,old)
            original=c.fault;c.step(.016,[0,0,0],[0,1.],[0,0])
            self.assertEqual(c.fault,original)

    def test_cancel_waits_for_both_paw_landings_before_changing_seated_attitude(self):
        c=self.at_clap();self.assertTrue(c.request('stop'))
        for _ in range(260):
            old=c.gait.q.copy();self.step(c)
            self.assertLessEqual(np.max(np.abs(c.gait.q-old)),1.2*.016+1e-9)
            self.assertFalse(c.clap_landed)
            self.assertAlmostEqual(c.pose_last_rpy[1],c.clap_trajectory.pitch)
        c.observation['contact_forces_n']=[3.,3.,17.,17.]
        self.step(c);self.assertTrue(c.clap_landed)
        self.assertGreater(c.clap_recenter_s,0.)

    def test_direct_buttons_deadman_edge_stop_and_new_catalog(self):
        m=JoyMapping();b=[0]*14;b[8]=1
        self.assertEqual(m.events([0]*4,b)[0],[])
        b[8]=0;m.events([0]*4,b);b[5]=b[8]=1
        self.assertEqual(m.events([0]*4,b)[0],['seated_clap'])
        self.assertEqual(m.events([0]*4,b)[0],[])
        b[5]=0;self.assertTrue(m.cancel_dance_if_unsafe([0]*4,b))
        b=[0]*14;b[5]=b[12]=1
        self.assertEqual(m.events([0]*4,b)[0],['dance_funk'])
        b[1]=1;self.assertEqual(m.events([0]*4,b),(['stop'],False))
        self.assertEqual(len(SELECTABLE),30)


if __name__=='__main__':unittest.main()
