import unittest
import numpy as np
import test_dance as fixture
from bigspot_sim.funk import FunkDancePlan
from bigspot_sim.joy_mapping import JoyMapping,SELECTABLE
from bigspot_sim.kinematics import rotation


class FunkTests(unittest.TestCase):
    def controller(self,**kw):
        c=fixture.DanceTests().controller(**kw)
        c.observation['contact_forces_n']=[10.]*4
        return c

    def test_full_beat_locked_path_reachable_inside_existing_walking_rate_ceiling(self):
        c=self.controller();p=FunkDancePlan();a=p.audit(c.gait,c.gait.q)
        self.assertTrue(a['reachable'],a);self.assertLessEqual(a['peak_planned_joint_speed_rad_s'],3.8)
        self.assertLessEqual(p.joint_rate,c.joint_rate)
        self.assertEqual(p.beat_seconds,.5);self.assertEqual(p.period,1.)

    def test_zero_velocity_march_alternates_diagonal_pairs_and_normal_idle_stays_planted(self):
        c=self.controller();g=c.gait;p=FunkDancePlan();p.configure(g)
        for phase,air in [(.3,[1,2]),(.8,[0,3])]:
            g.phase=phase-.016;g.blend=1.;g.speed[:]=0.
            feet=g.targets(.016,[0.,0.,0.],marching=True)
            self.assertEqual(np.flatnonzero(feet[:,2]-g.rest[:,2]>.010).tolist(),air)
        g.blend=0.;g.speed[:]=0.;g.speed_acceleration[:]=0.
        np.testing.assert_allclose(g.targets(.016,[0.,0.,0.]),g.rest)

    def test_shuffle_turn_and_body_tracks_overlap_and_reverse_in_each_phrase(self):
        p=FunkDancePlan();p.configure(self.controller().gait)
        for start,axis in [(32,1),(40,0),(48,2)]:
            a=p.intro+(start+2)*p.beat_seconds;b=p.intro+(start+6)*p.beat_seconds
            self.assertGreater(p.velocity(a)[axis],.04)
            self.assertLess(p.velocity(b)[axis],-.04)
            self.assertGreater(np.linalg.norm(p.body(a+.15)[1]),.01)
        np.testing.assert_array_equal(p.velocity(p.main_end),np.zeros(3))

    def test_start_requires_trot_profile_four_contacts_imu_joints_and_front_clearance(self):
        for key,value in [('imu_valid',False),('contact_known',False),('joint_feedback_fresh',False),('support_count',3)]:
            c=self.controller();c.observation[key]=value;self.assertFalse(c.request('dance_funk'),key)
        c=self.controller();c.gait.mode='crawl';self.assertFalse(c.request('dance_funk'))
        c=self.controller();c.observation['ultrasonic']['distance_m']=[.25,4.];self.assertFalse(c.request('dance_funk'))

    def test_hip_and_front_emphasis_are_distinct_using_cad_reference_points(self):
        p=FunkDancePlan();p.configure(self.controller().gait)
        points=np.array([p.front_pivot,p.rear_pivot])
        for name,axis,emphasis in [('hip_wiggle',1,1),('head_shake',1,0),('head_bob',2,0)]:
            start=p.intro+p.phrases.index(name)*8*p.beat_seconds
            measured=[]
            for t in np.arange(start,start+2,.016):
                shift,rpy=p.body(t)
                R=rotation([0,0,1],rpy[2])@rotation([0,1,0],rpy[1])@rotation([1,0,0],rpy[0])
                measured.append(shift+points@R.T)
            travel=np.ptp(measured,axis=0)[:,axis]
            self.assertGreater(travel[emphasis],.035,name)
            self.assertGreater(travel[emphasis],2*travel[1-emphasis],name)
        self.assertEqual(p.beat_seconds,.5)

    def test_body_track_transitions_and_exit_are_continuous(self):
        p=FunkDancePlan();p.configure(self.controller().gait)
        h=1e-5
        for boundary in [p.intro+8*p.beat_seconds*i for i in range(1,len(p.phrases))]:
            a=np.concatenate(p.body(boundary-h));b=np.concatenate(p.body(boundary));d=np.concatenate(p.body(boundary+h))
            self.assertLess(np.max(abs(a-d)),2e-5)
            self.assertLess(np.max(abs((b-a)/h-(d-b)/h)),.001)
        np.testing.assert_allclose(np.concatenate(p.body(p.duration)),np.zeros(6),atol=1e-12)

    def test_unvalidated_tempo_rejected_without_experimental_opt_in(self):
        c=self.controller(funk_bpm=110.)
        self.assertFalse(c.request('dance_funk'));self.assertIn('validated 120',c.action_result)

    def test_cancel_decelerates_before_restoring_gait_and_all_joints_respect_rate(self):
        c=self.controller();old=(c.gait.period,c.gait.lift,c.gait.trot_duty)
        self.assertTrue(c.request('dance_funk'),c.action_result)
        for _ in range(140):c.step(.016,[0,0,0],[0,0],[0,0],imu_fresh=True,joint_error=0.)
        self.assertTrue(c.request('stop'));self.assertNotEqual((c.gait.period,c.gait.lift,c.gait.trot_duty),old)
        for _ in range(375):
            before=c.gait.q.ravel().copy();q=c.step(.016,[0,0,0],[0,0],[0,0],imu_fresh=True,joint_error=0.)
            self.assertLessEqual(np.max(abs(q-before)),3.8*.016+1e-12)
            if not c.action:break
        self.assertEqual(c.state,'READY');self.assertEqual((c.gait.period,c.gait.lift,c.gait.trot_duty),old)
        self.assertTrue(c.action_events[-1]['result'].startswith('Cancelled'))

    def test_critical_sensor_loss_holds_last_command_and_emits_one_fault(self):
        for key in ['imu_valid','contact_known','joint_feedback_fresh']:
            c=self.controller();self.assertTrue(c.request('dance_funk'))
            c.step(.016,[0,0,0],[0,0],[0,0],imu_fresh=True,joint_error=0.)
            q=c.gait.q.ravel().copy();c.observation[key]=False
            out=c.step(.016,[0,0,0],[0,0],[0,0],imu_fresh=key!='imu_valid',joint_error=0.)
            self.assertEqual(c.state,'FAULT');np.testing.assert_array_equal(out,q)
            self.assertEqual(len(c.action_events),1)

    def test_support_debounce_preserves_64ms_boundary_despite_ros_clock_roundoff(self):
        c=self.controller();self.assertTrue(c.request('dance_funk'))
        c.observation['support_count']=1;c.observation['contact_forces_n']=[10.,0.,0.,0.]
        for _ in range(4):
            c.step(.016+1e-15,[0,0,0],[0,0],[0,0],imu_fresh=True,joint_error=0.)
        self.assertFalse(c.funk_cancelled,'64 ms plus clock roundoff must not become an early cancellation')
        c.step(.016,[0,0,0],[0,0],[0,0],imu_fresh=True,joint_error=0.)
        self.assertTrue(c.funk_cancelled);self.assertEqual(c.funk_cancel_reason,'insufficient_support')

    def test_cancel_finishes_body_track_before_returning_to_normal_standing_controller(self):
        c=self.controller();self.assertTrue(c.request('dance_funk'))
        for _ in range(420):c.step(.016,[0,0,0],[0,0],[0,0],imu_fresh=True,joint_error=0.)
        self.assertGreater(np.linalg.norm(c.funk_body[1]),.03)
        c.request('stop')
        for _ in range(160):
            c.step(.016,[0,0,0],[0,0],[0,0],imu_fresh=True,joint_error=0.)
            if not c.action:break
        self.assertEqual(c.state,'READY');self.assertGreaterEqual(c.funk_recovery_s,1.5)
        np.testing.assert_allclose(np.concatenate(c.funk_body),np.zeros(6),atol=1e-12)

    def test_default_button_is_edge_triggered_deadman_gated_and_old_dances_remain_selectable(self):
        m=JoyMapping();b=[0]*14;b[12]=1
        self.assertEqual(m.events([0]*4,b)[0],[])
        b[12]=0;m.events([0]*4,b);b[5]=b[12]=1
        self.assertEqual(m.events([0]*4,b)[0],['dance_funk']);self.assertEqual(m.events([0]*4,b)[0],[])
        self.assertIn('dance_steps',SELECTABLE);self.assertIn('dance_show',SELECTABLE);self.assertEqual(len(SELECTABLE),30)


if __name__=='__main__':unittest.main()
