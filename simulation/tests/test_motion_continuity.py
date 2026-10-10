"""Motion boundary regressions: stance speed, landing feedback and ROS timing."""
import unittest
import numpy as np
import test_dance
from bigspot_sim.awareness import SensorConstraints


class ContinuityTests(unittest.TestCase):
    def controller(self):return test_dance.DanceTests().controller()

    def test_horizontal_swing_joins_stance_at_same_velocity_and_acceleration(self):
        g=self.controller().gait;g.smooth_velocity=False;g.blend=1.
        v=np.array([.10,.03,.20]);g.speed=v.copy()
        h=1e-6
        def sample(phase):
            g.phase=(phase-.001/g.period)%1
            return g.targets(.001,v)[0,:2]
        for boundary in [g.trot_duty,1.]:
            left,center,right=[sample(boundary+x*h) for x in [-1,0,1]]
            a=(center-left)/(h*g.period);b=(right-center)/(h*g.period)
            expected=-np.array([v[0]-v[2]*g.rest[0,1],v[1]+v[2]*g.rest[0,0]])
            np.testing.assert_allclose(a,expected,atol=1e-5)
            np.testing.assert_allclose(b,expected,atol=1e-5)
            self.assertLess(np.max(abs(a-b)),1e-5)

    def test_salute_cancel_holds_transfer_until_measured_paw_landing(self):
        c=self.controller();self.assertTrue(c.request('salute_right'))
        for _ in range(155):c.step(.016,[0,0,0],[0,0],[0,0])
        c.observation.update(support_count=3,contact_forces_n=[0.,10.,10.,10.])
        c.request('stop')
        for _ in range(70):c.step(.016,[0,0,0],[0,0],[0,0])
        self.assertFalse(c.pose_recovery_landed)
        self.assertEqual(c.pose_recovery_recenter_time,0.)
        self.assertEqual(c.state,'POSE_RECOVERY')
        c.observation.update(support_count=4,contact_forces_n=[10.]*4)
        for _ in range(160):
            c.step(.016,[0,0,0],[0,0],[0,0])
            if not c.action:break
        self.assertEqual(c.state,'READY');self.assertTrue(c.pose_recovery_landed)
        self.assertTrue(c.action_events[-1]['result'].startswith('Cancelled'))

    def test_normal_salute_cannot_transfer_back_before_landing_feedback(self):
        c=self.controller();self.assertTrue(c.request('salute_left'))
        c.observation.update(support_count=3,contact_forces_n=[10.,0.,10.,10.])
        for _ in range(245):c.step(.016,[0,0,0],[0,0],[0,0])
        self.assertAlmostEqual(c.action_time,3.5)
        self.assertIsNotNone(c.pose_landing_gate)
        c.observation.update(support_count=4,contact_forces_n=[10.]*4)
        for _ in range(10):c.step(.016,[0,0,0],[0,0],[0,0])
        self.assertIsNone(c.pose_landing_gate);self.assertGreater(c.action_time,3.5)

    def test_eighty_ms_support_loss_has_same_boundary_with_ros_roundoff(self):
        c=self.controller();self.assertTrue(c.request('dance_rhythm'))
        c.observation['support_count']=2
        for _ in range(5):c.step(.016+1e-15,[0,0,0],[0,0],[0,0])
        self.assertFalse(c.pose_cancelled)
        c.step(.016,[0,0,0],[0,0],[0,0]);self.assertTrue(c.pose_cancelled)
        guard=SensorConstraints([.22,.08,.6]);o={'contact_known':True,'support_count':0}
        for _ in range(5):guard.limit([.1,0,0],.016+1e-15,o)
        self.assertNotIn('support_lost',guard.status['reasons'])
        guard.limit([.1,0,0],.016,o);self.assertIn('support_lost',guard.status['reasons'])

    def test_stopped_filter_does_not_allow_a_pose_until_sensors_settle(self):
        c=self.controller();c.state='STOPPING';c.gait.speed[:]=0.;c.gait.blend=0.
        c.observation.update(support_count=2,contact_forces_n=[0.,0.,10.,10.])
        for _ in range(40):c.step(.016,[0,0,0],[0,0],[0,0])
        self.assertEqual(c.state,'STOPPING');self.assertFalse(c.request('look_up'))
        c.observation.update(support_count=4,contact_forces_n=[10.]*4,joint_feedback_fresh=False)
        for _ in range(40):c.step(.016,[0,0,0],[0,0],[0,0])
        self.assertEqual(c.state,'STOPPING')
        c.observation['joint_feedback_fresh']=True
        for _ in range(40):c.step(.016,[0,0,0],[0,0],[.2,0])
        self.assertEqual(c.state,'STOPPING')
        for _ in range(35):c.step(.016,[0,0,0],[0,0],[0,0])
        self.assertEqual(c.state,'READY');self.assertTrue(c.request('look_up'))


if __name__=='__main__':unittest.main()
