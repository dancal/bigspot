import unittest
import numpy as np
import test_dance as fixture
from bigspot_sim.dance_steps import StepDanceTrajectory


class StepDanceTests(unittest.TestCase):
    def controller(self):
        c=fixture.DanceTests().controller()
        c.observation['contact_forces_n']=[10.,10.,10.,10.]
        self.assertTrue(c.request('dance_steps'),c.action_result)
        return c

    def tick(self,c):
        return np.array(c.pose_step(.016,[0.,0.],[0.,0.],imu_fresh=True,joint_error=0.))

    def place(self,c,t):
        c.action_time=t
        targets=c.pose_trajectory.sample(t)[0]
        c.gait.q=np.array([l.ik(p,q) for l,p,q in zip(c.gait.legs,targets,c.gait.q)])

    def test_all_four_feet_lift_and_tap_after_transfer_with_reachable_tripods(self):
        c=self.controller();p=c.pose_trajectory
        self.assertTrue(c.pose_audit['reachable'])
        self.assertLess(c.pose_audit['peak_planned_joint_speed_rad_s'],1.)
        self.assertGreater(c.pose_audit['support_margin_m'],.012)
        for index,foot in enumerate(p.order):
            t=p.ramp_seconds+(index*p.step_beats+4.2)*60/p.bpm
            targets,_=p.sample(t);shift,_=p.body(t)
            motion=targets-(p.standing-shift)
            self.assertEqual(p.lift_at(t),foot)
            self.assertAlmostEqual(motion[foot,2],p.clearance)
            self.assertGreater(np.linalg.norm(motion[foot,:2]),.035)
            np.testing.assert_allclose(np.delete(motion,foot,axis=0),0.,atol=1e-12)
            self.assertEqual(len(p.support_polygon(t)),3)

    def test_no_lift_without_other_three_measured_stance_forces(self):
        c=self.controller();p=c.pose_trajectory
        boundary=p.ramp_seconds+2*60/p.bpm
        self.place(c,boundary)
        c.observation['contact_forces_n']=[10.,.5,10.,10.]
        self.tick(c)
        self.assertEqual(c.action_time,boundary)
        c.observation['contact_forces_n']=[10.]*4
        self.tick(c)
        self.assertGreater(c.action_time,boundary)

    def test_touchdown_wait_holds_transfer_until_continuous_measured_contact(self):
        c=self.controller();p=c.pose_trajectory
        checkpoint=p.landing_between(0.,p.duration)[2]
        self.place(c,checkpoint-.008)
        c.observation['contact_forces_n']=[0.,13.,13.,13.]
        for _ in range(20):self.tick(c)
        self.assertEqual(c.action_time,checkpoint)
        self.assertEqual(c.pose_landed_steps,[])
        frozen_shift=p.body(c.action_time)[0]
        c.observation['contact_forces_n'][0]=2.
        self.tick(c);self.tick(c)
        c.observation['contact_forces_n'][0]=0.
        self.tick(c)
        self.assertEqual(c.pose_landing_confirmed_s,0.)
        c.observation['contact_forces_n'][0]=2.
        for _ in range(6):self.tick(c)
        self.assertEqual(len(c.pose_landed_steps),1)
        self.assertEqual(c.pose_landed_steps[0]['foot'],0)
        np.testing.assert_allclose(p.body(c.action_time)[0],frozen_shift)
        self.tick(c)
        self.assertGreater(c.action_time,checkpoint)

    def test_cancel_lands_paw_before_recenter_and_caps_joint_command_speed(self):
        c=self.controller();p=c.pose_trajectory
        self.place(c,p.ramp_seconds+4.2*60/p.bpm)
        c.observation['contact_forces_n']=[0.,13.,13.,13.]
        self.assertTrue(c.request('stop'))
        for _ in range(70):
            previous=c.gait.q.ravel().copy();q=self.tick(c)
            self.assertLessEqual(np.max(abs(q-previous)),1.2*.016+1e-12)
        self.assertFalse(c.pose_recovery_landed)
        self.assertEqual(c.pose_recovery_recenter_time,0.)
        c.observation['contact_forces_n'][0]=2.
        self.tick(c)
        self.assertTrue(c.pose_recovery_landed)
        self.assertGreater(c.pose_recovery_recenter_time,0.)

    def test_sensor_loss_holds_commands_during_swing(self):
        for key,value in [('imu_valid',False),('contact_known',False),('joint_feedback_fresh',False)]:
            c=self.controller();self.place(c,4.8)
            previous=c.gait.q.ravel().copy();c.observation[key]=value
            q=c.pose_step(.016,[0.,0.],[0.,0.],imu_fresh=key!='imu_valid',joint_error=0.)
            self.assertEqual(c.state,'FAULT')
            np.testing.assert_array_equal(q,previous)

    def test_step_and_finish_boundaries_have_no_position_velocity_or_acceleration_snap(self):
        c=self.controller();p=c.pose_trajectory;h=.0001
        boundaries=[p.steps_end,p.steps_end+2.,p.steps_end+3.5,p.steps_end+5.5,p.duration]
        for index in range(4):
            boundaries.extend(p.ramp_seconds+(index*p.step_beats+beat)*60/p.bpm
                              for beat in [0.,2.,3.1,3.2,4.2,5.2,6.5,7.,8.5])
        for t in boundaries:
            a,b,d=[p.sample(t+x)[0] for x in [-h,0.,h]]
            self.assertLess(np.max(abs((b-a)/h-(d-b)/h)),.0002,t)
            before,after=[p.sample(t+x)[0] for x in [-2*h,2*h]]
            left=(b-2*a+before)/h**2
            right=(after-2*d+b)/h**2
            self.assertLess(np.max(abs(right-left)),.005,t)


if __name__=='__main__':unittest.main()
