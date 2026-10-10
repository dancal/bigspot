import unittest
import numpy as np
import test_dance
from bigspot_sim.dance import DanceTrajectory,DANCES
from bigspot_sim.expressions import CUTES
from bigspot_sim.poses import BODY_POSES
from bigspot_sim.clap import CLAPS
from bigspot_sim.joy_mapping import JoyMapping,SELECTABLE


class RhythmMappingTests(unittest.TestCase):
    def test_default_rhythm_is_faster_and_has_planned_joint_reserve(self):
        c=test_dance.DanceTests().controller()
        q=c.gait.q.copy()
        rhythm=DanceTrajectory('dance_rhythm',c.standing,c.standing,c.rhythm_bpm)
        slow=DanceTrajectory('dance_big',c.standing,c.standing,c.dance_bpm)
        self.assertAlmostEqual(4*60/rhythm.bpm,8*60/slow.bpm/3.)
        self.assertLess(rhythm.duration,slow.duration/2.)
        audit=rhythm.audit(c.gait.legs,q)
        self.assertTrue(audit['reachable'],audit)
        self.assertLess(audit['peak_planned_joint_speed_rad_s'],1.)
        self.assertTrue(c.request('dance_rhythm'))
        self.assertFalse(c.enable_experiments)
        self.assertTrue(c.request('stop'))

    def test_rhythm_onset_and_phrase_transitions_are_c2(self):
        c=test_dance.DanceTests().controller()
        plan=DanceTrajectory('dance_rhythm',c.standing,c.standing,90.)
        h=1e-4
        for beat in [0.,2.,4.,6.,8.,10.,12.]:
            t=plan.ramp_seconds+beat*60/plan.bpm
            p=[plan.sample(t+i*h)[0] for i in [-2,-1,0,1,2]]
            self.assertLess(np.max(np.abs((p[2]-p[1])/h-(p[3]-p[2])/h)),.0001)
            self.assertLess(np.max(np.abs((p[2]-2*p[1]+p[0])/h**2-(p[4]-2*p[3]+p[2])/h**2)),.001)

    def test_other_rhythm_tempos_need_explicit_experiments(self):
        c=test_dance.DanceTests().controller(rhythm_bpm=75.)
        self.assertFalse(c.request('dance_rhythm'))
        c.enable_experiments=True
        self.assertTrue(c.request('dance_rhythm'))
        for bpm in [59.,91.,float('nan')]:
            with self.assertRaises(ValueError):
                test_dance.DanceTests().controller(rhythm_bpm=bpm)

    def test_all_grounded_actions_can_be_selected_then_executed_once(self):
        self.assertEqual(set(SELECTABLE),DANCES|CUTES|CLAPS|set(BODY_POSES)|{'turn_left','turn_right'})
        mapper=JoyMapping()
        axes=[0.]*4
        for i,action in enumerate(SELECTABLE):
            with self.subTest(action=action):
                self.assertEqual(mapper.selected_action,action)
                b=[0]*14;b[4]=b[5]=b[6]=1
                self.assertEqual(mapper.events(axes,b)[0],[action])
                self.assertEqual(mapper.events(axes,b)[0],[])
                self.assertTrue(mapper.cancel_dance_if_unsafe(axes,b,fresh=False))
                self.assertFalse(mapper.cancel_dance_if_unsafe(axes,b,fresh=False))
                b[6]=0;mapper.events(axes,b)
                b[13]=1
                self.assertEqual(mapper.events(axes,b)[0],[])
                self.assertEqual(mapper.events(axes,b)[0],[])
                self.assertEqual(mapper.selected_index,(i+1)%len(SELECTABLE))
                b[13]=0;mapper.events(axes,b)

    def test_selector_is_deadman_gated_stop_prioritised_and_blocks_velocity(self):
        mapper=JoyMapping()
        b=[0]*14;b[4]=b[13]=1
        self.assertEqual(mapper.events([0]*4,b)[0],[])
        self.assertEqual(mapper.selected_index,0)
        b[13]=0;mapper.events([0]*4,b)
        b[5]=b[13]=1
        self.assertEqual(mapper.events([0]*4,b)[0],[])
        self.assertEqual(mapper.selected_index,1)
        np.testing.assert_array_equal(mapper.velocity([0,1,0,0],b),[0,0,0])
        b[13]=0;mapper.events([0]*4,b)
        b[1]=b[6]=1
        self.assertEqual(mapper.events([0]*4,b),(['stop'],False))
        b=[0]*14;b[4]=b[5]=1
        self.assertGreater(mapper.velocity([0,1,0,0],b)[0],.06)

    def test_custom_catalog_rejects_unknown_duplicate_or_experimental_actions(self):
        for names in ['', 'cute,cute','jump','handstand_experimental','typo']:
            with self.assertRaises(ValueError):JoyMapping({'selection_actions':names})
        mapper=JoyMapping({'selection_actions':'cute_wave,look_down'})
        self.assertEqual(mapper.selected_action,'cute_wave')
        self.assertFalse(mapper.status()['physical_button_mapping_verified'])


if __name__=='__main__':unittest.main()
