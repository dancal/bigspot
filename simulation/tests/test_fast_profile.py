import json
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
PACKAGE=ROOT/'simulation/ros2_ws/src/bigspot_sim'
sys.path.insert(0,str(PACKAGE))
from bigspot_sim.posture import PostureController


class FastProfileTests(unittest.TestCase):
    def setUp(self):
        model=json.loads((PACKAGE/'config/model.json').read_text())
        policy=json.loads((PACKAGE/'config/control_policy_fast.json').read_text())['parameters']
        self.controller=PostureController(model,**policy)

    def test_fast_profile_reaches_speed_and_command_loss_stops_with_bounded_angles(self):
        c=self.controller
        c.gait.q=np.array([leg.ik(p,q) for leg,p,q in zip(c.gait.legs,c.standing,c.gait.q)])
        c.transition('READY')
        previous=c.gait.q.ravel().copy()
        for step in range(400):
            q=np.array(c.step(.016,[.1,0,0],[0,0],[0,0],command_fresh=step<300))
            self.assertTrue(np.isfinite(q).all())
            self.assertLessEqual(np.max(abs(q-previous)),c.joint_rate*.016+1e-9)
            for leg,angles in zip(c.gait.legs,q.reshape(4,3)):
                self.assertTrue(np.all(angles>=leg.lower-1e-9))
                self.assertTrue(np.all(angles<=leg.upper+1e-9))
            if step==299:
                self.assertGreater(c.gait.speed[0],.1)
                self.assertEqual(c.state,'WALKING')
            previous=q
        self.assertEqual(c.state,'READY')
        np.testing.assert_allclose(c.gait.speed,0.,atol=1e-9)

    def test_fast_request_cannot_skip_standing_sequence(self):
        c=self.controller
        for _ in range(20):
            c.step(.016,[.1,0,0],[0,0],[0,0])
        self.assertEqual(c.state,'CROUCH')
        c.start()
        for _ in range(240):
            c.step(.016,[.1,0,0],[0,0],[0,0])
            self.assertEqual(c.state,'RISING')
            np.testing.assert_allclose(c.gait.speed,0.,atol=1e-9)


if __name__=='__main__':
    unittest.main()
