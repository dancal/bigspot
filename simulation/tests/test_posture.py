import json
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'simulation/ros2_ws/src/bigspot_sim'))
from bigspot_sim.posture import PostureController, quaternion_rpy


class PostureTests(unittest.TestCase):
    def setUp(self):
        model=json.loads((ROOT/'simulation/ros2_ws/src/bigspot_sim/config/model.json').read_text())
        self.controller=PostureController(model)

    def test_start_does_not_allow_motion_before_standing_and_balance(self):
        c=self.controller
        for _ in range(30):
            c.step(.02,[.01,0,0],[0,0],[0,0])
        self.assertEqual(c.state,'CROUCH')
        c.start()
        for _ in range(180):
            c.step(.02,[.01,0,0],[0,0],[0,0])
            self.assertEqual(c.state,'RISING')
            np.testing.assert_allclose(c.gait.speed,0.)
        for _ in range(25):
            c.step(.02,[0,0,0],[0,0],[0,0])
        self.assertEqual(c.state,'BALANCING')
        for _ in range(60):
            c.step(.02,[0,0,0],[0,0],[0,0])
        self.assertEqual(c.state,'READY')

    def test_stale_command_decelerates_and_stale_imu_holds(self):
        c=self.controller
        c.transition('READY')
        for _ in range(50):
            c.step(.02,[.01,0,0],[0,0],[0,0])
        self.assertEqual(c.state,'WALKING')
        for _ in range(100):
            c.step(.02,[.01,0,0],[0,0],[0,0],command_fresh=False)
        self.assertEqual(c.state,'READY')
        previous=c.gait.q.ravel().copy()
        actual=c.step(.02,[.01,0,0],[0,0],[0,0],imu_fresh=False)
        self.assertEqual(c.state,'FAULT')
        np.testing.assert_array_equal(actual,previous)

    def test_quaternion_rejects_invalid_and_preserves_known_roll(self):
        angle=.2
        np.testing.assert_allclose(quaternion_rpy([np.sin(angle/2),0,0,np.cos(angle/2)]),[angle,0],atol=1e-9)
        for q in [[0,0,0,0],[0,0,0,2],[float('nan'),0,0,1]]:
            with self.assertRaises(ValueError):
                quaternion_rpy(q)

    def test_tilt_fault_never_commands_another_pose(self):
        c=self.controller
        before=c.gait.q.ravel().copy()
        result=c.step(.02,[.01,0,0],[np.deg2rad(40),0],[0,0])
        self.assertEqual(c.state,'FAULT')
        np.testing.assert_array_equal(result,before)


if __name__=='__main__':
    unittest.main()
