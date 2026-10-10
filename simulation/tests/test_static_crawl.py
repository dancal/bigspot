import json
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'simulation/ros2_ws/src/bigspot_sim'))
from bigspot_sim.kinematics import Gait


class StaticCrawlTests(unittest.TestCase):
    def setUp(self):
        model=json.loads((ROOT/'simulation/ros2_ws/src/bigspot_sim/config/model.json').read_text())
        self.gait=Gait(model,period=6.,lift=.025,center_stance=True,stance_center_x=-.02,
                       static_crawl=True,support_margin=.025,transfer_fraction=.4)

    def test_one_swing_and_supported_world_targets_stay_planted(self):
        gait=self.gait
        previous=gait.rest.copy()
        for _ in range(900):
            targets=gait.targets(.016,[.01,0,0])
            world=targets.copy()
            world[:,:2]+=gait.static.body
            self.assertLessEqual(np.count_nonzero(targets[:,2]>gait.rest[:,2]+1e-6),1)
            if gait.static.running:
                for leg in range(4):
                    if leg!=gait.static.leg:
                        np.testing.assert_allclose(world[leg],previous[leg],atol=1e-9)
            previous=world

    def test_stop_lands_current_swing_and_keeps_final_footprint(self):
        gait=self.gait
        for _ in range(190):
            gait.targets(.016,[.01,0,0])
        for _ in range(150):
            stopped=gait.targets(.016,[0,0,0])
        self.assertFalse(gait.static.running)
        np.testing.assert_allclose(stopped[:,2],gait.rest[:,2],atol=1e-9)
        self.assertGreater(np.linalg.norm(stopped[:,:2]-gait.rest[:,:2]),.001)
        for _ in range(50):
            np.testing.assert_allclose(gait.targets(.016,[0,0,0]),stopped,atol=1e-9)


if __name__=='__main__':
    unittest.main()
