import json
import sys
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import types

# Pure gait tests disable IMU; provide only the ROS clock dependency used by legacy PID.
# This is not a ROS runtime integration test.
if 'rospy' not in sys.modules:
    class _Clock:
        @staticmethod
        def now():
            return 0
    sys.modules['rospy'] = types.SimpleNamespace(Time=_Clock)

ROOT = Path(__file__).resolve().parents[2]
PKG = ROOT/'simulation/ros2_ws/src/bigspot_sim'
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(ROOT/'src/bigspot_controller/scripts'))
from bigspot_sim.kinematics import Gait, Leg
from RobotController.TrotGaitController import TrotGaitController
from RobotController.StateCommand import State, Command, BehaviorState
from InverseKinematics.robot_IK import InverseKinematics


class RegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = json.loads((PKG/'config/model.json').read_text())

    def test_legacy_laydown_state_exists(self):
        self.assertEqual(BehaviorState.LAYDOWN.value, 6)

    def test_legacy_trot_runs_all_four_legs(self):
        stance = np.array([[.2,.2,-.2,-.2],[-.1,.1,-.1,.1],[-.25]*4])
        controller = TrotGaitController(stance, .18, .24, .02, False)
        state, command = State(.25), Command(.25)
        state.foot_locations = stance.copy()
        command.velocity[:] = [.01, .005]
        for _ in range(controller.phase_length*3):
            feet = controller.run(state, command)
            self.assertEqual(feet.shape, (3,4))
            self.assertTrue(np.isfinite(feet).all())

    def test_legacy_stance_matches_command_velocity(self):
        stance = np.array([[.2,.2,-.2,-.2],[-.1,.1,-.1,.1],[-.25]*4])
        controller = TrotGaitController(stance,.18,.24,.02,False)
        state, command = State(.25), Command(.25)
        state.foot_locations = stance.copy()
        command.velocity[:] = [.01,.005]
        p = controller.stanceController.next_foot_location(0,state,command)
        np.testing.assert_allclose(p[:2], stance[:2,0]-np.array([.01,.005])*.02)

    def test_legacy_ik_rejects_unreachable_instead_of_fake_angle(self):
        with self.assertRaises(ValueError):
            InverseKinematics.check_domain(1.2)
        with self.assertRaises(ValueError):
            InverseKinematics.check_domain(float('nan'))

    def test_legacy_ik_law_of_cosines(self):
        ik = InverseKinematics([.4,.14],[0,.04,.183,.2])
        feet = np.array([[.2,.2,-.2,-.2],[-.11,.11,-.11,.11],[-.25]*4])
        angles = ik.inverse_kinematics(feet,0,0,0,0,0,0)
        # Shoulder lateral offset is .04, so planar distance is exactly .25.
        expected = (.25**2-.183**2-.2**2)/(2*.183*.2)
        self.assertAlmostEqual(np.cos(angles[2]), expected, places=6)

    def test_urdf_is_connected_and_has_12_bounded_dofs(self):
        tree = ET.parse(PKG/'urdf/bigspot.urdf')
        links = {l.get('name') for l in tree.findall('link')}
        children = set()
        for j in tree.findall('joint'):
            self.assertIn(j.find('parent').get('link'),links)
            child = j.find('child').get('link')
            self.assertIn(child,links)
            self.assertNotIn(child,children)
            children.add(child)
        self.assertEqual(links-children, {'base_link'})
        self.assertEqual(len(tree.findall("joint[@type='revolute']")),12)
        for l in tree.findall('link'):
            inertia = l.find('inertial/inertia')
            if inertia is not None:
                a = {k:float(v) for k,v in inertia.attrib.items()}
                tensor = [[a['ixx'],a['ixy'],a['ixz']],[a['ixy'],a['iyy'],a['iyz']],[a['ixz'],a['iyz'],a['izz']]]
                self.assertTrue(np.all(np.linalg.eigvalsh(tensor)>0))

    def test_urdf_frame_shift_preserves_joint_geometry(self):
        old = ET.parse(ROOT/'src/bigspot_description/urdf/bigspot.xacro')
        new = ET.parse(PKG/'urdf/bigspot.urdf')
        shift = np.array(self.model['root_shift_m'])
        for j in old.findall('joint'):
            n = new.find(f"joint[@name='{j.get('name')}']")
            old_p = np.fromstring(j.find('origin').get('xyz'),sep=' ')
            new_p = np.fromstring(n.find('origin').get('xyz'),sep=' ')
            expected = old_p-shift if j.find('parent').get('link')=='base_link' else old_p
            np.testing.assert_allclose(new_p,expected,atol=1e-12)

    def test_urdf_ik_roundtrip(self):
        for desc, seed in zip(self.model['legs'],self.model['initial_angles']):
            leg = Leg(desc)
            q = np.array(seed)+np.array([.02,.02,.02])
            target = leg.fk(q)
            actual = leg.ik(target,seed,tolerance=1e-5)
            np.testing.assert_allclose(leg.fk(actual),target,atol=1e-5)
            with self.assertRaises(ValueError):
                leg.ik([10,10,10],seed)

    def test_both_gaits_are_finite_bounded_and_stop(self):
        for mode in ['crawl','trot']:
            gait = Gait(self.model,mode)
            previous = gait.q.ravel().copy()
            for i in range(800):
                command = [.015,.01,.1] if i<400 else [0,0,0]
                q = np.array(gait.step(.02,command))
                self.assertTrue(np.isfinite(q).all())
                self.assertLessEqual(np.max(abs(q-previous)),.03+1e-12)
                for leg,angles in zip(gait.legs,q.reshape(4,3)):
                    self.assertTrue(np.all(angles>=leg.lower))
                    self.assertTrue(np.all(angles<=leg.upper))
                previous = q
            self.assertEqual(gait.blend,0)
            np.testing.assert_allclose(gait.speed,0,atol=1e-12)


if __name__ == '__main__':
    unittest.main()
