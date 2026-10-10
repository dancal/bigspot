import unittest
from pathlib import Path
import numpy as np
from bigspot_autonomy.design import load_design
from bigspot_autonomy.control_core import ControlCore, JOINT_NAMES, ordered_joints
from bigspot_autonomy.sim_backend import EffortWatchdog
from bigspot_autonomy.camera_geometry import camera_matrices, depth_points, WIDTH, HEIGHT, FX, BASELINE_M

DESIGN = load_design(Path(__file__).resolve().parents[1]/'config/pi4_design.json')

class TransportTests(unittest.TestCase):
    def setUp(self):
        self.core = ControlCore(DESIGN)
        self.q = np.tile([0.,-1.43,2.86],4)

    def sensors(self, t):
        self.core.joints(JOINT_NAMES,self.q,np.zeros(12),t)
        self.core.imu([0,0,0,1],[0,0,0],t)

    def test_explicit_arming_requires_live_sensors(self):
        self.assertFalse(self.core.enable(True,0));self.sensors(1)
        self.assertFalse(self.core.step(.005,1).torque_enabled)
        self.assertTrue(self.core.enable(True,1))
        self.assertTrue(self.core.step(.005,1).torque_enabled)

    def test_sensor_fault_latches_and_cannot_rearm(self):
        self.sensors(1);self.core.enable(True,1)
        output=self.core.step(.005,1.2)
        self.assertEqual(output.state,'FAULT');np.testing.assert_array_equal(output.efforts,0)
        self.core.enable(False,1.2);self.sensors(1.3)
        self.assertFalse(self.core.enable(True,1.3))

    def test_replayed_future_and_delayed_sensor_stamps_rejected(self):
        self.assertTrue(self.core.accept_stamp('imu',1,1.01))
        for stamp,now in [(1,1.02),(.9,1.02),(1.2,1.1),(1.1,1.3),(float('nan'),1.3)]:
            self.assertFalse(self.core.accept_stamp('imu',stamp,now))

    def test_invalid_measurements_do_not_refresh_sensor_health(self):
        self.sensors(1)
        with self.assertRaises(ValueError):self.core.joints(JOINT_NAMES,[float('nan')]*12,[0]*12,2)
        with self.assertRaises(ValueError):self.core.imu([0]*4,[0]*3,2)
        self.assertEqual(self.core.supervisor.stamps['joints'],1)
        self.assertEqual(self.core.supervisor.stamps['imu'],1)

    def test_joint_order_is_named_and_duplicates_rejected(self):
        q,v=ordered_joints(list(reversed(JOINT_NAMES)),self.q[::-1],[0]*12)
        np.testing.assert_array_equal(q,self.q)
        with self.assertRaises(ValueError):ordered_joints([JOINT_NAMES[0]]*12,self.q,[0]*12)

    def test_clock_jump_disables_torque(self):
        self.sensors(1);self.core.enable(True,1)
        self.assertFalse(self.core.step(-.01,1).torque_enabled)

    def test_autonomy_stops_on_stale_odometry(self):
        self.core=ControlCore(DESIGN,autonomous=True)
        self.sensors(1);self.core.command([.2,0,0],1)
        self.core.supervisor.update('camera',1)
        self.core.supervisor.update('odom',.5)
        self.core.state='READY';self.core.target=self.core.gait.stand()
        output=self.core.step(.005,1)
        self.assertEqual(output.reason,'STALE_ODOMETRY')
        self.assertTrue(output.torque_enabled)
        np.testing.assert_array_equal(self.core.gait.velocity,0)

    def test_watchdog_uses_wall_time_when_simulation_clock_stalls(self):
        watchdog=EffortWatchdog(DESIGN)
        self.assertTrue(watchdog.receive_target(JOINT_NAMES,self.q,[0]*12,1,1,10))
        np.testing.assert_array_equal(watchdog.motor_efforts(1,10.2,self.q,np.zeros(12)),0)
        self.assertTrue(watchdog.expired)
        self.assertFalse(watchdog.receive_target(JOINT_NAMES,self.q,[0]*12,1.1,1.1,10.21))

    def test_empty_effort_disables_local_motor_pd_atomically(self):
        watchdog=EffortWatchdog(DESIGN)
        self.assertTrue(watchdog.receive_target(JOINT_NAMES,self.q,[],1,1,10))
        np.testing.assert_array_equal(watchdog.motor_efforts(1,10,self.q+.05,np.zeros(12)),0)

    def test_invalid_target_and_peak_effort_rejected(self):
        watchdog=EffortWatchdog(DESIGN)
        self.assertFalse(watchdog.receive_target(JOINT_NAMES,[0]*12,[0]*12,1,1,10))
        self.assertFalse(watchdog.receive_target(JOINT_NAMES,self.q,[100]*12,1,1,10))
        self.assertFalse(watchdog.receive_target(JOINT_NAMES,self.q,[0]*12,1,1,float('nan')))

    def test_camera_depth_and_stereo_baseline(self):
        _,left=camera_matrices(False);_,right=camera_matrices(True)
        self.assertEqual(left[3],0.);self.assertAlmostEqual(right[3],-FX*BASELINE_M)
        depth=np.full((HEIGHT,WIDTH),np.nan,np.float32);depth[0,0]=1
        points=depth_points(depth)
        self.assertEqual(points.shape,(1,3));self.assertAlmostEqual(float(points[0,2]),1.)

if __name__=='__main__':unittest.main()
