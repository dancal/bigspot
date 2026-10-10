import math,sys,unittest,struct
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
deps=ROOT.parent/'bigspot/simulation/generated/python_deps'
if sys.platform=='win32' and deps.exists():sys.path.insert(0,str(deps))
import numpy as np
from bigspot_autonomy.kinematics import leg_ik,leg_fk
from bigspot_autonomy.design import load_design,mass_and_cost,variant
from bigspot_autonomy.can_protocol import control,feedback,Frame,stop,bus_load
from bigspot_autonomy.safety import Supervisor
from bigspot_autonomy.navigation import GridPlanner
from bigspot_autonomy.gait import Gait

class Tests(unittest.TestCase):
    def test_waiting_for_first_sensor_does_not_latch(self):
        s=Supervisor();self.assertEqual(s.evaluate(0).reason,'WAITING_SENSORS')
        for source in ('imu','joints','command'):s.update(source,1.)
        self.assertTrue(s.evaluate(1).torque_enabled)
    def test_ik_roundtrip_both_sides(self):
        rng=np.random.default_rng(12)
        for side in (-1,1):
            for _ in range(100):
                p=(rng.uniform(-.035,.055),side*.055+rng.uniform(-.015,.015),rng.uniform(-.24,-.18))
                q=leg_ik(p,.14,.14,.055,side)
                np.testing.assert_allclose(leg_fk(q,.14,.14,.055,side),p,atol=1e-9)
    def test_outside_workspace(self):
        with self.assertRaises(ValueError):leg_ik((0,.055,-1),.14,.14,.055,1)
    def test_can_zero_golden_vector(self):
        f=control('RS02',7,0,0,0,0)
        self.assertEqual(f.arbitration_id,0x017fff07)
        self.assertEqual(f.data.hex(),'7fff7fff00000000')
        self.assertTrue(f.is_extended_id)
    def test_can_endpoints_and_model_difference(self):
        self.assertEqual(control('RS00',1,4*math.pi,33,500,5,14).data,b'\xff'*8)
        with self.assertRaises(ValueError):control('RS00',1,0,34,0,0)
        with self.assertRaises(ValueError):control('RS02',1,float('nan'),0,0,0)
        self.assertEqual(stop(1).data,b'\0'*8)
    def test_feedback_source_and_temperature(self):
        f=Frame((2<<24)|(2<<22)|(5<<8)|253,struct.pack('>4H',32768,32768,32768,523))
        r=feedback('RS02',f);self.assertEqual(r['motor_id'],5);self.assertEqual(r['mode'],2)
        self.assertEqual(r['temperature_c'],52.3);self.assertLess(abs(r['torque']),.001)
    def test_fault_latches_and_camera_keeps_standing(self):
        s=Supervisor()
        for source in ('imu','joints','command','camera'):s.update(source,1.)
        self.assertTrue(s.evaluate(1.,True).motion_enabled)
        self.assertFalse(s.evaluate(1.151).torque_enabled)
        for source in ('imu','joints'):s.update(source,1.2)
        self.assertFalse(s.evaluate(1.2).torque_enabled)
        s.reset();s.update('command',1.4);s.update('imu',1.4);s.update('joints',1.4)
        result=s.evaluate(1.4,True);self.assertTrue(result.torque_enabled);self.assertFalse(result.motion_enabled)
    def test_command_timeout_and_estop(self):
        s=Supervisor();s.update('command',0.)
        s.update('imu',1.);s.update('joints',1.)
        self.assertTrue(s.evaluate(1.).torque_enabled);self.assertFalse(s.evaluate(1.).motion_enabled)
        s.estop=True;self.assertFalse(s.evaluate(1.).torque_enabled)
    def test_no_mixed_quantity_discount(self):
        d=load_design();self.assertEqual(mass_and_cost(d)['motor_cost_usd'],1660)
        self.assertEqual(mass_and_cost(variant(d,'all02',.14))['motor_cost_usd'],1656)
    def test_planner_obstacle_inflation(self):
        p=GridPlanner();p.integrate(np.array([[1.,0.,.3]]));route=p.path((0,0),(2,0))
        self.assertTrue(route);self.assertTrue(all(np.linalg.norm(x-[1,0])>.30 for x in route))
        self.assertGreater(max(abs(x[1]) for x in route),.3)
    def test_acceleration_limit_and_bus_budget(self):
        g=Gait(load_design());g.targets(.01,[100,0,0]);self.assertLessEqual(g.velocity[0],.00701)
        self.assertLess(bus_load(),.4);self.assertGreater(bus_load(12),.75)

if __name__=='__main__':unittest.main(verbosity=2)
