"""CPU plant driven by external targets and motor PD, with its own watchdog."""
import math
import numpy as np
from .control_core import JOINT_NAMES, JOINT_LOWER, JOINT_UPPER, limited_effort
from .design import motor_names
from .model import make_mjcf


class EffortWatchdog:
    def __init__(self, design, timeout=.15):
        self.peak = np.array([design['actuators'][n]['peak_torque_nm'] for n in motor_names(design)])
        self.timeout = timeout
        self.stamp = self.wall_stamp = -math.inf
        self.value = np.zeros(12)
        self.expired = False
        self.design = design
        self.target = None
        self.enabled = False

    def receive(self, names, efforts, stamp, sim_now, wall_now):
        if self.expired or len(names) != 12 or len(set(names)) != 12 or set(names) != set(JOINT_NAMES):
            return False
        if len(efforts) != 12 or not all(math.isfinite(t) for t in (stamp,sim_now,wall_now)) or not 0 <= sim_now-stamp <= self.timeout or stamp <= self.stamp:
            return False
        value = np.asarray(efforts, float)[[list(names).index(n) for n in JOINT_NAMES]]
        if not np.isfinite(value).all() or np.any(abs(value) > self.peak+1e-6):
            return False
        self.value, self.stamp, self.wall_stamp = value, stamp, wall_now
        return True

    def efforts(self, sim_now, wall_now):
        if self.stamp != -math.inf and (not 0 <= sim_now-self.stamp <= self.timeout or
                                       not 0 <= wall_now-self.wall_stamp <= self.timeout):
            self.expired = True
        return np.zeros(12) if self.expired or self.stamp == -math.inf else self.value.copy()

    def receive_target(self,names,positions,efforts,stamp,sim_now,wall_now):
        if len(positions)!=12 or len(efforts) not in (0,12):return False
        if len(names)!=12 or len(set(names))!=12 or set(names)!=set(JOINT_NAMES):return False
        q=np.asarray(positions,float)[[list(names).index(n) for n in JOINT_NAMES]]
        if not np.isfinite(q).all() or np.any(q<JOINT_LOWER) or np.any(q>JOINT_UPPER):return False
        if not self.receive(names,efforts if efforts else [0.]*12,stamp,sim_now,wall_now):return False
        self.target=q;self.enabled=bool(efforts)
        return True

    def motor_efforts(self,sim_now,wall_now,q,v):
        self.efforts(sim_now,wall_now)
        if self.expired or not self.enabled or self.target is None:return np.zeros(12)
        # Emulate the motor's local high-rate PD loop, not a delayed DDS torque hold.
        return limited_effort(self.design,self.target,q,v)


class CpuPlant:
    def __init__(self, design, friction=.8, mass_scale=1., obstacles=True):
        import mujoco
        self.mj = mujoco
        blocks = ((1.1,0.,.18,.25,.5),(2.1,.65,.2,.18,.5)) if obstacles else ()
        self.model = mujoco.MjModel.from_xml_string(make_mjcf(design, blocks))
        self.model.geom_friction[:,0] = friction
        self.model.body_mass[:] *= mass_scale
        self.model.body_inertia[:] *= mass_scale
        self.data = mujoco.MjData(self.model)
        mujoco.mj_setConst(self.model, self.data)
        ids = [mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,n) for n in JOINT_NAMES]
        self.qadr, self.vadr = self.model.jnt_qposadr[ids], self.model.jnt_dofadr[ids]
        self.data.qpos[self.qadr] = np.tile([0.,-1.43,2.86],4)
        self.data.qpos[2] = .12
        mujoco.mj_forward(self.model,self.data)
        floor = mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,'floor')
        lowest = min(mujoco.mj_geomDistance(self.model,self.data,g,floor,1.,None)
                     for g in range(self.model.ngeom) if g!=floor and self.model.geom_bodyid[g]!=0 and self.model.geom_contype[g])
        self.data.qpos[2] += .002-lowest
        mujoco.mj_forward(self.model,self.data)
        self.watchdog = EffortWatchdog(design)
        self.dt = float(self.model.opt.timestep)

    def step(self, wall_now):
        self.data.ctrl[:] = self.watchdog.motor_efforts(float(self.data.time),wall_now,
                                                       self.data.qpos[self.qadr],self.data.qvel[self.vadr])
        self.mj.mj_step(self.model,self.data)
        if not np.isfinite(self.data.qpos).all():
            raise RuntimeError('Non-finite physics state')

    def measurements(self):
        # Simulator attitude is explicitly ground truth. Not physical IMU fusion.
        w,x,y,z = self.data.qpos[3:7]
        return self.data.qpos[self.qadr].copy(), self.data.qvel[self.vadr].copy(), [x,y,z,w], self.data.sensordata[:3].copy()
