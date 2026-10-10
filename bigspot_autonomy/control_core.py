"""Engine-independent QDD controller. No CAN device or simulator pose writes."""
import math
from dataclasses import dataclass
import numpy as np
from .design import LEGS, motor_names
from .gait import Gait
from .kinematics import smoothstep
from .safety import Supervisor, Decision

JOINT_NAMES = tuple(f'{leg}_{axis}' for leg, _, _ in LEGS for axis in ('abad', 'hip', 'knee'))
JOINT_LOWER = np.tile([-.65, -2.1, .10], 4)
JOINT_UPPER = np.tile([.65, 1.25, 3.0], 4)


def ordered_joints(names, positions, velocities):
    if len(names) != 12 or len(set(names)) != 12 or set(names) != set(JOINT_NAMES):
        raise ValueError('Expected twelve unique named joints')
    if len(positions) != 12 or len(velocities) != 12:
        raise ValueError('Measured position and velocity required for every joint')
    order = [names.index(n) for n in JOINT_NAMES]
    q, v = np.asarray(positions, float)[order], np.asarray(velocities, float)[order]
    if not np.isfinite(q).all() or not np.isfinite(v).all():
        raise ValueError('Non-finite joint state')
    if np.any(q < JOINT_LOWER-.05) or np.any(q > JOINT_UPPER+.05):
        raise ValueError('Measured joint outside model limits')
    return q, v


def attitude(quaternion):
    q = np.asarray(quaternion, float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > .05:
        raise ValueError('Invalid xyzw attitude quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                     math.asin(np.clip(2*(w*y-z*x), -1, 1))])


def limited_effort(design, target, positions, velocities):
    names = motor_names(design)
    peak = np.array([design['actuators'][n]['peak_torque_nm'] for n in names])
    speed = np.array([design['actuators'][n]['no_load_rpm'] for n in names])*math.pi/30
    cfg = design['control']
    effort = cfg['kp']*(target-positions)-cfg['kd']*velocities
    available = peak*np.clip(1-np.abs(velocities)/speed, 0, 1)
    effort = np.clip(effort, -available, available)
    watts = np.maximum(effort*velocities, 0).sum()/.8
    return effort*min(1., design['battery']['maximum_motor_bus_power_w']/max(watts, 1.))


@dataclass
class Output:
    positions: np.ndarray
    efforts: np.ndarray
    state: str
    reason: str
    torque_enabled: bool


class ControlCore:
    def __init__(self, design, autonomous=False):
        self.design, self.autonomous = design, autonomous
        cfg = design['control']
        self.supervisor = Supervisor(cfg['sensor_timeout_s'], cfg['command_timeout_s'])
        self.gait = Gait(design)
        self.q = self.v = None
        self.rp = np.zeros(2)
        self.velocity = np.zeros(3)
        self.state = 'DISARMED'
        self.rise_seed = self.target = None
        self.elapsed = 0.
        self.settled = 0.
        self.source_stamps = {}

    def accept_stamp(self, source, stamp, now):
        if not math.isfinite(stamp) or not math.isfinite(now) or not 0 <= now-stamp <= .15:
            return False
        if stamp <= self.source_stamps.get(source, -math.inf):
            return False
        self.source_stamps[source] = stamp
        return True

    def joints(self, names, positions, velocities, now):
        self.q, self.v = ordered_joints(list(names), positions, velocities)
        self.supervisor.update('joints', now)

    def imu(self, quaternion, gyro, now):
        g = np.asarray(gyro, float)
        if g.shape != (3,) or not np.isfinite(g).all() or max(abs(g)) > 35:
            raise ValueError('Invalid gyro')
        self.rp = attitude(quaternion)
        self.supervisor.update('imu', now)

    def command(self, velocity, now):
        v = np.asarray(velocity, float)
        if v.shape != (3,) or not np.isfinite(v).all():
            raise ValueError('Invalid velocity command')
        self.velocity = v.copy()
        self.supervisor.update('command', now)

    def enable(self, requested, now):
        if not requested:
            self.state = 'DISARMED'
            self.velocity[:] = 0
            self.gait.velocity[:] = 0
            return True
        if self.state != 'DISARMED' or self.supervisor.latched or self.q is None:
            return False
        if not self.supervisor.evaluate(now, tilt=max(abs(self.rp))).torque_enabled:
            return False
        self.rise_seed = self.q.copy()
        self.target = self.q.copy()
        self.elapsed = self.settled = 0.
        self.state = 'RISING'
        return True

    def step(self, dt, now):
        target = self.q.copy() if self.target is None and self.q is not None else (
            self.gait.stand() if self.target is None else self.target.copy())
        if not math.isfinite(dt) or not 0 < dt <= .05:
            self.supervisor.latched = 'CLOCK_DISCONTINUITY'
        decision = self.supervisor.evaluate(now, autonomous=self.autonomous, tilt=max(abs(self.rp)))
        if self.autonomous and decision.motion_enabled and not 0 <= now-self.supervisor.stamps.get('odom', -math.inf) <= .3:
            decision = Decision(decision.torque_enabled,False,'STALE_ODOMETRY')
        if self.state == 'DISARMED':
            return Output(target, np.zeros(12), self.state, 'DISARMED', False)
        if not decision.torque_enabled:
            self.state = 'FAULT'
            return Output(target, np.zeros(12), self.state, decision.reason, False)
        try:
            self.elapsed += dt
            if self.state == 'RISING':
                target = self.rise_seed+(self.gait.stand()-self.rise_seed)*smoothstep(self.elapsed/4.)
                if self.elapsed >= 4.:
                    self.state = 'SETTLING'
            elif self.state == 'SETTLING':
                target = self.gait.stand()
                stable = max(abs(self.rp)) < math.radians(12) and max(abs(self.q-target)) < .18
                self.settled = self.settled+dt if stable else 0.
                if self.settled >= .4:
                    self.state = 'READY'
                elif self.elapsed > 10.:
                    raise ValueError('Stand did not settle')
            elif self.state == 'READY':
                moving = decision.motion_enabled
                target = self.gait.targets(dt, self.velocity if moving else np.zeros(3),
                                           roll=self.rp[0]*.5, pitch=self.rp[1]*.5)
            if not np.isfinite(target).all() or np.any(target < JOINT_LOWER) or np.any(target > JOINT_UPPER):
                raise ValueError('Joint target outside limits')
            self.target = target
            effort = limited_effort(self.design, target, self.q, self.v)
        except ValueError as error:
            self.supervisor.latched = 'INVALID_TARGET: '+str(error)
            self.state = 'FAULT'
            return Output(self.target.copy(), np.zeros(12), self.state, self.supervisor.latched, False)
        return Output(target, effort, self.state, decision.reason, True)
