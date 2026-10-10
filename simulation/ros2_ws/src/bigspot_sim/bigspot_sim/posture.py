"""Shared startup and feedback controller. No motor drivers are imported here."""
import numpy as np
from .kinematics import Gait, rotation
from .ground_startup import GroundStartup
from .hardware_reference import load_reference,reference_model


class PostureController:
    def __init__(self, model, support_shift=.6, balance_kp=.8, balance_kd=.05,
                 speed_scale=.7, rise_seconds=4., crouch_height=.20, period=4.,
                 lift=.012, support_forward_ratio=1., wave_sequence=False, center_stance=False,crawl_duty=.8,
                 static_crawl=False,support_margin=.012,transfer_fraction=.45,com_x=-.020,stance_center_x=0.,
                 gait_mode='crawl',max_forward_speed=.015,max_lateral_speed=.01,max_yaw_rate=.1,
                 acceleration=.03,trot_duty=.6,standing_height=0.,joint_rate=1.5,
                 smooth_velocity=False,linear_jerk=1.2,yaw_acceleration=.6,yaw_jerk=3.,
                 ground_rest=False,ground_prepare_seconds=7.,ground_arm_seconds=.8,hardware_reference=False):
        self.hardware_reference=load_reference() if hardware_reference else None
        if self.hardware_reference:
            model=reference_model(model,self.hardware_reference)
            standing_height=self.hardware_reference['standing_height_m']
            center_stance=False
        self.gait = Gait(model,mode=gait_mode,support_shift=support_shift, period=period, lift=lift,
                         support_forward_ratio=support_forward_ratio,wave_sequence=wave_sequence,
                         center_stance=center_stance,crawl_duty=crawl_duty,static_crawl=static_crawl,
                         support_margin=support_margin,transfer_fraction=transfer_fraction,
                         com_x=com_x,stance_center_x=stance_center_x,max_forward_speed=max_forward_speed,
                         max_lateral_speed=max_lateral_speed,max_yaw_rate=max_yaw_rate,
                         acceleration=acceleration,trot_duty=trot_duty,standing_height=standing_height,
                         smooth_velocity=smooth_velocity,linear_jerk=linear_jerk,
                         yaw_acceleration=yaw_acceleration,yaw_jerk=yaw_jerk)
        self.standing = self.gait.rest.copy()
        self.height = -float(self.standing[:, 2].mean())
        self.crouch_height = crouch_height
        self.balance_kp, self.balance_kd = balance_kp, balance_kd
        self.speed_scale, self.rise_seconds = speed_scale, rise_seconds
        motor_rate_limit=min(float(j['velocity']) for leg in model['legs'] for j in leg['joints'])
        if not 0 < joint_rate <= motor_rate_limit:
            raise ValueError('Joint command rate exceeds configured motor no-load speed')
        self.joint_rate = joint_rate
        self.state = 'CROUCH'
        self.stop_settled_s = 0.
        self.stop_gyro = np.zeros(2)
        self.ground_balance_support_loss=0.
        self.state_time = 0.
        self.stable_time = 0.
        self.start_requested = False
        self.correction = np.zeros(2)
        self.fault = ''
        feet = self.standing.copy()
        feet[:, 2] = -crouch_height
        self.gait.q = np.array([leg.ik(p, q) for leg, p, q in zip(self.gait.legs, feet, self.gait.q)])
        self.observation={}
        self.ground_startup=(GroundStartup(model,self.gait.q,ground_prepare_seconds,ground_arm_seconds)
                             if ground_rest else None)
        if self.ground_startup:
            self.state='GROUND_OFF'
            self.gait.q=self.ground_startup.q.reshape(4,3).copy()

    def start(self):
        if self.ground_startup and self.state=='GROUND_OFF':
            self.ground_startup.request()
            return
        if self.state == 'CROUCH':
            self.start_requested = True

    def transition(self, state):
        self.state, self.state_time, self.stable_time = state, 0., 0.

    def step(self, dt, velocity, roll_pitch, angular_velocity, imu_fresh=True,
             command_fresh=True, measured_height=None, joint_error=0.):
        if self.fault:
            self.state='FAULT'
            return self.gait.q.ravel().tolist()
        if not np.isfinite(dt) or not 0 < dt <= .1:
            raise ValueError('Invalid simulation interval')
        rp, gyro = np.asarray(roll_pitch), np.asarray(angular_velocity)
        if rp.shape != (2,) or gyro.shape != (2,) or not np.isfinite(rp).all() or not np.isfinite(gyro).all():
            self.fault = 'Invalid IMU'
        elif np.max(np.abs(rp)) > np.deg2rad(35):
            self.fault = 'Body tilt exceeds 35 degrees'
        elif not imu_fresh and self.state not in ('CROUCH','GROUND_OFF'):
            self.fault = 'IMU timeout'
        if self.fault:
            self.state = 'FAULT'
            return self.gait.q.ravel().tolist()
        self.state_time += dt
        if self.ground_startup and self.state in GroundStartup.phases:
            complete=self.ground_startup.step(dt,rp,gyro,imu_fresh,self.observation)
            self.gait.q=self.ground_startup.q.reshape(4,3).copy()
            self.state=self.ground_startup.phase
            if self.ground_startup.fault:
                self.fault=self.ground_startup.fault;self.state='FAULT'
            elif complete:
                self.transition('RISING')
            return self.gait.q.ravel().tolist()
        if self.ground_startup and self.state in ('RISING','BALANCING') and not self.ground_startup.valid_joints(self.observation):
            self.fault='Ground rise lost joint feedback';self.state='FAULT'
            return self.gait.q.ravel().tolist()
        if self.state == 'CROUCH' and self.start_requested:
            self.transition('RISING')
        if self.state == 'RISING' and self.state_time >= self.rise_seconds:
            self.transition('BALANCING')
        if self.state == 'BALANCING':
            stable = (np.max(np.abs(rp)) < np.deg2rad(3) and joint_error < .12
                      and (measured_height is None or measured_height > self.height-.03))
            if self.ground_startup:
                ground=self.observation.get('ground_rest_feedback',{})
                forces=np.asarray(self.observation.get('contact_forces_n',[]))
                known=(self.observation.get('contact_known',False) and forces.shape==(4,) and np.isfinite(forces).all())
                support=int(np.count_nonzero(forces>1.)) if known else 0
                self.ground_balance_support_loss=(0. if support==4 else self.ground_balance_support_loss+dt)
                stable=(stable and ground.get('fresh',False) and ground.get('base_height_m',0.)>self.height-.03
                        and known and support>=3 and self.ground_balance_support_loss<=.08+1e-9)
            self.stable_time = self.stable_time + dt if stable else 0.
            if self.stable_time >= 1. and (not self.ground_startup or support==4):
                self.transition('READY')
            elif self.state_time > 10.:
                self.fault = 'Standing did not stabilize'
                self.state = 'FAULT'
                return self.gait.q.ravel().tolist()
        desired = np.asarray(velocity, dtype=float)
        if desired.shape != (3,) or not np.isfinite(desired).all():
            raise ValueError('Invalid velocity')
        allowed = self.state in ('READY', 'WALKING', 'STOPPING')
        requested = desired*self.speed_scale if allowed and command_fresh else np.zeros(3)
        if allowed:
            self.stop_gyro+=min(1.,dt/.12)*(gyro-self.stop_gyro)
            if np.linalg.norm(requested) > 1e-4:
                self.state = 'WALKING'
                self.stop_settled_s=0.
            elif np.linalg.norm(self.gait.speed) > 1e-4 or self.gait.blend > 1e-4:
                self.state = 'STOPPING'
                self.stop_settled_s=0.
            elif self.state in ('WALKING','STOPPING'):
                stable=self.stop_is_stable(rp,self.stop_gyro,joint_error,dt)
                self.stop_settled_s=self.stop_settled_s+dt if stable else 0.
                self.state=('READY' if self.stop_settled_s>=.4-1e-9 and
                            self.stop_contacts_confirmed() else 'STOPPING')
            else:
                self.state = 'READY'
        targets = self.gait.targets(min(dt, .04), requested)
        if self.state == 'CROUCH':
            targets[:, 2] = -self.crouch_height
        elif self.state == 'RISING':
            u = np.clip(self.state_time/self.rise_seconds, 0., 1.)
            u = u*u*u*(10-15*u+6*u*u)
            targets[:, 2] = -(self.crouch_height + u*(self.height-self.crouch_height))
        desired_correction = np.zeros(2) if self.state == 'CROUCH' else np.clip(self.balance_kp*rp + self.balance_kd*gyro, -.10, .10)
        self.correction += np.clip(desired_correction-self.correction, -.3*dt, .3*dt)
        transform = rotation([1,0,0], self.correction[0]) @ rotation([0,1,0], self.correction[1])
        targets = targets @ transform.T
        solved = np.array([leg.ik(p, q, tolerance=.00005)
                           for leg, p, q in zip(self.gait.legs, targets, self.gait.q)])
        rate = self.joint_rate if allowed else .8
        delta=solved-self.gait.q
        if self.ground_startup:
            # The stiff servo model otherwise amplifies tiny IMU/IK changes
            # into intermittent two-foot contact during initial settling.
            delta*=1.-np.exp(-dt/.04)
        self.gait.q += np.clip(delta, -rate*dt, rate*dt)
        return self.gait.q.ravel().tolist()

    def stop_is_stable(self,rp,gyro,joint_error,dt):
        return bool(np.max(np.abs(rp))<np.deg2rad(3) and
                    np.max(np.abs(gyro))<.15 and joint_error<.12)

    def stop_contacts_confirmed(self):return True


def quaternion_rpy(values):
    q = np.asarray(values, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1.) > .05:
        raise ValueError('Invalid unit quaternion')
    x,y,z,w = q/np.linalg.norm(q)
    return np.array([np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                     np.arcsin(np.clip(2*(w*y-z*x), -1, 1))])
