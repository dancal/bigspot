"""Simulation startup from a folded, unpowered body-on-ground rest.

Real use requires calibrated rest angles, measured joints and a motor-enable
adapter. An IMU alone cannot establish joint angles or body-ground support.
"""
import numpy as np


def ease(u):
    u=float(np.clip(u,0.,1.))
    return u**3*(10.-15.*u+6.*u*u)


class GroundStartup:
    phases={'GROUND_OFF','SERVO_ARMING','LEG_PREPARING'}

    def __init__(self,model,crouch_q,prepare_seconds=7.,arm_seconds=.8):
        if not np.isfinite([prepare_seconds,arm_seconds]).all() or prepare_seconds<5. or arm_seconds<.5:
            raise ValueError('Ground startup needs >=5 s preparation and >=0.5 s arming')
        self.lower=np.array([j['lower'] for leg in model['legs'] for j in leg['joints']])
        self.upper=np.array([j['upper'] for leg in model['legs'] for j in leg['joints']])
        # This folded seed is a simulator initial condition, not a physical calibration.
        rear_hip=min(np.pi/2,self.upper[7],self.upper[10])
        self.folded_q=np.array([[0.,-np.pi/2,self.lower[2]], [0.,-np.pi/2,self.upper[5]],
            [0.,rear_hip,self.lower[8]], [0.,rear_hip,self.upper[11]]]).ravel()
        self.folded_q=np.clip(self.folded_q,self.lower,self.upper)
        if model.get('hardware_reference'):
            # Simulated relaxed fold within original numerical servo bounds.
            # This is not a measurement of the real unpowered robot's pose.
            self.folded_q[[1,4]]=np.deg2rad(70.)
            self.folded_q[[7,10]]=np.deg2rad(-60.)
        if np.any(self.folded_q<self.lower-.00001) or np.any(self.folded_q>self.upper+.00001):
            raise ValueError('Folded simulator rest outside joint range')
        self.q=self.folded_q.copy()
        self.crouch_q=np.asarray(crouch_q,dtype=float).ravel().copy()
        self.prepare_seconds,self.arm_seconds=prepare_seconds,arm_seconds
        self.phase='GROUND_OFF';self.time=0.;self.stable=0.;self.requested=False
        self.servo_enabled=False;self.strength=0.;self.fault=''

    def request(self):
        if self.phase=='GROUND_OFF':self.requested=True

    def snapshot(self):
        return {'phase':self.phase,'servo_enabled':self.servo_enabled,'strength':self.strength,
            'simulation_only':True,'physical_rest_calibrated':False}

    def valid_joints(self,observation):
        q=np.asarray(observation.get('measured_joint_angles_rad',[]),dtype=float)
        return (observation.get('joint_feedback_fresh',False) and q.shape==(12,) and
            np.isfinite(q).all() and np.all(q>=self.lower-.02) and np.all(q<=self.upper+.02))

    def step(self,dt,rp,gyro,imu_fresh,observation):
        valid=self.valid_joints(observation)
        sensor_ok=bool(imu_fresh and observation.get('imu_valid',False) and
            observation.get('gyro_available',False) and valid)
        feedback=observation.get('ground_rest_feedback',{})
        ground_known=feedback.get('fresh',False)
        if self.phase=='GROUND_OFF':
            if valid:self.q=np.asarray(observation['measured_joint_angles_rad']).copy()
            stable=(sensor_ok and ground_known and feedback.get('body_force_n',0.)>1. and
                abs(feedback.get('body_clearance_m',1.))<.002 and
                np.max(np.abs(rp))<np.deg2rad(5.) and np.max(np.abs(gyro))<.10)
            self.stable=self.stable+dt if stable else 0.
            if self.requested and self.stable>=.4-1e-9:
                self.seed=self.q.copy();self.phase='SERVO_ARMING';self.time=0.
                self.stable=0.;self.servo_enabled=True
            return False
        if not sensor_ok or not ground_known:
            self.fault='Ground startup lost IMU, joint or ground feedback'
            return False
        if np.max(np.abs(rp))>np.deg2rad(20.):
            self.fault='Ground startup tilt exceeds 20 degrees'
            return False
        self.time+=dt
        if self.phase=='SERVO_ARMING':
            self.strength=ease(self.time/self.arm_seconds)
            if self.time>=self.arm_seconds:
                self.phase='LEG_PREPARING';self.time=0.
        elif self.phase=='LEG_PREPARING':
            self.strength=1.
            self.q=self.seed+(self.crouch_q-self.seed)*ease(self.time/self.prepare_seconds)
            forces=np.asarray(observation.get('contact_forces_n',[]),dtype=float)
            settled=(self.time>=self.prepare_seconds and forces.shape==(4,) and
                observation.get('contact_known',False) and np.all(forces>1.) and
                np.max(np.abs(rp))<np.deg2rad(5.) and np.max(np.abs(gyro))<.10 and
                np.max(np.abs(np.asarray(observation['measured_joint_angles_rad'])-self.q))<.08)
            self.stable=self.stable+dt if settled else 0.
            if self.stable>=.4-1e-9:
                self.phase='COMPLETE';return True
            if self.time>self.prepare_seconds+5.:
                self.fault='Ground startup did not obtain stable four-foot support'
        return False
