"""IMU attitude and contact state; never claim IMU-only position/velocity accuracy.

Quaternion convention: ROS xyzw. Acceleration is body-frame specific force
(stationary upright sensor reads +9.81 on z). DMP orientation, when available,
is used directly; gyro/accelerometer complementary fallback has drifting yaw.
Contact forces and encoder feedback require separate physical sensors.
"""
import numpy as np


def stamp_seconds(sec, nanosec):
    """Match the ROS clock's integer-nanosecond conversion exactly."""
    return (int(sec)*1_000_000_000+int(nanosec))/1e9


def quaternion_matrix(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def multiply(a,b):
    av,bv=np.asarray(a[:3]),np.asarray(b[:3])
    return np.r_[a[3]*bv+b[3]*av+np.cross(av,bv),a[3]*b[3]-av@bv]


class StateEstimator:
    def __init__(self):
        self.q=np.array([0.,0.,0.,1.])
        self.gyro=np.zeros(3)
        self.accel=np.zeros(3)
        self.linear_accel=np.zeros(3)
        self.rpy=np.zeros(3)
        self.initialized=False
        self.orientation_source='unavailable'
        self.contact_forces=np.zeros(4)
        self.contact_time=float('-inf')
        self.imu_time=float('-inf')
        self.air_time=0.
        self.motion_state='UNKNOWN'
        self.gyro_available=False
        self.accel_available=False

    def contacts(self,forces,t):
        f=np.asarray(forces,dtype=float)
        if f.shape!=(4,) or not np.isfinite(f).all() or np.any(f<0):
            raise ValueError('Invalid foot contact forces')
        self.contact_forces=f.copy()
        self.contact_time=float(t)

    def update(self,q,gyro,accel,t,orientation_available=True,accel_available=True,gyro_available=True):
        gyro,accel=np.asarray(gyro,dtype=float),np.asarray(accel,dtype=float)
        if gyro.shape!=(3,) or not np.isfinite(gyro).all() or np.max(np.abs(gyro))>35:
            raise ValueError('Invalid gyroscope')
        if accel.shape!=(3,) or (accel_available and not np.isfinite(accel).all()):
            raise ValueError('Invalid accelerometer')
        dt=float(t-self.imu_time) if self.initialized else .016
        if not 0<=dt<=.25:
            raise ValueError('IMU timestamp discontinuity')
        if orientation_available:
            q=np.asarray(q,dtype=float)
            if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.05:
                raise ValueError('Invalid IMU orientation')
            self.q=q/np.linalg.norm(q)
            self.orientation_source='provided_fused_orientation'
        else:
            if not gyro_available:
                raise ValueError('Orientation and gyro both unavailable')
            omega=gyro.copy()
            # Dynamic acceleration and freefall are excluded from gravity correction.
            norm=np.linalg.norm(accel)
            if not self.initialized:
                if not accel_available or abs(norm-9.81)>=1.5:
                    raise ValueError('Stationary gravity reference required to initialize attitude')
                roll=np.arctan2(accel[1],accel[2])
                pitch=np.arctan2(-accel[0],np.hypot(accel[1],accel[2]))
                cr,sr=np.cos(roll/2),np.sin(roll/2)
                cp,sp=np.cos(pitch/2),np.sin(pitch/2)
                self.q=np.array([sr*cp,cr*sp,-sr*sp,cr*cp])
            if accel_available and abs(norm-9.81)<1.5:
                gravity=quaternion_matrix(self.q).T@np.array([0.,0.,1.])
                omega+=1.5*np.cross(accel/norm,gravity)
            dq=np.r_[omega*dt/2,1.]
            self.q=multiply(self.q,dq/np.linalg.norm(dq))
            self.q/=np.linalg.norm(self.q)
            self.orientation_source='gyro_accel_relative_yaw'
        r=quaternion_matrix(self.q)
        self.rpy=np.array([np.arctan2(r[2,1],r[2,2]),np.arcsin(np.clip(-r[2,0],-1,1)),
                           np.arctan2(r[1,0],r[0,0])])
        self.gyro=gyro.copy()
        self.gyro_available=bool(gyro_available)
        self.accel_available=bool(accel_available)
        self.accel=accel.copy() if accel_available else np.zeros(3)
        self.linear_accel=r@accel-np.array([0.,0.,9.81]) if accel_available else np.zeros(3)
        self.imu_time=float(t)
        self.initialized=True
        known=t-self.contact_time<.1
        count=int(np.count_nonzero(self.contact_forces>1.)) if known else None
        self.air_time=self.air_time+dt if known and count==0 else 0.
        if count==0 and self.air_time>=.032:
            self.motion_state='AIRBORNE'
        elif count is not None:
            self.motion_state='SUPPORTED' if count>=3 else 'PARTIAL_SUPPORT'
        elif accel_available and np.linalg.norm(accel)<2.:
            self.motion_state='POSSIBLE_FREEFALL'
        else:
            self.motion_state='CONTACT_UNKNOWN'
        return self.rpy.copy()

    def snapshot(self,t):
        known=bool(0<=t-self.contact_time<.1)
        count=int(np.count_nonzero(self.contact_forces>1.)) if known else None
        motion=('AIRBORNE' if count==0 and self.air_time>=.032 else
                'SUPPORTED' if count is not None and count>=3 else
                'PARTIAL_SUPPORT' if count is not None else
                'POSSIBLE_FREEFALL' if self.accel_available and np.linalg.norm(self.accel)<2. and self.initialized else 'CONTACT_UNKNOWN')
        return {'imu_valid':bool(self.initialized and 0<=t-self.imu_time<.25),
                'gyro_available':self.gyro_available,
                'accel_available':self.accel_available,
                'rpy_rad':self.rpy.tolist(),'angular_velocity_rad_s':self.gyro.tolist(),
                'specific_force_m_s2':self.accel.tolist(),'world_linear_acceleration_m_s2':self.linear_accel.tolist(),
                'orientation_source':self.orientation_source,'absolute_yaw_observable':False,
                'contact_known':known,'support_count':count,
                'contact_forces_n':self.contact_forces.tolist() if known else None,
                'motion_state':motion,'airborne_seconds':self.air_time,
                'position_velocity_estimated':False}
