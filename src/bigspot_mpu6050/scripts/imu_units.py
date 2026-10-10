"""MPU6050 register counts to ROS SI units, using actual configured FS bits."""
import numpy as np


def raw_to_si(gyro_counts,accel_counts,gyro_fs,accel_fs):
    if gyro_fs not in range(4) or accel_fs not in range(4):
        raise ValueError('Invalid MPU6050 full-scale register')
    gyro=np.asarray(gyro_counts,dtype=float)
    accel=np.asarray(accel_counts,dtype=float)
    if gyro.shape!=(3,) or accel.shape!=(3,) or not np.isfinite(gyro).all() or not np.isfinite(accel).all():
        raise ValueError('Invalid raw IMU sample')
    gyro=gyro/[131.,65.5,32.8,16.4][gyro_fs]*np.pi/180.
    accel=accel/[16384.,8192.,4096.,2048.][accel_fs]*9.80665
    return gyro,accel


def remap_mount(orientation_xyzw,gyro,accel,body_from_sensor_xyzw):
    q=np.asarray(orientation_xyzw,dtype=float)
    mount=np.asarray(body_from_sensor_xyzw,dtype=float)
    for value in [q,mount]:
        if value.shape!=(4,) or not np.isfinite(value).all() or abs(np.linalg.norm(value)-1)>.05:
            raise ValueError('Invalid mounting/orientation quaternion')
    q=q/np.linalg.norm(q)
    mount=mount/np.linalg.norm(mount)
    x,y,z,w=mount
    r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    inverse=np.r_[-mount[:3],mount[3]]
    mapped=np.r_[q[3]*inverse[:3]+inverse[3]*q[:3]+np.cross(q[:3],inverse[:3]),
                 q[3]*inverse[3]-q[:3]@inverse[:3]]
    return mapped,r@np.asarray(gyro),r@np.asarray(accel)
