"""Conservative sensor constraints, independent of ROS and motor drivers.

Two forward sonars do not observe lateral/rear clearance. Contacts/encoders
are optional observations, never inferred from commanded PWM angles.
"""
from collections import deque
import numpy as np


class RangePair:
    def __init__(self, timeout=.35):
        self.timeout=timeout
        self.samples=[deque(maxlen=3),deque(maxlen=3)]
        self.times=np.full(2,float('-inf'))
        self.valid=[False,False]
        self.raw=np.full(2,np.nan)

    def update(self, side, distance, stamp, now, minimum=.02, maximum=4.):
        if side not in (0,1) or not np.isfinite([stamp,now,minimum,maximum]).all():
            raise ValueError('Invalid range metadata')
        if not 0<minimum<maximum or not 0<=now-stamp<=self.timeout or stamp<self.times[side]:
            return False
        self.times[side]=float(stamp)
        if distance==float('-inf'):
            distance=minimum  # ROS Range convention: obstacle below minimum.
        elif distance==float('inf'):
            distance=maximum  # Only an explicitly reported beyond-range reading.
        if not np.isfinite(distance) or not minimum<=distance<=maximum:
            self.valid[side]=False
            self.samples[side].clear()
            self.raw[side]=np.nan
            return False
        self.valid[side]=True
        self.raw[side]=float(distance)
        self.samples[side].append(float(distance))
        return True

    def snapshot(self, now):
        fresh=[bool(self.valid[i] and 0<=now-self.times[i]<=self.timeout) for i in range(2)]
        # A close return takes effect immediately; an isolated far return cannot
        # remove an obstacle held in the three-sample median.
        distances=[min(self.raw[i],float(np.median(self.samples[i]))) if fresh[i] else None for i in range(2)]
        return {'fresh':fresh,'distance_m':distances,'coverage':'front_only',
                'mount_calibrated':False,'both_fresh':all(fresh)}


class SensorConstraints:
    def __init__(self, limits, require_ultrasonic=False, clearance=.22, slow_distance=.60,
                 deceleration=.2, jerk=1.2, latency=.35, require_joint_feedback=False):
        if not 0<clearance<slow_distance or min(deceleration,jerk,latency)<=0:
            raise ValueError('Invalid obstacle braking parameters')
        self.limits=np.asarray(limits,dtype=float)
        self.required=require_ultrasonic
        self.require_joints=require_joint_feedback
        self.clearance,self.slow_distance=clearance,slow_distance
        self.deceleration,self.jerk,self.latency=deceleration,jerk,latency
        self.accel_filtered=np.zeros(3)
        self.gyro_filtered=np.zeros(3)
        self.lag_seconds=0.
        self.air_seconds=0.
        self.heading=None
        self.obstacle_latched=False
        self.status={}

    def limit(self, velocity, dt, observation, joint_error=0., heading_hold=True):
        command=np.clip(np.asarray(velocity,dtype=float),-self.limits,self.limits)
        if command.shape!=(3,) or not np.isfinite(command).all():
            raise ValueError('Invalid constrained velocity')
        reasons=[]
        alpha=min(1.,dt/.15)
        self.accel_filtered+=alpha*(np.asarray(observation.get('world_linear_acceleration_m_s2',[0,0,0]))-self.accel_filtered)
        self.gyro_filtered+=alpha*(np.asarray(observation.get('angular_velocity_rad_s',[0,0,0]))-self.gyro_filtered)
        tilt=float(np.max(np.abs(observation.get('rpy_rad',[0,0,0])[:2])))
        scale=float(np.clip((np.deg2rad(20)-tilt)/np.deg2rad(12),0.,1.))
        if scale<1:
            reasons.append('body_tilt')
        horizontal_accel=float(np.linalg.norm(self.accel_filtered[:2]))
        dynamic_scale=float(np.clip(1.4/max(horizontal_accel,1.4),.25,1.))
        if dynamic_scale<1:
            reasons.append('dynamic_acceleration')
        scale=min(scale,dynamic_scale)
        feedback=observation.get('joint_feedback_fresh',False)
        if self.require_joints and not feedback:
            scale=0.
            reasons.append('joint_feedback_unavailable')
        self.lag_seconds=self.lag_seconds+dt if feedback and joint_error>.25 else max(0.,self.lag_seconds-dt)
        if self.lag_seconds>.25:
            scale=min(scale,0. if joint_error>.5 else .4)
            reasons.append('joint_tracking_lag')
        known=observation.get('contact_known',False)
        self.air_seconds=self.air_seconds+dt if known and observation.get('support_count')==0 else 0.
        if self.air_seconds>.08+1e-9:
            scale=0.
            reasons.append('support_lost')
        command*=scale
        if heading_hold and observation.get('imu_valid') and np.linalg.norm(command[:2])>.015 and abs(velocity[2])<.02:
            yaw=observation['rpy_rad'][2]
            if self.heading is None:
                self.heading=float(yaw)
            error=(self.heading-yaw+np.pi)%(2*np.pi)-np.pi
            command[2]=float(np.clip(.8*error-.06*self.gyro_filtered[2],-.12,.12))
        else:
            self.heading=None
        sonar=observation.get('ultrasonic',{})
        both=sonar.get('both_fresh',False)
        distances=[v for v in sonar.get('distance_m',[]) if v is not None]
        distance=min(distances) if distances else None
        cap=float(self.limits[0])
        if self.required and not both:
            cap=0.
            reasons.append('ultrasonic_unavailable')
        if distance is not None:
            if distance<=self.clearance+.03:
                self.obstacle_latched=True
            elif both and distance>=self.clearance+.12:
                self.obstacle_latched=False
            free=max(0.,distance-self.clearance)
            delay=self.latency+self.deceleration/self.jerk
            braking=np.sqrt((self.deceleration*delay)**2+2*self.deceleration*free)-self.deceleration*delay
            proximity=self.limits[0]*np.clip(free/(self.slow_distance-self.clearance),0.,1.)
            obstacle_cap=min(float(braking),float(proximity))
            cap=min(cap,obstacle_cap)
            if obstacle_cap<max(0.,command[0]):
                reasons.append('obstacle_braking')
        if self.obstacle_latched:
            cap=0.
            command[2]=0.
            reasons.append('front_obstacle')
        if command[0]>0:
            command[0]=min(command[0],cap)
        # Heading hold must not keep marching in place after a sensor blocks
        # translation. Explicit operator turns are handled separately.
        if heading_hold and abs(velocity[2])<.02 and np.linalg.norm(command[:2])<.015:
            command[2]=0.
            self.heading=None
        self.status={'reasons':reasons,'speed_scale':scale,'front_speed_cap_m_s':cap,
                     'nearest_front_m':distance,'ultrasonic_required':self.required,
                     'requested_velocity':np.asarray(velocity).tolist(),'constrained_velocity':command.tolist(),
                     'rear_lateral_obstacle_coverage':False,'joint_feedback_used':bool(feedback),
                     'contact_feedback_used':bool(known),'heading_hold_relative_only':self.heading is not None,
                     'obstacle_stop_latched':self.obstacle_latched}
        return command
