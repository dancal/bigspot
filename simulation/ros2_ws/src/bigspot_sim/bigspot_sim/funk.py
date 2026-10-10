"""Beat-locked diagonal steps with layered body motion and steering phrases.

Inspired by the public Spot choreography concept of simultaneous body/leg
tracks, not a port of Spot controllers. Free-body physics must validate this
robot's two-paw dynamic support; no static tripod guarantee is made.
"""
from copy import deepcopy
import numpy as np
from .kinematics import rotation
from .poses import smooth


class FunkDancePlan:
    phrases=('groove','hip_wiggle','head_shake','head_bob','side_shuffle','forward_back','twist','combo')
    joint_rate=3.8  # Below the existing 4 rad/s walking command ceiling.
    hip_yaw=.15
    front_yaw=.12
    yaw_pivot_fraction=.4

    def __init__(self,bpm=120.,body_gain=1.,lift=.024,speed_gain=1.):
        if not np.isfinite([bpm,body_gain,lift,speed_gain]).all() or not 100<=bpm<=130:
            raise ValueError('Invalid funk dance parameters')
        if not .25<=body_gain<=1.2 or not .018<=lift<=.035 or not .5<=speed_gain<=1.2:
            raise ValueError('Funk gain/lift outside audited range')
        self.bpm,self.body_gain,self.lift,self.speed_gain=float(bpm),body_gain,lift,speed_gain
        self.beat_seconds=60/self.bpm
        self.period=2*self.beat_seconds
        self.intro=2*self.beat_seconds
        self.main_end=self.intro+len(self.phrases)*8*self.beat_seconds
        self.duration=self.main_end+4*self.beat_seconds
        self.duty=.67
        self.front_pivot=None
        self.rear_pivot=None

    def part(self,t):
        b=max(0.,(t-self.intro)/self.beat_seconds)
        i=min(len(self.phrases)-1,int(b//8))
        w=smooth((b-8*i-6)/2) if i<len(self.phrases)-1 else 0.
        return b,i,w

    def phrase(self,t):
        return 'intro' if t<self.intro else 'finish' if t>=self.main_end else self.phrases[self.part(t)[1]]

    def velocity(self,t):
        if t>=self.main_end:return np.zeros(3)
        b,i,w=self.part(t);a=2*np.pi*b/8
        v=np.array([[0.,0.,0.],[0.,0.,0.],[0.,0.,0.],[0.,0.,0.],
                    [0.,.065*np.sin(a),0.],[.13*np.sin(a),0.,0.],
                    [0.,0.,.60*np.sin(a)],[.09*np.sin(a),.05*np.sin(2*a),.30*np.sin(a)]])
        return self.speed_gain*(v[i]*(1-w)+v[min(i+1,len(self.phrases)-1)]*w)*smooth(t/self.intro)

    def body(self,t):
        b,i,w=self.part(t);phase=2*np.pi*b/2
        onset=smooth(b/2)
        s=np.sin(phase)*onset;c=np.cos(phase)*onset
        sway=np.sin(phase/2)*onset  # Four-beat accents, while feet stay on every beat.
        if self.front_pivot is None or self.rear_pivot is None:
            raise ValueError('Bind CAD hip pivots with configure(gait) before body motion')
        # Front and rear refer to rigid-body reference points, not neck joints.
        # Moving the rotation center toward the shoulders emphasizes the hips;
        # moving it rearward emphasizes the front. P-RP fixes the chosen pivot.
        attitudes=np.array([[.045*s,.025*c,0.],[.012*sway,0.,self.hip_yaw*sway],
                            [.015*sway,.010*c,self.front_yaw*sway],[.015*c,.065*s,0.],
                            [.070*s,-.010*s,0.],[.020*s,.065*s,0.],
                            [.040*s,.040*c,0.],[.060*s,.050*c,.030*s]])
        shifts=np.tile([0.,.005*s,-.018+.006*c],(len(self.phrases),1))
        pivots=np.zeros((len(self.phrases),3))
        pivots[1]=self.front_pivot*self.yaw_pivot_fraction
        pivots[2]=self.rear_pivot*self.yaw_pivot_fraction;pivots[3]=self.rear_pivot
        for n in range(1,4):
            rpy=attitudes[n]
            R=rotation([0,0,1],rpy[2])@rotation([0,1,0],rpy[1])@rotation([1,0,0],rpy[0])
            shifts[n]+=pivots[n]-R@pivots[n]
        # Front/rear emphasis uses a smaller common lateral bounce and no
        # extra vertical bounce for nodding, so the pitch supplies the accent.
        shifts[1,1]-=.005*s;shifts[2,1]-=.005*s;shifts[1,2]-=.006*c;shifts[2,2]-=.004*c;shifts[3,2]-=.006*c
        envelope=smooth(t/self.intro)*smooth((self.duration-t)/(3*self.beat_seconds))
        j=min(i+1,len(self.phrases)-1)
        return (self.body_gain*envelope*(shifts[i]*(1-w)+shifts[j]*w),
                self.body_gain*envelope*(attitudes[i]*(1-w)+attitudes[j]*w))

    def configure(self,gait):
        gait.period,gait.lift,gait.trot_duty=self.period,self.lift,self.duty
        self.front_pivot=np.mean([leg.joints[0]['origin'] for leg in gait.legs[:2]],axis=0)
        self.rear_pivot=np.mean([leg.joints[0]['origin'] for leg in gait.legs[2:]],axis=0)

    def targets(self,gait,dt,t,velocity=None,body=None,marching=None):
        self.configure(gait)
        feet=gait.targets(dt,self.velocity(t) if velocity is None else velocity,
            marching=t<self.main_end if marching is None else marching)
        shift,rpy=self.body(t) if body is None else body
        transform=rotation([0,0,1],rpy[2])@rotation([0,1,0],rpy[1])@rotation([1,0,0],rpy[0])
        return (feet-shift)@transform,rpy

    def audit(self,gait,initial):
        g=deepcopy(gait);g.q=initial.copy();g.phase=0.;g.speed[:]=0.;g.speed_acceleration[:]=0.;g.blend=0.
        peak=0.;dt=.016
        for t in np.arange(dt,self.duration+2,dt):
            targets,_=self.targets(g,dt,t)
            try:q=np.array([l.ik(p,v,tolerance=.00005) for l,p,v in zip(g.legs,targets,g.q)])
            except ValueError as e:return {'reachable':False,'reason':str(e),'time_s':float(t)}
            peak=max(peak,float(np.max(abs(q-g.q))/dt));g.q=q
        return {'reachable':peak<=self.joint_rate,'peak_planned_joint_speed_rad_s':peak,
            'reason':'Joint speed exceeds dance reserve' if peak>self.joint_rate else '',
            'bpm':self.bpm,'pair_step_period_s':self.beat_seconds,'duration_s':self.duration,
            'minimum_planned_support_feet':2,'collision_and_real_mass_verified':False}


def restore(c):
    c.gait.period,c.gait.lift,c.gait.trot_duty=c.funk_previous_gait


def request_funk(c):
    if c.action or c.state!='READY' or np.linalg.norm(c.gait.speed)>.005:
        c.action_result='Rejected: stop and settle before funk dance';return False
    if c.gait.mode!='trot' or c.gait.static is not None:
        c.action_result='Rejected: funk needs dynamic trot profile';return False
    obs=c.observation
    if (not obs.get('imu_valid') or np.max(abs(np.asarray(obs.get('rpy_rad',[99,99]))[:2]))>np.deg2rad(5)
        or not obs.get('joint_feedback_fresh') or not obs.get('contact_known') or obs.get('support_count',0)<4):
        c.action_result='Rejected: upright IMU, measured joints and four contacts required';return False
    sonar=obs.get('ultrasonic',{});front=[v for v in sonar.get('distance_m',[]) if v is not None]
    if c.constraints.required and (not sonar.get('both_fresh') or not front or min(front)<c.constraints.clearance+.08):
        c.action_result='Rejected: fresh front clearance required';return False
    if c.funk_bpm!=120. and not c.enable_experiments:
        c.action_result='Rejected: funk tempo outside validated 120 BPM';return False
    c.funk_plan=FunkDancePlan(c.funk_bpm,c.funk_body_gain,c.funk_lift,c.funk_speed_gain)
    audit=c.funk_audit or c.funk_plan.audit(c.gait,c.gait.q)
    if not audit['reachable']:
        c.action_result='Rejected: funk path '+audit['reason'];return False
    seed=np.array([l.fk(q) for l,q in zip(c.gait.legs,c.gait.q)])
    if np.max(np.linalg.norm(seed-c.standing,axis=1))>.01:
        c.action_result='Rejected: settle within 10 mm of nominal stance';return False
    c.funk_previous_gait=(c.gait.period,c.gait.lift,c.gait.trot_duty)
    c.funk_plan.configure(c.gait)
    c.funk_cancelled=False;c.funk_cancel_reason='';c.funk_recovery_s=0.;c.funk_support_bad_s=0.;c.funk_settled_s=0.
    c.funk_air_s=np.zeros(4);c.funk_touch_s=np.zeros(4);c.funk_armed=np.zeros(4,dtype=bool);c.funk_landings=[]
    c.funk_filtered_gyro=np.zeros(2);c.funk_body=(np.zeros(3),np.zeros(3))
    c.gait.phase=0.;c.gait.speed[:]=0.;c.gait.speed_acceleration[:]=0.;c.gait.blend=0.
    c.action='dance_funk';c.action_time=0.;c.action_result='Running: dance_funk';c.state='DANCE'
    c.pose_last_rpy=np.zeros(3)
    return True


def stop_funk(c,reason='command'):
    if not c.funk_cancelled:
        c.funk_cancelled=True;c.funk_recovery_s=0.;c.funk_recovery_body=tuple(a.copy() for a in c.funk_body)
        c.funk_cancel_reason=reason
        c.action_result='Stop requested: decelerate diagonal steps and settle all paws'
    return True


def funk_step(c,dt,roll_pitch,angular_velocity,**kwargs):
    if not np.isfinite(dt) or not 0<dt<=.1:raise ValueError('Invalid funk interval')
    rp,gyro=np.asarray(roll_pitch),np.asarray(angular_velocity);o=c.observation
    if not kwargs.get('imu_fresh',True) or not np.isfinite(rp).all() or not np.isfinite(gyro).all():
        c.fault='IMU unavailable during funk dance'
    elif np.max(abs(rp))>np.deg2rad(25):c.fault='Funk exceeded 25 degree envelope'
    elif not o.get('contact_known') or not o.get('joint_feedback_fresh'):
        c.fault='Measured contact/joint feedback lost during funk dance'
    if c.fault:
        c.action_result='FAULT: '+c.fault;c.action_events.append({'action':c.action,'result':c.action_result,'duration_s':c.action_time})
        c.action='';c.state='FAULT';restore(c);return c.gait.q.ravel().tolist()
    c.action_time+=dt;t=c.action_time;p=c.funk_plan
    forces=o.get('contact_forces_n')
    if forces is not None:
        f=np.asarray(forces)
        c.funk_air_s=np.where(f<.2,c.funk_air_s+dt,0.)
        c.funk_touch_s=np.where(f>1.,c.funk_touch_s+dt,0.)
        c.funk_armed|=c.funk_air_s>=.064
        for foot in np.flatnonzero(c.funk_armed&(c.funk_touch_s>=.048)):
            c.funk_landings.append({'foot':int(foot),'elapsed_s':t});c.funk_armed[foot]=False
    sonar=o.get('ultrasonic',{});front=[v for v in sonar.get('distance_m',[]) if v is not None]
    blocked=c.constraints.required and (not sonar.get('both_fresh') or not front or min(front)<c.constraints.clearance+.04)
    c.funk_support_bad_s=c.funk_support_bad_s+dt if o.get('support_count',0)<2 else 0.
    if blocked:stop_funk(c,'front_clearance_or_sonar')
    elif kwargs.get('joint_error',0.)>.35:stop_funk(c,'joint_tracking')
    # ROS clock subtraction can represent 64 ms as 0.064000000000001.
    # Preserve the same strict debounce boundary as the fixed-step controller.
    elif c.funk_support_bad_s>.064+1e-9:stop_funk(c,'insufficient_support')
    if c.funk_cancelled:
        c.funk_recovery_s+=dt;u=1-smooth(c.funk_recovery_s/1.5)
        body=tuple(a*u for a in c.funk_recovery_body);desired=np.zeros(3);marching=False
    else:
        body=p.body(t);desired=p.velocity(t);marching=t<p.main_end
    constrained=c.constraints.limit(desired,dt,o,kwargs.get('joint_error',0.),heading_hold=False)
    targets,expected=p.targets(c.gait,dt,t,velocity=constrained,body=body,marching=marching)
    c.funk_body=tuple(a.copy() for a in body);c.pose_last_rpy=expected.copy()
    c.funk_filtered_gyro+=min(1.,dt/.10)*(gyro-c.funk_filtered_gyro)
    correction=np.clip(c.balance_kp*(rp-expected[:2])+c.balance_kd*c.funk_filtered_gyro,-.08,.08)
    c.correction+=np.clip(correction-c.correction,-.3*dt,.3*dt)
    targets=targets@(rotation([1,0,0],c.correction[0])@rotation([0,1,0],c.correction[1])).T
    solved=np.array([l.ik(v,q,tolerance=.00005) for l,v,q in zip(c.gait.legs,targets,c.gait.q)])
    c.gait.q+=np.clip(solved-c.gait.q,-p.joint_rate*dt,p.joint_rate*dt)
    ended=c.funk_cancelled or t>=p.duration
    c.state='DANCE_RECOVERY' if ended else 'DANCE'
    body_returned=not c.funk_cancelled or c.funk_recovery_s>=1.5
    stable=(body_returned and np.linalg.norm(c.gait.speed)<.003 and c.gait.blend<.001 and o.get('support_count',0)>=4 and
            np.max(abs(rp))<np.deg2rad(3) and np.max(abs(c.funk_filtered_gyro))<.15 and kwargs.get('joint_error',0.)<.12)
    c.funk_settled_s=c.funk_settled_s+dt if ended and stable else 0.
    if c.funk_settled_s>.4:
        restore(c);c.finish('Cancelled funk dance; recovered upright ('+c.funk_cancel_reason+')' if c.funk_cancelled else 'Completed sensor-settled funk dance')
    elif (c.funk_cancelled and c.funk_recovery_s>6.) or t>p.duration+6.:
        restore(c);c.finish('Failed: funk did not settle')
    return c.gait.q.ravel().tolist()
