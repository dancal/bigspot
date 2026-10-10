"""Rear-shank-supported sitting and three measured front-paw claps.

Requires rear-support contact and front-paw contact telemetry. The stock PWM/IMU
hardware does not supply this feedback; no real robot execution is claimed.
"""
import numpy as np
from .kinematics import rotation
from .poses import smooth
from .expressions import keyframes

CLAPS={'seated_clap'}


class SeatedClapTrajectory:
    duration=43.5
    return_duration=18.
    lift_start=6.
    landing_start=22.
    landing_end=25.5
    clap_start=9.
    clap_period=4.2
    clap_repetitions=3
    pitch=-np.deg2rad(35.)
    seat_height=.24528418156202506
    rear_world_angle=-np.deg2rad(88.5)
    seat_shift=np.array([-.10,0.,seat_height-.28])

    def __init__(self,standing,seed,legs):
        self.standing,self.seed=standing.copy(),seed.copy()
        self.legs=legs
        self.seat_shift=self.seat_shift.copy()
        self.seat_shift[2]=self.seat_height+float(standing[0,2])
        self.open_paws=np.array([[.127,-.10,-.30],[.127,.10,-.30]])
        self.open_q=np.array([leg.ik(p,[0.,.5,(-1 if i==0 else 1)*.8],tolerance=.00001)
            for i,(leg,p) in enumerate(zip(legs[:2],self.open_paws))])
        self.closed_q=np.array([[legs[0].lower[0]+.00005,.12,-.20],
                               [legs[1].upper[0]-.00005,.12,.20]])
        self.closed_paws=np.array([leg.fk(q) for leg,q in zip(legs[:2],self.closed_q)])
        # Place the existing rear lower-leg meshes flat on the floor. Trying
        # to lower the torso onto the floor drove the rear knees below it.
        knee=-self.rear_world_angle+self.pitch+.8
        self.rear_seated=np.array([legs[2].fk([0.,-.8,-knee]),legs[3].fk([0.,-.8,knee])])
        self.rear_standing_q=np.array([leg.ik(p,[0.,.5,sign*1.]) for leg,p,sign in zip(legs[2:],standing[2:],[-1,1])])

    def body(self,t):
        if t>self.landing_end:return self.return_body(t-self.landing_end)[:2]
        zero=[0.,0.,0.]
        return keyframes(t,[(0.,zero,zero),(2.,[-.03,0.,-.015],[0.,-.1,0.]),
            (6.,self.seat_shift,[0.,self.pitch,0.]),
            (self.landing_end,self.seat_shift,[0.,self.pitch,0.])])

    def planted(self,t):
        if t>self.landing_end:return self.return_sample(t-self.landing_end)
        raw_t=t
        shift,rpy=self.body(t)
        ground=self.standing.copy()
        landing_relocation=.04*smooth((raw_t-self.landing_start)/3.5)*(1-smooth((raw_t-self.landing_end)/6.5))
        relocation=.04*smooth(t/2.)+landing_relocation
        ground[:2,0]-=relocation
        targets=(ground-shift)@rotation([0,1,0],rpy[1])
        rear=smooth((t-2.)/4.)
        targets[2:]=targets[2:]*(1-rear)+self.rear_seated*rear
        return targets,rpy

    def return_body(self,u):
        # Unfolding the rear knees first drove their long meshes into the
        # floor. Lower the head while keeping both rear shanks flat, then
        # transfer to a normal four-paw crouch before standing.
        pitch=self.pitch+(np.deg2rad(-5.)-self.pitch)*smooth(u/8.)
        knee=-self.rear_world_angle+pitch+.8
        rear_q=np.array([[0.,-.8,-knee],[0.,-.8,knee]])
        rear=np.array([leg.fk(q) for leg,q in zip(self.legs[2:],rear_q)])
        initial=np.mean(self.rear_seated@rotation([0,1,0],self.pitch).T,axis=0)
        delta=np.mean(rear@rotation([0,1,0],pitch).T,axis=0)-initial
        shift=self.seat_shift-delta;shift[1]=0.
        rpy=np.array([0.,pitch,0.])
        if u>8.:
            blend=smooth((u-8.)/10.)
            rear_q=np.array([[0.,-.8,-knee],[0.,-.8,knee]])*(1-blend)+self.rear_standing_q*blend
            rpy*=1-blend
            rear=np.array([leg.fk(q) for leg,q in zip(self.legs[2:],rear_q)])
            height=max(leg.support_height(q,rpy[1]) for leg,q in zip(self.legs[2:],rear_q))
            shift[0]*=1-blend;shift[2]=height+self.standing[0,2]
        return shift,rpy,rear,rear_q

    def return_sample(self,u):
        shift,rpy,rear,_=self.return_body(u)
        ground=self.standing.copy()
        ground[:2,0]-=.08*(1-smooth(u/8.))
        targets=(ground-shift)@rotation([0,1,0],rpy[1])
        targets[2:]=rear
        return targets,rpy

    def clap_weight(self,t):
        elapsed=t-self.clap_start
        if not 0.<=elapsed<self.clap_period*self.clap_repetitions:return 0.
        phase=(elapsed%self.clap_period)/(self.clap_period/2)
        return smooth(phase) if phase<=1. else 1-smooth(phase-1.)

    def lifted(self,t):
        return self.lift_start<t<self.landing_end

    def sample(self,t):
        t=float(np.clip(t,0.,self.duration))
        targets,rpy=self.planted(t)
        lift=smooth((t-6.)/2.5)*(1-smooth((t-self.landing_start)/3.5))
        # Interpolate joints during clapping. Cartesian interpolation near full
        # leg extension has a large IK derivative despite a slow paw path.
        front=np.array([leg.fk(q) for leg,q in zip(self.legs[:2],
            self.open_q+(self.closed_q-self.open_q)*self.clap_weight(t))])
        targets[:2]=targets[:2]*(1-lift)+front*lift
        targets+=(self.seed-self.standing)*(1-smooth(t/2.))
        return targets,rpy

    def audit(self,legs,initial):
        q=initial.copy();peak=0.;previous=0.;peak_t=0.
        for t in np.linspace(0.,self.duration,601):
            targets,_=self.sample(t)
            try:solved=np.array([leg.ik(p,v) for leg,p,v in zip(legs,targets,q)])
            except ValueError as error:return {'reachable':False,'time_s':float(t),'reason':str(error)}
            if t>self.landing_end:solved[2:]=self.return_body(t-self.landing_end)[3]
            if t>0:
                speed=float(np.max(np.abs(solved-q))/(t-previous))
                if speed>peak:peak,peak_t=speed,float(t)
            previous,q=t,solved
        return {'reachable':peak<=1.,'peak_planned_joint_speed_rad_s':peak,
                'reason':'Joint speed reserve exceeded' if peak>1. else '',
                'peak_time_s':peak_t,'mechanical_collision_verified':False,'rear_support_contact_required':True}


def request_clap(control):
    obs=control.observation
    if control.action or control.state!='READY' or np.linalg.norm(control.gait.speed)>.005:
        control.action_result='Rejected: stop and settle before seated clap';return False
    if not obs.get('imu_valid') or np.max(np.abs(obs.get('rpy_rad',[1,1])[:2]))>np.deg2rad(5):
        control.action_result='Rejected: fresh upright IMU required';return False
    if not obs.get('contact_known') or obs.get('support_count',0)<4 or not obs.get('joint_feedback_fresh'):
        control.action_result='Rejected: four measured contacts and joints required';return False
    if not obs.get('seat_feedback',{}).get('fresh'):
        control.action_result='Rejected: rear-support and paw contact telemetry required';return False
    sonar=obs.get('ultrasonic',{})
    if control.constraints.required and (not sonar.get('both_fresh') or
            min([v for v in sonar.get('distance_m',[]) if v is not None] or [0.])<.30):
        control.action_result='Rejected: fresh front clearance required';return False
    seed=np.array([leg.fk(q) for leg,q in zip(control.gait.legs,control.gait.q)])
    plan=SeatedClapTrajectory(control.standing,seed,control.gait.legs)
    audit=control.clap_audit or plan.audit(control.gait.legs,control.gait.q)
    if not audit['reachable']:
        control.action_result='Rejected: seated clap path '+audit['reason'];return False
    if np.max(np.linalg.norm(seed-control.standing,axis=1))>.01:
        control.action_result='Rejected: settle near nominal stance';return False
    control.clap_trajectory=plan
    control.clap_cancelled=False;control.clap_recovery_s=0.;control.clap_recenter_s=0.
    control.clap_landed=False;control.clap_count=0;control.clap_contact_armed=True
    control.clap_wait_s=0.;control.clap_support_bad_s=0.;control.clap_settled_s=0.
    control.clap_gyro=np.zeros(2);control.pose_last_rpy=np.zeros(3)
    control.action='seated_clap';control.action_time=0.;control.action_result='Started sensor-gated seated clap'
    control.state='SEATED_LOWER'
    return True


def stop_clap(control):
    if control.clap_cancelled:return True
    control.clap_cancelled=True;control.clap_recovery_s=0.
    control.clap_recovery_seed=np.array([leg.fk(q) for leg,q in zip(control.gait.legs,control.gait.q)])
    control.clap_recovery_rpy=control.pose_last_rpy.copy()
    control.clap_recovery_start_t=control.action_time
    control.clap_recovery_lifted=control.clap_trajectory.lifted(control.action_time)
    control.clap_landing_target=control.clap_trajectory.planted(control.clap_trajectory.landing_end)[0]
    control.action_result='Stopping clap; open paws, confirm landing, then stand'
    return True


def clap_step(control,dt,rp,gyro,**kwargs):
    if not np.isfinite(dt) or not 0<dt<=.1:raise ValueError('Invalid seated clap interval')
    rp,gyro=np.asarray(rp),np.asarray(gyro)
    obs=control.observation;seat=obs.get('seat_feedback',{});forces=obs.get('contact_forces_n',[0.]*4)
    if not kwargs.get('imu_fresh',True) or not np.isfinite(rp).all() or not np.isfinite(gyro).all():
        control.fault='IMU unavailable during seated clap'
    elif not obs.get('contact_known') or not obs.get('joint_feedback_fresh') or not seat.get('fresh'):
        control.fault='Seat, paw, foot or joint feedback unavailable during seated clap'
    elif abs(rp[0])>np.deg2rad(22.) or abs(rp[1])>np.deg2rad(43.):
        control.fault='Seated clap exceeded roll/pitch envelope'
    if control.fault:
        control.finish('FAULT: '+control.fault);control.state='FAULT'
        return control.gait.q.ravel().tolist()
    t=control.action_time;plan=control.clap_trajectory
    seated=(seat.get('rear_support_force_n',0.)>3. and min(forces[2:])>2.
            and seat.get('rear_support_margin_m',-1.)>.012)
    landed=min(forces[:2])>1.
    gate=(t<6.<=t+dt and not seated) or (t<plan.landing_end<=t+dt and not landed)
    if gate and not control.clap_cancelled:
        control.clap_wait_s+=dt
        control.action_result='Waiting for measured rear support' if t<6. else 'Waiting for both paw landings'
        if control.clap_wait_s>3.:stop_clap(control)
    else:
        control.clap_wait_s=0.
        control.action_time+=dt
    t=control.action_time
    if control.clap_cancelled:
        control.clap_recovery_s+=dt
        r=control.clap_recovery_s
        if control.clap_recovery_lifted:
            open_target=control.clap_recovery_seed.copy();open_target[:2]=plan.open_paws
            if r<1.:
                targets=control.clap_recovery_seed+(open_target-control.clap_recovery_seed)*smooth(r)
            else:
                targets=open_target+(control.clap_landing_target-open_target)*smooth((r-1.)/2.5)
            expected=control.clap_recovery_rpy.copy()
            if r>=3.5 and landed:control.clap_landed=True
            if control.clap_landed:
                control.clap_recenter_s+=dt
                targets,expected=plan.sample(plan.landing_end+control.clap_recenter_s)
            elif r>6.5:control.fault='Both paw landings unconfirmed; seated body held'
            ended=control.clap_recenter_s>plan.return_duration+.5
        else:
            start_t=min(6.,control.clap_recovery_start_t)
            replay=start_t*(1-smooth(r/6.5))
            targets,expected=plan.planted(replay)
            original,original_rpy=plan.planted(start_t)
            targets+=(control.clap_recovery_seed-original)*(1-smooth(r/.8))
            expected+=(control.clap_recovery_rpy-original_rpy)*(1-smooth(r/.8))
            ended=r>7.
        control.state='SEATED_RECOVERY'
    else:
        targets,expected=plan.sample(t)
        ended=t>plan.duration
        control.state=('SEATED_LOWER' if t<6. else 'SEATED_PAWS_UP' if t<9. else
            'SEATED_CLAP' if t<plan.landing_start else 'SEATED_PAWS_DOWN' if t<plan.landing_end else 'SEATED_RISE')
    if plan.clap_start<=t<plan.landing_start and not control.clap_cancelled:
        contact=seat.get('paw_contact_force_n',0.)>.5 and seat.get('paw_contact_tip_offset_m',1.)<.065
        contact=contact and seat.get('paw_gap_m',1.)<.09 and min(seat.get('front_paw_height_m',[0.,0.]))>.035
        if seat.get('paw_gap_m',0.)>.14:control.clap_contact_armed=True
        if contact and control.clap_contact_armed:
            control.clap_count+=1;control.clap_contact_armed=False
    support_bad=plan.lifted(t) and not seated
    control.clap_support_bad_s=control.clap_support_bad_s+dt if support_bad else 0.
    sonar=obs.get('ultrasonic',{})
    blocked=control.constraints.required and (not sonar.get('both_fresh') or
            min([v for v in sonar.get('distance_m',[]) if v is not None] or [0.])<.26)
    if not control.clap_cancelled and (control.clap_support_bad_s>.12 or blocked or kwargs.get('joint_error',0.)>.35):
        stop_clap(control)
    control.pose_last_rpy=expected.copy()
    rising=(not control.clap_cancelled and t>=plan.landing_end) or (control.clap_cancelled and control.clap_landed)
    control.clap_gyro+=min(1.,dt/.12)*(gyro-control.clap_gyro)
    correction=np.clip(control.balance_kp*(rp-expected[:2])+control.balance_kd*control.clap_gyro,-.05,.05)
    control.correction+=np.clip(correction-control.correction,-.25*dt,.25*dt)
    corrected=targets@(rotation([1,0,0],control.correction[0])@rotation([0,1,0],control.correction[1])).T
    # Only planted paws can exert a balancing ground force. Rotating lifted
    # paws with the attitude correction also pushed their inward reach beyond
    # the shoulder limit during closure.
    front_ground=1-smooth((t-plan.lift_start)/2.5)*(1-smooth((t-plan.landing_start)/3.5))
    targets[2:]=corrected[2:]
    targets[:2]+=front_ground*(corrected[:2]-targets[:2])
    solved=[]
    for i,(leg,p,q) in enumerate(zip(control.gait.legs,targets,control.gait.q)):
        if rising and i>=2:
            return_t=control.clap_recenter_s if control.clap_cancelled else t-plan.landing_end
            rear_q=plan.return_body(return_t)[3][i-2].copy()
            rear_q[0]+=float(np.clip(-.25*rp[0]-.02*gyro[0],-.035,.035))
            solved.append(rear_q);continue
        try:solved.append(leg.ik(p,q,tolerance=.00005 if i<2 and plan.lifted(t) and not control.clap_cancelled else .0005))
        except ValueError as error:
            raise ValueError(str(error)+f'; clap t={t:.3f}, target={p.tolist()}, seed={q.tolist()}') from error
    solved=np.array(solved)
    control.gait.q+=np.clip(solved-control.gait.q,-1.2*dt,1.2*dt)
    stable=max(abs(rp))<np.deg2rad(3.) and max(abs(control.clap_gyro))<.15 and kwargs.get('joint_error',0.)<.12 and obs.get('support_count',0)>=4
    control.clap_settled_s=control.clap_settled_s+dt if ended and stable else 0.
    if control.fault:
        control.finish('FAULT: '+control.fault);control.state='FAULT'
    elif control.clap_settled_s>.4:
        control.finish('Cancelled seated clap; recovered upright' if control.clap_cancelled else
            'Completed seated clap; three measured paw contacts' if control.clap_count>=3 else 'Failed: fewer than three measured paw claps')
    elif control.clap_recovery_s>plan.return_duration+8. or t>plan.duration+6.:
        control.finish('Failed: seated clap did not recover')
    return control.gait.q.ravel().tolist()
