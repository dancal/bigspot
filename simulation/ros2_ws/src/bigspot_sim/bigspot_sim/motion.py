"""Sensor-gated action sequences sharing the upright startup controller.

Experiments are simulation-only until mechanics, motor speed/torque, supply,
contact sensing and joint feedback are measured. No base pose manipulation.
"""
import numpy as np
from .posture import PostureController
from .kinematics import rotation
from .awareness import SensorConstraints
from .poses import POSES, BODY_POSES, PoseTrajectory
from .dance import DANCES, DYNAMIC_DANCES, DanceTrajectory
from .dance_steps import StepDanceTrajectory
from .expressions import CUTES,CuteTrajectory
from .clap import CLAPS,SeatedClapTrajectory,request_clap,stop_clap,clap_step
from .funk import FunkDancePlan,request_funk,stop_funk,funk_step

PERFORMANCES=(DANCES-DYNAMIC_DANCES)|CUTES
BODY_ACTIONS=set(BODY_POSES)|PERFORMANCES


def smooth(u):
    u=np.clip(u,0.,1.)
    return u*u*u*(10-15*u+6*u*u)


class MotionController(PostureController):
    JUMPS={'jump_left':np.array([0.,1.]),'jump_right':np.array([0.,-1.]),
           'jump_back':np.array([-1.,0.]),'jump_forward':np.array([1.,0.]),'jump':np.zeros(2)}

    def __init__(self,model,enable_experiments=False,require_ultrasonic=False,
                 obstacle_clearance=.22,obstacle_slow_distance=.6,require_joint_feedback=False,
                 dance_bpm=60.,rhythm_bpm=90.,funk_bpm=120.,funk_body_gain=1.,funk_lift=.024,funk_speed_gain=1.,**parameters):
        if not np.isfinite(dance_bpm) or not 40.<=dance_bpm<=90.:
            raise ValueError('Dance tempo must be between 40 and 90 BPM')
        if not np.isfinite(rhythm_bpm) or not 60.<=rhythm_bpm<=90.:
            raise ValueError('Rhythm tempo must be between 60 and 90 BPM')
        super().__init__(model,**parameters)
        self.dance_bpm=float(dance_bpm)
        self.rhythm_bpm=float(rhythm_bpm)
        FunkDancePlan(funk_bpm,funk_body_gain,funk_lift,funk_speed_gain)
        self.funk_bpm,self.funk_body_gain,self.funk_lift,self.funk_speed_gain=funk_bpm,funk_body_gain,funk_lift,funk_speed_gain
        self.funk_audit=None
        self.dance_audits={}
        self.enable_experiments=enable_experiments
        self.action=''
        self.action_time=0.
        self.action_result=''
        self.action_events=[]
        self.observation={}
        self.mode='walk'
        self.normal_period=self.gait.period
        self.normal_lift=self.gait.lift
        self.normal_duty=self.gait.trot_duty
        self.seen_flight=False
        self.landing_stable=0.
        self.jump_extension=.32
        self.jump_crouch=.22
        self.jump_push_seconds=.22
        self.jump_shift=.025
        self.jump_joint_rate=6.5
        self.jump_retract_height=.22
        self.constraints=SensorConstraints(self.gait.speed_limits,require_ultrasonic,
            obstacle_clearance,obstacle_slow_distance,self.gait.acceleration[0],self.gait.jerk_limits[0],
            require_joint_feedback=require_joint_feedback)
        self.pose_feedback=[]
        self.pose_cancelled=False
        self.clap_audit=None

    def observe(self,state):
        self.observation=state

    def prepare_dances(self):
        """Run expensive full-path IK checks before starting ROS sensor timers."""
        seed=(self.ground_startup.crouch_q.reshape(4,3) if self.ground_startup else self.gait.q)
        q=np.array([leg.ik(p,v) for leg,p,v in zip(self.gait.legs,self.standing,seed)])
        for name in sorted(PERFORMANCES):
            plan=(StepDanceTrajectory(self.standing,self.standing,self.tempo(name)) if name=='dance_steps' else
                  CuteTrajectory(name,self.standing,self.standing) if name in CUTES else
                  DanceTrajectory(name,self.standing,self.standing,self.tempo(name)))
            self.dance_audits[name]=plan.audit(self.gait.legs,q)
        self.clap_audit=SeatedClapTrajectory(self.standing,self.standing,self.gait.legs).audit(self.gait.legs,q)
        self.funk_audit=FunkDancePlan(self.funk_bpm,self.funk_body_gain,self.funk_lift,self.funk_speed_gain).audit(self.gait,q)

    def tempo(self,name):
        if name in DYNAMIC_DANCES:return self.funk_bpm
        return self.rhythm_bpm if name in ('dance_rhythm','dance_show','dance_steps') else self.dance_bpm

    def dance_status(self):
        if self.action in DYNAMIC_DANCES:
            return {'bpm':self.funk_bpm,'elapsed_s':self.action_time,'duration_s':self.funk_plan.duration,
                    'phrase':self.funk_plan.phrase(self.action_time),'dynamic':True,
                    'intro_s':self.funk_plan.intro,'main_end_s':self.funk_plan.main_end,
                    'pair_step_period_s':self.funk_plan.beat_seconds,'measured_landings':self.funk_landings.copy(),
                    'cancel_reason':self.funk_cancel_reason,
                    'planned_lift_foot':None,'grounded_trajectory':False}
        if self.action in PERFORMANCES:
            return {'bpm':self.pose_trajectory.bpm,'elapsed_s':self.action_time,'duration_s':self.pose_trajectory.duration,
                    'planned_lift_foot':self.pose_trajectory.lift_at(self.action_time),
                    'grounded_trajectory':self.action in DANCES and self.action!='dance_steps',
                    'measured_landings':getattr(self,'pose_landed_steps',[]).copy(),
                    'waiting_for_landing':getattr(self,'pose_landing_gate',None) is not None}
        return None

    def start(self):
        if self.state=='READY' and not self.action:
            self.mode='walk'
            self.gait.period=self.normal_period
            self.gait.lift=self.normal_lift
            self.gait.trot_duty=self.normal_duty
        super().start()

    def request(self,name):
        if name in DYNAMIC_DANCES:return request_funk(self)
        if name in CLAPS:return request_clap(self)
        if name=='stop':
            if self.action in DYNAMIC_DANCES:return stop_funk(self)
            if self.action in CLAPS:return stop_clap(self)
            if self.action in BODY_ACTIONS:
                if self.pose_cancelled:
                    return True
                self.pose_recovery_seed=np.array([leg.fk(q) for leg,q in zip(self.gait.legs,self.gait.q)])
                self.pose_recovery_rpy=self.pose_last_rpy.copy()
                self.pose_recovery_lift=(self.pose_trajectory.lift_at(self.action_time)
                    if self.action in PERFORMANCES else self.pose_trajectory.lift)
                self.pose_recovery_landed=False
                self.pose_recovery_recenter_time=0.
                if self.pose_recovery_lift is not None:
                    self.pose_recovery_landing_seed=self.pose_recovery_seed.copy()
                    i=self.pose_recovery_lift
                    self.pose_recovery_landing_seed[i,2]=np.mean(np.delete(self.pose_recovery_seed[:,2],i))
                    if hasattr(self.pose_trajectory,'landing_target'):
                        self.pose_recovery_landing_seed[i]=self.pose_trajectory.landing_target(self.action_time)[i]
                self.pose_recovery_time=0.
                self.pose_cancelled=True
                self.action_result='Stop requested; smoothly lower feet and return upright'
                return True
            if self.action in ('turn_left','turn_right'):
                self.finish('Cancelled turn; stopping')
            if self.action:
                # Do not restart/abort thrust in midair; complete landing, cancel later actions.
                self.action_result='Stop requested; complete current landing/recovery'
            else:
                self.action_result='Stopped; upright posture held'
            return True
        if name in ('rear_stand','handstand'):
            self.action_result='Rejected: vertical acrobatics unvalidated; simulation probes available separately'
            return False
        if name not in {'walk','run','bow','crouch','rear_stand_probe','handstand_probe',
                         'rear_stand_experimental','handstand_experimental','turn_left','turn_right',*self.JUMPS,*POSES,*PERFORMANCES}:
            self.action_result='Rejected: unknown action'
            return False
        if self.action or self.state not in ('READY','WALKING','STOPPING'):
            self.action_result='Rejected: robot not ready or action already active'
            return False
        if name in ('turn_left','turn_right') and self.gait.static is not None:
            self.action_result='Rejected: load multi-action trot profile for turning'
            return False
        if name in ('walk','run'):
            if self.gait.static is not None:
                self.action_result='Rejected: load multi-action trot profile first'
                return False
            self.mode=name
            if not self.gait.smooth_velocity:
                self.gait.period=self.normal_period*(.8 if name=='run' else 1.)
                self.gait.lift=self.normal_lift*(1.25 if name=='run' else 1.)
                self.gait.trot_duty=.6 if name=='run' else self.normal_duty
            self.action_result='Mode: '+name
            return True
        if name in self.JUMPS or name in ('rear_stand_probe','handstand_probe','rear_stand_experimental','handstand_experimental'):
            if not self.enable_experiments:
                self.action_result='Rejected: experimental actions disabled; simulation opt-in required'
                return False
            if self.state!='READY' or np.linalg.norm(self.gait.speed)>.005:
                self.action_result='Rejected: stop and settle before experimental action'
                return False
            if not self.observation.get('contact_known') or self.observation.get('support_count',0)<3:
                self.action_result='Rejected: fresh foot contacts required'
                return False
            if not self.observation.get('joint_feedback_fresh',False):
                self.action_result='Rejected: fresh measured joint feedback required'
                return False
        if not self.observation.get('imu_valid',False):
            self.action_result='Rejected: fresh valid IMU required'
            return False
        if np.max(np.abs(self.observation['rpy_rad'][:2]))>np.deg2rad(5):
            self.action_result='Rejected: settle upright before action'
            return False
        if name in BODY_ACTIONS:
            if self.state!='READY' or np.linalg.norm(self.gait.speed)>.005:
                self.action_result='Rejected: stop and settle before body pose'
                return False
            if name in DANCES and name not in ('dance_rhythm','dance_show','dance_steps') and self.dance_bpm>60. and not self.enable_experiments:
                self.action_result='Rejected: dance above 60 BPM requires simulation experiments; support unvalidated'
                return False
            if name in ('dance_rhythm','dance_show','dance_steps') and self.rhythm_bpm!=90. and not self.enable_experiments:
                self.action_result='Rejected: rhythm tempo other than validated 90 BPM requires simulation experiments'
                return False
        if name in BODY_ACTIONS:
            sonar=self.observation.get('ultrasonic',{})
            front=[v for v in sonar.get('distance_m',[]) if v is not None]
            if self.constraints.required and (not sonar.get('both_fresh') or not front or
                                              min(front)<self.constraints.clearance+.08):
                self.action_result='Rejected: fresh front clearance required for body pose'
                return False
            if self.constraints.require_joints and not self.observation.get('joint_feedback_fresh'):
                self.action_result='Rejected: measured joint feedback required by active profile'
                return False
            if name in PERFORMANCES and (not self.observation.get('contact_known') or
                    self.observation.get('support_count',0)<4 or not self.observation.get('joint_feedback_fresh')):
                self.action_result='Rejected: performance requires all four contacts and measured joints'
                return False
            seed=np.array([leg.fk(q) for leg,q in zip(self.gait.legs,self.gait.q)])
            try:
                trajectory=(StepDanceTrajectory(self.standing,seed,self.tempo(name)) if name=='dance_steps' else
                            CuteTrajectory(name,self.standing,seed) if name in CUTES else
                            DanceTrajectory(name,self.standing,seed,self.tempo(name)) if name in DANCES
                            else PoseTrajectory(name,self.standing,seed))
                audit=(trajectory.audit(self.gait.legs,self.gait.q,self.dance_audits.get(name))
                       if name in PERFORMANCES else trajectory.audit(self.gait.legs,self.gait.q))
            except ValueError as error:
                self.action_result='Rejected: '+str(error)
                return False
            if not audit['reachable']:
                self.action_result='Rejected: pose path '+audit['reason']
                return False
            if trajectory.lift is not None and (not self.observation.get('contact_known') or
                    self.observation.get('support_count',0)<4 or not self.observation.get('joint_feedback_fresh')):
                self.action_result='Rejected: foot lift requires all four contacts and measured joints'
                return False
            self.pose_trajectory=trajectory
            self.pose_cancelled=False
            self.pose_last_rpy=np.zeros(3)
            self.pose_filtered_gyro=np.zeros(2)
            self.pose_settled=0.
            self.pose_support_bad_time=0.
            self.pose_wait_support_s=0.
            self.pose_landing_gate=None
            self.pose_landing_confirmed_s=0.
            self.pose_landed_steps=[]
            self.pose_audit=audit
            self.pose_feedback=[]
        if name in ('rear_stand_experimental','handstand_experimental'):
            audit=self.acrobatics_path_audit(name)
            if not audit['reachable']:
                self.action_result='Rejected: '+audit['reason']
                return False
        self.action=name
        self.action_time=0.
        self.action_result='Running: '+name
        self.seen_flight=False
        self.landing_stable=0.
        if name not in ('turn_left','turn_right'):
            self.gait.speed[:]=0.
            self.gait.speed_acceleration[:]=0.
            self.gait.blend=0.
        self.action_seed=np.array([leg.fk(q) for leg,q in zip(self.gait.legs,self.gait.q)])
        if name in ('turn_left','turn_right'):
            self.turn_target=self.observation['rpy_rad'][2]+(1 if name=='turn_left' else -1)*np.pi/2
            self.turn_filtered_rate=0.
        self.transition('ACTION_PREPARE')
        return True

    def acrobatics_path_audit(self,name):
        stance=[2,3] if name=='rear_stand_experimental' else [0,1]
        sign=-1. if stance==[2,3] else 1.
        ground=self.standing[stance].copy()
        ground[:,2]=0.
        q=self.gait.q.copy()
        hip=abs(float(self.gait.legs[stance[0]].joints[0]['origin'][0]))
        for deg in range(76):
            angle=np.deg2rad(deg)
            body=np.array([ground[:,0].mean()+.02*np.cos(angle),0.,.25+hip*np.sin(angle)])
            targets=(ground-body)@rotation([0,1,0],sign*angle)
            try:
                for i,p in zip(stance,targets):
                    q[i]=self.gait.legs[i].ik(p,q[i])
            except ValueError as error:
                return {'reachable':False,'first_unreachable_pitch_deg':deg,
                        'reason':f'{name} at {deg} degrees: {error}; verify CAD joint limits'}
        return {'reachable':True,'checked_pitch_degrees':[0,75],
                'reason':'Geometric reach only; balance and collision unverified'}

    def finish(self,result):
        finishing_turn=self.action in ('turn_left','turn_right') and self.gait.smooth_velocity
        self.action_events.append({'action':self.action,'result':result,'duration_s':self.action_time})
        self.action_result=result
        self.action=''
        self.transition('STOPPING' if finishing_turn else 'READY')
        if finishing_turn:
            # The velocity filter completes deceleration. Resetting velocity or
            # acceleration here would bypass its jerk limit at turn completion.
            return
        self.gait.phase=0.
        self.gait.speed[:]=0.
        self.gait.speed_acceleration[:]=0.
        self.gait.blend=0.

    def step(self,dt,velocity,roll_pitch,angular_velocity,**kwargs):
        if self.action in DYNAMIC_DANCES:return funk_step(self,dt,roll_pitch,angular_velocity,**kwargs)
        if self.gait.smooth_velocity:
            run=self.mode=='run'
            for key,target,rate in [('period',self.normal_period*(.8 if run else 1.),.12),
                                    ('lift',self.normal_lift*(1.25 if run else 1.),.008),
                                    ('trot_duty',.6 if run else self.normal_duty,.05)]:
                current=getattr(self.gait,key)
                setattr(self.gait,key,current+np.clip(target-current,-rate*dt,rate*dt))
        if not self.action:
            desired=np.asarray(velocity)*self.speed_scale if kwargs.get('command_fresh',True) else np.zeros(3)
            constrained=self.constraints.limit(desired,dt,self.observation,kwargs.get('joint_error',0.),
                                               heading_hold=self.gait.static is None)
            return super().step(dt,constrained/self.speed_scale,roll_pitch,angular_velocity,**kwargs)
        if self.action in CLAPS:
            return clap_step(self,dt,roll_pitch,angular_velocity,**kwargs)
        if self.action in BODY_ACTIONS:
            return self.pose_step(dt,roll_pitch,angular_velocity,**kwargs)
        if self.action in ('turn_left','turn_right'):
            self.action_time+=dt
            yaw=self.observation.get('rpy_rad',[0,0,0])[2]
            error=(self.turn_target-yaw+np.pi)%(2*np.pi)-np.pi
            rate=float(np.clip(2*error,-.4,.4))
            rate=np.sign(rate)*min(abs(rate),np.sqrt(.3*abs(error)))
            self.state='READY' if self.state not in ('READY','WALKING','STOPPING') else self.state
            constrained=self.constraints.limit(np.array([0.,0.,rate])*self.speed_scale,dt,
                self.observation,kwargs.get('joint_error',0.),heading_hold=False)
            result=super().step(dt,constrained/self.speed_scale,roll_pitch,angular_velocity,**kwargs)
            if self.state=='FAULT':
                self.action=''
                return result
            self.turn_filtered_rate+=min(1.,dt/.15)*(self.observation.get('angular_velocity_rad_s',[0,0,0])[2]-self.turn_filtered_rate)
            self.landing_stable=self.landing_stable+dt if abs(error)<np.deg2rad(2) and abs(self.turn_filtered_rate)<.08 else 0.
            if self.landing_stable>.3:
                self.finish('Completed relative 90 degree turn')
            elif self.action_time>12.:
                self.finish('Failed: turn did not converge')
            return result
        if not np.isfinite(dt) or not 0<dt<=.1:
            raise ValueError('Invalid action interval')
        rp,gyro=np.asarray(roll_pitch),np.asarray(angular_velocity)
        if not kwargs.get('imu_fresh',True) or not np.isfinite(rp).all() or not np.isfinite(gyro).all():
            self.fault='IMU unavailable during action'
        elif self.action in ('rear_stand_experimental','handstand_experimental') and (abs(rp[0])>.4 or abs(rp[1])>np.deg2rad(85)):
            self.fault='Acrobatics exceeded simulated roll/pitch envelope'
        elif self.action not in ('rear_stand_experimental','handstand_experimental') and np.max(np.abs(rp))>np.deg2rad(35):
            # Acrobatics never silently bypass the existing upright fault envelope.
            self.fault='Experimental action exceeded 35 degree envelope'
        elif (self.action in self.JUMPS or self.action.endswith('_experimental')) and not self.observation.get('contact_known',False):
            self.fault='Contact feedback lost during jump'
        elif (self.action in self.JUMPS or self.action.endswith('_experimental')) and not self.observation.get('joint_feedback_fresh',False):
            self.fault='Joint feedback lost during jump'
        if self.fault:
            self.action_events.append({'action':self.action,'result':'FAULT: '+self.fault,'duration_s':self.action_time})
            self.action=''
            self.state='FAULT'
            return self.gait.q.ravel().tolist()
        self.action_time+=dt
        t=self.action_time
        targets=self.standing.copy()
        rate=self.joint_rate
        support=self.observation.get('support_count',0)
        expected_pitch=0.
        if self.action in self.JUMPS:
            direction=self.JUMPS[self.action]
            if t<.8:
                self.state='JUMP_CROUCH'
                targets=self.action_seed+(self.standing-self.action_seed)*smooth(t/.8)
                targets[:,2]=-(self.height+(self.jump_crouch-self.height)*smooth(t/.8))
            elif t<.8+self.jump_push_seconds:
                self.state='JUMP_THRUST'
                u=smooth((t-.8)/self.jump_push_seconds)
                targets[:,2]=-(self.jump_crouch+(self.jump_extension-self.jump_crouch)*u)
                targets[:,:2]-=direction*self.jump_shift*u
                rate=self.jump_joint_rate
            elif t<.8+self.jump_push_seconds+.14:
                self.state='JUMP_RETRACT'
                targets[:,2]=-self.jump_retract_height
                rate=self.jump_joint_rate
            else:
                self.state='JUMP_LAND'
                # Extend for contact; no timed reversal that forces a second bounce.
                targets[:,2]=-self.height
                stable=support>=3 and np.max(np.abs(rp))<.10 and np.max(np.abs(gyro))<.4
                self.landing_stable=max(0.,self.landing_stable+(dt if stable else -.5*dt))
                if self.landing_stable>.6:
                    self.finish('Landed after flight' if self.seen_flight else 'Failed: no measured flight')
                elif t>4.:
                    self.finish('Failed: landing did not settle')
            if self.observation.get('motion_state')=='AIRBORNE':
                self.seen_flight=True
                self.state='JUMP_FLIGHT'
        elif self.action in ('rear_stand_experimental','handstand_experimental'):
            self.state='ACROBATICS_EXPERIMENT'
            stance=[2,3] if self.action=='rear_stand_experimental' else [0,1]
            lifted=[i for i in range(4) if i not in stance]
            sign=-1. if stance==[2,3] else 1.
            ground=self.standing[stance].copy()
            ground[:,2]=0.
            shift=float(ground[:,0].mean()+.02)
            height=.25
            if t<2.:
                u=smooth(t/2.)
                targets[:,0]+=shift*u
                targets[:,2]=-(self.height+(height-self.height)*u)
            elif t<3.:
                u=smooth(t-2.)
                targets[:,0]+=shift
                targets[:,2]=-height
                targets[lifted,0]-=shift*u
                targets[lifted,2]+=.07*u
            elif t<13.:
                u=smooth((t-3.)/4.) if t<7. else (1. if t<9. else 1-smooth((t-9.)/4.))
                expected_pitch=sign*np.deg2rad(75)*u
                angle=abs(expected_pitch)
                hip_x=abs(float(self.gait.legs[stance[0]].joints[0]['origin'][0]))
                body=np.array([ground[:,0].mean()+.02*np.cos(angle),0.,height+hip_x*np.sin(angle)])
                desired_rotation=rotation([0,1,0],expected_pitch)
                targets[stance]=(ground-body)@desired_rotation
                targets[lifted,2]=-.18
            elif t<14.:
                u=smooth(t-13.)
                targets[:,0]+=shift
                targets[:,2]=-height
                targets[lifted,0]-=shift*(1-u)
                targets[lifted,2]+=.07*(1-u)
            else:
                u=smooth((t-14.)/2.)
                targets[:,0]+=shift*(1-u)
                targets[:,2]=-(height+(self.height-height)*u)
                if t>16.5:
                    self.finish('Experimental trajectory completed; success evaluated from measured contacts and pitch')
        else:
            # Deliberately bounded feasibility probes. A 15-degree weight transfer
            # is not labelled a successful two-leg stand or handstand.
            self.state='ACROBATICS_PROBE'
            u=smooth(t/1.5) if t<1.5 else (1. if t<2.5 else 1-smooth((t-2.5)/1.5))
            sign=1. if self.action=='rear_stand_probe' else -1.
            targets=targets@rotation([0,1,0],sign*.25*u).T
            lifted=[0,1] if self.action=='rear_stand_probe' else [2,3]
            targets[lifted,2]+=.05*u
            if t>4.5:
                self.finish('Probe only: vertical acrobatics unvalidated')
        # Gyro damping + attitude feedback remain active during thrust and landing.
        correction=np.clip(self.balance_kp*rp+.025*gyro,-.1,.1)
        if self.action in ('rear_stand_experimental','handstand_experimental'):
            correction=np.clip(.5*(rp-np.array([0.,expected_pitch]))+.03*gyro,-.06,.06)
        if self.action in self.JUMPS and self.state=='JUMP_THRUST':
            correction=np.clip(correction,-.02,.02)
        if self.action in ('rear_stand_probe','handstand_probe'):
            correction[1]=np.clip(.3*rp[1]+.04*gyro[1],-.08,.08)
        transform=rotation([1,0,0],correction[0])@rotation([0,1,0],correction[1])
        targets=targets@transform.T
        solved=np.array([leg.ik(p,q) for leg,p,q in zip(self.gait.legs,targets,self.gait.q)])
        self.gait.q+=np.clip(solved-self.gait.q,-rate*dt,rate*dt)
        return self.gait.q.ravel().tolist()

    def stop_is_stable(self,rp,gyro,joint_error,dt):
        obs=self.observation
        contacts=self.stop_contacts_confirmed()
        gap=getattr(self,'stop_contact_gap_s',0.)
        self.stop_contact_gap_s=0. if contacts else gap+dt
        # A brief threshold crossing must not prevent settling forever.
        # Four measured contacts are still mandatory on the READY transition.
        supported=not obs.get('contact_known') or obs.get('support_count',0)>=3
        contacts=supported and self.stop_contact_gap_s<=.08+1e-9
        joints=not self.constraints.require_joints or obs.get('joint_feedback_fresh',False)
        return super().stop_is_stable(rp,gyro,joint_error,dt) and contacts and joints

    def stop_contacts_confirmed(self):
        return not self.observation.get('contact_known') or self.observation.get('support_count',0)>=4

    def pose_step(self,dt,roll_pitch,angular_velocity,**kwargs):
        if not np.isfinite(dt) or not 0<dt<=.1:
            raise ValueError('Invalid pose interval')
        rp,gyro=np.asarray(roll_pitch),np.asarray(angular_velocity)
        if not kwargs.get('imu_fresh',True) or not np.isfinite(rp).all() or not np.isfinite(gyro).all():
            self.fault='IMU unavailable during body pose'
        elif np.max(np.abs(rp))>np.deg2rad(25):
            self.fault='Body pose exceeded 25 degree envelope'
        performance=self.action in PERFORMANCES
        lifted=(self.pose_recovery_lift if self.pose_cancelled else self.pose_trajectory.lift_at(self.action_time)) if performance else self.pose_trajectory.lift
        dancing=self.action in DANCES
        cute=self.action in CUTES
        if (((lifted is not None or performance) and not self.observation.get('contact_known')) or
            ((lifted is not None or performance or self.constraints.require_joints) and not self.observation.get('joint_feedback_fresh'))):
            self.fault='Measured contact/joint feedback lost during body action'
        if self.fault:
            self.action_result='FAULT: '+self.fault
            self.action_events.append({'action':self.action,'result':'FAULT: '+self.fault,'duration_s':self.action_time})
            self.action=''
            self.state='FAULT'
            return self.gait.q.ravel().tolist()
        advance=True
        if performance and not self.pose_cancelled and lifted is None:
            next_lift=self.pose_trajectory.lift_at(self.action_time+dt)
            forces=self.observation.get('contact_forces_n')
            if next_lift is not None and (self.observation.get('support_count',0)<4 if forces is None else
                    min(np.delete(forces,next_lift))<2.):
                advance=False
                self.pose_wait_support_s+=dt
                self.action_result='Waiting: measured stance support before lifting paw'
                if self.pose_wait_support_s>2.:
                    self.request('stop')
            else:
                self.pose_wait_support_s=0.
        static_lift=self.pose_trajectory.lift if not performance else None
        if (self.action=='dance_steps' or static_lift is not None) and not self.pose_cancelled:
            if self.pose_landing_gate is None:
                gate=(self.pose_trajectory.landing_between(self.action_time,self.action_time+dt)
                      if self.action=='dance_steps' else
                      (0,static_lift,3.5) if self.action_time<3.5<=self.action_time+dt else None)
                if gate is not None:
                    self.pose_landing_gate=gate
                    self.action_time=gate[2]
            if self.pose_landing_gate is not None:
                index,foot,_=self.pose_landing_gate
                forces=self.observation.get('contact_forces_n')
                contact=forces is not None and forces[foot]>1.
                self.pose_landing_confirmed_s=self.pose_landing_confirmed_s+dt if contact else 0.
                self.pose_wait_support_s+=dt
                advance=False
                self.action_result='Waiting: measured step landing before weight transfer'
                if self.pose_landing_confirmed_s>=.096-1e-9:
                    self.pose_landed_steps.append({'index':index,'foot':foot,'trajectory_time_s':self.action_time})
                    self.pose_landing_gate=None
                    self.pose_landing_confirmed_s=0.
                    self.pose_wait_support_s=0.
                elif self.pose_wait_support_s>2.:
                    self.request('stop')
        self.action_time+=dt if advance else 0.
        t=self.action_time
        if self.pose_cancelled:
            self.pose_recovery_time+=dt
            if self.pose_recovery_lift is not None:
                # Keep weight transferred until measured paw landing, then
                # recenter. Simultaneous unloading/landing can leave the tripod.
                landing=smooth(self.pose_recovery_time/.9)
                targets=self.pose_recovery_seed+(self.pose_recovery_landing_seed-self.pose_recovery_seed)*landing
                expected=self.pose_recovery_rpy.copy()
                forces=self.observation.get('contact_forces_n')
                contact=(self.observation.get('support_count',0)>=4 if forces is None else
                         forces[self.pose_recovery_lift]>1.)
                if self.pose_recovery_time>=.9 and contact:
                    self.pose_recovery_landed=True
                if self.pose_recovery_landed:
                    self.pose_recovery_recenter_time+=dt
                    u=smooth(self.pose_recovery_recenter_time/1.5)
                    targets=self.pose_recovery_landing_seed+(self.standing-self.pose_recovery_landing_seed)*u
                    expected=self.pose_recovery_rpy*(1-u)
                elif self.pose_recovery_time>2.5:
                    self.fault='Paw landing feedback not confirmed; weight transfer held'
                ended=self.pose_recovery_recenter_time>1.7
            else:
                u=smooth(self.pose_recovery_time/1.8)
                targets=self.pose_recovery_seed+(self.standing-self.pose_recovery_seed)*u
                expected=self.pose_recovery_rpy*(1-u)
                ended=self.pose_recovery_time>2.
        else:
            targets,expected=self.pose_trajectory.sample(t)
            ended=t>self.pose_trajectory.duration
        self.pose_last_rpy=expected.copy()
        lifted=(self.pose_recovery_lift if self.pose_cancelled else self.pose_trajectory.lift_at(t)) if performance else lifted
        prefix='DANCE' if dancing else 'CUTE' if cute else 'POSE'
        self.state=prefix+'_RECOVERY' if self.pose_cancelled or ended else prefix
        # Track desired tilt rather than fighting it with an always-level target.
        self.pose_filtered_gyro+=min(1.,dt/.12)*(gyro-self.pose_filtered_gyro)
        error=rp-expected[:2]
        correction=np.clip(self.balance_kp*error+self.balance_kd*self.pose_filtered_gyro,-.07,.07)
        self.correction+=np.clip(correction-self.correction,-.25*dt,.25*dt)
        transform=rotation([1,0,0],self.correction[0])@rotation([0,1,0],self.correction[1])
        targets=targets@transform.T
        measured_lag=kwargs.get('joint_error',0.)
        sonar=self.observation.get('ultrasonic',{})
        front=[v for v in sonar.get('distance_m',[]) if v is not None]
        blocked=self.constraints.required and (not sonar.get('both_fresh') or not front or
                                                min(front)<self.constraints.clearance+.04)
        measured_forces=self.observation.get('contact_forces_n')
        support_bad=(self.observation.get('support_count',0)<3 if measured_forces is None else
                     min(np.delete(measured_forces,lifted))<=1.) if lifted is not None else False
        if performance:
            instant_bad=support_bad if lifted is not None else self.observation.get('support_count',0)<3
            self.pose_support_bad_time=(self.pose_support_bad_time+dt
                if instant_bad else 0.)
            support_bad=self.pose_support_bad_time>.08+1e-9
        if not self.pose_cancelled and (blocked or measured_lag>.35 or (support_bad and (performance or t>1.5))):
            self.request('stop')
            self.action_result='Recovering: front clearance, joint lag or support insufficient'
        solved=np.array([leg.ik(p,q,tolerance=.00005)
            for leg,p,q in zip(self.gait.legs,targets,self.gait.q)])
        # Apply the same precision and 40 ms command filter to all body routines.
        # A coarse IK deadband turns a smooth foot path into stair-step angles.
        gain=min(1.,dt/.04)
        self.gait.q+=np.clip((solved-self.gait.q)*gain,-1.2*dt,1.2*dt)
        self.pose_feedback.append({'time_s':t,'target_rpy_rad':expected.tolist(),'attitude_error_rad':error.tolist()})
        stable=np.max(np.abs(rp))<np.deg2rad(3) and measured_lag<.12 and np.max(np.abs(self.pose_filtered_gyro))<.15
        if performance or static_lift is not None:
            stable=stable and self.observation.get('support_count',0)>=4
        self.pose_settled=self.pose_settled+dt if ended and stable else 0.
        if self.pose_settled>.4:
            kind='dance' if dancing else 'gesture' if cute else 'pose'
            self.finish(f'Cancelled {kind}; recovered upright' if self.pose_cancelled else f'Completed sensor-settled {kind}')
        elif (self.pose_cancelled and self.pose_recovery_time>6.) or t>self.pose_trajectory.duration+6:
            self.finish('Failed: body pose did not settle')
        return self.gait.q.ravel().tolist()
