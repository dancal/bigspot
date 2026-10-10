"""Tempo-based grounded dances; no foot lifts, jumps or body teleportation.

The planned support polygon and IK check are geometric checks only. Actual
joint/contact feedback and IMU are required by MotionController throughout.
"""
import numpy as np
from .kinematics import rotation
from .poses import smooth, support_margin


GROUNDED_DANCES={'dance','dance_sway','dance_bounce','dance_big','dance_rhythm','dance_show'}
DYNAMIC_DANCES={'dance_funk'}
DANCES=GROUNDED_DANCES|{'dance_steps'}|DYNAMIC_DANCES


class DanceTrajectory:
    beats=16
    ramp_seconds=2.
    amplitude=.6
    lift=None
    minimum_support_margin=.025

    def __init__(self,name,standing,seed,bpm=60.):
        if name not in GROUNDED_DANCES:
            raise ValueError('Unknown dance')
        if not np.isfinite(bpm) or not 40.<=bpm<=90.:
            raise ValueError('Dance tempo must be between 40 and 90 BPM')
        self.name,self.bpm=name,float(bpm)
        if name=='dance_big':
            self.beats,self.ramp_seconds=32,3.
        elif name=='dance_rhythm':
            self.ramp_seconds=3.
        elif name=='dance_show':
            self.beats,self.ramp_seconds=24,3.
        self.duration=2*self.ramp_seconds+self.beats*60./self.bpm
        self.standing,self.seed=standing.copy(),seed.copy()
        self.support_margin_m=None
        feet=standing[:,:2]
        center=feet.mean(axis=0)
        self.polygon=feet[np.argsort(np.arctan2(feet[:,1]-center[1],feet[:,0]-center[0]))]

    def body(self,t):
        t=float(np.clip(t,0.,self.duration))
        envelope=smooth(t/self.ramp_seconds)*smooth((self.duration-t)/self.ramp_seconds)
        beat=max(0.,(t-self.ramp_seconds)*self.bpm/60.)
        phase=2*np.pi*t*self.bpm/60.
        if self.name=='dance_show':
            phase=2*np.pi*beat/6.
            onset=smooth(beat/2.)
            sway,bob=np.sin(phase)*onset,1.+(np.cos(phase)-1.)*onset
            shifts=np.array([[0.,.035*sway,-.025],
                             [0.,0.,-.040+.040*bob],
                             [.012*sway,.035*bob,-.030+.010*bob],
                             [0.,0.,-.040+.040*bob]])
            attitudes=np.array([[.18*sway,0.,0.],
                                [0.,0.,0.],
                                [.16*sway,.06*(1+bob),0.],
                                [.06*sway*(1+bob)/2,.10*(1+bob)/2,0.]])
            index=min(3,int(beat//6))
            weight=smooth((beat-(6*index+4))/2.) if index<3 else 0.
            following=min(3,index+1)
            return (.9*envelope*(shifts[index]*(1-weight)+shifts[following]*weight),
                    .9*envelope*(attitudes[index]*(1-weight)+attitudes[following]*weight))
        if self.name=='dance_big':
            # Larger excursions use an eight-beat period. Increasing amplitude
            # without slowing the trajectory caused foot unloading in trials.
            phase/=8.
            sway,bob=np.sin(phase),np.cos(phase)
            shifts=np.array([[0.,.035*sway,-.025],
                             [0.,0.,-.040+.035*bob],
                             [.018*sway,.030*bob,-.035+.010*bob],
                             [0.,.025*sway,-.045+.020*bob]])
            attitudes=np.array([[.12*sway,0.,0.],
                                [0.,.14*sway*(1+bob)/2,0.],
                                [.10*sway,.06*(1+bob),0.],
                                [.10*sway,.07*(1+bob),0.]])
            index=min(3,int(beat//8))
            weight=smooth((beat-(8*index+6))/2.) if index<3 else 0.
            following=min(3,index+1)
            shift=shifts[index]*(1-weight)+shifts[following]*weight
            rpy=attitudes[index]*(1-weight)+attitudes[following]*weight
            scale=min(1.,(60./self.bpm)**2)
            return scale*envelope*shift,scale*envelope*rpy
        if self.name=='dance_rhythm':
            # Four-beat rhythm: three times the cadence of the large 60 BPM
            # dance at 90 BPM. Moderate excursion retains grounded support.
            phase=2*np.pi*beat/4.
            onset=smooth(beat/2.)
            sway,bob=np.sin(phase)*onset,1.+(np.cos(phase)-1.)*onset
            shifts=np.array([[0.,0.,-.020],
                             [0.,0.,-.023+.015*bob],
                             [0.,0.,-.023+.010*bob],
                             [0.,0.,-.024+.012*bob]])
            attitudes=np.array([[.070*sway,0.,0.],
                                [0.,.070*sway*(1+bob)/2,0.],
                                [.060*sway,.035*(1+bob),0.],
                                [.065*sway,.040*(1+bob),0.]])
            index=min(3,int(beat//4))
            weight=smooth((beat-(4*index+2))/2.) if index<3 else 0.
            following=min(3,index+1)
            return (.8*envelope*(shifts[index]*(1-weight)+shifts[following]*weight),
                    .8*envelope*(attitudes[index]*(1-weight)+attitudes[following]*weight))
        sway,bob=np.sin(phase/2),np.cos(phase/2)
        # Shift values are body motion with all nominal foot positions fixed.
        shifts=np.array([[0.,.018*sway,0.],
                         [0.,0.,-.014+.008*bob],
                         [.006*np.sin(phase/2),.012*sway,-.010+.005*bob],
                         [.009*np.sin(phase/2),.015*np.cos(phase/2),-.012+.005*bob]])
        attitudes=np.array([[.050*sway,0.,0.],
                            [0.,.035*np.sin(phase/2),0.],
                            [.035*sway,.030*np.cos(phase/2),0.],
                            [.040*sway,.035*np.cos(phase/2),0.]])
        if self.name=='dance_sway':
            shift,rpy=shifts[0],attitudes[0]
        elif self.name=='dance_bounce':
            shift,rpy=shifts[1],attitudes[1]
        else:
            # Each four-beat phrase crossfades over its last beat. The common
            # continuous phase prevents a position/velocity snap at a phrase.
            index=min(3,int(beat//4))
            weight=smooth(beat-(4*index+3)) if index<3 else 0.
            following=min(3,index+1)
            shift=shifts[index]*(1-weight)+shifts[following]*weight
            rpy=attitudes[index]*(1-weight)+attitudes[following]*weight
        # Faster tempo reduces excursion to keep nominal oscillation
        # acceleration from rising with frequency squared.
        scale=self.amplitude*min(1.,(60./self.bpm)**2)
        return scale*envelope*shift,scale*envelope*rpy

    def sample(self,t):
        shift,rpy=self.body(t)
        transform=rotation([0,0,1],rpy[2])@rotation([0,1,0],rpy[1])@rotation([1,0,0],rpy[0])
        targets=(self.standing-shift)@transform
        targets+=(self.seed-self.standing)*(1-smooth(t/self.ramp_seconds))
        return targets,rpy

    def lift_at(self,t):
        return None

    def support_polygon(self,t):
        return self.polygon

    def audit(self,legs,initial,prepared=None):
        if prepared is not None and prepared.get('bpm',self.bpm)!=self.bpm:
            prepared=None
        if prepared is not None and not prepared['reachable']:
            return prepared
        # The seed blend is exactly zero after the two-second ramp. ROS can
        # reuse the full nominal audit and check only this short entry segment.
        if prepared is not None and np.max(np.linalg.norm(self.seed-self.standing,axis=1))>.01:
            return {'reachable':False,'reason':'Settle within 10 mm of nominal stance before dance'}
        q=initial.copy()
        times=(np.linspace(0,self.ramp_seconds,33) if prepared is not None else
               np.linspace(0,self.duration,int(np.ceil(self.duration/.04))+1))
        peak_speed=prepared['peak_planned_joint_speed_rad_s'] if prepared is not None else 0.
        margin=prepared['support_margin_m'] if prepared is not None else float('inf')
        for index,t in enumerate(times):
            targets,_=self.sample(t)
            shift,_=self.body(t)
            margin=min(margin,support_margin(np.array([-.02,0.])+shift[:2],self.support_polygon(t)))
            try:
                solved=np.array([leg.ik(p,v) for leg,p,v in zip(legs,targets,q)])
            except ValueError as error:
                return {'reachable':False,'reason':str(error),'time_s':float(t)}
            if index:
                peak_speed=max(peak_speed,float(np.max(np.abs(solved-q))/(t-times[index-1])))
            q=solved
        self.support_margin_m=margin
        if margin<self.minimum_support_margin or peak_speed>1.:
            return {'reachable':False,'reason':'Dance exceeds planned support margin or joint speed reserve',
                    'support_margin_m':margin,'peak_planned_joint_speed_rad_s':peak_speed}
        return {'reachable':True,'support_margin_m':margin,
                'peak_planned_joint_speed_rad_s':peak_speed,'bpm':self.bpm,'duration_s':self.duration,
                'full_nominal_path_prepared':prepared is not None,
                'collision_and_real_mass_verified':False}
