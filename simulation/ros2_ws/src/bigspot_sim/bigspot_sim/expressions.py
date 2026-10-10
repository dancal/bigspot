"""Visible dog-like body gestures and a supported front-paw wave.

There is no neck joint. 'Tilt' and 'bow' describe the whole body. Foot lifting
follows weight transfer and ends before that transfer is reversed.
"""
import numpy as np
from .dance import DanceTrajectory
from .poses import smooth,support_margin

CUTES={'cute','cute_tilt','cute_wave','cute_bow'}


def keyframes(t,frames):
    for (a,shift_a,rpy_a),(b,shift_b,rpy_b) in zip(frames,frames[1:]):
        if t<=b:
            u=smooth((t-a)/(b-a))
            return (np.array(shift_a)*(1-u)+np.array(shift_b)*u,
                    np.array(rpy_a)*(1-u)+np.array(rpy_b)*u)
    return np.array(frames[-1][1],dtype=float),np.array(frames[-1][2],dtype=float)


class CuteTrajectory(DanceTrajectory):
    minimum_support_margin=.012
    durations={'cute_tilt':10.,'cute_wave':12.,'cute_bow':9.5}

    def __init__(self,name,standing,seed):
        if name not in CUTES:
            raise ValueError('Unknown cute gesture')
        super().__init__('dance',standing,seed,60.)
        self.name=name
        self.duration=sum(self.durations.values()) if name=='cute' else self.durations[name]
        self.has_foot_lift=name in ('cute','cute_wave')
        self.triangle=np.delete(standing[:,:2],0,axis=0)
        self.transfer=.45*(self.triangle.mean(axis=0)-np.array([-.02,0.]))
        if support_margin(np.array([-.02,0.])+self.transfer,self.triangle)<self.minimum_support_margin:
            raise ValueError('Front-paw wave has insufficient planned support margin')

    def part(self,t):
        t=float(np.clip(t,0.,self.duration))
        if self.name!='cute':
            return self.name,t
        for name in ['cute_tilt','cute_wave','cute_bow']:
            if t<=self.durations[name]:
                return name,t
            t-=self.durations[name]
        return 'cute_bow',self.durations['cute_bow']

    def body(self,t):
        name,t=self.part(t)
        zero=[0.,0.,0.]
        if name=='cute_wave':
            u=smooth(t/1.8)*(1-smooth((t-9.)/2.))
            return np.array([*self.transfer,0.])*u,np.zeros(3)
        if name=='cute_tilt':
            left,right=[.10,-.04,0.],[-.10,-.04,0.]
            low=[0.,0.,-.035]
            return keyframes(t,[(0.,zero,zero),(1.8,low,left),(2.8,low,left),
                (4.6,low,right),(5.6,low,right),(7.4,low,left),(8.,low,left),(10.,zero,zero)])
        shift,rpy=keyframes(t,[(0.,zero,zero),(2.,[-.015,0.,-.045],[0.,.18,0.]),
            (6.5,[-.015,0.,-.045],[0.,.18,0.]),(8.5,zero,zero),(9.5,zero,zero)])
        wiggle=smooth((t-2.)/.8)*(1-smooth((t-5.7)/.8))
        shift[1]+=.015*wiggle*np.sin(np.pi*(t-2.))
        rpy[0]+=.04*wiggle*np.sin(np.pi*(t-2.))
        return shift,rpy

    def foot_weight(self,t):
        name,t=self.part(t)
        return smooth((t-2.)/1.5)*(1-smooth((t-7.5)/1.5)) if name=='cute_wave' else 0.

    def lift_at(self,t):
        return 0 if self.foot_weight(t)>1e-6 else None

    def support_polygon(self,t):
        return self.triangle if self.lift_at(t) is not None else self.polygon

    def sample(self,t):
        targets,rpy=super().sample(t)
        name,local=self.part(t)
        if name=='cute_wave':
            foot=self.foot_weight(t)
            targets[0,2]+=.075*foot
            targets[0,0]+=.025*foot
            wave=smooth((local-3.5)/.5)*(1-smooth((local-7.)/.5))
            targets[0,1]+=.022*wave*np.sin(np.pi*(local-3.5))
            targets[0,2]+=.006*wave*np.sin(np.pi*(local-3.5))
        return targets,rpy
