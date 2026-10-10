"""One-paw dance steps with weight transfer and measured landing checkpoints.

Targets are joint-driven foot paths, never base poses or external forces. The
geometric COM estimate is only a planning check; physical support is measured.
"""
import numpy as np
from .dance import DanceTrajectory
from .poses import smooth,support_margin


class StepDanceTrajectory(DanceTrajectory):
    has_foot_lift=True
    minimum_support_margin=.012
    ramp_seconds=2.
    order=(0,3,1,2)  # FR, RL, FL, RR: alternating side and front/rear.
    step_beats=8.5
    clearance=.040

    def __init__(self,standing,seed,bpm=90.):
        super().__init__('dance',standing,seed,bpm)
        self.name='dance_steps'
        self.step_seconds=self.step_beats*60/self.bpm
        self.steps_end=self.ramp_seconds+len(self.order)*self.step_seconds
        self.duration=self.steps_end+8*60/self.bpm+self.ramp_seconds
        self.triangles=[np.delete(standing[:,:2],i,axis=0) for i in range(4)]
        self.transfers=[.45*(p.mean(axis=0)-np.array([-.02,0.])) for p in self.triangles]
        for shift,p in zip(self.transfers,self.triangles):
            if support_margin(np.array([-.02,0.])+shift,p)<self.minimum_support_margin:
                raise ValueError('Step has insufficient planned tripod support margin')

    def part(self,t):
        local=(float(t)-self.ramp_seconds)*self.bpm/60
        if 0<=local<self.step_beats*len(self.order):
            i=int(local//self.step_beats)
            return i,self.order[i],local-self.step_beats*i
        return None,None,0.

    def body(self,t):
        index,foot,beat=self.part(t)
        if foot is not None:
            transfer=smooth(beat/2)*(1-smooth((beat-7)/1.5))
            return np.array([*self.transfers[foot],-.020])*transfer,np.zeros(3)
        if self.steps_end<=t<=self.duration:
            local=t-self.steps_end
            envelope=smooth(local/2.)*(1-smooth((local-3.5)/2.))
            phase=2*np.pi*local/(6*60/self.bpm)
            # A grounded bounce phrase links the last landing to the finish.
            return np.array([0.,.020*np.sin(phase),-.050])*envelope,np.zeros(3)
        return np.zeros(3),np.zeros(3)

    def lift_at(self,t):
        _,foot,beat=self.part(t)
        return foot if foot is not None and 2<beat<7 else None

    def support_polygon(self,t):
        foot=self.lift_at(t)
        return self.triangles[foot] if foot is not None else self.polygon

    def sample(self,t):
        targets,rpy=super().sample(t)
        _,foot,beat=self.part(t)
        if foot is not None:
            height=smooth((beat-2)/1.1)*(1-smooth((beat-5.2)/1.3))
            tap=smooth(beat-3.2)*(1-smooth(beat-4.2))
            # Tap away and retract before touchdown: all other paws stay put.
            targets[foot,2]+=self.clearance*height
            targets[foot,0]+=(.045 if foot<2 else -.035)*tap
            targets[foot,1]+=(.018 if foot in (1,3) else -.018)*tap
        return targets,rpy

    def landing_between(self,a,b):
        """Pause at planted height, before reversing the weight transfer."""
        for index,foot in enumerate(self.order):
            checkpoint=self.ramp_seconds+(index*self.step_beats+6.65)*60/self.bpm
            if a<checkpoint<=b:
                return index,foot,checkpoint
        return None

    def landing_target(self,t):
        targets,_=self.sample(t)
        shift,_=self.body(t)
        foot=self.lift_at(t)
        if foot is not None:
            targets[foot]=self.standing[foot]-shift
        return targets
