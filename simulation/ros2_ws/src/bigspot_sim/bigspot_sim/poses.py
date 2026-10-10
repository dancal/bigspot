"""Slow, reachable body poses with explicit support transfer before foot lift."""
import numpy as np
from .kinematics import rotation


def smooth(u):
    u=float(np.clip(u,0.,1.))
    return u**3*(10-15*u+6*u*u)


POSES={
    'low':{'shift':[0,0,-.055]},
    'high':{'shift':[0,0,.02]},
    'sit':{'shift':[-.018,0,-.05],'rpy':[0,-.08,0]},
    'lean_left':{'shift':[0,.025,0]},
    'lean_right':{'shift':[0,-.025,0]},
    'look_up':{'rpy':[0,-.10,0]},
    'look_down':{'rpy':[0,.10,0]},
    'roll_left':{'rpy':[.08,0,0]},
    'roll_right':{'rpy':[-.08,0,0]},
    'stretch':{'shift':[0,0,-.02],'rpy':[0,.14,0]},
    'body_circle':{'circle':True},
    'salute_right':{'lift':0},
    'salute_left':{'lift':1},
}
BODY_POSES={**POSES,'bow':{'rpy':[0,.12,0]},'crouch':{'shift':[0,0,-.06]}}


def support_margin(point, triangle):
    p=np.asarray(triangle)
    ab,ac=p[1]-p[0],p[2]-p[0]
    if ab[0]*ac[1]-ab[1]*ac[0]<0:
        p=p[::-1]
    return min(float(((b-a)[0]*(np.asarray(point)-a)[1]-(b-a)[1]*(np.asarray(point)-a)[0])/np.linalg.norm(b-a))
               for a,b in zip(p,np.roll(p,-1,axis=0)))


class PoseTrajectory:
    duration=5.5

    def __init__(self,name,standing,seed,com_x=-.02):
        self.name,self.description=name,BODY_POSES[name]
        self.standing,self.seed=standing.copy(),seed.copy()
        self.com=np.array([com_x,0.])
        self.lift=self.description.get('lift')
        self.shift=np.array(self.description.get('shift',[0,0,0]),dtype=float)
        self.rpy=np.array(self.description.get('rpy',[0,0,0]),dtype=float)
        self.support_margin_m=None
        if self.lift is not None:
            triangle=np.delete(standing[:,:2],self.lift,axis=0)
            self.shift[:2]=.45*(triangle.mean(axis=0)-self.com)
            self.support_margin_m=support_margin(self.com+self.shift[:2],triangle)
            if self.support_margin_m<.012:
                raise ValueError('Pose has insufficient planned three-foot support margin')

    def sample(self,t):
        # Body transfer finishes before lift, and foot landing precedes return.
        body_u=smooth(t/1.5) if t<1.5 else 1. if t<3.5 else 1-smooth((t-3.5)/1.5)
        shift=self.shift*body_u
        rpy=self.rpy*body_u
        if self.description.get('circle'):
            theta=2*np.pi*np.clip((t-1.)/3.,0,1)
            envelope=smooth(t/1.) if t<1 else 1. if t<4 else 1-smooth((t-4)/1.)
            shift[:2]=.018*envelope*np.array([np.sin(theta),1-np.cos(theta)])
        body_rotation=rotation([0,0,1],rpy[2])@rotation([0,1,0],rpy[1])@rotation([1,0,0],rpy[0])
        targets=(self.standing-shift)@body_rotation
        if self.lift is not None:
            foot_u=0. if t<1.5 else smooth((t-1.5)/.8) if t<2.3 else 1. if t<2.7 else 1-smooth((t-2.7)/.8)
            targets[self.lift,2]+=.04*foot_u
            targets[self.lift,0]+=.015*foot_u
        # Blend measured command geometry into the initial stance without a snap.
        targets+=(self.seed-self.standing)*(1-smooth(t/.8))
        return targets,rpy

    def audit(self,legs,initial):
        q=initial.copy()
        for t in np.linspace(0,self.duration,81):
            targets,_=self.sample(t)
            try:
                q=np.array([leg.ik(p,v) for leg,p,v in zip(legs,targets,q)])
            except ValueError as error:
                return {'reachable':False,'reason':str(error),'time_s':float(t)}
        return {'reachable':True,'support_margin_m':self.support_margin_m,
                'collision_and_real_mass_verified':False}
