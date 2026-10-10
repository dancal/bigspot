from motion_simulation import ROOT,MotionController
from bigspot_sim.clap import SeatedClapTrajectory,rotation
import numpy as np,json
f=ROOT/'simulation/ros2_ws/src/bigspot_sim/config'
c=MotionController(json.loads((f/'model.json').read_text()),**json.loads((f/'control_policy_adaptive.json').read_text())['parameters'])
p=SeatedClapTrajectory(c.standing,c.standing,c.gait.legs)
reference=np.mean(p.rear_seated@rotation([0,1,0],p.pitch).T,axis=0)
for deg in [35,30,25,20,15,10,5,0]:
 pitch=-np.deg2rad(deg);knee=-p.rear_world_angle+pitch+.8
 rear=np.array([leg.fk([0,-.8,sign*knee]) for leg,sign in zip(c.gait.legs[2:],[-1,1])])
 delta=np.mean(rear@rotation([0,1,0],pitch).T,axis=0)-reference
 shift=p.seat_shift-delta;shift[1]=0
 target=(c.standing-shift)@rotation([0,1,0],pitch);target[2:]=rear
 try:q=np.array([leg.ik(pos,q) for leg,pos,q in zip(c.gait.legs,target,c.gait.q)])
 except ValueError as err:q=str(err)
 print(deg,shift.tolist(),'height',.28+shift[2],q)
