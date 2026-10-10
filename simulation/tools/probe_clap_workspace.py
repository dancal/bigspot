"""Geometric seated-clap candidates. This is not a balance/physics result."""
import json
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
if sys.platform=='win32':sys.path.insert(0,str(ROOT/'simulation/generated/python_deps'))
sys.path.insert(0,str(ROOT/'simulation/ros2_ws/src/bigspot_sim'))
import numpy as np
from bigspot_sim.motion import MotionController
from bigspot_sim.kinematics import rotation

folder=ROOT/'simulation/ros2_ws/src/bigspot_sim/config'
c=MotionController(json.loads((folder/'model.json').read_text()),
    **json.loads((folder/'control_policy_adaptive.json').read_text())['parameters'])
q=np.array([leg.ik(p,v) for leg,p,v in zip(c.gait.legs,c.standing,c.gait.q)])
print('Standing feet',c.standing.tolist(),flush=True)
candidates=[]
for degrees in [28,30,32,35,38,40]:
    pitch=-np.deg2rad(degrees)
    # Rear body corner approaches the floor: no added seat or fixed body.
    height=.207*np.sin(-pitch)+.053*np.cos(pitch)
    for shift_x in [-.22,-.24,-.26,-.28]:
        rear=(c.standing[2:]-np.array([shift_x,0.,height-.28]))@rotation([0,1,0],pitch)
        try:rq=np.array([leg.ik(p,v) for leg,p,v in zip(c.gait.legs[2:],rear,q[2:])])
        except ValueError:continue
        for x in [.26,.28,.30,.32]:
            for z in [-.25,-.27,-.29,-.31]:
                for y in [.018,.025,.032,.04]:
                    front=np.array([[x,-y,z],[x,y,z]])
                    try:fq=np.array([leg.ik(p,v) for leg,p,v in zip(c.gait.legs[:2],front,q[:2])])
                    except ValueError:continue
                    clearance=height+(front@rotation([0,1,0],pitch).T)[:,2].min()
                    if clearance>.04:
                        candidates.append({'pitch_deg':-degrees,'height':height,'shift_x':shift_x,
                            'paw_x':x,'paw_z':z,'paw_y':y,'clearance':float(clearance),
                            'q':np.concatenate([fq,rq]).tolist()})
print('Candidates',len(candidates),flush=True)
out=ROOT/'simulation/generated/show'
out.mkdir(parents=True,exist_ok=True)
(out/'workspace_candidates.json').write_text(json.dumps(candidates,indent=2))
print(json.dumps(sorted(candidates,key=lambda r:(r['paw_y'],abs(r['pitch_deg'])))[:8],indent=2))
