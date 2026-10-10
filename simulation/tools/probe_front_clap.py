"""Near-midline front sole workspace within unchanged URDF joint limits."""
import json
import numpy as np
import xml.etree.ElementTree as ET
from build_model import vertices
from motion_simulation import ROOT
from bigspot_sim.motion import MotionController
folder=ROOT/'simulation/ros2_ws/src/bigspot_sim/config'
c=MotionController(json.loads((folder/'model.json').read_text()),
    **json.loads((folder/'control_policy_adaptive.json').read_text())['parameters'])
out=[]
for x in [.11,.127,.14,.15,.17]:
    for z in [-.330,-.335,-.340,-.342,-.344]:
        for y in [.005,.006,.008,.010,.012,.016,.02]:
            try:
                q=np.array([leg.ik([x,sign*y,z],seed) for leg,sign,seed in zip(c.gait.legs[:2],[-1,1],c.gait.q[:2])])
                out.append({'x':x,'y':y,'z':z,'q':q.tolist()})
            except ValueError:pass
print(json.dumps(sorted(out,key=lambda p:(p['y'],abs(p['x']-.127)))[:15],indent=2))
robot=ET.parse(folder.parent/'urdf/bigspot.urdf').getroot()
for name in ['FR','FL']:
    link=robot.find("link[@name='"+name+"F_1']")
    v=vertices(ROOT/'src/bigspot_description/meshes'/(name+'F_1.stl'))
    v+=np.fromstring(link.find('visual/origin').get('xyz'),sep=' ')
    tip=v[v[:,2]<-.175]
    print(name,'CAD tip region bounds',tip.min(0).tolist(),tip.max(0).tolist())
(ROOT/'simulation/generated/show/front_workspace.json').write_text(json.dumps(out,indent=2))
