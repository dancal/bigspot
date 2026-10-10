"""CAD-derived rear support-face angles and free-body physics candidates."""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from motion_simulation import episode,ROOT
from build_model import vertices
from bigspot_sim.clap import SeatedClapTrajectory
from bigspot_sim.kinematics import Gait,rotation
import numpy as np

parser=argparse.ArgumentParser()
parser.add_argument('--angles',nargs='+',type=float,default=[86.,84.,82.])
parser.add_argument('--geometry-only',action='store_true')
parser.add_argument('--holdout',action='store_true')
args=parser.parse_args()
folder=ROOT/'simulation/ros2_ws/src/bigspot_sim'
robot=ET.parse(folder/'urdf/bigspot.urdf').getroot()
gait=Gait(json.loads((folder/'config/model.json').read_text()))
for angle in args.angles:
    SeatedClapTrajectory.rear_world_angle=-np.deg2rad(angle)
    knee=-SeatedClapTrajectory.rear_world_angle+SeatedClapTrajectory.pitch+.8
    cloud=[]
    for index,sign in [(2,-1.),(3,1.)]:
        leg=gait.legs[index];q=[0.,-.8,sign*knee]
        R=np.eye(3);p=np.zeros(3)
        for joint,value in zip(leg.joints,q):
            p+=R@np.array(joint['origin']);R=R@rotation(joint['axis'],value)
        link=robot.find("link[@name='"+leg.name+"F_1']")
        mesh=vertices(ROOT/'src/bigspot_description/meshes'/(leg.name+'F_1.stl'))
        mesh+=np.fromstring(link.find('visual/origin').get('xyz'),sep=' ')
        world=(p+mesh@R.T)@rotation([0,1,0],SeatedClapTrajectory.pitch).T
        cloud.append(world)
    height=-min(v[:,2].min() for v in cloud)-.0005
    SeatedClapTrajectory.seat_height=height
    print(json.dumps({'rear_world_angle_deg':-angle,'cad_height_m':height,
        'lowest_points':[v[np.argmin(v[:,2])].tolist() for v in cloud]}),flush=True)
    if args.geometry_only:continue
    report=episode('seated_clap',seconds=40.,profile='adaptive',output_folder='show',
        case=('holdout_seat_' if args.holdout else 'seat_')+str(round(angle*10)),
        **({'mass_scale':1.15,'friction':.55,'imu_noise':True} if args.holdout else {}))
    print(json.dumps({k:report.get(k) for k in ['end_state','fault','action_result','measured_paw_clap_count',
        'seated_clap_success','rear_support_min_margin_during_clap_m','front_paw_min_gap_m']}),flush=True)
