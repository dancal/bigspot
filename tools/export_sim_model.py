"""Export each profile's mass/inertia and geometry to MJCF and URDF together."""
import argparse
import json
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import mujoco
from bigspot_autonomy.design import load_design
from bigspot_autonomy.model import make_mjcf
from generate_design import urdf


def export(profile):
    d=load_design(ROOT/'config'/f'{profile}_design.json')
    xml=make_mjcf(d)
    model=mujoco.MjModel.from_xml_string(xml)
    folder=ROOT/'design'/profile
    folder.mkdir(exist_ok=True)
    (folder/'bigspot.xml').write_text(xml+'\n',encoding='utf-8')
    (folder/'bigspot.urdf').write_text(urdf(model,d),encoding='utf-8')
    report={'profile':profile,'mass_kg':float(model.body_mass.sum()),'actuated_joints':model.nu,
            'cad_inertia_used':bool(d.get('link_inertials')),'collision_geometry':'analytic proxies',
            'hardware_tested':False,'isaac_tested':False}
    (folder/'model_manifest.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--profile',choices=['pi4','selected'],default='pi4')
    export(parser.parse_args().profile)
