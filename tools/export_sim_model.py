"""Export each profile's mass/inertia and geometry to MJCF and URDF together."""
import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import mujoco
from bigspot_autonomy.design import load_design
from bigspot_autonomy.model import make_mjcf
from generate_design import urdf

def cad_visuals(text,folder):
    data=json.loads((folder/'cad_meshes.json').read_text(encoding='utf-8'))
    root=ET.fromstring(text);links={link.get('name'):link for link in root.findall('link')}
    for group in {p['group'] for p in data['parts']}:
        for node in list(links[group].findall('visual')):links[group].remove(node)
    batches={}
    for part in data['parts']:batches.setdefault((part['group'],tuple(part['color'])),[]).append(part)
    directory=folder/'visual_meshes';directory.mkdir(exist_ok=True)
    for index,((group,color),parts) in enumerate(sorted(batches.items())):
        filename=f'{group}_{index:02}.obj';offset=0
        with (directory/filename).open('w',encoding='utf-8') as out:
            for part in parts:
                vertices=part['vertices'];faces=part['indices']
                for i in range(0,len(vertices),3):out.write('v '+' '.join(f'{v/1000:.9g}' for v in vertices[i:i+3])+'\n')
                for i in range(0,len(faces),3):out.write('f '+' '.join(str(v+offset+1) for v in faces[i:i+3])+'\n')
                offset+=len(vertices)//3
        visual=ET.SubElement(links[group],'visual')
        ET.SubElement(visual,'origin',xyz='0 0 0',rpy='0 0 0')
        geometry=ET.SubElement(visual,'geometry')
        ET.SubElement(geometry,'mesh',filename=f'package://bigspot_autonomy/design/{folder.name}/visual_meshes/{filename}',scale='1 1 1')
        material=ET.SubElement(visual,'material',name=f'cad_color_{index}')
        ET.SubElement(material,'color',rgba=' '.join(map(str,(*color,1.))))
    ET.indent(root);return ET.tostring(root,encoding='unicode')+'\n',len(batches)


def export(profile):
    d=load_design(ROOT/'config'/f'{profile}_design.json')
    xml=make_mjcf(d)
    model=mujoco.MjModel.from_xml_string(xml)
    folder=ROOT/'design'/profile
    folder.mkdir(exist_ok=True)
    (folder/'bigspot.xml').write_text(xml+'\n',encoding='utf-8')
    text=urdf(model,d);mesh_count=0
    if (folder/'cad_meshes.json').is_file():text,mesh_count=cad_visuals(text,folder)
    (folder/'bigspot.urdf').write_text(text,encoding='utf-8')
    report={'profile':profile,'mass_kg':float(model.body_mass.sum()),'actuated_joints':model.nu,
            'cad_inertia_used':bool(d.get('link_inertials')),'collision_geometry':'analytic proxies',
            'hardware_tested':False,'isaac_tested':False}
    report['cad_visual_meshes']=mesh_count
    (folder/'model_manifest.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--profile',choices=['pi4','selected'],default='pi4')
    export(parser.parse_args().profile)
