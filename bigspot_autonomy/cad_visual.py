"""Non-colliding CAD render meshes; existing inertia/contact physics are retained."""
import json
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

class CadVisuals:
    def __init__(self,source,output):
        self.parts=json.loads(Path(source).read_text(encoding='utf-8'))['parts']
        self.output=Path(output);self.output.mkdir(parents=True,exist_ok=True)

    def decorate(self,xml):
        root=ET.fromstring(xml);asset=ET.SubElement(root,'asset')
        bodies={b.get('name'):b for b in root.findall('.//body')}
        for i,p in enumerate(self.parts):
            vertices=np.asarray(p['vertices']).reshape(-1,3)/1000
            faces=np.asarray(p['indices']).reshape(-1,3)+1
            path=self.output/f'part_{i:03}.obj'
            with path.open('w') as f:
                for v in vertices:f.write('v '+' '.join(f'{x:.9g}' for x in v)+'\n')
                for face in faces:f.write('f '+' '.join(str(x) for x in face)+'\n')
            name=f'cad_{i:03}'
            ET.SubElement(asset,'mesh',name=name,file=str(path.resolve()))
            ET.SubElement(bodies[p['group']],'geom',name=name,type='mesh',mesh=name,
                group='3',mass='0',contype='0',conaffinity='0',rgba=' '.join(map(str,p['color']+[1.])))
        visual=root.find('visual');ET.SubElement(visual,'global',offwidth='1280',offheight='720')
        ET.SubElement(visual,'quality',shadowsize='2048')
        ET.SubElement(visual,'headlight',ambient='0.4 0.4 0.4',diffuse='0.65 0.65 0.65',specular='0.15 0.15 0.15')
        return ET.tostring(root,encoding='unicode')
