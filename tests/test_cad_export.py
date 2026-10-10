"""CAD-to-URDF contracts: units, body-local frames, inertias and offline assets."""
import json
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from bigspot_autonomy.asset_paths import resolve_meshes

ROOT=Path(__file__).resolve().parents[1]

class CadExportTests(unittest.TestCase):
    def test_cad_frames_units_and_physical_model_are_preserved(self):
        root=ET.parse(ROOT/'design/pi4/bigspot.urdf')
        source=json.loads((ROOT/'design/pi4/cad_meshes.json').read_text(encoding='utf-8'))
        groups={}
        for part in source['parts']:groups.setdefault(part['group'],[]).extend(np.asarray(part['vertices']).reshape(-1,3)/1000)
        for link in root.findall('link'):
            if link.get('name') not in groups:continue
            vertices=[]
            for visual in link.findall('visual'):
                np.testing.assert_array_equal(np.fromstring(visual.find('origin').get('xyz'),sep=' '),0)
                mesh=visual.find('geometry/mesh')
                path=ROOT/mesh.get('filename').removeprefix('package://bigspot_autonomy/')
                for line in path.read_text(encoding='utf-8').splitlines():
                    if line.startswith('v '):vertices.append([float(v) for v in line.split()[1:]])
            actual=np.asarray(vertices);expected=np.asarray(groups[link.get('name')])
            np.testing.assert_allclose([actual.min(axis=0),actual.max(axis=0)],[expected.min(axis=0),expected.max(axis=0)],atol=1e-8,rtol=0)
        design=json.loads((ROOT/'config/pi4_design.json').read_text())
        self.assertAlmostEqual(sum(float(m.get('value')) for m in root.findall('.//inertial/mass')),sum(x['mass_kg'] for x in design['link_inertials'].values()),places=8)
        self.assertEqual(sum(j.get('type')=='revolute' for j in root.findall('joint')),12)
        self.assertFalse(root.findall('.//collision/geometry/mesh'))

    def test_isaac_assets_resolve_without_a_ros_workspace(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'generated',prefix='urdf-test-') as tmp:
            path=resolve_meshes(ROOT/'design/pi4/bigspot.urdf',ROOT,Path(tmp)/'resolved.urdf')
            original=ET.parse(ROOT/'design/pi4/bigspot.urdf').findall('.//mesh')
            resolved=ET.parse(path).findall('.//mesh')
            self.assertTrue(original);self.assertEqual(len(original),len(resolved))
            for before,after in zip(original,resolved):
                asset=ROOT/before.get('filename').removeprefix('package://bigspot_autonomy/')
                self.assertTrue(asset.is_file())
                self.assertEqual(after.get('filename'),asset.resolve().as_uri())

if __name__=='__main__':unittest.main()
