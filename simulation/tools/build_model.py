"""Convert this repository's known Fusion-export xacro to portable ROS 2 URDF.

No arbitrary xacro evaluator: refuses unexpected macros rather than silently dropping them.
STL contact points are estimates, not replacement CAD dimensions.
"""
from pathlib import Path
import copy
import json
import struct
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / 'simulation/ros2_ws/src/bigspot_sim'
XACRO = '{http://www.ros.org/wiki/xacro}'


def vertices(path):
    raw = path.read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    if len(raw) != 84 + 50 * count:
        raise ValueError(f'Expected binary STL: {path}')
    dtype = np.dtype([('normal', '<f4', 3), ('v', '<f4', (3, 3)), ('attr', '<u2')])
    return np.frombuffer(raw, dtype=dtype, count=count, offset=84)['v'].reshape(-1, 3).astype(float) * .001


def vec(node, key='xyz'):
    return np.fromstring(node.get(key, '0 0 0'), sep=' ')


def build():
    source = ROOT / 'src/bigspot_description'
    robot = ET.parse(source / 'urdf/bigspot.xacro').getroot()
    for element in list(robot):
        if element.tag == XACRO + 'include':
            filename = element.get('filename')
            if filename.endswith('materials.xacro'):
                for material in ET.parse(source / 'urdf/materials.xacro').getroot():
                    robot.append(copy.deepcopy(material))
            elif not filename.endswith(('bigspot.trans', 'bigspot.gazebo')):
                raise ValueError(f'Unexpected include {filename}')
            robot.remove(element)
        elif element.tag == XACRO + 'property':
            robot.remove(element)
    if any(e.tag.startswith(XACRO) or '${' in str(e.attrib) for e in robot.iter()):
        raise ValueError('Unexpanded xacro: use a proper xacro processor')
    base = robot.find("link[@name='base_link']")
    shift = vec(base.find('inertial/origin'))
    # Change the root frame only; every link geometry and joint relationship is preserved.
    for origin in base.findall('.//origin'):
        origin.set('xyz', ' '.join(map(str, vec(origin) - shift)))
    for joint in robot.findall('joint'):
        if joint.find('parent').get('link') == 'base_link':
            origin = joint.find('origin')
            origin.set('xyz', ' '.join(map(str, vec(origin) - shift)))
    joints = {j.get('name'): j for j in robot.findall("joint[@type='revolute']")}
    servo_path=PACKAGE/'config/servo_rds5160.json'
    if servo_path.exists():
        servo=json.loads(servo_path.read_text(encoding='utf-8'))
        for j in joints.values():
            # Stall torque / no-load speed are limits, never simultaneous continuous ratings.
            j.find('limit').set('effort',str(servo['stall_torque_nm_at_assumed_supply']))
            j.find('limit').set('velocity',str(servo['no_load_speed_rad_s_at_assumed_supply']))
    legs = []
    mesh_audit = {}
    for link in robot.findall('link'):
        mesh = link.find('visual/geometry/mesh')
        if mesh is not None:
            points = vertices(source / 'meshes' / Path(mesh.get('filename')).name)
            points += vec(link.find('visual/origin'))
            mesh_audit[link.get('name')] = {'min_m': points.min(0).tolist(), 'max_m': points.max(0).tolist(), 'triangles': len(points)//3}
    for leg in ('FR', 'FL', 'RR', 'RL'):
        names = [leg + part + '_Joint' for part in ('S', 'L', 'F')]
        chain = []
        for name in names:
            j = joints[name]
            assert np.allclose(vec(j.find('origin'), 'rpy'), 0)
            limit = j.find('limit')
            chain.append({'name': name, 'origin': vec(j.find('origin')).tolist(),
                          'axis': vec(j.find('axis')).tolist(),
                          'lower': float(limit.get('lower')), 'upper': float(limit.get('upper')),
                          'velocity': float(limit.get('velocity')), 'effort': float(limit.get('effort'))})
        distal = robot.find(f"link[@name='{leg}F_1']")
        mesh = distal.find('visual/geometry/mesh')
        points = vertices(source / 'meshes' / Path(mesh.get('filename')).name)
        points += vec(distal.find('visual/origin'))
        bottom = points[points[:, 2] <= points[:, 2].min() + .003]
        foot = bottom.mean(0)
        foot[2] = points[:, 2].min()
        # Fixed point frame at the estimated sole center; no invented foot mass/collider.
        robot.append(ET.Element('link', name=leg.lower() + '_foot'))
        fixed = ET.SubElement(robot, 'joint', name=leg.lower() + '_foot_joint', type='fixed')
        ET.SubElement(fixed, 'parent', link=leg+'F_1')
        ET.SubElement(fixed, 'child', link=leg.lower()+'_foot')
        ET.SubElement(fixed, 'origin', xyz=' '.join(map(str, foot)), rpy='0 0 0')
        definition={'name': leg, 'joints': chain, 'foot': foot.tolist()}
        if leg in ('RR','RL'):
            definition['support_vertices_m']=np.unique(points,axis=0).tolist()
        legs.append(definition)
    # Synthetic camera frame: actual Pi camera calibration must replace this estimate.
    robot.append(ET.Element('link', name='camera_link'))
    camera_joint = ET.SubElement(robot, 'joint', name='camera_joint', type='fixed')
    ET.SubElement(camera_joint, 'parent', link='base_link')
    ET.SubElement(camera_joint, 'child', link='camera_link')
    ET.SubElement(camera_joint, 'origin', xyz='0.18 0 0.04', rpy='0 0 0')
    robot.append(ET.Element('link', name='camera_optical_frame'))
    optical_joint = ET.SubElement(robot, 'joint', name='camera_optical_joint', type='fixed')
    ET.SubElement(optical_joint, 'parent', link='camera_link')
    ET.SubElement(optical_joint, 'child', link='camera_optical_frame')
    ET.SubElement(optical_joint, 'origin', xyz='0 0 0', rpy='-1.5707963267948966 0 -1.5707963267948966')
    for mesh in robot.findall('.//mesh'):
        mesh.set('filename', 'package://bigspot_sim/meshes/' + Path(mesh.get('filename')).name)
    out = PACKAGE / 'urdf'
    out.mkdir(parents=True, exist_ok=True)
    ET.indent(robot)
    ET.ElementTree(robot).write(out/'bigspot.urdf', encoding='utf-8', xml_declaration=True)
    config = {'root_shift_m': shift.tolist(), 'legs': legs,
              'initial_angles': [[0, .5, -1], [0, .5, 1], [0, -.5, -1], [0, -.5, 1]],
              'foot_estimate': 'STL sole points; verify with Fusion CAD',
              'mesh_audit': mesh_audit}
    cfg = PACKAGE/'config'
    cfg.mkdir(parents=True, exist_ok=True)
    (cfg/'model.json').write_text(json.dumps(config, indent=2)+'\n', encoding='utf-8')
    print(f'Generated {len(legs)*3} DOF model: {out / "bigspot.urdf"}')


if __name__ == '__main__':
    build()
