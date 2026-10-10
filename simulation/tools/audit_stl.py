"""Inspect STL topology before converting reference geometry to CAD solids.

Coordinates are welded at 0.00001 mm only for topology inspection. No STL is edited.
Closed edge topology does not prove absence of self intersections or usable materials.
"""
from pathlib import Path
import json
import sys
import xml.etree.ElementTree as ET
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_model import ROOT, vertices


def inspect(path):
    triangles = vertices(path).reshape(-1, 3, 3)
    points, inverse = np.unique(np.round(triangles.reshape(-1, 3) / 1e-8).astype(np.int64),
                                axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    ordered = np.sort(edges, axis=1)
    unique, mapping, counts = np.unique(ordered, axis=0, return_inverse=True, return_counts=True)
    signs = np.where(edges[:, 0] < edges[:, 1], 1, -1)
    balance = np.bincount(mapping, weights=signs, minlength=len(unique))
    parent = np.arange(len(points))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a, b in unique:
        a, b = find(a), find(b)
        if a != b:
            parent[b] = a
    shells = len({find(i) for i in range(len(points))})
    area = np.linalg.norm(np.cross(triangles[:, 1]-triangles[:, 0],
                                   triangles[:, 2]-triangles[:, 0]), axis=1) / 2
    signed_volume = np.einsum('ij,ij->i', triangles[:, 0],
                             np.cross(triangles[:, 1], triangles[:, 2])).sum() / 6
    xyz = triangles.reshape(-1, 3) * 1000
    return {
        'file': path.name, 'triangles': len(faces), 'unique_vertices': len(points),
        'connected_vertex_sets': shells, 'boundary_edges': int(np.count_nonzero(counts == 1)),
        'nonmanifold_edges': int(np.count_nonzero(counts > 2)),
        'nonmanifold_edge_points_mm': (points[unique[counts > 2]] * 0.00001).tolist(),
        'nonmanifold_edge_face_counts': counts[counts > 2].tolist(),
        'inconsistent_shared_edge_directions': int(np.count_nonzero((counts == 2) & (balance != 0))),
        'degenerate_triangles': int(np.count_nonzero(area < 1e-16)),
        'duplicate_triangles': int(len(faces)-len(np.unique(np.sort(faces, axis=1), axis=0))),
        'bounds_min_mm': xyz.min(0).tolist(), 'bounds_max_mm': xyz.max(0).tolist(),
        'extent_mm': np.ptp(xyz, axis=0).tolist(), 'surface_area_mm2': float(area.sum()*1e6),
        'signed_volume_mm3': float(signed_volume*1e9),
        'closed_oriented_edge_topology': bool(np.all(counts == 2) and np.all(balance == 0)),
        'self_intersections_checked': False,
    }


def assembly():
    source = ROOT/'src/bigspot_description'
    robot = ET.parse(source/'urdf/bigspot.xacro').getroot()
    links = {link.get('name'): link for link in robot.findall('link')}
    world = {'base_link': np.zeros(3)}
    pending = list(robot.findall('joint'))
    joints = []
    while pending:
        ready = [j for j in pending if j.find('parent').get('link') in world]
        if not ready:
            raise ValueError('Disconnected joint tree')
        for joint in ready:
            origin = joint.find('origin')
            if not np.allclose(np.fromstring(origin.get('rpy', '0 0 0'), sep=' '), 0):
                raise ValueError('Nonzero joint rpy requires full transform composition')
            p, c = joint.find('parent').get('link'), joint.find('child').get('link')
            world[c] = world[p]+np.fromstring(origin.get('xyz'), sep=' ')
            limit = joint.find('limit')
            if joint.get('type') == 'revolute':
                joints.append({'name': joint.get('name'), 'parent': p, 'child': c,
                               'point_mm': (world[c]*1000).tolist(),
                               'axis': np.fromstring(joint.find('axis').get('xyz'), sep=' ').tolist(),
                               'lower_rad': float(limit.get('lower')), 'upper_rad': float(limit.get('upper'))})
            elif joint.get('type') != 'fixed':
                raise ValueError(f"Unsupported joint type: {joint.get('type')}")
            pending.remove(joint)
    meshes = []
    for name, link in links.items():
        mesh = link.find('visual/geometry/mesh')
        if mesh is None:
            continue
        origin = link.find('visual/origin')
        # Original exporter stored vertices in world space and compensated in each visual.
        compensation = world[name]+np.fromstring(origin.get('xyz'), sep=' ')
        if not np.allclose(compensation, 0, atol=1e-6):
            raise ValueError(f'{name}: STL is not in CAD world coordinates')
        if not np.allclose(np.fromstring(origin.get('rpy', '0 0 0'), sep=' '), 0):
            raise ValueError('Visual rotation must be composed')
        meshes.append({'link': name, **inspect(source/'meshes'/Path(mesh.get('filename')).name)})
    return {'units': 'mm', 'source': 'STL geometry and original URDF zero pose; axes not measured from CAD',
            'weld_tolerance_mm': 0.00001, 'meshes': meshes, 'joints': joints}


def main():
    report = assembly()
    output = ROOT/'simulation/ros2_ws/src/bigspot_sim/config/reconstruction.json'
    output.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    for m in report['meshes']:
        print(f"{m['link']}: {m['triangles']} triangles, {m['connected_vertex_sets']} vertex sets, "
              f"boundary={m['boundary_edges']}, nonmanifold={m['nonmanifold_edges']}, "
              f"orientation={m['inconsistent_shared_edge_directions']}, degenerate={m['degenerate_triangles']}")
    print(f"Saved {len(report['meshes'])} mesh audits and {len(report['joints'])} joints: {output}")


if __name__ == '__main__':
    main()
