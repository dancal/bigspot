"""Reconstruct closed planar CAD solids from STL and export a named STEP assembly.

This retains STL tessellation error; it does not recover original sketches/cylinders.
Touching shells are separated using face connectivity through manifold edges.
Every exported solid must pass OCCT validity, volume and bounding-box checks.
"""
from pathlib import Path
import argparse
import json
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'simulation/generated/python_deps'))
from build_model import vertices
from audit_stl import assembly
from OCP.BRep import BRep_Builder
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace, BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeSolid
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.BRepGProp import BRepGProp
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepLib import BRepLib
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Pnt
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.TopAbs import TopAbs_FACE, TopAbs_SHELL, TopAbs_SOLID, TopAbs_IN
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDocStd import TDocStd_Document
from OCP.TDataStd import TDataStd_Name
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_Reader, STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone


def face_components(triangles):
    """Return edge-connected face sets; a shared edge with >2 faces is not a join."""
    _, indices = np.unique(np.round(triangles.reshape(-1,3)/1e-5).astype(np.int64),
                           axis=0, return_inverse=True)
    faces = indices.reshape(-1,3)
    edge_faces = {}
    for i, face in enumerate(faces):
        for a,b in zip(face,np.roll(face,-1)):
            edge_faces.setdefault(tuple(sorted((int(a),int(b)))),[]).append(i)
    parent = np.arange(len(faces))
    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for adjacent in edge_faces.values():
        if len(adjacent)==2:
            a,b = map(find,adjacent)
            parent[b]=a
    groups = {}
    for i in range(len(faces)):
        groups.setdefault(find(i),[]).append(i)
    for group in groups.values():
        subset = faces[group]
        edges = np.sort(np.concatenate([subset[:,[0,1]],subset[:,[1,2]],subset[:,[2,0]]]),axis=1)
        _, counts = np.unique(edges,axis=0,return_counts=True)
        if not np.all(counts==2):
            raise ValueError('An edge-connected shell is not closed; repair must be reviewed')
        yield triangles[group]


def count(shape, kind):
    explorer=TopExp_Explorer(shape,kind)
    n=0
    while explorer.More():
        n+=1
        explorer.Next()
    return n


def compound(shapes):
    builder=BRep_Builder()
    result=TopoDS_Compound()
    builder.MakeCompound(result)
    for shape in shapes:
        builder.Add(result,shape)
    return result


def bounds(shape):
    box=Bnd_Box()
    BRepBndLib.AddOptimal_s(shape,box,False,False)
    return np.array(box.Get()).reshape(2,3)


def volume(shape):
    prop=GProp_GProps()
    BRepGProp.VolumeProperties_s(shape,prop)
    return prop.Mass()


def reconstruct(triangles):
    solids=[]
    rejected_merges=0
    shells=[]
    for shell_number, faces in enumerate(face_components(triangles),1):
        sewing=BRepBuilderAPI_Sewing(1e-5)
        for triangle in faces:
            polygon=BRepBuilderAPI_MakePolygon()
            for point in triangle:
                polygon.Add(gp_Pnt(*point.tolist()))
            polygon.Close()
            face=BRepBuilderAPI_MakeFace(polygon.Wire(),True)
            if not face.IsDone():
                raise ValueError('Planar triangle face construction failed')
            sewing.Add(face.Face())
        sewing.Perform()
        sewed=sewing.SewedShape()
        if count(sewed,TopAbs_SHELL)!=1:
            raise ValueError(f'Expected one sewn shell, found {count(sewed,TopAbs_SHELL)}')
        if sewed.ShapeType()==TopAbs_SHELL:
            shell=TopoDS.Shell_s(sewed)
        else:
            shell=TopoDS.Shell_s(TopExp_Explorer(sewed,TopAbs_SHELL).Current())
        signed=float(np.einsum('ij,ij->i',faces[:,0],np.cross(faces[:,1],faces[:,2])).sum()/6)
        shells.append({'shell':shell,'volume':signed,'sample':faces[0].mean(0)})
    # Negative shells are internal voids, not extra solid matter. Preserve their orientation.
    outer=[s for s in shells if s['volume']>0]
    cavities=[s for s in shells if s['volume']<0]
    makers=[BRepBuilderAPI_MakeSolid(s['shell']) for s in outer]
    for cavity in cavities:
        containers=[]
        for i,s in enumerate(outer):
            classifier=BRepClass3d_SolidClassifier(makers[i].Solid(),gp_Pnt(*cavity['sample'].tolist()),1e-6)
            if classifier.State()==TopAbs_IN:
                containers.append(i)
        if not containers:
            raise ValueError('Reversed shell is not contained in an outer shell; inspect normals')
        selected=min(containers,key=lambda i:outer[i]['volume'])
        makers[selected].Add(cavity['shell'])
    for solid_number,maker in enumerate(makers,1):
        solid=maker.Solid()
        if not BRepLib.OrientClosedSolid_s(solid):
            raise ValueError('Cannot orient closed solid with cavities')
        if not BRepCheck_Analyzer(solid).IsValid():
            raise ValueError(f'Invalid CAD solid, solid {solid_number}')
        unify=ShapeUpgrade_UnifySameDomain(solid,True,True,False)
        unify.SetLinearTolerance(1e-7)
        unify.SetAngularTolerance(1e-7)
        unify.Build()
        optimized=unify.Shape()
        if (not BRepCheck_Analyzer(optimized).IsValid() or count(optimized,TopAbs_SOLID)!=1
                or abs(volume(optimized)-volume(solid))/volume(solid)>1e-5):
            # Preserve the verified faceted solid when simplification is invalid.
            optimized=solid
            rejected_merges+=1
        solids.append(optimized)
    result=compound(solids)
    reference_volume=abs(np.einsum('ij,ij->i',triangles[:,0],
                                  np.cross(triangles[:,1],triangles[:,2])).sum()/6)
    relative_error=abs(volume(result)-reference_volume)/reference_volume
    reference_bounds=np.array([triangles.reshape(-1,3).min(0),triangles.reshape(-1,3).max(0)])
    bound_error=np.max(np.abs(bounds(result)-reference_bounds))
    if relative_error>1e-5 or bound_error>.001:
        raise ValueError(f'Geometry changed: volume error={relative_error}, bounds={bound_error} mm')
    return result,{'solids':len(solids),'cad_faces':count(result,TopAbs_FACE),
                   'closed_shells':len(shells),'internal_cavity_shells':len(cavities),
                   'rejected_coplanar_merges':rejected_merges,
                   'valid':True,'volume_mm3':volume(result),'volume_relative_error':relative_error,
                   'max_bound_error_mm':float(bound_error)}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--link',help='Optional single-link conversion for diagnostics')
    args=parser.parse_args()
    output=ROOT/'simulation/generated/cad'
    output.mkdir(parents=True,exist_ok=True)
    reconstruction=assembly()
    doc=TDocStd_Document(TCollection_ExtendedString('BigSpot STL reconstruction'))
    shape_tool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    assembly_label=shape_tool.AddShape(compound([]),False)
    TDataStd_Name.Set_s(assembly_label,TCollection_ExtendedString('BigSpot'))
    report={'units':'mm','method':'closed planar BRep; coplanar merge attempted with validity/volume fallback',
            'faceted_geometry':True,
            'original_parametric_history_recovered':False,'parts':[],
            'joint_source':'original URDF, not newly measured axes','joints':reconstruction['joints']}
    for mesh in reconstruction['meshes']:
        if args.link and mesh['link']!=args.link:
            continue
        started=time.monotonic()
        print(f"Converting {mesh['link']}: {mesh['triangles']} triangles",flush=True)
        triangles=vertices(ROOT/'src/bigspot_description/meshes'/mesh['file']).reshape(-1,3,3)*1000
        shape,metrics=reconstruct(triangles)
        part_label=shape_tool.AddShape(shape,False)
        TDataStd_Name.Set_s(part_label,TCollection_ExtendedString(mesh['link']))
        instance=shape_tool.AddComponent(assembly_label,part_label,TopLoc_Location())
        TDataStd_Name.Set_s(instance,TCollection_ExtendedString(mesh['link']))
        report['parts'].append({'link':mesh['link'],**metrics})
        print(f"  {metrics['solids']} valid solids, {metrics['cad_faces']} faces, "
              f"volume error={metrics['volume_relative_error']:.2g}, {time.monotonic()-started:.1f}s",flush=True)
    shape_tool.UpdateAssemblies()
    target=output/(f"{args.link}.step" if args.link else 'BigSpot_reconstructed.step')
    writer=STEPCAFControl_Writer()
    writer.SetNameMode(True)
    if not writer.Transfer(doc,STEPControl_AsIs) or writer.Write(str(target))!=IFSelect_RetDone:
        raise RuntimeError('STEP export failed')
    # Read the file independently through the STEP reader: verifies persisted geometry.
    reader=STEPControl_Reader()
    if reader.ReadFile(str(target))!=IFSelect_RetDone or reader.TransferRoots()==0:
        raise RuntimeError('STEP re-import failed')
    persisted=reader.OneShape()
    expected=sum(p['solids'] for p in report['parts'])
    if count(persisted,TopAbs_SOLID)!=expected or not BRepCheck_Analyzer(persisted).IsValid():
        raise RuntimeError('STEP round-trip solid count/validity failed')
    report['step_roundtrip']={'valid':True,'solids':expected,'volume_mm3':volume(persisted)}
    expected_volume=sum(p['volume_mm3'] for p in report['parts'])
    if abs(volume(persisted)-expected_volume)/expected_volume>1e-5:
        raise RuntimeError('STEP round-trip volume changed')
    target.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(f'Exported and re-imported: {target}; {expected} validated solids',flush=True)


if __name__=='__main__':
    main()
