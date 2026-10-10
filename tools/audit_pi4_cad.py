"""Exact BRep overlaps, with AABB broad phase. No blanket contact exemptions."""
import json
import sys
import argparse
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'tools'))
import build_pi4_assembly as cad
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

def bounds(shape):
    box=Bnd_Box();BRepBndLib.Add_s(shape,box)
    return np.asarray(box.Get()).reshape(2,3)

def overlaps(parts,pose):
    solids=[cad.transformed(p['shape'],pose[p['group']]) for p in parts]
    boxes=[bounds(s) for s in solids];result=[]
    for i,a in enumerate(parts):
        for j in range(i+1,len(parts)):
            b=parts[j]
            if np.any(np.minimum(boxes[i][1],boxes[j][1])-np.maximum(boxes[i][0],boxes[j][0])<=.01):continue
            op=BRepAlgoAPI_Common(solids[i],solids[j]);op.Build()
            if not op.IsDone():raise RuntimeError('Intersection calculation failed')
            props=GProp_GProps();BRepGProp.VolumeProperties_s(op.Shape(),props)
            if props.Mass()>1:
                result.append({'a':a['name'],'b':b['name'],'groups':[a['group'],b['group']],
                    'overlap_mm3':round(props.Mass(),3),'status':'requires_review'})
    return sorted(result,key=lambda x:-x['overlap_mm3'])

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--sweep',action='store_true');parser.add_argument('--recorded',action='store_true');args=parser.parse_args()
    cad.frame();cad.electronics();cad.legs();cad.consolidate_structures();cad.exterior()
    stand=cad.Gait(cad.D).stand();seed=np.tile(cad.C['startup_joint_angles_rad'],4)
    cases={'stand':stand}
    if args.sweep:
        for fraction in np.linspace(0,1,17):cases[f'rise_{fraction:.4f}']=seed+(stand-seed)*cad.smoothstep(fraction)
        for label,velocity in [('forward',[.24,0,0]),('lateral',[0,.15,0]),('turn',[0,0,.3])]:
            gait=cad.Gait(cad.D)
            for index in range(12):
                q=gait.targets(1/24,np.asarray(velocity))
                cases[f'{label}_{index}']=q
    if args.recorded:
        seen=set()
        for report in json.loads((ROOT/'validation/control_loop.json').read_text())['reports']:
            for sample in report.get('cad_pose_samples',[]):
                key=tuple(np.round(sample['joints_rad'],3))
                if key in seen:continue
                seen.add(key)
                cases[f'recorded_{report["case"]}_{sample["time_s"]:.3f}']=np.asarray(sample['joints_rad'])
        if not seen:raise RuntimeError('No recorded CAD poses. Run validate_control_loop.py --render first.')
    reports=[]
    for name,q in cases.items():
        rows=overlaps(cad.PARTS,cad.poses(q))
        reports.append({'pose':name,'joints_rad':q.tolist(),'overlaps':rows,'passed':not rows})
        print(json.dumps({'pose':name,'count':len(rows),'largest':rows[:4]}),flush=True)
    report={'revision':cad.C['revision'],'method':'Exact BRep intersections after AABB broad phase',
        'overlap_threshold_mm3':1,'sampled_poses_only':True,'continuous_sweep_certified':False,
        'strength_verified':False,'manufacturing_released':False,'reports':reports,'passed':all(r['passed'] for r in reports)}
    out=ROOT/'generated/cad_audit';out.mkdir(parents=True,exist_ok=True)
    (out/'stand_overlaps.json').write_text(json.dumps(reports[0]['overlaps'],indent=2)+'\n')
    target=ROOT/'design/pi4/interference_audit.json'
    target.write_text(json.dumps(report,indent=2)+'\n')
    raise SystemExit(0 if report['passed'] else 2)
