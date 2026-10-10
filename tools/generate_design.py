"""Export analytic CAD layout and dynamic URDF from the same selected parameters."""
import json,math,sys,xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
deps=ROOT.parent/'bigspot/simulation/generated/python_deps'
if sys.platform=='win32' and deps.exists():sys.path.insert(0,str(deps))
import numpy as np,mujoco
from bigspot_autonomy.design import load_design,LEGS
from bigspot_autonomy.model import make_mjcf,vec
from bigspot_autonomy.gait import Gait

def rpy(rotation):
    return (math.atan2(rotation[2,1],rotation[2,2]),math.atan2(-rotation[2,0],math.hypot(rotation[2,1],rotation[2,2])),math.atan2(rotation[1,0],rotation[0,0]))
def rotation(quat):
    m=np.zeros(9);mujoco.mju_quat2Mat(m,quat);return m.reshape(3,3)

def urdf(model,d):
    root=ET.Element('robot',name='bigspot_autonomy')
    for b in range(1,model.nbody):
        name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,b);link=ET.SubElement(root,'link',name=name)
        inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'origin',xyz=vec(model.body_ipos[b]),rpy='0 0 0');ET.SubElement(inertial,'mass',value=str(model.body_mass[b]))
        rot=rotation(model.body_iquat[b]);inertia=rot@np.diag(model.body_inertia[b])@rot.T
        ET.SubElement(inertial,'inertia',**{key:str(inertia[i,j]) for key,i,j in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]})
        for g in range(model.ngeom):
            if model.geom_bodyid[g]!=b:continue
            for kind in ('visual','collision'):
                if kind=='collision' and not (model.geom_contype[g] or model.geom_conaffinity[g]):continue
                node=ET.SubElement(link,kind);ET.SubElement(node,'origin',xyz=vec(model.geom_pos[g]),rpy=vec(rpy(rotation(model.geom_quat[g]))))
                geometry=ET.SubElement(node,'geometry');typ=model.geom_type[g];s=model.geom_size[g]
                if typ==mujoco.mjtGeom.mjGEOM_BOX:ET.SubElement(geometry,'box',size=vec(s*2))
                elif typ==mujoco.mjtGeom.mjGEOM_SPHERE:ET.SubElement(geometry,'sphere',radius=str(s[0]))
                else:ET.SubElement(geometry,'cylinder',radius=str(s[0]),length=str(s[1]*2))
                if kind=='visual':material=ET.SubElement(node,'material',name=f'color_{g}');ET.SubElement(material,'color',rgba=vec(model.geom_rgba[g]))
        if b>1:
            jid=model.body_jntadr[b];j=ET.SubElement(root,'joint',name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,jid),type='revolute')
            ET.SubElement(j,'parent',link=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,model.body_parentid[b]));ET.SubElement(j,'child',link=name)
            ET.SubElement(j,'origin',xyz=vec(model.body_pos[b]),rpy=vec(rpy(rotation(model.body_quat[b]))));ET.SubElement(j,'axis',xyz=vec(model.jnt_axis[jid]))
            ai=jid-1;ET.SubElement(j,'limit',lower=str(model.jnt_range[jid,0]),upper=str(model.jnt_range[jid,1]),effort=str(max(abs(model.actuator_ctrlrange[ai]))),velocity='12')
    for name,xyz,angles in [('camera_link',d['camera_xyz_m'],[0,math.radians(d['camera_downward_pitch_deg']),0]),('body_imu_link',[0,0,0],[0,0,0])]:
        ET.SubElement(root,'link',name=name);joint=ET.SubElement(root,'joint',name=name+'_mount',type='fixed');ET.SubElement(joint,'parent',link='base_link');ET.SubElement(joint,'child',link=name);ET.SubElement(joint,'origin',xyz=vec(xyz),rpy=vec(angles))
    ET.indent(root);return ET.tostring(root,encoding='unicode')+'\n'

def step_layout(model,data,out):
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox,BRepPrimAPI_MakeCylinder,BRepPrimAPI_MakeSphere
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.gp import gp_Pnt,gp_Dir,gp_Ax2,gp_Trsf
    from OCP.STEPControl import STEPControl_Writer,STEPControl_AsIs
    writer=STEPControl_Writer();solids=[]
    for g in range(model.ngeom):
        if model.geom_bodyid[g]==0:continue
        s=model.geom_size[g]*1000;p=data.geom_xpos[g]*1000;r=data.geom_xmat[g].reshape(3,3);typ=model.geom_type[g]
        if typ==mujoco.mjtGeom.mjGEOM_BOX:
            shape=BRepPrimAPI_MakeBox(gp_Pnt(-s[0],-s[1],-s[2]),2*s[0],2*s[1],2*s[2]).Shape()
            tr=gp_Trsf();tr.SetValues(*[float(x) for row in range(3) for x in (*r[row],p[row])]);shape=BRepBuilderAPI_Transform(shape,tr,True).Shape()
        elif typ==mujoco.mjtGeom.mjGEOM_SPHERE:shape=BRepPrimAPI_MakeSphere(gp_Pnt(*p),float(s[0])).Shape()
        else:
            start=p-r[:,2]*s[1];shape=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*start),gp_Dir(*r[:,2])),float(s[0]),float(s[1]*2)).Shape()
        valid=BRepCheck_Analyzer(shape).IsValid()
        if not valid:raise RuntimeError('Invalid CAD solid')
        writer.Transfer(shape,STEPControl_AsIs);solids.append({'name':mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,g),'valid':valid})
    status=writer.Write(str(out))
    return {'solid_count':len(solids),'solids':solids,'step_status':str(status),'units':'millimetres','mounting_holes_verified':False,'manufacturing_ready':False,'rails':'cylinder envelopes; brackets and actuator mounting drawings require detailed CAD'}

if __name__=='__main__':
    d=load_design(ROOT/'config/selected_design.json');out=ROOT/'design';out.mkdir(exist_ok=True)
    xml=make_mjcf(d);(out/'bigspot.xml').write_text(xml+'\n',encoding='utf-8');m=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(m)
    data.qpos[7:]=Gait(d).stand();mujoco.mj_forward(m,data)
    (out/'bigspot.urdf').write_text(urdf(m,d),encoding='utf-8')
    report=step_layout(m,data,out/'BigSpot_QDD_layout.step');(out/'cad_validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'solids':report['solid_count'],'valid':all(s['valid'] for s in report['solids']),'mass':float(m.body_mass.sum()),'manufacturing_ready':False}))
