"""Build named, articulated Pi4 CAD; unknown supplier parts stay marked reservations.

Coordinates are mm, base origin is the body centre, +X forward, +Y left.
Only the specified mounting patterns are released as geometric patterns; no
strength, motor-thread revision or electrical commissioning is implied.
"""
import copy, csv, json, math, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
if sys.platform == 'win32':
    sys.path.insert(0, str(ROOT.parent/'bigspot/simulation/generated/python_deps'))
import numpy as np
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeSphere
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse, BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Pnt, gp_Dir, gp_Ax2, gp_Trsf
from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location
from bigspot_autonomy.design import load_design, LEGS
from bigspot_autonomy.gait import Gait
from bigspot_autonomy.kinematics import smoothstep

OUT = ROOT/'design/pi4'
PARTS = []
D = load_design(ROOT/'config/selected_design.json')
C = json.loads((ROOT/'config/pi4_assembly.json').read_text(encoding='utf-8'))
D['standing_leg_height_m']=C.get('standing_leg_height_mm',245)/1000
D['neutral_foot_x_m']=C.get('neutral_foot_x_mm',45)/1000

def box(size, xyz=(0,0,0)):
    p = np.asarray(xyz)-np.asarray(size)/2
    return BRepPrimAPI_MakeBox(gp_Pnt(*map(float,p)), *map(float,size)).Shape()

def cylinder(radius, length, xyz=(0,0,0), axis=(0,0,1)):
    p = np.asarray(xyz)-np.asarray(axis)*length/2
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*map(float,p)),gp_Dir(*map(float,axis))), float(radius), float(length)).Shape()

def cut(shape, tool):
    op=BRepAlgoAPI_Cut(shape, tool); op.Build()
    if not op.IsDone(): raise RuntimeError('Boolean cut failed')
    return op.Shape()

def fuse(a,b):
    op=BRepAlgoAPI_Fuse(a,b);op.Build()
    if not op.IsDone():raise RuntimeError('Boolean fuse failed')
    return op.Shape()

def holes(shape, points, radius, length, axis=(0,0,1)):
    for xyz in points: shape=cut(shape,cylinder(radius,length,xyz,axis))
    return shape

def circle_points(pcd, count, xyz=(0,0,0), axis='y', phase=0):
    out=[]
    for i in range(count):
        a=phase+2*math.pi*i/count; p=list(xyz)
        axes=(0,2) if axis=='y' else (1,2) if axis=='x' else (0,1)
        p[axes[0]]+=pcd/2*math.cos(a);p[axes[1]]+=pcd/2*math.sin(a);out.append(p)
    return out

def add(name, shape, group='base_link', color=(.65,.69,.73), material='6061-T6', mass=None, status='custom_geometry', notes=''):
    if not BRepCheck_Analyzer(shape).IsValid():raise RuntimeError('Invalid solid: '+name)
    p=GProp_GProps();BRepGProp.VolumeProperties_s(shape,p)
    volume=p.Mass()
    if mass is None:mass=volume*(.0000027 if material=='6061-T6' else .00000125)
    com=p.CentreOfMass(); inertia=p.MatrixOfInertia();scale=mass/volume/1e6
    record={'name':name,'group':group,'color':list(color),'material':material,'mass_kg':mass,'volume_mm3':volume,
            'com_mm':[com.X(),com.Y(),com.Z()], 'inertia_kg_m2':[[inertia.Value(i+1,j+1)*scale for j in range(3)] for i in range(3)],
            'status':status,'notes':notes,'valid':True,'shape':shape}
    PARTS.append(record);return record

def rotor(name, group, origin, side, pcd, radius, thread):
    # Outer adapter face lives inward of the motor face, leaving the stator ring free.
    y=origin[1]-side*5
    shape=cylinder(radius,4,(origin[0],y,origin[2]),(0,1,0))
    pts=circle_points(pcd,6,(origin[0],y,origin[2]),axis='y')
    shape=holes(shape,pts,1.7 if thread=='M3' else 2.25,8,(0,1,0))
    shape=holes(shape,[(origin[0],y,origin[2])],6,8,(0,1,0))
    return add(name,shape,group,notes=f'{thread} clearance holes; screw engagement measured before assembly')

def frame():
    f=C['frame'];length,width,t=f['length'],f['width'],f['plate_thickness']
    mountpts=[]
    for x in (-178,-92,92,178):
        for y in (-85,85):mountpts.append((x,y,0))
    bottom=box((length,width,t),(0,0,f['bottom_z']))
    # Weight relief avoids battery tray and board fixing strips.
    for x in (-150,-58,48,152):
        for y in (-58,58):bottom=cut(bottom,box((43,29,6),(x,y,f['bottom_z'])))
    bottom=holes(bottom,[(x,y,f['bottom_z']) for x,y,_ in mountpts],1.7,6)
    # General slot grid, including battery straps and interchangeable power modules.
    for x in (-120,-20,60,120):
        for y in (-36,36):bottom=cut(bottom,box((23,3.4,6),(x,y,f['bottom_z'])))
    add('Lower_Frame_380x190_t2',bottom,notes='Laser/waterjet blank; deburr, keep reinforced motor posts')
    top=box((length,width,t),(0,0,f['top_z']))
    for x in (-120,-10,105):top=cut(top,box((88,65,6),(x,0,f['top_z'])))
    top=holes(top,[(x,y,f['top_z']) for x,y,_ in mountpts],1.7,6)
    top=holes(top,[(100,-65,f['top_z'])],11.1,6)
    add('Removable_Top_Service_Frame',top,notes='22mm NC E-stop opening; top access to battery, Pi4 and MCU')
    for x,y,_ in mountpts:
        post=cylinder(4,84,(x,y,0));post=holes(post,[(x,y,0)],1.25,90)
        add(f'Frame_M3_Standoff_{x}_{y}',post)
    for side in (-1,1):
        wall=box((300,2,68),(0,side*92,0))
        for x in (-95,0,95):wall=cut(wall,box((66,6,40),(x,side*92,0)))
        add(f'Side_Web_{side}',wall)
    # Ground support sits below all body electronics; the robot settles on this when OFF.
    skid=box((300,160,3),(0,0,-63.5))
    for x in (-90,0,90):skid=cut(skid,box((55,110,8),(x,0,-63.5)))
    add('Belly_Ground_Skid',skid,material='UHMW-PE',color=(.15,.18,.21),notes='Bottom z=-65mm; body supports weight while unpowered')
    for x in (-130,130):
        for y in (-65,65):add(f'Skid_Support_{x}_{y}',box((10,10,18),(x,y,-53)))

def electronics():
    for e in C['electronics']:
        shape=box(e['size'],e['xyz'])
        if e['id']=='pi4':
            # Official board holes are 58 x 49, offset from the board centre.
            cx,cy,z=e['xyz'];px=cx-42.5+3.5
            pts=[(px+dx,cy-28+3.5+dy,z) for dx in (0,58) for dy in (0,49)]
            shape=holes(shape,pts,1.35,6)
            for i,(x,y,z) in enumerate(pts):
                p=cylinder(3,9,(x,y,z-5.3));p=holes(p,[(x,y,z-5.3)],1.2,12)
                add(f'Pi4_M2_5_Standoff_{i}',p,notes='Official 58x49mm pitch, asymmetric on 85x56mm PCB')
            trayz=z-11.5
            tray=box((90,64,2),(cx-8,0,trayz))
            tray=holes(tray,[(x,y,trayz) for x,y,_ in pts],1.35,6)
            add('Pi4_Replaceable_Tray',tray)
            for hy in (-31,31):
                # Feet stay forward of the battery; tray bridges backwards over it.
                add(f'Pi4_Tray_Support_{hy}',box((10,10,trayz+42),(145,hy,(trayz-42)/2)))
            for i,y in enumerate((-15,3,20)):
                size=(18,15,15) if i else (21,17,13)
                add(f'Pi4_USB_Ethernet_Port_{i}',box(size,(cx+37,y,z+8.5)),mass=0,status='connector_envelope',material='purchased',color=(.68,.70,.74))
            add('Pi4_40_Pin_Header',box((51,5,8),(cx-11,cy+24,z+4.8)),mass=0,material='purchased',color=(.1,.1,.1))
        add(e['name'],shape,mass=e['mass_kg'],material='purchased',color=e['color'],status=e['footprint'],notes=e.get('requires',''))
        if e['id'] in ('dcdc','mcu','bms','switch','fuses','regen','current','watchdog'):
            sx,sy,sz=e['size'];x,y,z=e['xyz']
            # Slotted strap-mount plinths do not invent an unverified supplier hole pattern.
            base=box((sx+8,sy+6,2),(x,y,z-sz/2-2))
            for hx in (-sx/2+8,sx/2-8):base=cut(base,box((3.2,sy-3,6),(x+hx,y,z-sz/2-2)))
            add(e['id']+'_Slotted_Mounting_Plinth',base,notes='Supplier-specific holes remain un-drilled; retention straps through slots')
    # Actual cells are visible independently; insulating tray encloses a supplier-built pack.
    b=C['battery'];x,y,z=b['position'];tray=box(b['pack_envelope'],(x,y,z))
    inner=np.asarray(b['pack_envelope'])-np.array([4,4,2]);inner[2]+=6
    tray=cut(tray,box(inner,(x,y,z+3)))
    for hx in (-45,45):
        for hy in (-37,37):tray=cut(tray,box((10,3,65),(x+hx,y+hy,z)))
    add('12S_Battery_Insulated_Tray_and_Strap_Slots',tray,material='PETG',color=(.18,.23,.3))
    for col in range(6):
        for row in range(2):
            xyz=(x+(col-2.5)*23,y,z+(row-.5)*23)
            add(f'P42A_21700_Cell_{col*2+row+1:02}',cylinder(b['cell_diameter']/2,b['cell_length'],xyz,(0,1,0)),mass=b['cell_mass_kg'],material='purchased',color=(.21,.42,.69),status='official_envelope',notes='12S1P supplier assembled/insulated pack, not a loose-cell assembly instruction')
    add('Pi4_Heatsink',box((20,20,8),(113,0,29)),mass=.012,color=(.6,.63,.65))
    add('EStop_22mm_NC_Contact_Block',cylinder(10,20,(100,-65,33)),mass=.020,color=(.25,.25,.28),status='reservation_only')
    add('EStop_Red_Mushroom',cylinder(15,11,(100,-65,51)),mass=.013,color=(.85,.1,.08),status='reservation_only')
    add('Brake_Resistor_47ohm_100W_PulseRating_REQUIRED',box((100,20,20),(-119,64,2)),mass=.09,color=(.9,.57,.13),status='reservation_only',notes='54V clamp gives ~62W; pulse-energy and cooling require selected resistor datasheet')
    camera=np.asarray(D['camera_xyz_m'])*1000
    add('RealSense_D435i_90x25x25',box((25,90,25),camera),mass=.072,material='purchased',color=(.17,.18,.21),status='official_envelope')
    cradle=box((39,102,3),(camera[0],0,camera[2]-16))
    cradle=cut(cradle,box((27,92,6),(camera[0],0,camera[2]-16)))
    add('D435i_Edge_Clamp_Cradle',cradle,notes='Edge clamp avoids unverified camera screw lengths; 12deg pitch in Fusion occurrence')
    add('D435i_Front_Riser',box((12,18,38),(182,0,56)))
    for y in (-22,22):
        add(f'Camera_Riser_Arm_{y}',box((42,7,3),(197,y,61)))
    # Cable routes are visible separate objects and are excluded from moving-part collision tests.
    for side in (-1,1):
        add(f'CAN_and_Power_Separated_Trunk_{side}',cylinder(3,260,(0,side*82,-32),(1,0,0)),mass=.012,material='purchased',color=(.85,.3,.12),status='route_envelope')
    add('USB3_Camera_Cable_Service_Route',cylinder(3,58,(167,0,30),(1,0,0)),mass=.012,material='purchased',color=(.1,.12,.16),status='route_envelope')
    add('Ultrasonic_Front_Mount_Plate',box((2,55,30),(195,0,-10)))
    for hy in (-23,23):add(f'Ultrasonic_Body_Attachment_{hy}',box((26,6,6),(184,hy,-26)))

def legs():
    for leg,front,side in LEGS:
        x,y=front*D['hip_x_m']*1000,side*D['hip_y_m']*1000
        # RS00 output-face plane is the abduction pivot, not its casing centre.
        m=C['motor_mounts']['RS00'];mc=(x-front*(m['length']/2+2),y,0)
        add(leg+'_RS00_Stator',cylinder(28.5,51,mc,(1,0,0)),mass=D['actuators']['RS00']['mass_kg'],material='purchased',color=(.09,.10,.13),status='official_envelope')
        rearx=x-front*54
        rear=cylinder(34,3,(rearx,y,0),(1,0,0))
        rear=holes(rear,circle_points(38,4,(rearx,y,0),'x',math.pi/4),1.7,8,(1,0,0))
        rear=holes(rear,[(rearx,y,0)],17,8,(1,0,0))
        rear=fuse(rear,box((3,18,84),(rearx,side*82,0)))
        add(leg+'_RS00_Rear_Bolt_Mount',rear,notes='4x M3 PCD38; engagement <=4mm in RS00; rear flange connected to both frame plates')
        for z in (-32,32):add(leg+f'_Body_Motor_Thermal_Bridge_{z}',box((47,20,6),(rearx+front*16,side*82,z)))
        group=leg+'_abduction_link'
        output=cylinder(20,4,(front*2,0,0),(1,0,0))
        output=holes(output,circle_points(27,6,(front*2,0,0),'x'),1.7,8,(1,0,0))
        add(leg+'_Abduction_Output_M3_PCD27',output,group,notes='6x M3 output bolts; <=5mm engagement; auxiliary support needed before impact motions')
        offset=side*D['hip_offset_m']*1000
        add(leg+'_Abduction_Axial_Adapter',cylinder(9,33,(front*18.5,0,0),(1,0,0)),group)
        beam=box((8,abs(offset),18),(front*35,offset/2,0))
        beam=cut(beam,box((4,abs(offset)-16,10),(front*35,offset/2,0)))
        add(leg+'_Abduction_Yoke_55mm',beam,group)
        stator_motor(leg+'_Hip',group,(0,offset,0),side)
        # These link primitives share the kinematic pivot, but offset their material
        # away from the rotating motor face. The foot centre returns to pivot Y.
        uppergroup=leg+'_upper_link'
        rotor(leg+'_Hip_Output_6M4_PCD24',uppergroup,(0,0,0),side,24,21.5,'M4')
        rail_tube(leg+'_Upper_Tube',uppergroup,140,-side*14,upper=True)
        stator_motor(leg+'_Knee',uppergroup,(0,0,-140),side)
        add(leg+'_Knee_Stator_Ring_Tab',box((24,16,22),(0,-side*8,-105)),uppergroup)
        lowergroup=leg+'_lower_link'
        rotor(leg+'_Knee_Output_6M4_PCD24',lowergroup,(0,0,0),side,24,21.5,'M4')
        rail_tube(leg+'_Lower_Tube',lowergroup,140,-side*19)
        add(leg+'_Foot_Return_Bracket',box((25,26,7),(0,-side*10,-140)),lowergroup)
        add(leg+'_FSR_402_Contact_Disc',cylinder(7.3,1,(0,0,-146)),lowergroup,mass=.002,material='purchased',color=(.95,.48,.06),status='reservation_only',notes='Contact sensor only; force accuracy and overload bypass require bench calibration')
        add(leg+'_Elastomer_Foot',BRepPrimAPI_MakeSphere(gp_Pnt(0,0,-140),18).Shape(),lowergroup,mass=.033,material='purchased',color=(.12,.13,.15),status='custom_elastomer_foot',notes='FSR pad is inside foot; mechanical overload stop prevents full impact on film')
        add(leg+'_Cable_Service_Loop',cylinder(2,72,(14,-side*26,-70)),lowergroup,mass=.008,material='purchased',color=(.8,.3,.13),status='route_envelope',notes='Route reservation; 30mm minimum bend radius and strain relief to be measured')

def stator_motor(name,group,origin,side):
    ox,oy,oz=origin
    m=C['motor_mounts']['RS02'];centre=(ox,oy+side*(m['length']/2+2),oz)
    add(name+'_RS02_Stator',cylinder(m['diameter']/2,m['length'],centre,(0,1,0)),group,mass=D['actuators']['RS02']['mass_kg'],material='purchased',color=(.09,.10,.13),status='official_envelope')
    y=oy-side*1.5
    ring=cylinder(43,3,(ox,y,oz),(0,1,0))
    ring=cut(ring,cylinder(32,8,(ox,y,oz),(0,1,0)))
    ring=holes(ring,circle_points(73,9,(ox,y,oz),'y',math.pi/2),1.7,8,(0,1,0))
    add(name+'_Stator_Ring_9M3_PCD73',ring,group,notes='9x M3 PCD73; <=8mm thread engagement; current vendor drawing required')

def rail_tube(name,group,length,y,upper=False):
    tube=box((26,12,length-30),(0,y,-length/2))
    tube=cut(tube,box((23,9,length-28),(0,y,-length/2)))
    add(name+'_26x12x1_5',tube,group)
    for z in ((-14,) if upper else (-14,-length+14)):
        connector=box((26,abs(y)+4,8),(0,y/2,z))
        connector=holes(connector,[(-8,y,z),(8,y,z)],1.7,30,(0,1,0))
        add(name+'_End_Adapter_'+str(z),connector,group)

def poses(q):
    result={'base_link':np.eye(4)}
    for i,(leg,front,side) in enumerate(LEGS):
        a,h,k=q[i*3:i*3+3]
        rx=np.array([[1,0,0],[0,math.cos(a),-math.sin(a)],[0,math.sin(a),math.cos(a)]])
        def ry(t):return np.array([[math.cos(t),0,math.sin(t)],[0,1,0],[-math.sin(t),0,math.cos(t)]])
        def t(r,p):m=np.eye(4);m[:3,:3]=r;m[:3,3]=p;return m
        hip=[front*D['hip_x_m']*1000,side*D['hip_y_m']*1000,0]
        ab=t(rx,hip);up=ab@t(ry(h),[0,side*D['hip_offset_m']*1000,0]);low=up@t(ry(k),[0,0,-140])
        result.update({leg+'_abduction_link':ab,leg+'_upper_link':up,leg+'_lower_link':low})
    return result

def transformed(shape,mat):
    tr=gp_Trsf();tr.SetValues(*[float(v) for row in range(3) for v in (*mat[row,:3],mat[row,3])])
    return BRepBuilderAPI_Transform(shape,tr,True).Shape()

def step(shapes,path):
    w=STEPControl_Writer()
    for shape in shapes:w.Transfer(shape,STEPControl_AsIs)
    if w.Write(str(path))!=IFSelect_RetDone:raise RuntimeError('STEP export failed: '+str(path))

def mesh(shape):
    BRepMesh_IncrementalMesh(shape,.55,False,.3,True).Perform()
    vertices=[];indices=[];exp=TopExp_Explorer(shape,TopAbs_FACE)
    while exp.More():
        face=TopoDS.Face_s(exp.Current());loc=TopLoc_Location();tri=BRep_Tool.Triangulation_s(face,loc)
        if tri:
            start=len(vertices)//3
            for i in range(1,tri.NbNodes()+1):
                p=tri.Node(i).Transformed(loc.Transformation());vertices.extend((p.X(),p.Y(),p.Z()))
            for i in range(1,tri.NbTriangles()+1):
                a,b,c=tri.Triangle(i).Get();indices.extend((start+a-1,start+b-1,start+c-1))
        exp.Next()
    return {'vertices':vertices,'indices':indices}

def inertials():
    groups={}
    for p in PARTS:groups.setdefault(p['group'],[]).append(p)
    out={}
    for g,pp in groups.items():
        mass=sum(p['mass_kg'] for p in pp)
        com=sum(np.array(p['com_mm'])*p['mass_kg'] for p in pp)/mass
        inertia=np.zeros((3,3))
        for p in pp:
            r=(np.asarray(p['com_mm'])-com)/1000
            inertia+=np.asarray(p['inertia_kg_m2'])+p['mass_kg']*(np.eye(3)*np.dot(r,r)-np.outer(r,r))
        out[g]={'mass_kg':mass,'com_m':(com/1000).tolist(),'inertia_kg_m2':inertia.tolist()}
    # Wire/fastener allowance is carried by the base, without double-counting cell masses.
    allowance=C['non_geometric_mass_allowance_kg'];out['base_link']['mass_kg']+=allowance
    out['base_link']['inertia_kg_m2']=(np.asarray(out['base_link']['inertia_kg_m2'])+np.diag([.003,.009,.01])*allowance).tolist()
    return out

def validate_static_electronics():
    # Exact booleans between purchased-box reservations, not a bounding-box overlap shortcut.
    ee=[p for p in PARTS if p['name'] in [e['name'] for e in C['electronics']] and p['name']!='MicroSD_64GB_A2']
    collisions=[]
    for i,a in enumerate(ee):
        for b in ee[i+1:]:
            op=BRepAlgoAPI_Common(a['shape'],b['shape']);op.Build();props=GProp_GProps();BRepGProp.VolumeProperties_s(op.Shape(),props)
            if props.Mass()>.5:collisions.append({'a':a['name'],'b':b['name'],'overlap_mm3':props.Mass()})
    return collisions

def main():
    OUT.mkdir(parents=True,exist_ok=True);(OUT/'groups').mkdir(exist_ok=True)
    frame();electronics();legs()
    collision=validate_static_electronics()
    stand=Gait(D).stand();ts=poses(stand)
    groups={}
    for p in PARTS:groups.setdefault(p['group'],[]).append(p)
    for g,pp in groups.items():step([p['shape'] for p in pp],OUT/'groups'/f'{g}.step')
    step([transformed(p['shape'],ts[p['group']]) for p in PARTS],OUT/'BigSpot_Pi4_assembly.step')
    inertia=inertials()
    q=copy.deepcopy(D);q['name']='BigSpot Pi4 assembly R1';q['hardware_measured']=False;q['mechanical_revision']='pi4-r1';q['link_inertials']=inertia
    # Base excludes RS00 motor masses, which are already added by the dynamics model.
    q['base_without_actuators_kg']=inertia['base_link']['mass_kg']-4*D['actuators']['RS00']['mass_kg']
    ab=inertia['FL_abduction_link']['mass_kg']-D['actuators']['RS02']['mass_kg']
    up=inertia['FL_upper_link']['mass_kg']-D['actuators']['RS02']['mass_kg']
    low=inertia['FL_lower_link']['mass_kg']
    q.update(hip_bracket_kg=ab,upper_structure_kg=up,lower_structure_kg=low)
    q['battery'].update(chemistry='12S1P Molicel P42A Li-ion, supplier pack required',nominal_voltage_v=43.2,capacity_ah=4.2)
    q['body_height_m']=.088
    (ROOT/'config/pi4_design.json').write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8')
    clean=[{k:v for k,v in p.items() if k!='shape'} for p in PARTS]
    joints=[]
    for i,(leg,front,side) in enumerate(LEGS):
        for j,(suffix,parent,child) in enumerate((('abad','base_link',leg+'_abduction_link'),('hip',leg+'_abduction_link',leg+'_upper_link'),('knee',leg+'_upper_link',leg+'_lower_link'))):
            child_t=ts[child];axis=ts[parent][:3,:3]@np.array([1,0,0] if j==0 else [0,1,0])
            joints.append({'name':leg+'_'+suffix,'parent':parent,'child':child,'pivot_mm':child_t[:3,3].tolist(),'axis':axis.tolist(),'initial_rad':float(stand[i*3+j]),'range_rad':[-.65,.65] if j==0 else [-2.1,1.25] if j==1 else [.1,3.]})
    manifest={'revision':C['revision'],'units':'mm','parts':clean,'groups':{g:{'step':'groups/'+g+'.step','transform_mm':ts[g].tolist(),'parts':[p['name'] for p in pp]} for g,pp in groups.items()},'joints':joints,'link_inertials':inertia}
    (OUT/'assembly_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    with (OUT/'bom.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.writer(f);w.writerow(['name','quantity','group','mass_kg','material','confirmation','notes'])
        for p in clean:w.writerow([p['name'],1,p['group'],round(p['mass_kg'],5),p['material'],p['status'],p['notes']])
        for p in C['external_parts']:w.writerow([p['name'],p['quantity'],'external',0,'purchased','supplier_required','Not carried on robot'])
    viewer=[]
    for p in PARTS:viewer.append({**{k:p[k] for k in ('name','group','color','status','mass_kg')},**mesh(p['shape'])})
    (OUT/'cad_meshes.json').write_text(json.dumps({'parts':viewer,'joints':joints,'groups':manifest['groups']},separators=(',',':')),encoding='utf-8')
    report={'revision':C['revision'],'valid_breps':len(PARTS),'kinematic_revolute_joints':12,'purchased_envelope_overlaps':collision,'static_electronics_clearance_passed':not collision,'estimated_mass_kg':sum(x['mass_kg'] for x in inertia.values()),'unverified_part_reservations':[e['id'] for e in C['electronics'] if e['footprint']=='reservation_only'],'motor_mount_patterns_from_manufacturer':True,'thread_revision_measured':False,'mount_bearing_load_fea_tested':False,'wire_bend_clearance_tested':False,'hardware_function_verified':False,'manufacturing_released':False}
    (OUT/'validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('unverified_part_reservations',)},ensure_ascii=False))
    if collision:raise SystemExit(2)

if __name__=='__main__':main()
