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
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform, BRepBuilderAPI_MakePolygon
from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Pnt, gp_Dir, gp_Ax2, gp_Trsf
from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from bigspot_autonomy.design import load_design, LEGS
from bigspot_autonomy.gait import Gait
from bigspot_autonomy.kinematics import smoothstep

OUT = ROOT/'design/pi4'
PARTS = []
YOKE_GUARDS=[]
D = load_design(ROOT/'config/selected_design.json')
C = json.loads((ROOT/'config/pi4_assembly.json').read_text(encoding='utf-8'))
D['standing_leg_height_m']=C.get('standing_leg_height_mm',245)/1000
D['neutral_foot_x_m']=C.get('neutral_foot_x_mm',45)/1000
D['camera_xyz_m'][2]=C.get('camera_height_mm',75)/1000

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

def rounded_plate(size,xyz,radius):
    length,width,t=size;x,y,z=xyz;r=radius
    shape=box((length-2*r,width,t),xyz)
    if width>2*r:shape=fuse(shape,box((length,width-2*r,t),xyz))
    for sx in (-1,1):
        for sy in (-1,1):shape=fuse(shape,cylinder(r,t,(x+sx*(length/2-r),y+sy*(width/2-r),z)))
    return shape

def chamfer_loft(sections):
    loft=BRepOffsetAPI_ThruSections(True,True)
    for length,width,z,corner in sections:
        x,y=length/2,width/2;c=corner
        points=[(-x+c,-y),(x-c,-y),(x,-y+c),(x,y-c),(x-c,y),(-x+c,y),(-x,y-c),(-x,-y+c)]
        wire=BRepBuilderAPI_MakePolygon()
        for px,py in points:wire.Add(gp_Pnt(px,py,z))
        wire.Close();loft.AddWire(wire.Wire())
    loft.Build()
    if not loft.IsDone():raise RuntimeError('Industrial enclosure loft failed')
    return loft.Shape()

def end_loft(sections):
    loft=BRepOffsetAPI_ThruSections(True,True)
    for x,width,bottom,top,corner in sections:
        y=width/2;c=corner
        points=[(-y+c,bottom),(y-c,bottom),(y,bottom+c),(y,top-c),(y-c,top),(-y+c,top),(-y,top-c),(-y,bottom+c)]
        wire=BRepBuilderAPI_MakePolygon()
        for py,pz in points:wire.Add(gp_Pnt(x,py,pz))
        wire.Close();loft.AddWire(wire.Wire())
    loft.Build()
    if not loft.IsDone():raise RuntimeError('End enclosure loft failed')
    return loft.Shape()

def aabb(shape):
    b=Bnd_Box();BRepBndLib.Add_s(shape,b)
    return np.asarray(b.Get()).reshape(2,3)

def motion_clearance(shape):
    # Bound the yoke's entire +/-0.65rad abduction sweep, including extrema
    # between samples. Only the removable shell is trimmed, never the mechanism.
    for front,side,yoke in YOKE_GUARDS:
        low,high=aabb(yoke);points=[]
        for x in (low[0],high[0]):
            for y in (low[1],high[1]):
                for z in (low[2],high[2]):
                    phi=math.atan2(z,y);angles=[-.65,.65]
                    for k in range(-2,3):
                        for angle in (-phi+k*math.pi,math.pi/2-phi+k*math.pi):
                            if -.65<=angle<=.65:angles.append(angle)
                    for angle in angles:
                        c,s=math.cos(angle),math.sin(angle)
                        points.append((x,y*c-z*s,y*s+z*c))
        lo,hi=np.min(points,axis=0),np.max(points,axis=0)
        origin=np.array([front*D['hip_x_m']*1000,side*D['hip_y_m']*1000,0])
        guard=box(hi-lo+2,origin+(lo+hi)/2)
        bb,gg=aabb(shape),aabb(guard)
        if np.all(np.minimum(bb[1],gg[1])-np.maximum(bb[0],gg[0])>.01):shape=cut(shape,guard)
    return shape

def camera_rotation():
    angle=math.radians(D['camera_downward_pitch_deg']);c,s=math.cos(angle),math.sin(angle)
    origin=np.asarray(D['camera_xyz_m'])*1000
    matrix=np.eye(4);matrix[:3,:3]=[[c,0,s],[0,1,0],[-s,0,c]]
    matrix[:3,3]=origin-matrix[:3,:3]@origin
    return matrix

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
    bottom=holes(bottom,[(x,y,f['bottom_z']) for x in (-60,60) for y in (-92,92)],1.7,6)
    bottom=holes(bottom,[(x,y,f['bottom_z']) for x in (-184,184) for y in (-62,62)],1.7,6)
    bx,by,_=C['battery']['position']
    bottom=holes(bottom,[(bx+x,by+y,f['bottom_z']) for x in (-60,60) for y in (-32,32)],1.7,6)
    # General slot grid, including battery straps and interchangeable power modules.
    for x in (-120,-20,60,120):
        for y in (-36,36):bottom=cut(bottom,box((23,3.4,6),(x,y,f['bottom_z'])))
    add('Lower_Frame_380x190_t2',bottom,notes='Laser/waterjet blank; deburr, keep reinforced motor posts')
    top=box((length,width,t),(0,0,f['top_z']))
    for x in (-120,-10,105):top=cut(top,box((100 if x==105 else 88,65,6),(x,0,f['top_z'])))
    top=holes(top,[(x,y,f['top_z']) for x,y,_ in mountpts],1.7,6)
    top=holes(top,[(x,y,f['top_z']) for x in (-60,60) for y in (-92,92)],1.7,6)
    top=holes(top,[(100,-55,f['top_z'])],11.1,6)
    top=holes(top,[(182,y,f['top_z']) for y in (-48,48)],1.7,6)
    add('Removable_Top_Service_Frame',top,notes='22mm NC E-stop opening; top access to battery, Pi4 and MCU')
    for x,y,_ in mountpts:
        post=cylinder(4,84,(x,y,0));post=holes(post,[(x,y,0)],1.25,90)
        add(f'Frame_M3_Standoff_{x}_{y}',post)
    for side in (-1,1):
        wall=box((300,2,68),(0,side*92,0))
        for x in (-95,0,95):wall=cut(wall,box((66,6,40),(x,side*92,0)))
        # The side webs previously intruded into the four fixed motor casings.
        for front in (-1,1):
            mx=front*D['hip_x_m']*1000-front*27.5
            wall=cut(wall,cylinder(29.5,53,(mx,side*105,0),(1,0,0)))
        add(f'Side_Web_{side}',wall,color=(.10,.51,.49))
    # Ground support sits below all body electronics; the robot settles on this when OFF.
    skid=box((300,160,3),(0,0,-63.5))
    for x in (-90,0,90):skid=cut(skid,box((55,110,8),(x,0,-63.5)))
    add('Belly_Ground_Skid',skid,material='UHMW-PE',color=(.15,.18,.21),notes='Bottom z=-65mm; body supports weight while unpowered')
    for x in (-130,130):
        for y in (-65,65):add(f'Skid_Support_{x}_{y}',box((10,10,18),(x,y,-53)))

def electronics():
    for e in C['electronics']:
        shape=box(e['size'],e['xyz'])
        if e['id']=='ultrasound':
            cx,cy,cz=e['xyz'];shape=box((2,45,20),(cx-6.5,cy,cz))
            for sy in (-13,13):
                shell=cylinder(8,13,(cx+1,cy+sy,cz),(1,0,0))
                shell=cut(shell,cylinder(6.2,1,(cx+7.5,cy+sy,cz),(1,0,0)))
                shape=fuse(shape,shell)
                add(f'Acoustic_Diaphragm_Reservation_{sy}',cylinder(6,.5,(cx+7.25,cy+sy,cz),(1,0,0)),mass=0,material='purchased',color=(.12,.16,.18),status='reservation_only',notes='Schematic two-port module within original reserved envelope; actual vendor dimensions unverified')
        if e['id']=='pi4':
            # Official board holes are 58 x 49, offset from the board centre.
            cx,cy,z=e['xyz'];px=cx-42.5+3.5
            pts=[(px+dx,cy-28+3.5+dy,z) for dx in (0,58) for dy in (0,49)]
            shape=holes(shape,pts,1.35,6)
            for i,(x,y,z) in enumerate(pts):
                p=cylinder(3,9.7,(x,y,z-5.65));p=holes(p,[(x,y,z-5.65)],1.2,12)
                add(f'Pi4_M2_5_Standoff_{i}',p,notes='Official 58x49mm pitch, asymmetric on 85x56mm PCB')
            trayz=z-11.5
            tray=box((90,64,2),(cx-8,0,trayz))
            tray=holes(tray,[(x,y,trayz) for x,y,_ in pts],1.35,6)
            add('Pi4_Replaceable_Tray',tray)
            for hy in (-28,28):
                # Feet stay forward of the battery; tray bridges backwards over it.
                height=trayz-1+42
                add(f'Pi4_Tray_Support_{hy}',box((8,8,height),(134,hy,(-42+trayz-1)/2)),notes='Full 8x8mm tray footprint; lower-frame to tray load path, fastener/FEA pending')
            for i,y in enumerate((-15,3,20)):
                size=(18,15,15) if i else (21,17,13)
                add(f'Pi4_USB_Ethernet_Port_{i}',box(size,(cx+37,y,z+8.5)),mass=0,status='connector_envelope',material='purchased',color=(.68,.70,.74))
            add('Pi4_40_Pin_Header',box((51,5,8),(cx-11,cy+24,z+4.8)),mass=0,material='purchased',color=(.1,.1,.1))
        color=(.52,.56,.60) if e['id']=='ultrasound' else e['color']
        add(e['name'],shape,mass=e['mass_kg'],material='purchased',color=color,status=e['footprint'],notes=e.get('requires',''))
        if e['id'] in ('dcdc','mcu','bms','switch','fuses','regen','current','watchdog'):
            sx,sy,sz=e['size'];x,y,z=e['xyz']
            # Slotted strap-mount plinths do not invent an unverified supplier hole pattern.
            plinthz=z-sz/2-1
            base=box((sx+8,sy+6,2),(x,y,plinthz))
            if e['id']=='dcdc':base=cut(base,box((147,83,58),C['battery']['position']))
            if e['id']=='switch':
                for hy in (-28,28):base=cut(base,box((10,10,6),(134,hy,plinthz)))
            for hx in (-sx/2+8,sx/2-8):base=cut(base,box((3.2,sy-3,6),(x+hx,y,plinthz)))
            add(e['id']+'_Slotted_Mounting_Plinth',base,notes='Supplier-specific holes remain un-drilled; retention straps through slots')
            top=plinthz-1;bottom=-42
            feet=[(x,y-min(6,sy/4)),(x,y+min(6,sy/4))]
            if e['id']=='watchdog':feet=[(x-6,-76),(x+6,-76)]
            for index,(fx,fy) in enumerate(feet):
                add(e['id']+f'_Frame_Support_{index}',box((6,6,top-bottom),(fx,fy,(top+bottom)/2)),notes='Compression support rests on lower plate; strap/fastener details unverified')
    # Actual cells are visible independently; insulating tray encloses a supplier-built pack.
    b=C['battery'];x,y,z=b['position'];tray=box(b['pack_envelope'],(x,y,z))
    inner=np.asarray(b['pack_envelope'])-np.array([4,4,2]);inner[2]+=6
    tray=cut(tray,box(inner,(x,y,z+3)))
    for hx in (-45,45):
        for hy in (-37,37):tray=cut(tray,box((10,3,65),(x+hx,y+hy,z)))
    tray=holes(tray,[(x+dx,y+dy,z-27.5) for dx in (-60,60) for dy in (-32,32)],1.7,6)
    add('12S_Battery_Insulated_Tray_and_Strap_Slots',tray,material='PETG',color=(.18,.23,.3))
    for dx in (-60,60):
        for dy in (-32,32):
            support=cylinder(4,1.5,(x+dx,y+dy,-41.25));support=holes(support,[(x+dx,y+dy,-41.25)],1.7,5)
            add(f'Battery_Tray_Compression_Spacer_{dx}_{dy}',support,notes='M3 through-bolt spacer; pack insulation and fastener retention require verification')
    for col in range(6):
        for row in range(2):
            xyz=(x+(col-2.5)*23,y,z+(row-.5)*23)
            add(f'P42A_21700_Cell_{col*2+row+1:02}',cylinder(b['cell_diameter']/2,b['cell_length'],xyz,(0,1,0)),mass=b['cell_mass_kg'],material='purchased',color=(.21,.42,.69),status='official_envelope',notes='12S1P supplier assembled/insulated pack, not a loose-cell assembly instruction')
    add('Pi4_CPU_Thermal_Interface_Reservation',box((20,20,2),(113,0,29.8)),mass=0,status='reservation_only',color=(.6,.63,.65))
    add('Pi4_Heatsink',box((20,20,8),(113,0,34.8)),mass=.012,color=(.6,.63,.65))
    add('EStop_22mm_NC_Contact_Block',cylinder(10,20,(100,-55,33)),mass=.020,color=(.25,.25,.28),status='reservation_only')
    add('EStop_Panel_Stem_Reservation',cylinder(10,13,(100,-55,49.5)),mass=0,status='reservation_only',color=(.25,.25,.28))
    add('EStop_Red_Mushroom',cylinder(15,11,(100,-55,61.5)),mass=.013,color=(.85,.1,.08),status='reservation_only')
    add('Brake_Resistor_47ohm_100W_PulseRating_REQUIRED',box((100,20,20),(-119,55,2)),mass=.09,color=(.9,.57,.13),status='reservation_only',notes='54V clamp gives ~62W; pulse-energy and cooling require selected resistor datasheet')
    camera=np.asarray(D['camera_xyz_m'])*1000
    rotate=camera_rotation()
    housing=box((25,90,25),camera)
    for sy in (-25,25):
        housing=cut(housing,cylinder(5.2,1,(camera[0]+12.35,sy,camera[2]),(1,0,0)))
        lens=cylinder(5,.5,(camera[0]+12.2,sy,camera[2]),(1,0,0))
        add(f'D435_Stereo_Optical_Marker_{sy}',transformed(lens,rotate),mass=0,material='purchased',color=(.05,.14,.20),status='visual_reference',notes='Stereo baseline reference, not supplier lens machining detail; mass included in camera estimate')
    add('RealSense_D435i_90x25x25',transformed(housing,rotate),mass=.072,material='purchased',color=(.52,.58,.60),status='official_envelope',notes='Outer dimensions retained; optical details are schematic')
    cradle=box((39,102,3),(camera[0],0,camera[2]-14))
    cradle=cut(cradle,box((27,92,6),(camera[0],0,camera[2]-14)))
    for dx in (-7,7):cradle=fuse(cradle,box((6,102,3),(camera[0]+dx,0,camera[2]-14)))
    for sy in (-1,1):cradle=fuse(cradle,box((16,3,15),(camera[0],sy*46.5,camera[2]-5.5)))
    cradle=holes(cradle,[(camera[0],sy*46.5,camera[2]-2) for sy in (-1,1)],1.7,4,(0,1,0))
    add('D435i_Edge_Clamp_Cradle',transformed(cradle,rotate),notes='CAD and optical mount both 12deg down; bottom rails and side lips support envelope; retention hardware pending')
    upper_halfspace=transformed(box((1000,1000,1000),(205,0,camera[2]+484.5)),rotate)
    for y in (-48,48):
        beam=box((30,8,4),(191,y,46));beam=holes(beam,[(182,y,46)],1.7,8)
        post=cut(box((8,8,25),(202,y,60.5)),upper_halfspace)
        add(f'Camera_Frame_Mount_{y}',fuse(beam,post),notes='Base on top frame z44; sloped post face meets pitched cradle underside; M3 fastening/strength pending')
    # Cable routes are visible separate objects and are excluded from moving-part collision tests.
    for side in (-1,1):
        add(f'CAN_and_Power_Separated_Trunk_{side}',cylinder(3,164,(0,side*82,-39),(1,0,0)),mass=.012,material='purchased',color=(.85,.3,.12),status='route_envelope',notes='Central route only; motor branches and bend radii not released')
    add('USB3_Camera_Cable_Service_Route',cylinder(3,52,(178,0,30),(1,0,0)),mass=.012,material='purchased',color=(.1,.12,.16),status='route_envelope')
    ultrasonic=box((2,55,30),(195,0,-10))
    for hy in (-27,27):
        ultrasonic=fuse(ultrasonic,box((26,6,6),(184,hy,-26)))
        ultrasonic=fuse(ultrasonic,box((6,6,17),(182,hy,-33.5)))
    add('Ultrasonic_Integrated_Frame_Bracket',ultrasonic,notes='Integrated bracket reaches lower frame z-42; retention/strength pending')

def exterior():
    # A tapered, hollow roof replaces the flat sheet; electronics stay accessible.
    cover=chamfer_loft([(404,206,44,35),(390,186,56,38)])
    cover=cut(cover,chamfer_loft([(400,202,43.8,35),(386,182,54,38)]))
    for x in range(-145,-45,12):
        for y in (-34,34):cover=cut(cover,rounded_plate((5,30,6),(x,y,55),2))
    for y in range(-16,17,4):cover=cut(cover,rounded_plate((36,1.8,6),(113,y,55),.8))
    cover=holes(cover,[(100,-55,55)],11,8)
    cover=holes(cover,[(x,y,55) for x in (-92,92) for y in (-85,85)],1.7,8)
    # Service-panel upper mounting tabs occupy z44..46; keep a real fitting gap.
    for x in (-60,60):
        for side in (-1,1):cover=cut(cover,box((12.8,14,5),(x,side*96,45)))
    for p in PARTS:
        if p['name'].startswith('Camera_Frame_Mount_'):
            bounds=aabb(p['shape']);cover=cut(cover,box(bounds[1]-bounds[0]+.8,(bounds[1]+bounds[0])/2))
    add('Sport_Low_Profile_Service_Hood',motion_clearance(cover),material='PETG',color=(.70,.74,.77),notes='2mm nonstructural shell; lower tapered roof; conservatively bounded shoulder sweep cutouts; retained fasteners and cooling unverified')
    add('Sport_Centre_Spine',rounded_plate((176,22,.6),(0,0,56.3),8),material='PETG',color=(.13,.18,.21),notes='Attached nonstructural trim on roof')
    for y in (-14,14):add(f'Sport_Orange_Pinstripe_{y}',rounded_plate((160,2,.5),(0,y,56.25),1),material='PETG',color=(.94,.34,.10),notes='Attached non-powered trim')
    for x in (-92,92):
        for y in (-85,85):
            support=holes(cylinder(3,10,(x,y,49)),[(x,y,49)],1.25,14)
            add(f'Hood_M3_Spacer_{x}_{y}',support,notes='Top frame z44 to hood underside z54; retained screw design pending')
    for side in (-1,1):
        panel=box((220,2,82),(0,side*101.5,0))
        # Keep the folded lower link clear even as unpowered legs settle.
        for x in (-140,140):panel=cut(panel,cylinder(60,8,(x,side*101.5,0),(0,1,0)))
        for x in (-60,60):
            for z in (-45,45):
                tab=holes(box((12,12,2),(x,side*96,z)),[(x,side*92,z)],1.7,6)
                panel=fuse(panel,tab)
                panel=fuse(panel,box((12,2,6),(x,side*101,z/45*43)))
        for x in (-65,-50,-35,-20,-5,10,25,40,55,70):
            panel=cut(panel,box((3,8,22),(x,side*101.5,18)))
        add(f'Removable_Side_Enclosure_{side}',panel,material='PETG',color=(.17,.22,.25),notes='Ventilated centre panel; upper/lower M3 tabs touch frame surfaces; does not bridge shoulder sweep zones')
        add(f'Orange_Side_Identity_{side}',box((108,.7,5),(0,side*102.85,-10)),material='PETG',color=(.94,.34,.10),notes='Attached trim on side panel')
    # Protective camera collar leaves the entire supplier envelope and optical face open.
    camera=np.asarray(D['camera_xyz_m'])*1000;rotate=camera_rotation()
    shroud=cut(box((42,110,42),camera),box((44,104,36),camera))
    # Existing cradle has a wider floor, so the underside remains open.
    shroud=cut(shroud,box((46,114,16),(camera[0],0,camera[2]-18)))
    for sy in (-1,1):
        clip=holes(box((8,4,6),(camera[0],sy*50,camera[2]-2)),[(camera[0],sy*50,camera[2]-2)],1.7,6,(0,1,0))
        shroud=fuse(shroud,clip)
    for p in PARTS:
        if p['name'].startswith('Camera_Frame_Mount_'):shroud=cut(shroud,transformed(p['shape'],np.linalg.inv(rotate)))
    add('Integrated_Camera_Protective_Collar',transformed(shroud,rotate),material='PETG',color=(.70,.74,.77),notes='Cradle and optical clearance; clips/retention require detailing')
    for sign,label in [(1,'Front'),(-1,'Rear')]:
        end=end_loft([(sign*187,176,-44,44,14),(sign*217,118,-28,28,18)])
        end=cut(end,end_loft([(sign*186,172,-41.8,42,14),(sign*214.8,113.6,-25.8,25.8,18)]))
        for sy in (-1,1):
            mount=holes(box((26,10,2),(sign*193,sy*62,-45)),[(sign*184,sy*62,-45)],1.7,6)
            end=fuse(end,mount)
            # Tapered belly rises toward the nose; bridge each lower-frame tab
            # to the shell so the tabs remain attached after shoulder reliefs.
            end=fuse(end,box((12,2,16),(sign*193,sy*62,-37)))
        if sign==1:
            end=cut(end,box((12,54,24),(216,0,-10)))
            grille=box((1.2,54,24),(216.4,0,-10))
            grille=holes(grille,[(216.4,sy,-10) for sy in (-13,13)],9,5,(1,0,0))
            end=fuse(end,grille)
            neck=cut(box((22,110,30),(205,0,50)),box((24,104,32),(205,0,50)))
            end=fuse(end,neck)
        else:
            for y in range(-48,49,12):end=cut(end,box((16,3,27),(-214,y,-2)))
        # Actual stationary envelopes cut fitting reliefs; moving links are never cut away.
        for p in PARTS:
            if p['group']!='base_link':continue
            bb,pp=aabb(end),aabb(p['shape'])
            if np.all(np.minimum(bb[1],pp[1])-np.maximum(bb[0],pp[0])>.01):end=cut(end,p['shape'])
        add(f'{label}_Tapered_End_Enclosure',motion_clearance(end),material='PETG',color=(.61,.66,.69),notes='Lower-frame M3 mounts; swept shoulder reliefs; tapered nonstructural fairing; port seals and fasteners unverified')
    # Motor end covers are separate cosmetic parts, not altered supplier housings.
    for leg,front,side in LEGS:
        for joint,group,origin in [('Hip',leg+'_abduction_link',(0,side*D['hip_offset_m']*1000,0)),('Knee',leg+'_upper_link',(0,0,-140))]:
            y=origin[1]+side*(C['motor_mounts']['RS02']['length']+3)
            cap=cylinder(39.5,2,(0,y,origin[2]),(0,1,0))
            cap=holes(cap,[(0,y,origin[2])],9,6,(0,1,0))
            cap=holes(cap,circle_points(64,8,(0,y,origin[2]),'y'),1.7,6,(0,1,0))
            cap=cut(cap,box((12,6,8),(0,y,origin[2]+38)))
            add(f'{leg}_{joint}_Removable_End_Cover',cap,group,material='PETG',color=(.17,.22,.25),notes='Removable cosmetic cap; supplier cable connector and retention unverified')
    for p in PARTS:
        if p['name'].endswith('_Integrated_Structure'):p['color']=[.36,.43,.47]

def legs():
    for leg,front,side in LEGS:
        x,y=front*D['hip_x_m']*1000,side*D['hip_y_m']*1000
        # RS00 output-face plane is the abduction pivot, not its casing centre.
        m=C['motor_mounts']['RS00'];mc=(x-front*(m['length']/2+2),y,0)
        add(leg+'_RS00_Stator',cylinder(28.5,51,mc,(1,0,0)),mass=D['actuators']['RS00']['mass_kg'],material='purchased',color=(.09,.10,.13),status='official_envelope')
        rearx=x-front*54.5
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
        beam=box((8,abs(offset),18),(front*35,offset/2,50))
        beam=cut(beam,box((4,abs(offset)-16,10),(front*35,offset/2,50)))
        beam=fuse(beam,box((8,18,50),(front*35,0,25)))
        direction=np.array([-front*15.,0.,-15.]);length=float(np.linalg.norm(direction))
        beam=fuse(beam,cylinder(4,length,(front*27.5,offset-side*1.5,42.5),direction/length))
        add(leg+'_Abduction_Yoke_55mm',beam,group,notes='Raised yoke clears upper-link swept plane; stiffness and bearing load still require FEA')
        YOKE_GUARDS.append((front,side,beam))
        stator_motor(leg+'_Hip',group,(0,offset,0),side)
        # These link primitives share the kinematic pivot, but offset their material
        # away from the rotating motor face. The foot centre returns to pivot Y.
        uppergroup=leg+'_upper_link'
        rotor(leg+'_Hip_Output_6M4_PCD24',uppergroup,(0,0,0),side,24,21.5,'M4')
        rail_tube(leg+'_Upper_Tube',uppergroup,140,-side*14,upper=True)
        stator_motor(leg+'_Knee',uppergroup,(0,0,-140),side)
        add(leg+'_Knee_Stator_Ring_Tab',box((28,16,22),(24,-side*8,-105)),uppergroup,notes='Positive-X tab clears lower-link fold sweep')
        lowergroup=leg+'_lower_link'
        rotor(leg+'_Knee_Output_6M4_PCD24',lowergroup,(0,0,0),side,24,21.5,'M4')
        rail_tube(leg+'_Lower_Tube',lowergroup,140,-side*30)
        add(leg+'_Foot_Return_Bracket',box((25,40,7),(0,-side*15,-140)),lowergroup)
        add(leg+'_Foot_Load_Path_Neck',box((18,12,14),(0,-side*30,-133)),lowergroup,notes='Connects tube end adapter to foot return bracket; no floating foot carrier')
        add(leg+'_FSR_402_Contact_Disc',cylinder(7.3,1,(0,0,-146)),lowergroup,mass=.002,material='purchased',color=(.95,.48,.06),status='reservation_only',notes='Contact sensor only; force accuracy and overload bypass require bench calibration')
        add(leg+'_Elastomer_Foot',BRepPrimAPI_MakeSphere(gp_Pnt(0,0,-140),18).Shape(),lowergroup,mass=.033,material='purchased',color=(.12,.13,.15),status='custom_elastomer_foot',notes='FSR pad is inside foot; mechanical overload stop prevents full impact on film')
        add(leg+'_Cable_Service_Loop',cylinder(2,72,(17,-side*26,-70)),lowergroup,mass=.008,material='purchased',color=(.8,.3,.13),status='route_envelope',notes='Route reservation; 30mm minimum bend radius and strain relief to be measured')

def consolidate_structures():
    # Former overlapping custom solids now represent one integrated load-bearing
    # link, not separate machined parts occupying the same volume.
    for leg,_,_ in LEGS:
        for suffix in ('abduction_link','upper_link','lower_link'):
            group=leg+'_'+suffix
            records=[p for p in PARTS if p['group']==group and p['material']=='6061-T6']
            shape=records[0]['shape']
            for p in records[1:]:shape=fuse(shape,p['shape'])
            exp=TopExp_Explorer(shape,TopAbs_SOLID);count=0
            while exp.More():count+=1;exp.Next()
            if count!=1:raise RuntimeError(group+' structure has a disconnected load path')
            for p in records:PARTS.remove(p)
            add(leg+'_'+suffix+'_Integrated_Structure',shape,group,notes='Integrated prototype geometry from '+', '.join(p['name'] for p in records)+'; machining process, retained fasteners and strength unverified')
            if suffix=='lower_link':
                foot=next(p for p in PARTS if p['name']==leg+'_Elastomer_Foot')
                sensor=next(p for p in PARTS if p['name']==leg+'_FSR_402_Contact_Disc')
                fitted=cut(cut(foot['shape'],shape),sensor['shape'])
                PARTS.remove(foot)
                add(foot['name'],fitted,group,foot['color'],foot['material'],foot['mass_kg'],foot['status'],
                    'Rubber pocket clears integrated carrier and sensor; retention and force transfer require bench tests')
        mount=next(p for p in PARTS if p['name']==leg+'_RS00_Rear_Bolt_Mount')
        bridges=[p for p in PARTS if p['name'].startswith(leg+'_Body_Motor_Thermal_Bridge_')]
        shape=mount['shape']
        for p in bridges:shape=fuse(shape,p['shape']);PARTS.remove(p)
        PARTS.remove(mount)
        add(mount['name'],shape,notes=mount['notes']+'; thermal bridges integrated into bracket')
        for wall in [p for p in PARTS if p['name'].startswith('Side_Web_')]:
            fitted=cut(wall['shape'],shape);PARTS.remove(wall)
            add(wall['name'],fitted,color=wall['color'],notes='Web relief matches integrated motor bracket; interface fasteners/strength pending')

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
    span=length-40 if upper else length-20
    tube=box((26,12,span),(0,y,-length/2))
    tube=cut(tube,box((23,9,span+2),(0,y,-length/2)))
    add(name+'_26x12x1_5',tube,group)
    for z in ((-20,) if upper else (-9,-length+14)):
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
    BRepMesh_IncrementalMesh(shape,.35,False,.2,True).Perform()
    vertices=[];indices=[];exp=TopExp_Explorer(shape,TopAbs_FACE)
    while exp.More():
        face=TopoDS.Face_s(exp.Current());loc=TopLoc_Location();tri=BRep_Tool.Triangulation_s(face,loc)
        if tri:
            start=len(vertices)//3
            for i in range(1,tri.NbNodes()+1):
                p=tri.Node(i).Transformed(loc.Transformation());vertices.extend((p.X(),p.Y(),p.Z()))
            for i in range(1,tri.NbTriangles()+1):
                a,b,c=tri.Triangle(i).Get()
                if face.Orientation()==TopAbs_REVERSED:b,c=c,b
                indices.extend((start+a-1,start+b-1,start+c-1))
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
    frame();electronics();legs();consolidate_structures();exterior()
    collision=validate_static_electronics()
    stand=Gait(D).stand();ts=poses(stand)
    groups={}
    for p in PARTS:groups.setdefault(p['group'],[]).append(p)
    for g,pp in groups.items():step([p['shape'] for p in pp],OUT/'groups'/f'{g}.step')
    step([transformed(p['shape'],ts[p['group']]) for p in PARTS],OUT/'BigSpot_Pi4_assembly.step')
    inertia=inertials()
    q=copy.deepcopy(D);q['name']='BigSpot Pi4 assembly '+C['revision'];q['hardware_measured']=False;q['mechanical_revision']=C['revision'];q['link_inertials']=inertia
    q['enclosure_style']=C.get('enclosure_style','industrial')
    # Base excludes RS00 motor masses, which are already added by the dynamics model.
    q['base_without_actuators_kg']=inertia['base_link']['mass_kg']-4*D['actuators']['RS00']['mass_kg']
    ab=inertia['FL_abduction_link']['mass_kg']-D['actuators']['RS02']['mass_kg']
    up=inertia['FL_upper_link']['mass_kg']-D['actuators']['RS02']['mass_kg']
    low=inertia['FL_lower_link']['mass_kg']
    q.update(hip_bracket_kg=ab,upper_structure_kg=up,lower_structure_kg=low)
    q['battery'].update(chemistry='12S1P Molicel P42A Li-ion, supplier pack required',nominal_voltage_v=43.2,capacity_ah=4.2)
    q['body_height_m']=.112
    q['startup_joint_angles_rad']=C['startup_joint_angles_rad']
    q['startup_base_height_m']=.002+max(.065,.14*(math.cos(C['startup_joint_angles_rad'][1])+math.cos(sum(C['startup_joint_angles_rad'][1:])))+.018)
    q['joint_limits_rad']={'abad':[-.65,.65],'hip':[-2.1,1.25],'knee':[.1,C['knee_operating_max_rad']]}
    (ROOT/'config/pi4_design.json').write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8')
    clean=[{k:v for k,v in p.items() if k!='shape'} for p in PARTS]
    joints=[]
    for i,(leg,front,side) in enumerate(LEGS):
        for j,(suffix,parent,child) in enumerate((('abad','base_link',leg+'_abduction_link'),('hip',leg+'_abduction_link',leg+'_upper_link'),('knee',leg+'_upper_link',leg+'_lower_link'))):
            child_t=ts[child];axis=ts[parent][:3,:3]@np.array([1,0,0] if j==0 else [0,1,0])
            joints.append({'name':leg+'_'+suffix,'parent':parent,'child':child,'pivot_mm':child_t[:3,3].tolist(),'axis':axis.tolist(),'initial_rad':float(stand[i*3+j]),'range_rad':q['joint_limits_rad'][suffix]})
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
