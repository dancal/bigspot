"""Generate analytic collision geometry; motor shells are envelopes, not machining drawings."""
import math
import xml.etree.ElementTree as ET
from .design import LEGS, motor_names


def vec(values):
    return " ".join(f"{float(value):.9g}" for value in values)


def make_mjcf(d, obstacles=()):
    root = ET.Element("mujoco", model="BigSpot Autonomy QDD")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(root, "option", timestep="0.002", gravity="0 0 -9.81", integrator="implicitfast", iterations="80")
    defaults = ET.SubElement(root, "default")
    ET.SubElement(defaults, "joint", damping="0.12", armature="0.007", frictionloss="0.03")
    ET.SubElement(defaults, "geom", group="1", friction="0.8 0.01 0.001", solref="0.008 1", solimp="0.95 0.99 0.001")
    ET.SubElement(root, "visual")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="0 -2 5", dir="0 0 -1", diffuse="0.9 0.9 0.9")
    ET.SubElement(world, "geom", name="floor", group="0", type="plane", size="8 8 0.1", rgba="0.18 0.21 0.24 1")
    for i, (x,y,sx,sy,h) in enumerate(obstacles):
        ET.SubElement(world, "geom", name=f"obstacle_{i}", group="2", type="box", pos=vec((x,y,h/2)), size=vec((sx,sy,h/2)), rgba="0.9 0.45 0.18 1")
    base = ET.SubElement(world, "body", name="base_link", pos=vec((0,0,d["standing_leg_height_m"]+d["foot_radius_m"])))
    ET.SubElement(base, "freejoint", name="floating_base")
    motors = motor_names(d)
    base_mass = d["base_without_actuators_kg"]
    ET.SubElement(base, "geom", name="body_shell", type="box", size=vec((d["body_length_m"]/2,d["body_width_m"]/2,d["body_height_m"]/2)), mass=str(base_mass), rgba="0.18 0.52 0.67 1")
    ET.SubElement(base,'geom',name='belly_skid',type='box',pos='0 0 -0.050',size='0.15 0.080 0.015',mass='0',rgba='0.1 0.13 0.15 1')
    cad_profile=bool(d.get('link_inertials'))
    if cad_profile:
        shell=base.find("geom[@name='body_shell']");shell.set('contype','0');shell.set('conaffinity','0')
        skid=base.find("geom[@name='belly_skid']");skid.set('pos','0 0 -0.0635');skid.set('size','0.15 0.08 0.0015')
        for name,z,halfwidth,halfheight in [('lower_frame',-.043,.095,.001),('top_frame',.043,.095,.001),('service_cover',.054,.096,.00075)]:
            ET.SubElement(base,'geom',name=name+'_collision',type='box',pos=vec([0,0,z]),size=vec([.190,halfwidth,halfheight]),mass='0',rgba='0.2 0.3 0.35 1')
    # Camera local -Z points forward/down and +Y is optical up.
    pitch = math.radians(d["camera_downward_pitch_deg"])
    ET.SubElement(base, "camera", name="front_depth", pos=vec(d["camera_xyz_m"]), xyaxes=vec((0,-1,0,math.sin(pitch),0,math.cos(pitch))), fovy="58")
    ET.SubElement(base, "geom", name="camera_envelope", type="box", pos=vec(d["camera_xyz_m"]), size="0.013 0.045 0.013", mass="0", contype="0", conaffinity="0", rgba="0.1 0.1 0.1 1")
    sensors = ET.SubElement(root, "sensor")
    ET.SubElement(base, "site", name="body_imu", pos="0 0 0", size="0.006")
    ET.SubElement(sensors, "gyro", name="imu_gyro", site="body_imu")
    ET.SubElement(sensors, "accelerometer", name="imu_accel", site="body_imu")
    actuators = ET.SubElement(root, "actuator")
    for index, (leg, front, side) in enumerate(LEGS):
        motor=d['actuators'][motors[index*3]]
        motor_x=front*(d['hip_x_m']-motor['length_m']/2-.002) if cad_profile else front*d['hip_x_m']
        ET.SubElement(base,'geom',name=f'{leg}_abad_motor',type='cylinder',pos=vec((motor_x,side*d['hip_y_m'],0)),quat='0.70710678 0 0.70710678 0',size=vec((motor['diameter_m']/2,motor['length_m']/2)),mass=str(motor['mass_kg']),rgba='0.09 0.1 0.12 1')
        if cad_profile:
            rear_x=front*(d['hip_x_m']-.0545)
            ET.SubElement(base,'geom',name=f'{leg}_rear_mount_collision',type='cylinder',pos=vec((rear_x,side*d['hip_y_m'],0)),quat='0.70710678 0 0.70710678 0',size='.034 .0015',mass='0',margin='.001')
            ET.SubElement(base,'geom',name=f'{leg}_rear_web_collision',type='box',pos=vec((rear_x,side*.082,0)),size='.0015 .009 .042',mass='0',margin='.001')
            for z in (-.032,.032):ET.SubElement(base,'geom',name=f'{leg}_thermal_bridge_{z}',type='box',pos=vec((rear_x+front*.016,side*.082,z)),size='.0235 .010 .003',mass='0',margin='.001')
        abduction = ET.SubElement(base, "body", name=f"{leg}_abduction_link", pos=vec((front*d["hip_x_m"],side*d["hip_y_m"],0)))
        ET.SubElement(abduction, "joint", name=f"{leg}_abad", axis="1 0 0", range="-0.65 0.65")
        ET.SubElement(abduction, "geom", name=f"{leg}_hip_bracket", type="capsule", fromto=vec((0,0,0,0,side*d["hip_offset_m"],0)), size="0.012", mass=str(d["hip_bracket_kg"]), rgba="0.65 0.7 0.72 1")
        pitch_motor = d["actuators"][motors[index*3+1]]
        motor_y=side*(d['hip_offset_m']+pitch_motor['length_m']/2+.002) if cad_profile else side*d['hip_offset_m']
        ET.SubElement(abduction, "geom", name=f"{leg}_pitch_motor", type="cylinder", pos=vec((0,motor_y,0)), quat="0.70710678 0.70710678 0 0", size=vec((pitch_motor["diameter_m"]/2,pitch_motor["length_m"]/2)), mass=str(pitch_motor["mass_kg"]), rgba="0.09 0.1 0.12 1")
        upper = ET.SubElement(abduction, "body", name=f"{leg}_upper_link", pos=vec((0,side*d["hip_offset_m"],0)))
        ET.SubElement(upper, "joint", name=f"{leg}_hip", axis="0 1 0", range="-2.1 1.25")
        if cad_profile:
            ET.SubElement(upper,'geom',name=f'{leg}_upper_rail',type='box',pos=vec((0,-side*.014,-.070)),size='.013 .006 .050',mass=str(d['upper_structure_kg']),rgba='0.65 0.7 0.72 1')
        else:
            ET.SubElement(upper,'geom',name=f'{leg}_upper_rail',type='capsule',fromto=vec((0,0,0,0,0,-d['upper_length_m'])),size='.010',mass=str(d['upper_structure_kg']),rgba='0.65 0.7 0.72 1')
        knee_motor = d["actuators"][motors[index*3+2]]
        knee_y=side*(knee_motor['length_m']/2+.002) if cad_profile else 0.
        ET.SubElement(upper, "geom", name=f"{leg}_knee_motor", type="cylinder", pos=vec((0,knee_y,-d["upper_length_m"])), quat="0.70710678 0.70710678 0 0", size=vec((knee_motor["diameter_m"]/2,knee_motor["length_m"]/2)), mass=str(knee_motor["mass_kg"]), rgba="0.09 0.1 0.12 1")
        lower = ET.SubElement(upper, "body", name=f"{leg}_lower_link", pos=vec((0,0,-d["upper_length_m"])))
        ET.SubElement(lower, "joint", name=f"{leg}_knee", axis="0 1 0", range=vec(d.get('joint_limits_rad',{}).get('knee',[.1,3.])))
        if cad_profile:
            ET.SubElement(lower,'geom',name=f'{leg}_lower_rail',type='box',pos=vec((0,-side*.030,-.070)),size='.013 .006 .060',mass=str(d['lower_structure_kg']-.035),rgba='0.65 0.7 0.72 1')
            for z in (-.009,-.126):
                ET.SubElement(lower,'geom',name=f'{leg}_lower_adapter_{z}',type='box',pos=vec((0,-side*.015,z)),size='.013 .017 .004',mass='0')
            ET.SubElement(lower,'geom',name=f'{leg}_knee_output_collision',type='cylinder',pos=vec((0,-side*.005,0)),quat='0.70710678 0.70710678 0 0',size='.0215 .002',mass='0')
            ET.SubElement(lower,'geom',name=f'{leg}_foot_return_collision',type='box',pos=vec((0,-side*.015,-.140)),size='.0125 .020 .0035',mass='0')
            ET.SubElement(lower,'geom',name=f'{leg}_foot_neck_collision',type='box',pos=vec((0,-side*.030,-.133)),size='.009 .006 .007',mass='0')
        else:
            ET.SubElement(lower,'geom',name=f'{leg}_lower_rail',type='capsule',fromto=vec((0,0,0,0,0,-d['lower_length_m'])),size='.008',mass=str(d['lower_structure_kg']-.035),rgba='0.65 0.7 0.72 1')
        ET.SubElement(lower, "geom", name=f"{leg}_foot", type="sphere", pos=vec((0,0,-d["lower_length_m"])), size=str(d["foot_radius_m"]), mass="0.035", rgba="0.12 0.12 0.15 1")
        ET.SubElement(lower, "site", name=f"{leg}_contact_site", pos=vec((0,0,-d["lower_length_m"])), size=str(d["foot_radius_m"]*1.02))
        ET.SubElement(sensors, "touch", name=f"{leg}_contact", site=f"{leg}_contact_site")
        for j, suffix in enumerate(("abad","hip","knee")):
            motor = d["actuators"][motors[index*3+j]]
            peak = motor["peak_torque_nm"]
            ET.SubElement(actuators, "motor", name=f"{leg}_{suffix}_drive", joint=f"{leg}_{suffix}", ctrlrange=vec((-peak,peak)))
    # Neighbouring links share pivot bearings. Only non-neighbours can collide.
    contact = ET.SubElement(root, "contact")
    for leg, _, _ in LEGS:
        if not cad_profile:ET.SubElement(contact, "exclude", body1="base_link", body2=f"{leg}_upper_link")
        if not cad_profile:ET.SubElement(contact, "exclude", body1=f"{leg}_abduction_link", body2=f"{leg}_lower_link")
    # A detailed assembly carries measured/vendor mass estimates and CAD-derived
    # centres/inertias. Collision shapes remain simplified analytic proxies.
    if d.get('link_inertials'):
        for body in root.findall('.//body'):
            mass=d['link_inertials'].get(body.get('name'))
            if mass is None: continue
            for geom in body.findall('geom'): geom.set('mass','0')
            inertia=mass['inertia_kg_m2']
            ET.SubElement(body,'inertial',pos=vec(mass['com_m']),mass=str(mass['mass_kg']),
                          fullinertia=vec([inertia[0][0],inertia[1][1],inertia[2][2],inertia[0][1],inertia[0][2],inertia[1][2]]))
    return ET.tostring(root, encoding="unicode")
