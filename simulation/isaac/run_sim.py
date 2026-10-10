# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES.
# SPDX-License-Identifier: Apache-2.0
# Modified for BigSpot. Source API patterns:
# https://github.com/isaac-sim/IsaacSim/blob/v6.0.1/source/standalone_examples/api/isaacsim.ros2.bridge/moveit.py
# https://github.com/isaac-sim/IsaacSim/blob/v6.0.1/source/standalone_examples/api/isaacsim.ros2.bridge/camera_periodic.py
"""Isaac Sim 6.0.1 scene + standard ROS 2 message bridge.

Run with Isaac Sim's python.sh, in a shell separate from Isaac ROS 5.0's container.
Based on NVIDIA's v6.0.1 moveit.py and camera_periodic.py API patterns (Apache-2.0).
Runtime has NOT been validated on this AMD-only workstation.
"""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT/'simulation/ros2_ws/src/bigspot_sim'
sys.path.insert(0, str(PACKAGE))
from bigspot_sim.posture import PostureController

parser = argparse.ArgumentParser()
parser.add_argument('--headless', action='store_true')
parser.add_argument('--floating', action='store_true', help='Enable free base; fixed base is the joint/bridge test default')
parser.add_argument('--frames', type=int, default=0, help='0 runs until the window closes')
args, _ = parser.parse_known_args()
model = json.loads((PACKAGE/'config/model.json').read_text())
policy = json.loads((PACKAGE/'config/control_policy.json').read_text())['parameters']
posture = PostureController(model, **policy)
gait = posture.gait
generated = ROOT/'simulation/generated'
generated.mkdir(parents=True, exist_ok=True)
urdf = ET.parse(PACKAGE/'urdf/bigspot.urdf')
for mesh in urdf.findall('.//mesh'):
    path = ROOT/'src/bigspot_description/meshes'/Path(mesh.get('filename')).name
    mesh.set('filename', path.resolve().as_posix())
for limit in urdf.findall('.//joint/limit'):
    # Same unmeasured conservative cap as the local test; not a motor identification.
    limit.set('effort', '6.0')
urdf.write(generated/'bigspot.urdf', encoding='utf-8', xml_declaration=True)

from isaacsim import SimulationApp
simulation_app = SimulationApp({'headless': args.headless})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.experimental.prims import Articulation
import omni.usd
import omni.graph.core as og
import usdrt.Sdf
from pxr import Gf, UsdGeom, UsdPhysics, UsdLux, UsdShade

try:
    app_utils.enable_extension('isaacsim.asset.importer.urdf')
    app_utils.enable_extension('isaacsim.ros2.bridge')
    app_utils.enable_extension('isaacsim.sensors.experimental.physics')
    app_utils.enable_extension('isaacsim.sensors.physics.nodes')
    simulation_app.update()
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    config = URDFImporterConfig(
        urdf_path=str(generated/'bigspot.urdf'), usd_path=str(generated/'usd'),
        fix_base=not args.floating, merge_fixed_joints=False, merge_mesh=False,
        collision_type='Convex Decomposition', allow_self_collision=False,
        joint_drive_type='force', joint_target_type='position',
        override_joint_stiffness=300.0, override_joint_damping=8.0)
    usd_path = URDFImporter(config).import_urdf()
    stage_utils.set_stage_units(meters_per_unit=1.0)
    stage_utils.add_reference_to_stage(usd_path, '/World/BigSpot')
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath('/World/BigSpot')
    base_z = posture.crouch_height + .008
    UsdGeom.XformCommonAPI(robot_prim).SetTranslate(Gf.Vec3d(0, 0, base_z))
    scene = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    ground = UsdGeom.Cube.Define(stage, '/World/Ground')
    ground.GetSizeAttr().Set(1)
    transform = UsdGeom.XformCommonAPI(ground)
    transform.SetTranslate(Gf.Vec3d(0, 0, -.05))
    transform.SetScale(Gf.Vec3f(10, 10, .1))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    material = UsdShade.Material.Define(stage, '/World/GroundMaterial')
    physical = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physical.CreateStaticFrictionAttr().Set(.8)
    physical.CreateDynamicFrictionAttr().Set(.6)
    physical.CreateRestitutionAttr().Set(0)
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(material, materialPurpose='physics')
    light = UsdLux.DomeLight.Define(stage, '/World/Light')
    light.CreateIntensityAttr().Set(1000)
    simulation_app.update()
    subtree = list(stage.Traverse())
    roots = [p for p in subtree if str(p.GetPath()).startswith('/World/BigSpot/') and p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if robot_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        roots.append(robot_prim)
    if len(roots) != 1:
        raise RuntimeError(f'Expected one articulation root, found {[str(p.GetPath()) for p in roots]}')
    articulation_path = str(roots[0].GetPath())
    bases = [p for p in subtree if str(p.GetPath()).startswith('/World/BigSpot/') and p.GetName() == 'base_link' and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(bases) != 1:
        raise RuntimeError('Cannot locate base_link rigid body for camera attachment')
    base_path = str(bases[0].GetPath())
    # API and OGN ports checked against NVIDIA's v6.0.1 source. GPU runtime pending.
    from isaacsim.sensors.experimental.physics import IMU
    imu_path = base_path+'/SimIMU'
    IMU.create(imu_path, translations=np.array([[0., 0., 0.]]))
    camera_path = base_path+'/SimCamera'
    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera_transform = UsdGeom.XformCommonAPI(camera)
    camera_transform.SetTranslate(Gf.Vec3d(.18, 0, .04))
    # USD camera (-Z forward, +Y up) to robot (+X forward, +Z up).
    camera_transform.SetRotate((90, 0, -90), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    camera.GetHorizontalApertureAttr().Set(20.955)
    camera.GetVerticalApertureAttr().Set(15.71625)
    camera.GetFocalLengthAttr().Set(18.0)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(.01, 100.0))

    k = og.Controller.Keys
    nodes = [('Tick','omni.graph.action.OnPlaybackTick'),
             ('Time','isaacsim.core.nodes.IsaacReadSimulationTime'),
             ('Read','isaacsim.sensors.physics.IsaacReadJointState'),
             ('Context','isaacsim.ros2.bridge.ROS2Context'),
             ('State','isaacsim.ros2.bridge.ROS2PublishJointState'),
             ('Command','isaacsim.ros2.bridge.ROS2SubscribeJointState'),
             ('Control','isaacsim.core.nodes.IsaacArticulationController'),
             ('ReadIMU','isaacsim.sensors.physics.IsaacReadIMU'),
             ('IMU','isaacsim.ros2.bridge.ROS2PublishImu'),
             ('Clock','isaacsim.ros2.bridge.ROS2PublishClock')]
    connections = [('Tick.outputs:tick','Read.inputs:execIn'),
                   ('Read.outputs:execOut','State.inputs:execIn'),
                   ('Tick.outputs:tick','Command.inputs:execIn'),
                   ('Command.outputs:execOut','Control.inputs:execIn'),
                   ('Tick.outputs:tick','Clock.inputs:execIn'),
                   ('Tick.outputs:tick','ReadIMU.inputs:execIn'),
                   ('ReadIMU.outputs:execOut','IMU.inputs:execIn'),
                   ('ReadIMU.outputs:orientation','IMU.inputs:orientation'),
                   ('ReadIMU.outputs:angVel','IMU.inputs:angularVelocity'),
                   ('ReadIMU.outputs:sensorTime','IMU.inputs:timeStamp'),
                   ('Time.outputs:simulationTime','Clock.inputs:timeStamp')]
    for field in ['jointNames','jointPositions','jointVelocities','jointEfforts','jointDofTypes','stageMetersPerUnit','sensorTime']:
        connections.append((f'Read.outputs:{field}',f'State.inputs:{field}'))
    for node in ['State','Command','Clock','IMU']:
        connections.append(('Context.outputs:context',f'{node}.inputs:context'))
    connections.extend([('Command.outputs:jointNames','Control.inputs:jointNames'),
                        ('Command.outputs:positionCommand','Control.inputs:positionCommand')])
    og.Controller.edit({'graph_path':'/BigSpotBridge','evaluator_name':'execution'}, {
        k.CREATE_NODES:nodes, k.CONNECT:connections, k.SET_VALUES:[
            ('Context.inputs:useDomainIDEnvVar',True),
            ('Read.inputs:prim',[usdrt.Sdf.Path(articulation_path)]),
            ('Control.inputs:robotPath',articulation_path),
            ('ReadIMU.inputs:imuPrim',[usdrt.Sdf.Path(imu_path)]),
            ('IMU.inputs:topicName','/bigspot/imu'),
            ('IMU.inputs:frameId','base_link'),
            # 6.0.1 acceleration has a reported rotating-frame differentiation defect.
            # This controller uses orientation/angular rate, not acceleration.
            ('IMU.inputs:publishLinearAcceleration',False),
            ('State.inputs:topicName','/bigspot/joint_states'),
            ('Command.inputs:topicName','/bigspot/joint_commands'),
            ('Clock.inputs:topicName','/clock')]})
    camera_graph, _, _, _ = og.Controller.edit({
        'graph_path':'/BigSpotCamera','evaluator_name':'push',
        'pipeline_stage':og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND}, {
        k.CREATE_NODES:[('Tick','omni.graph.action.OnTick'),
                        ('Render','isaacsim.core.nodes.IsaacCreateRenderProduct'),
                        ('RGB','isaacsim.ros2.bridge.ROS2CameraHelper'),
                        ('Info','isaacsim.ros2.bridge.ROS2CameraInfoHelper')],
        k.CONNECT:[('Tick.outputs:tick','Render.inputs:execIn'),
                   ('Render.outputs:execOut','RGB.inputs:execIn'),
                   ('Render.outputs:execOut','Info.inputs:execIn'),
                   ('Render.outputs:renderProductPath','RGB.inputs:renderProductPath'),
                   ('Render.outputs:renderProductPath','Info.inputs:renderProductPath')],
        k.SET_VALUES:[('Render.inputs:cameraPrim',[usdrt.Sdf.Path(camera_path)]),
                      ('Render.inputs:width',640),('Render.inputs:height',480),
                      ('RGB.inputs:frameId','camera_optical_frame'),
                      ('RGB.inputs:topicName','/bigspot/camera/image_rect'),('RGB.inputs:type','rgb'),
                      ('Info.inputs:frameId','camera_optical_frame'),
                      ('Info.inputs:topicName','/bigspot/camera/camera_info')]})
    og.Controller.evaluate_sync(camera_graph)
    simulation_app.update()
    SimulationManager.setup_simulation(dt=1/60, device='cpu')
    app_utils.play()
    simulation_app.update()
    articulation = Articulation(articulation_path)
    names = [name for leg in gait.legs for name in leg.names]
    indices = articulation.get_dof_indices(names)
    # Set the initial pose once. Subsequent motion uses physical drives through the bridge.
    articulation.set_dof_positions(gait.q.ravel().tolist(), dof_indices=indices)
    articulation.set_dof_position_targets(gait.q.ravel().tolist(), dof_indices=indices)
    print(f'BigSpot bridge ready: {articulation_path}; floating={args.floating}')
    frame = 0
    while simulation_app.is_running() and (args.frames == 0 or frame < args.frames):
        simulation_app.update()
        frame += 1
finally:
    app_utils.stop()
    simulation_app.close()
