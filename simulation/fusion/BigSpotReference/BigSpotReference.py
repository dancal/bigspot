"""Reconstruct a mesh reference assembly from the repository's Fusion-export STLs.

Creates a new local design; it does not modify the user's original CAD or upload it.
Joint locations and limits come from the original URDF, not measurements of the STL.
Mesh references have no recovered feature history or solid material properties.
"""
from pathlib import Path
import traceback
import json
import adsk.core
import adsk.fusion


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        repo = Path(__file__).resolve().parents[3]
        source = repo/'src/bigspot_description'
        report_path = repo/'simulation/ros2_ws/src/bigspot_sim/config/reconstruction.json'
        reconstruction = json.loads(report_path.read_text(encoding='utf-8'))
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.DirectDesignType
        root = design.rootComponent
        root.name = 'BigSpot STL + URDF joint reconstruction'
        occurrences = {}
        for mesh in reconstruction['meshes']:
            name = mesh['file']
            occurrence = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            occurrences[mesh['link']] = occurrence
            component = occurrence.component
            component.name = mesh['link']
            bodies = component.meshBodies.add(str(source/'meshes'/name),adsk.fusion.MeshUnits.MillimeterMeshUnit)
            if bodies is None or bodies.count!=1:
                raise RuntimeError(f'Import failed: {name}')
            bodies.item(0).name = Path(name).stem
        occurrences['base_link'].isGrounded = True
        for joint in reconstruction['joints']:
            # Fusion internal lengths are centimeters; audit coordinates are millimeters.
            point = adsk.core.Point3D.create(*[v/10 for v in joint['point_mm']])
            direction = adsk.core.Vector3D.create(*joint['axis'])
            if not direction.normalize():
                raise RuntimeError(f"Invalid axis: {joint['name']}")
            pi = root.constructionPoints.createInput()
            if not pi.setByPoint(point):
                raise RuntimeError('Construction point creation failed')
            cp = root.constructionPoints.add(pi)
            cp.name = joint['name']+'_origin'
            ai = root.constructionAxes.createInput()
            if not ai.setByLine(adsk.core.InfiniteLine3D.create(point,direction)):
                raise RuntimeError('Construction axis creation failed')
            axis = root.constructionAxes.add(ai)
            axis.name = joint['name']+'_axis'
            geometry = adsk.fusion.JointGeometry.createByPoint(cp)
            ji = root.asBuiltJoints.createInput(occurrences[joint['parent']],
                                               occurrences[joint['child']],geometry)
            if not ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.CustomJointDirection,axis):
                raise RuntimeError('Revolute joint definition failed')
            built = root.asBuiltJoints.add(ji)
            if built is None:
                raise RuntimeError(f"Joint creation failed: {joint['name']}")
            built.name = joint['name']
            limits = adsk.fusion.RevoluteJointMotion.cast(built.jointMotion).rotationLimits
            limits.minimumValue = joint['lower_rad']
            limits.maximumValue = joint['upper_rad']
            limits.isMinimumValueEnabled = True
            limits.isMaximumValueEnabled = True
            cp.isLightBulbOn = False
            axis.isLightBulbOn = False
        if root.asBuiltJoints.count != 12:
            raise RuntimeError('Expected 12 reconstructed joints')
        app.activeViewport.fit()
        output = repo/'simulation/generated'
        output.mkdir(parents=True,exist_ok=True)
        options = design.exportManager.createFusionArchiveExportOptions(str(output/'BigSpot_kinematic_reference.f3d'))
        if not design.exportManager.execute(options):
            raise RuntimeError('Local Fusion archive export failed')
        (output/'fusion_reconstruction_result.json').write_text(json.dumps({
            'mesh_components':len(occurrences),'revolute_joints':root.asBuiltJoints.count,
            'joint_source':'Original URDF; verify axes and signs using Fusion joint animation',
            'solid_conversion_completed':False,
            'original_feature_history_recovered':False,
        },indent=2)+'\n',encoding='utf-8')
        ui.messageBox('13 mesh components and 12 URDF-derived revolute joints exported locally. '
                      'Verify each axis/sign. Solid conversion, feature reconstruction and mass calibration remain.')
    except Exception:
        ui.messageBox(traceback.format_exc())
