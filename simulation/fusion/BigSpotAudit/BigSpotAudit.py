"""Read-only geometry audit of the ACTIVE Fusion design. No cloud upload or geometry edits.

Run from Utilities > Scripts and Add-Ins after opening the original BigSpot design.
Exports to the path selected by the user. SI meters from Fusion's internal centimeters.
"""
import json
import traceback
import adsk.core
import adsk.fusion


def vector(v):
    return [v.x*.01,v.y*.01,v.z*.01]


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if design is None:
            ui.messageBox('Open a Fusion design first.')
            return
        dialog = ui.createFileDialog()
        dialog.title = 'Save BigSpot geometry audit'
        dialog.filter = 'JSON files (*.json)'
        dialog.initialFilename = 'bigspot_fusion_audit.json'
        if dialog.showSave()!=adsk.core.DialogResults.DialogOK:
            return
        root = design.rootComponent
        result = {'design':root.name,'units':'m, kg, kg*m^2', 'components':[], 'joints':[]}
        for component in design.allComponents:
            entry = {'name':component.name,'bodies':[],'mesh_bodies':component.meshBodies.count}
            for body in component.bRepBodies:
                properties = body.physicalProperties
                moments = properties.getXYZMomentsOfInertia()
                entry['bodies'].append({'name':body.name,'mass_kg':properties.mass,
                    'center_of_mass_local_m':vector(properties.centerOfMass),
                    'bbox_min_local_m':vector(body.boundingBox.minPoint),
                    'bbox_max_local_m':vector(body.boundingBox.maxPoint),
                    'moments_kg_m2': [float(v)*1e-4 for v in moments[1:]] if moments[0] else None})
            result['components'].append(entry)
        for collection in [root.joints,root.asBuiltJoints]:
            for joint in collection:
                result['joints'].append({'name':joint.name,'joint_type':int(joint.jointMotion.jointType),
                    'occurrence_one':joint.occurrenceOne.name if joint.occurrenceOne else None,
                    'occurrence_two':joint.occurrenceTwo.name if joint.occurrenceTwo else None})
        result['note'] = 'Component-local coordinates. Meshes have no solid mass audit. Moments require reference-frame verification before URDF replacement.'
        with open(dialog.filename,'w',encoding='utf-8') as stream:
            json.dump(result,stream,ensure_ascii=False,indent=2)
        ui.messageBox('Audit saved. Geometry was not changed.')
    except Exception:
        ui.messageBox(traceback.format_exc())
