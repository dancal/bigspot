"""Fusion native articulated Pi4 assembly, local archive export, no cloud upload.

Run in Fusion: Utilities > Scripts and Add-ins > add this folder > Run.
Hardware envelope reservations are marked in part names. Geometry is regenerated
from tools/build_pi4_assembly.py; joints remain editable in the native archive.
"""
import json, math, traceback
from pathlib import Path
import adsk.core, adsk.fusion

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'design/pi4'

def point(component,xyz,name):
    p=component.constructionPoints.createInput()
    p.setByPoint(adsk.core.Point3D.create(*(float(x)/10 for x in xyz)))
    p=component.constructionPoints.add(p);p.name=name;return p

def run(context):
    app=adsk.core.Application.get();ui=app.userInterface
    report={'completed':False,'archive_exported':False,'joint_count':0,'warnings':[]}
    try:
        manifest=json.loads((OUT/'assembly_manifest.json').read_text(encoding='utf-8'))
        doc=app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        doc.name='BigSpot Pi4 R1 - articulated engineering prototype'
        design=adsk.fusion.Design.cast(app.activeProduct)
        if hasattr(design,'designIntent'):
            design.designIntent=adsk.fusion.DesignIntentTypes.HybridDesignIntentType
        design.designType=adsk.fusion.DesignTypes.DirectDesignType
        root=design.rootComponent;root.name='BigSpot_Pi4_R1'
        occurrences={}
        for name,g in manifest['groups'].items():
            occ=root.occurrences.addNewComponent(adsk.core.Matrix3D.create());occ.component.name=name
            options=app.importManager.createSTEPImportOptions(str(OUT/g['step']))
            options.isViewFit=False
            if not app.importManager.importToTarget(options,occ.component):raise RuntimeError('Import failed: '+name)
            bodies=list(occ.component.bRepBodies)
            if len(bodies)!=len(g['parts']):
                report['warnings'].append(name+': imported body count differs from part manifest')
            for i,body in enumerate(bodies):
                if i<len(g['parts']):body.name=g['parts'][i]
            tr=adsk.core.Matrix3D.create();m=g['transform_mm']
            tr.setWithArray([m[r][c]/10 if c==3 and r<3 else m[r][c] for r in range(4) for c in range(4)])
            occ.transform2=tr;occurrences[name]=occ
            adsk.doEvents()
        occurrences['base_link'].isGrounded=True
        # Place geometry in the parent component so each joint axis follows the
        # parent when an upstream joint moves. Root-fixed axes would be wrong.
        for j in manifest['joints']:
            parent=occurrences[j['parent']];pc=parent.component
            m=manifest['groups'][j['parent']]['transform_mm']
            delta=[j['pivot_mm'][i]-m[i][3] for i in range(3)]
            local=[sum(m[r][i]*delta[r] for r in range(3)) for i in range(3)]
            direction=[sum(m[r][i]*j['axis'][r] for r in range(3)) for i in range(3)]
            p=point(pc,local,j['name']+'_pivot')
            end=[local[i]+10*direction[i] for i in range(3)]
            p2=point(pc,end,j['name']+'_axis_end')
            ai=pc.constructionAxes.createInput();ai.setByTwoPoints(p,p2)
            axis=pc.constructionAxes.add(ai);axis.name=j['name']+'_axis'
            geometry=adsk.fusion.JointGeometry.createByPoint(p.createForAssemblyContext(parent))
            ji=root.asBuiltJoints.createInput(occurrences[j['parent']],occurrences[j['child']],geometry)
            ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.CustomJointDirection,axis.createForAssemblyContext(parent))
            joint=root.asBuiltJoints.add(ji);joint.name=j['name']
            motion=adsk.fusion.RevoluteJointMotion.cast(joint.jointMotion)
            limits=motion.rotationLimits
            limits.isMinimumValueEnabled=True;limits.minimumValue=j['range_rad'][0]-j['initial_rad']
            limits.isMaximumValueEnabled=True;limits.maximumValue=j['range_rad'][1]-j['initial_rad']
            report['joint_count']+=1
            p.isLightBulbOn=False;p2.isLightBulbOn=False;axis.isLightBulbOn=False
        app.activeViewport.fit()
        report['completed']=report['joint_count']==12
        options=design.exportManager.createFusionArchiveExportOptions(str(OUT/'BigSpot_Pi4_R1.f3d'))
        report['archive_exported']=bool(design.exportManager.execute(options))
        app.activeViewport.saveAsImageFile(str(OUT/'fusion_preview.png'),1600,1000)
        report['component_count']=len(occurrences)
        report['hardware_verified']=False
        report['manufacturing_released']=False
        ui.messageBox('Pi4 assembly created: 13 rigid link groups, 12 revolute joints.\nLocal archive export result: '+str(report['archive_exported'])+'.\nMarked reservations require supplier verification.\nThis is an engineering prototype; electrical commissioning and load tests are pending.')
    except Exception:
        report['error']=traceback.format_exc();ui.messageBox(report['error'])
    finally:
        (OUT/'fusion_execution.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
