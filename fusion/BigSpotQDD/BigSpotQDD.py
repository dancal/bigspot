"""Fusion: create a separate design document and import the local QDD layout."""
import adsk.core,adsk.fusion,traceback
from pathlib import Path
def run(context):
    app=adsk.core.Application.get();ui=app.userInterface
    try:
        path=Path(__file__).resolve().parents[2]/'design/BigSpot_QDD_layout.step'
        if not path.exists():raise FileNotFoundError(str(path))
        doc=app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType);doc.name='BigSpot QDD - layout prototype'
        design=adsk.fusion.Design.cast(app.activeProduct);options=app.importManager.createSTEPImportOptions(str(path))
        app.importManager.importToTarget(options,design.rootComponent);design.rootComponent.name='BigSpot_QDD_prototype'
        app.activeViewport.fit()
        ui.messageBox('QDD layout imported. Mounting holes, thermal path and hardware clearances require detailed design verification.')
    except Exception:ui.messageBox(traceback.format_exc())
