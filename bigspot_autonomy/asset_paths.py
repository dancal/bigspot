"""Resolve our portable URDF mesh URIs before an offline Isaac import."""
import xml.etree.ElementTree as ET
from pathlib import Path

def resolve_meshes(source,repository,destination):
    root=ET.parse(source);prefix='package://bigspot_autonomy/'
    for mesh in root.findall('.//mesh'):
        uri=mesh.get('filename','')
        if uri.startswith(prefix):
            path=(Path(repository)/uri[len(prefix):]).resolve()
            if not path.is_file():raise FileNotFoundError(path)
            mesh.set('filename',path.as_uri())
    path=Path(destination);path.parent.mkdir(parents=True,exist_ok=True)
    root.write(path,encoding='utf-8',xml_declaration=True)
    return path
