"""Package source + existing STL assets for transfer to the target Ubuntu server."""
from pathlib import Path
import zipfile

repo = Path(__file__).resolve().parents[2]
output = repo.parent/'bigspot-isaacros5.zip'
excluded = {'.git','__pycache__','generated','build','install','log','.venv'}
with zipfile.ZipFile(output,'w',compression=zipfile.ZIP_DEFLATED) as archive:
    for name in ['src','simulation','README.md','LICENSE','.gitignore']:
        root = repo/name
        paths = root.rglob('*') if root.is_dir() else [root]
        for path in paths:
            relative = path.relative_to(repo)
            if path.is_file() and not excluded.intersection(relative.parts) and path.suffix not in {'.exe','.pyc','.zip'}:
                archive.write(path,Path('bigspot')/relative)
print(f'{output.name}: {output.stat().st_size/1024/1024:.1f} MiB')
