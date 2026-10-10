"""Serve only the generated video library on an automatically chosen loopback port."""
import functools
import http.server
import json
import os
from pathlib import Path

folder = Path(__file__).resolve().parents[1] / 'generated/joystick_videos'
if not (folder / 'index.html').is_file():
    raise RuntimeError('Build the video library first')
handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(folder))
server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), handler)
(folder / 'server.json').write_text(json.dumps({
    'pid': os.getpid(), 'url': f'http://127.0.0.1:{server.server_port}/index.html',
    'scope': 'loopback only; generated joystick video directory',
}, indent=2) + '\n', encoding='utf-8')
server.serve_forever()
