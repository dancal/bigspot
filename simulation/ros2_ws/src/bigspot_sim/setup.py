from pathlib import Path
import os
from setuptools import setup

here = Path(__file__).parent.resolve()
repo = here.parents[3]
setup(name='bigspot_sim', version='0.1.0', packages=['bigspot_sim'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/bigspot_sim']),
                  ('share/bigspot_sim', ['package.xml']),
                  ('share/bigspot_sim/urdf', ['urdf/bigspot.urdf']),
                  ('share/bigspot_sim/config', [str(p.relative_to(here)) for p in (here/'config').iterdir() if p.suffix in {'.json','.yaml'}]),
                  ('share/bigspot_sim/meshes', [os.path.relpath(p,here) for p in (repo/'src/bigspot_description/meshes').glob('*.stl')]),
                  ('share/bigspot_sim/launch', [str(p.relative_to(here)) for p in (here/'launch').glob('*.py')])],
      install_requires=['setuptools', 'numpy'], zip_safe=True,
      entry_points={'console_scripts': ['controller = bigspot_sim.posture_node:main',
                                      'joystick = bigspot_sim.joystick:main',
                                      'ultrasonic = bigspot_sim.ultrasonic:main']})
