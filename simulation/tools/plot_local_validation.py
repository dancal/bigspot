"""Plot measured CPU physics traces for an equal-parameter gait comparison."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'simulation/generated/python_deps'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

folder = ROOT / 'simulation/generated/local'
fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True, constrained_layout=True)
for name, label, color in [('crawl_kp300_noshift', 'Before: no torso shift', '#bb4433'),
                           ('crawl_validated', 'After: torso shift 0.6', '#007b88')]:
    data = np.genfromtxt(folder / (name + '.csv'), delimiter=',', names=True)
    axes[0].plot(data['time_s'], data['base_x_m'] - data['base_x_m'][0], label=label, color=color)
    tilt = np.maximum(np.abs(data['roll_rad']), np.abs(data['pitch_rad'])) * 180 / np.pi
    axes[1].plot(data['time_s'], tilt, color=color)
axes[0].set_ylabel('Forward displacement (m)')
axes[1].set_ylabel('Max |roll|, |pitch| (degrees)')
axes[1].set_xlabel('Simulation time (s)')
axes[0].legend(loc='upper left')
for ax in axes:
    ax.grid(alpha=0.25)
    ax.axvspan(2, 18, color='#999999', alpha=0.08)
fig.suptitle('BigSpot / MuJoCo 3.15.0 / floating body, gravity and ground contact\nSame speed 0.015 m/s, kp 300, kv 8; only torso shift changes')
fig.savefig(folder / 'validation.png', dpi=150)
plt.close(fig)
