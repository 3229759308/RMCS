"""Regenerate README figures: python3 docs/gantry/generate_plots.py.

Dependencies: numpy, matplotlib. Times are nominal CSV times, not video times.
"""
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
with (ROOT / 'data/gantry_motion_2026-09-21_20-45-21.csv').open() as f:
    reader = csv.reader(f)
    header = next(reader)
    data = np.asarray([r for r in reader if len(r) == len(header)], dtype=float)


def col(name):
    return data[:, header.index(name)]


index = col('index')
time = index * 0.005
target = col('/gantry/pitch/target')
feedback = col('/gantry/pitch/measurement')
valid = np.isfinite(target) & np.isfinite(feedback)
start = np.flatnonzero(valid)[0]
error = np.rad2deg(feedback - target)
blue, orange, green = '#2463a6', '#e1812c', '#208477'
plt.rcParams.update({
    'font.family': 'DejaVu Sans', 'font.size': 11,
    'axes.spines.top': False, 'axes.spines.right': False,
    'axes.titleweight': 'bold', 'axes.labelcolor': '#334155',
    'figure.facecolor': 'white', 'savefig.facecolor': 'white',
})


def figure(title):
    fig, axes = plt.subplots(2, 1, figsize=(11, 6.8), sharex=True,
                             layout='constrained')
    fig.suptitle(title, fontsize=17, fontweight='bold')
    for ax in axes:
        ax.grid(True, alpha=0.2)
        ax.set_axisbelow(True)
    axes[-1].set_xlabel('Nominal CSV time (s)')
    return fig, axes


def save(fig, name):
    fig.savefig(OUT / name, dpi=160)
    plt.close(fig)


fig, (ax, err_ax) = figure('Pitch tracking | measured angle and tracking error')
ax.plot(time[valid], np.rad2deg(feedback[valid] - target[start]),
        color=blue, linewidth=1.8, label='Measured')
ax.plot(time[valid], np.rad2deg(target[valid] - target[start]),
        color=orange, linestyle='--', linewidth=1.5, label='Target')
ax.set_ylabel('Pitch relative to entry (deg)')
ax.legend(loc='lower left')
ax.set_title('Target span: 15.9362 deg', fontsize=11, loc='left')
err_ax.plot(time[valid], error[valid], color=blue, linewidth=0.8)
err_ax.axhline(0, color='#64748b', linewidth=0.8)
err_ax.set_ylabel('Measured - target (deg)')
err_ax.set_title('RMS: 0.0307 deg | P95 absolute: 0.0453 deg | Max absolute: 0.1427 deg',
                 fontsize=11, loc='left')
save(fig, 'pitch_tracking.png')

hold = valid & (index >= 21555) & (index <= 30054)
fig, (ax, err_ax) = figure('Yaw motion | pitch target held constant')
ax.plot(time[hold], col('/dart/up_motor/control_velocity')[hold],
        color=orange, linewidth=1.5, label='Yaw speed command')
ax.plot(time[hold], col('/dart/up_motor/velocity')[hold],
        color=blue, linewidth=0.8, alpha=0.8, label='Yaw measured speed')
ax.set_ylabel('Motor output speed (rad/s)')
ax.legend(loc='lower left', ncol=2)
err_ax.plot(time[hold], error[hold], color=green, linewidth=0.8)
err_ax.axhline(0, color='#64748b', linewidth=0.8)
err_ax.set_ylabel('Pitch measured - target (deg)')
err_ax.set_ylim(-0.025, 0.025)
err_ax.set_title('Pitch RMS: 0.0072 deg | Peak-to-peak: 0.0372 deg | Max absolute: 0.0201 deg',
                 fontsize=11, loc='left')
save(fig, 'yaw_pitch_hold.png')

left = col('/dart/left_motor/angle') - col('/dart/left_motor/angle')[start]
right = col('/dart/right_motor/angle') - col('/dart/right_motor/angle')[start]
delta = left - right
fig, (ax, err_ax) = figure('Lift synchronization | relative motor output positions')
ax.plot(time[valid], left[valid], color=blue, linewidth=1.8, label='Left motor')
ax.plot(time[valid], right[valid], color=orange, linestyle='--',
        linewidth=1.5, label='Right motor')
ax.set_ylabel('Travel relative to entry (rad)')
ax.legend(loc='lower left')
err_ax.plot(time[valid], delta[valid], color=green, linewidth=0.9)
err_ax.axhline(0, color='#64748b', linewidth=0.8)
err_ax.set_ylabel('Left - right travel (rad)')
err_ax.set_title('RMS: 0.2956 rad | Max absolute: 0.8175 rad | Motor-axis difference, not tilt angle',
                 fontsize=11, loc='left')
save(fig, 'lift_sync.png')
print('Generated 3 figures in', OUT)
