"""Six source-auditable, monochrome TVC literature-positioning plots.

Run: python research_outputs/professor/build_ashby_plots.py
Only directly reported values, clearly identified calculations, and a marked
POSTECH design-model estimate are plotted. No marker aesthetics encode data.
"""

from pathlib import Path
import math
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "plots"
OUT.mkdir(exist_ok=True)
G = 9.80665

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "svg.fonttype": "none",
    "font.size": 11,
    "axes.labelsize": 12,
    "axes.titlesize": 17,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "axes.edgecolor": "0.4",
    "axes.linewidth": 0.8,
    "savefig.facecolor": "white",
})


def base(title, subtitle, xlabel, ylabel, *, bottom=0.18, left=0.12):
    fig, ax = plt.subplots(figsize=(15.6, 7.2))
    fig.subplots_adjust(left=left, right=0.94, top=0.81, bottom=bottom)
    fig.text(0.045, 0.95, title, ha="left", va="top", fontsize=21, weight="bold")
    fig.text(0.045, 0.895, subtitle, ha="left", va="top", fontsize=11, color="0.25")
    ax.set_xlabel(xlabel, labelpad=10)
    ax.set_ylabel(ylabel, labelpad=10)
    ax.grid(True, which="major", color="0.89", linewidth=0.75, zorder=0)
    ax.set_axisbelow(True)
    return fig, ax


def dot(ax, x, y, label, dx=7, dy=6, ha="left", va="bottom", size=10.4):
    ax.plot([x], [y], "o", color="black", markersize=6.7, zorder=5)
    ax.annotate(label, (x, y), xytext=(dx, dy), textcoords="offset points",
                ha=ha, va=va, fontsize=size, color="black", zorder=6)


def note(fig, content):
    fig.text(0.045, 0.055, content, ha="left", va="bottom", fontsize=9.6,
             color="0.25")


def save(fig, stem):
    fig.savefig(OUT / f"{stem}.png", dpi=200)
    fig.savefig(OUT / f"{stem}.svg")
    plt.close(fig)


# 1. True physical-property Ashby comparison. Chen is an estimate from the
# published thrust calibration at 1600 us, not the paper's certified T_max.
fig, ax = base("01  Vehicle mass vs available thrust",
               "Physical-property map | direct TVC comparators + marked POSTECH design model",
               "Vehicle mass (kg)", "Available thrust (N)")
ax.set_xlim(0.25, 1.9)
ax.set_ylim(0, 62)
xs = np.linspace(0.25, 1.9, 100)
for ratio in (1, 2, 3):
    ax.plot(xs, ratio * G * xs, color="0.65", linewidth=1,
            linestyle=(0, (4, 4)), zorder=1)
    ax.text(1.82, ratio * G * 1.82 + 0.5, f"T/W = {ratio}", fontsize=9,
            color="0.43", ha="right")
dot(ax, 0.366, 2 * 0.366 * G, "Denton (2022)\n~2 T/W", 9, 8)
dot(ax, 0.6523, 0.01671 * 1600 - 15.333, "Chen (2024)\nfit at 1600 us", 9, 8)
dot(ax, 1.328, 17.79, "POSTECH model*", 9, -9, va="top")
dot(ax, 1.7, 2.3 * G, "Linsen (2022)", 9, -5, va="top")
dot(ax, 1.6, 3 * G, "Santos (2026)\nE-Rocket (stated)", 9, 7)
dot(ax, 1.55, 55, "Santos (2026)\nQuadRocket", 9, 6)
note(fig, "Spannagl: thrust limit not stated. Osedo: constrained rig, not free-flight T/W.  "
          "*POSTECH mass is a design estimate; thrust is the current model/bench-derived limit.")
save(fig, "01_hardware_mass_thrust")


# 2. Mechanical/control-authority positioning. The pale isolines are simple
# ideal-static geometry, NOT achieved translational acceleration in flight.
fig, ax = base("02  Gimbal reach vs thrust margin",
               "Physical-property map | ideal upright lateral force contours are context, not flight results",
               "One-axis gimbal travel from neutral (degrees)", "Maximum thrust / weight")
ax.set_xlim(3.5, 35)
ax.set_ylim(1.05, 2.22)
ang = np.linspace(3.5, 35, 500)
for lateral in (2, 4, 6, 8):
    ratio = lateral / (G * np.sin(np.deg2rad(ang)))
    ax.plot(ang, ratio, color="0.76", linewidth=0.8, linestyle=(0, (3, 5)), zorder=1)
    idx = np.argmin(np.abs(ratio - 2.10))
    if 3.5 < ang[idx] < 35:
        ax.text(ang[idx] + .15, 2.105, f"{lateral} m/s²", fontsize=8.8,
                color="0.46", rotation=-22)
dot(ax, 7, 17.79 / (1.328 * G), "POSTECH model*", 8, 8)
dot(ax, 15, 2.3 / 1.7, "Linsen (2022)", 8, -10, va="top")
dot(ax, 30, 3 / 1.6, "Santos (2026)\nE-Rocket (stated)", -10, -9, ha="right", va="top")
dot(ax, 30, 2, "Denton (2022)", -10, 9, ha="right")
note(fig, "Contours: (Tmax/m) sin(angle), upright static geometry only.  "
          "QuadRocket's universal joint is not a two-axis servo gimbal; Chen does not state an angle limit.  "
          "*POSTECH mixed design estimate/bench model.")
save(fig, "02_hardware_gimbal_margin")


# 3. Categorical software positioning. Cell offsets only separate labels.
fig, ax = base("03  Control family vs demonstrated GNC scope",
               "Categorical positioning map | paper labels are not a numerical score or novelty proof",
               "Controller family", "Demonstrated GNC scope", bottom=.20, left=.17)
ax.set_xlim(-0.5, 3.5)
ax.set_ylim(-0.4, 2.45)
ax.set_xticks(range(4), ["Feedback / nonlinear\nallocation", "Optimization\nMPC / NMPC",
                           "Adaptive\nnonlinear", "Direct RL"])
ax.set_yticks(range(3), ["Attitude only", "Trajectory / ascent tracking", "Online guidance + tracking"])
ax.grid(False)
for x in (0.5, 1.5, 2.5):
    ax.axvline(x, color="0.82", linewidth=0.8)
for y in (0.5, 1.5):
    ax.axhline(y, color="0.82", linewidth=0.8)
for x, y, label, dy in [
    (0, 0, "Denton (2022)", 0),
    (0, 1, "Chen (2024)", -.22),
    (0, 1, "Santos (2026) E-Rocket", .20),
    (1, 2, "Spannagl (2021)", -.19),
    (1, 2, "Linsen (2022)", .19),
    (2, 1, "Santos (2026) QuadRocket", 0),
    (3, 0, "Osedo (2023) [rig]", 0),
    (1, 1, "Torrente (2021) [transfer]", 0),
    (3, 1, "Zhang (2026) [transfer]", 0),
]:
    xx = x - .40
    yy = y + dy
    dot(ax, xx, yy, label, dx=5, dy=0, va="center", size=9.0)
note(fig, "Core TVC papers plus two explicitly marked method-transfer studies.  "
          "'Direct RL' does not mean residual RL; no selected close free-flight paper here tests residual RL.")
save(fig, "03_software_control_gnc")


# 4. Implementation rates are deliberately loop-specific, not vehicle-wide.
fig, ax = base("04  Reported loop rate vs compute location",
               "Software-system map | different loop layers, so frequency is not a controller quality ranking",
               "Reported periodic function rate (Hz; log scale)", "Where the function runs", left=.18)
ax.set_xscale("log")
ax.set_xlim(18, 400)
ax.set_ylim(-.47, 2.47)
ax.set_xticks([25, 50, 100, 250], ["25", "50", "100", "250"])
ax.set_yticks([0, 1, 2], ["Onboard microcontroller", "Onboard companion computer",
                           "Offboard laboratory computer"])
ax.grid(True, axis="x", color="0.88")
ax.grid(False, axis="y")
for x, y, label, dy in [
    (250, 0, "Chen (2024) control", 7),
    (25, 1, "Spannagl (2021) MPC", 7),
    (50, 1, "Linsen (2022) NMPC", -18),
    (50, 1, "Osedo (2023) policy [rig]", 15),
    (100, 1, "Zhang (2026) policy [transfer]", 7),
    (50, 2, "Torrente (2021) GP-MPC [transfer]", 7),
    (100, 2, "Santos (2026) QuadRocket tracking", 7),
]:
    dot(ax, x, y, label, dx=7, dy=dy, va="bottom" if dy >= 0 else "top", size=9.7)
note(fig, "Denton reports 1 kHz gyro sampling / 200 Hz attitude estimation, not an explicit comparable controller update rate.  "
          "E-Rocket does not report its PID rate.")
save(fig, "04_software_rate_location")


# 5. The deadline plot uses three distinct timing statistics. State this on
# the plot so a mean is not silently compared to a bound as if equivalent.
fig, ax = base("05  Controller computation vs update deadline",
               "Software real-time evidence | only papers with a published numeric execution time appear",
               "Controller update period (ms)", "Reported computation / inference time (ms)")
ax.set_xlim(0, 46)
ax.set_ylim(0, 43)
line = np.linspace(0, 43, 100)
ax.plot(line, line, color="0.45", linewidth=1.15, linestyle=(0, (5, 4)))
ax.text(41.2, 40, "deadline", fontsize=9, color="0.38", ha="right", rotation=42)
dot(ax, 40, 30, "Spannagl (2021)\n30 ms mean MPC", -8, -7, ha="right", va="top")
dot(ax, 20, 18, "Linsen (2022)\n<=18 ms NMPC", -8, 8, ha="right")
dot(ax, 10, .3, "Zhang (2026) [transfer]\n~0.3 ms policy inference", 9, 7)
note(fig, "Mean, stated upper bound, and approximate inference time are different statistics.  "
          "Deadline line is y = x. This plot is not a worst-case safety certification.")
save(fig, "05_software_compute_deadline")


# 6. Highest demonstrated task/environment, not a vehicle performance score.
fig, ax = base("06  Experimental evidence vs demonstrated task",
               "Validation-positioning map | each study appears at its strongest reported task + setting",
               "Experimental setting", "Demonstrated task", bottom=.20, left=.17)
ax.set_xlim(-.5, 3.5)
ax.set_ylim(-.45, 3.45)
ax.set_xticks(range(4), ["Model / bench", "Constrained rig", "Indoor free flight", "Outdoor free flight"])
ax.set_yticks(range(4), ["Attitude regulation", "Ascent / hover", "Trajectory tracking",
                           "Landing / diversion"])
ax.grid(False)
for x in (.5, 1.5, 2.5):
    ax.axvline(x, color="0.82", linewidth=.8)
for y in (.5, 1.5, 2.5):
    ax.axhline(y, color="0.82", linewidth=.8)
for x, y, label, dy in [
    (0, 0, "POSTECH current*", 0),
    (1, 0, "Osedo (2023)", 0),
    (2, 1, "Chen (2024)", 0),
    (3, 1, "Denton (2022)", 0),
    (2, 2, "Santos (2026) E-Rocket", -.17),
    (2, 2, "Santos (2026) QuadRocket", .17),
    (3, 3, "Linsen (2022)", -.17),
    (3, 3, "Spannagl (2021)", .17),
]:
    dot(ax, x - .39, y + dy, label, dx=5, dy=0, va="center", size=9.1)
note(fig, "*POSTECH: design/simulation and actuator bench characterization, not free flight.  "
          "Chen point refers to custom-controller ascent; separate hover tests use a different stack.")
save(fig, "06_validation_task_setting")

assert len(list(OUT.glob("*.png"))) == 6
assert len(list(OUT.glob("*.svg"))) == 6
print("Created 6 PNG and 6 SVG plots in", OUT)
