"""Build four standalone, publication-review TVC figures.

Published observations use black dots and author/year labels. Our current
simulation/model point is a blue dot labeled "Our Vehicle" in three figures.
Separate figures distinguish close systems from method-transfer studies.
"""

from pathlib import Path
import os
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
OUT = Path(os.environ.get("TVC_FIGURE_OUT", ROOT / "focused_plots"))
OUT.mkdir(parents=True, exist_ok=True)
G = 9.80665
OUR_BLUE = "#1768ac"

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "svg.fonttype": "none",
    "font.size": 9,
    "axes.labelsize": 10,
    "xtick.labelsize": 8.5,
    "ytick.labelsize": 8.5,
    "axes.edgecolor": "0.4",
    "axes.linewidth": 0.8,
    "savefig.facecolor": "white",
})


def base(xlabel, ylabel, *, left=.12, bottom=.15):
    fig, ax = plt.subplots(figsize=(10, 5.5))
    fig.subplots_adjust(left=left, right=.97, top=.96, bottom=bottom)
    ax.set_xlabel(xlabel, labelpad=10)
    ax.set_ylabel(ylabel, labelpad=10)
    ax.grid(True, which="major", color=".90", linewidth=.75, zorder=0)
    ax.set_axisbelow(True)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    return fig, ax


def dot(ax, x, y, label, dx=7, dy=6, ha="left", va="bottom", size=8.6,
        color="black"):
    ax.plot([x], [y], "o", color=color, markersize=5.5, zorder=5)
    ax.annotate(label, (x, y), xytext=(dx, dy), textcoords="offset points",
                ha=ha, va=va, fontsize=size, color=color, zorder=6)


def save(fig, stem):
    fig.savefig(OUT / f"{stem}.png", dpi=450)
    fig.savefig(OUT / f"{stem}.svg")
    fig.savefig(OUT / f"{stem}.pdf")
    plt.close(fig)


# Figure 1 is the numerical Ashby-style property chart. Its thrust values are
# heterogeneous evidence, not a like-for-like measured-max comparison.
fig, ax = base("Vehicle mass (kg)", "Thrust reference (N)", left=.11)
ax.set_xscale("log")
ax.set_yscale("log")
ax.set_xlim(.30, 2.0)
ax.set_ylim(5, 40)
ax.set_xticks([.3, .5, .7, 1.0, 1.5, 2.0], labels=["0.3", "0.5", "0.7", "1.0", "1.5", "2.0"])
ax.set_yticks([5, 10, 20, 40], labels=["5", "10", "20", "40"])
ax.minorticks_off()
xs = np.geomspace(.30, 2.0, 100)
for ratio in (1, 2):
    ax.plot(xs, ratio * G * xs, color=".68", linewidth=1,
            linestyle=(0, (4, 4)), zorder=1)
    label_x = 1.91
    ax.text(label_x, ratio * G * label_x * 1.04, f"T/W = {ratio}",
            fontsize=8.4, color=".43", ha="right")

dot(ax, .366, 2 * .366 * G, "Denton (2022)", dx=6, dy=6)
dot(ax, .6523, .01671 * 1600 - 15.333, "Chen (2024)", dx=6, dy=6)
dot(ax, 1.16, 16.5, "Spannagl (2021)", dx=-6, dy=6, ha="right")
dot(ax, 1.328, 17.79, "Our Vehicle", dx=-6, dy=6, ha="right", color=OUR_BLUE)
dot(ax, 1.7, 2.3 * G, "Linsen (2022)", dx=-6, dy=6, ha="right")
dot(ax, 1.6, 3 * G, "Santos (2026)", dx=-6, dy=6, ha="right")
save(fig, "01_hardware_thrust")


# Figures 2a and 2b use the same categorical axes, but separate close
# systems from method-transfer evidence on different hardware.

control_ticks = ["PID /\ncascade", "Nonlinear\nfeedback", "MPC /\nNMPC",
                 "Residual-model\nlearning", "Direct\nRL policy"]
gnc_ticks = ["Fixed / pilot\ntarget", "Given\ntrajectory", "Onboard guidance\nor goal policy"]


def control_figure(observations, stem, ncols=5, compact=False):
    fig, ax = base("Implemented controller / policy architecture",
                   "GNC function demonstrated", left=.19, bottom=.23)
    if compact:
        fig.subplots_adjust(left=.25, right=.75, top=.90, bottom=.24)
    ax.grid(False)
    ax.set_xlim(-.5, ncols - .5)
    ax.set_ylim(-.43, 2.43)
    ax.set_xticks(range(ncols), control_ticks[:ncols])
    ax.set_yticks(range(3), gnc_ticks)
    ax.tick_params(length=0, axis="both", pad=6, labelsize=8.4)
    for x in range(ncols - 1):
        ax.axvline(x + .5, color=".84", linewidth=.8, zorder=0)
    for y in (.5, 1.5):
        ax.axhline(y, color=".84", linewidth=.8, zorder=0)
    for spine in ax.spines.values():
        spine.set_visible(True)
        spine.set_color(".75")
    for x, y, label, offset in observations:
        color = OUR_BLUE if label == "Our Vehicle" else "black"
        if x == 4:
            dot(ax, x + .43, y + offset, label, dx=-6, dy=0,
                ha="right", va="center", size=8.6, color=color)
        else:
            dot(ax, x - .43, y + offset, label, dx=6, dy=0,
                va="center", size=8.6, color=color)
    save(fig, stem)


control_figure([
    (0, 0, "Denton (2022)", -.18),
    (0, 0, "Our Vehicle", .18),
    (0, 1, "Santos (2026)", 0),
    (1, 0, "Chen (2024)", .13),
    (2, 2, "Spannagl (2021)", -.14),
    (2, 2, "Linsen (2022)", .14),
], "02a_close_tvc_control", ncols=3, compact=True)
control_figure([
    (2, 1, "Li (2024)", 0),
    (3, 1, "Torrente (2021)", 0),
    (4, 0, "Osedo (2023)", 0),
], "02b_transfer_methods")


# Figure 3: same close papers, plus the explicitly qualified current
# project stage. Task labels are demonstrated outcomes, not controller types.
fig, ax = base("Experimental setting", "Demonstrated task", left=.18, bottom=.16)
ax.set_xlim(-.5, 2.5)
ax.set_ylim(-.45, 3.45)
ax.set_xticks([0, 1, 2], ["Model / bench", "Indoor free flight", "Outdoor free flight"])
ax.set_yticks([0, 1, 2, 3], ["Attitude stabilization", "Ascent / hover",
                               "Trajectory tracking", "Landing / diversion"])
ax.grid(False)
for x in (.5, 1.5):
    ax.axvline(x, color=".82", linewidth=.8)
for y in (.5, 1.5, 2.5):
    ax.axhline(y, color=".82", linewidth=.8)
for x, y, label, offset in [
    (0, 0, "Our Vehicle", 0),
    (1, 1, "Chen (2024)", 0),
    (1, 2, "Santos (2026)", 0),
    (2, 1, "Denton (2022)", 0),
    (2, 3, "Spannagl (2021)", -.16),
    (2, 3, "Linsen (2022)", .16),
]:
    dot(ax, x - .37, y + offset, label, dx=6, dy=0,
        va="center", size=8.6,
        color=OUR_BLUE if label == "Our Vehicle" else "black")
save(fig, "03_validation")

STEMS = ("01_hardware_thrust", "02a_close_tvc_control",
         "02b_transfer_methods", "03_validation")
assert all((OUT / f"{stem}.{ext}").exists()
           for stem in STEMS for ext in ("png", "svg", "pdf"))
print("Created focused 4 PNG + 4 SVG + 4 vector PDF figures:", OUT)
