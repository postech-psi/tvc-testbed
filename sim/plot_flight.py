"""
Plot a hover flight logged by hover.py --log.

    python3 sim/plot_flight.py sim/flight_log.csv [out.png]

Small multiples on a shared time axis rather than one crowded plot with twin
y-axes: altitude (m), position (m), attitude (deg) and gimbal (deg) are four
different scales, and overlaying them on two axes makes the reader guess which
curve belongs to which axis.
"""
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from vehicle_params import load as _load_vehicle
    GIMBAL_LIMIT_DEG = _load_vehicle().gimbal_max_deg
except Exception:            # keep plotting even if the params file is absent
    GIMBAL_LIMIT_DEG = 15.0

# Categorical slots in fixed order (never cycled), from the validated palette.
C_BLUE, C_ORANGE, C_AQUA, C_RED = "#2a78d6", "#eb6834", "#1baf7a", "#e34948"
INK, MUTED, GRID = "#1a1a18", "#6b6b66", "#d8d8d4"


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "sim/flight_log.csv"
    out = sys.argv[2] if len(sys.argv) > 2 else "sim/flight_plot.png"
    d = load(src)
    t = d["t_s"]
    target = round(max(d["z_m"]), 1)

    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    fig.suptitle("Coaxial TVC vehicle - Gazebo hover\n"
                 "thrust vectoring only: 2-axis gimbal for yaw/pitch, "
                 "rotor differential for roll",
                 fontsize=12, color=INK, y=0.985)

    # --- altitude ---------------------------------------------------------
    ax = axes[0]
    ax.axhline(target, color=MUTED, lw=1, ls="--", zorder=1)
    ax.annotate("target %.1f m" % target, (t[-1], target), xytext=(-6, 6),
                textcoords="offset points", ha="right", fontsize=8, color=MUTED)
    ax.plot(t, d["z_m"], color=C_BLUE, lw=2, zorder=3)
    ax.set_ylabel("altitude [m]", color=INK)
    ax.set_title("Altitude hold", fontsize=10, color=INK, loc="left")

    # --- horizontal position ---------------------------------------------
    ax = axes[1]
    ax.axhline(0, color=GRID, lw=1, zorder=1)
    ax.plot(t, d["x_m"], color=C_BLUE, lw=2, label="x", zorder=3)
    ax.plot(t, d["y_m"], color=C_ORANGE, lw=2, label="y", zorder=3)
    ax.set_ylabel("position [m]", color=INK)
    ax.set_title("Station keeping (drift from origin)", fontsize=10, color=INK,
                 loc="left")
    ax.legend(fontsize=8, frameon=False, loc="upper right", ncol=2)

    # --- attitude ---------------------------------------------------------
    ax = axes[2]
    ax.axhline(0, color=GRID, lw=1, zorder=1)
    ax.plot(t, d["pitch_deg"], color=C_BLUE, lw=2, label="pitch", zorder=3)
    ax.plot(t, d["yaw_deg"], color=C_ORANGE, lw=2, label="yaw", zorder=3)
    ax.plot(t, d["roll_deg"], color=C_AQUA, lw=2, label="roll", zorder=3)
    ax.set_ylabel("attitude [deg]", color=INK)
    ax.set_title("Attitude - roll is held by rotor differential, the vehicle's "
                 "only roll authority", fontsize=10, color=INK, loc="left")
    ax.legend(fontsize=8, frameon=False, loc="upper right", ncol=3)

    # --- gimbal -----------------------------------------------------------
    ax = axes[3]
    glim = GIMBAL_LIMIT_DEG
    for lim in (glim, -glim):
        ax.axhline(lim, color=C_RED, lw=1, ls=":", zorder=1)
    ax.annotate("+/-%g deg mechanical limit" % glim, (t[-1], glim),
                xytext=(-6, -12), textcoords="offset points", ha="right",
                fontsize=8, color=C_RED)
    ax.plot(t, d["gimbal_outer_deg"], color=C_BLUE, lw=2, label="gimbal pitch",
            zorder=3)
    ax.plot(t, d["gimbal_inner_deg"], color=C_ORANGE, lw=2, label="gimbal yaw",
            zorder=3)
    ax.set_ylabel("deflection [deg]", color=INK)
    ax.set_xlabel("time [s]", color=INK)
    ax.set_ylim(-glim * 1.2, glim * 1.2)
    ax.set_title("Gimbal deflection - this is what tilts the thrust vector",
                 fontsize=10, color=INK, loc="left")
    ax.legend(fontsize=8, frameon=False, loc="upper right", ncol=2)

    for ax in axes:
        ax.grid(color=GRID, lw=0.6, alpha=0.7)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(GRID)
        ax.tick_params(colors=MUTED, labelsize=8)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out, dpi=130, facecolor="white")
    print("wrote %s" % out)
    n = len(t)
    print("final: z=%.2f m  drift=%.2f m  tilt=%.1f deg  roll=%.1f deg"
          % (d["z_m"][-1],
             (d["x_m"][-1] ** 2 + d["y_m"][-1] ** 2) ** 0.5,
             (d["pitch_deg"][-1] ** 2 + d["yaw_deg"][-1] ** 2) ** 0.5,
             d["roll_deg"][-1]))
    print("samples: %d over %.1f s" % (n, t[-1]))


if __name__ == "__main__":
    main()
