"""
The four-panel flight figure, for either plant.
================================================================================
    python tvc.py plot flight_log.csv [out.png]     a Gazebo run
    python tvc.py validate --plot out/              the analytic scenarios

Small multiples on a shared time axis rather than one crowded plot with twin
y-axes: altitude (m), position (m), attitude (deg) and gimbal (deg) are four
different scales, and overlaying them on two axes makes the reader guess which
curve belongs to which axis.

ONE FIGURE, TWO PLANTS. `four_panel` is the only place the layout exists, and
both the Gazebo CSV and the analytic harness feed it the same nine series. That
is the point: when the two plants disagree, the pictures have to be comparable
at a glance, and they cannot be if each pipeline draws its own.

The Gazebo log carries an axis-convention token in its first line and this
refuses to plot one it does not recognise. A pre-rename log has the same column
NAMES with different meanings, so plotting it would produce a picture that is
wrong in a way nothing about it looks wrong. See docs/4-CONVENTIONS.md.
"""
import argparse
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from ..config import SUPPORTED_AXIS_CONVENTIONS, load

# No fallback. This used to default to 15.0 deg if the params import failed --
# a value superseded by the 7 deg bench measurement, which would have silently
# drawn every gimbal trace against a limit line at twice the real travel. A plot
# that lies about a limit is worse than no plot.
GIMBAL_LIMIT_DEG = load().gimbal_max_deg

# Categorical slots in fixed order (never cycled), from the validated palette.
C_BLUE, C_ORANGE, C_AQUA, C_RED = "#2a78d6", "#eb6834", "#1baf7a", "#e34948"
INK, MUTED, GRID = "#1a1a18", "#6b6b66", "#d8d8d4"

# The nine series every panel needs, in the order they are drawn.
SERIES = ("t_s", "z_m", "x_m", "y_m", "pitch_deg", "yaw_deg", "roll_deg",
          "gimbal_outer_deg", "gimbal_inner_deg")


def read_log(path):
    """-> {column: [float]}, after checking the convention token."""
    with open(path, newline="", encoding="utf-8") as f:
        first = f.readline()
        if first.lstrip().startswith("#"):
            token = first.split(":", 1)[-1].strip()
            if token not in SUPPORTED_AXIS_CONVENTIONS:
                raise SystemExit(
                    "%s was written in axis convention %r, which this build does "
                    "not speak (%s). The column names are the same and the "
                    "meanings are not -- see docs/4-CONVENTIONS.md."
                    % (path, token, ", ".join(SUPPORTED_AXIS_CONVENTIONS)))
        else:
            raise SystemExit(
                "%s carries no axis-convention header, so it predates the rocket "
                "convention. Its roll/pitch/yaw columns mean different axes than "
                "they do now; re-fly it rather than plotting it. "
                "See docs/4-CONVENTIONS.md." % path)
        rows = list(csv.DictReader(f))
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def series_from_run(r):
    """-> the nine series, from a harness.mil.simulate() result dict.

    The analytic harness keeps its history as numpy arrays with a shape per
    quantity; this is the one place that knows which column is which, so the
    plotting code below never has to.
    """
    euler, delta, pos = r["euler_deg"], r["delta_deg"], r["pos"]
    return {
        "t_s": r["t"],
        "z_m": pos[:, 2], "x_m": pos[:, 0], "y_m": pos[:, 1],
        "pitch_deg": euler[:, 0], "yaw_deg": euler[:, 1], "roll_deg": euler[:, 2],
        # delta is stored [inner, outer] -- the order the controller emits.
        "gimbal_inner_deg": delta[:, 0], "gimbal_outer_deg": delta[:, 1],
    }


def four_panel(d, out, title, subtitle, target_m=None):
    """Draw altitude / position / attitude / gimbal and write `out`."""
    missing = [k for k in SERIES if k not in d]
    if missing:
        raise ValueError("series missing from the run: %s" % ", ".join(missing))
    t = d["t_s"]

    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    fig.suptitle("%s\n%s" % (title, subtitle), fontsize=12, color=INK, y=0.985)

    # --- altitude ---------------------------------------------------------
    ax = axes[0]
    if target_m is not None:
        ax.axhline(target_m, color=MUTED, lw=1, ls="--", zorder=1)
        ax.annotate("target %.1f m" % target_m, (t[-1], target_m),
                    xytext=(-6, 6), textcoords="offset points", ha="right",
                    fontsize=8, color=MUTED)
    ax.plot(t, d["z_m"], color=C_BLUE, lw=2, zorder=3)
    ax.set_ylabel("altitude [m]", color=INK)
    ax.set_title("Altitude", fontsize=10, color=INK, loc="left")

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
    plt.close(fig)
    print("wrote %s" % out)


def main(argv=None):
    """Read a Gazebo flight log and write the four-panel PNG."""
    ap = argparse.ArgumentParser(
        prog="tvc.py plot", description="Plot a Gazebo flight log CSV.")
    ap.add_argument("log", help="CSV written by `tvc.py hover --log`")
    ap.add_argument("out", nargs="?", default="flight_plot.png",
                    help="output PNG (default: flight_plot.png)")
    args = ap.parse_args(argv)
    d = read_log(args.log)
    four_panel(d, args.out,
               "Coaxial TVC vehicle - Gazebo hover",
               "thrust vectoring only: 2-axis gimbal for yaw/pitch, "
               "rotor differential for roll",
               target_m=round(max(d["z_m"]), 1))

    t = d["t_s"]
    print("final: z=%.2f m  drift=%.2f m  tilt=%.1f deg  roll=%.1f deg"
          % (d["z_m"][-1],
             (d["x_m"][-1] ** 2 + d["y_m"][-1] ** 2) ** 0.5,
             (d["pitch_deg"][-1] ** 2 + d["yaw_deg"][-1] ** 2) ** 0.5,
             d["roll_deg"][-1]))
    print("samples: %d over %.1f s" % (len(t), t[-1]))


if __name__ == "__main__":
    import sys
    sys.exit(main() or 0)
