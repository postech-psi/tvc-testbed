"""Read, save and plot simulation results using a shared CSV format."""
import argparse
import csv

from pathlib import Path
from matplotlib.figure import Figure

from .config import SUPPORTED_AXIS_CONVENTIONS, load

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
                    "meanings are not -- see docs/GUIDE.md."
                    % (path, token, ", ".join(SUPPORTED_AXIS_CONVENTIONS)))
        else:
            raise SystemExit(
                "%s carries no axis-convention header, so it predates the rocket "
                "convention. Its roll/pitch/yaw columns mean different axes than "
                "they do now; re-fly it rather than plotting it. "
                "See docs/GUIDE.md." % path)
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"No simulation samples in {path}")
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def series_from_run(r):
    """-> the nine series, from a simulation.simulate() result dict.

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


def make_figure(d, title="TVC simulation", subtitle="", target_m=None, figure=None):
    """Draw altitude, position, attitude and gimbal for either the CLI or GUI."""
    missing = [k for k in SERIES if k not in d]
    if missing:
        raise ValueError("series missing from the run: %s" % ", ".join(missing))
    t = d["t_s"]

    fig = figure if figure is not None else Figure(figsize=(11, 10))
    fig.clear()
    axes = fig.subplots(4, 1, sharex=True)
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
    ax.set_title("Horizontal position", fontsize=10, color=INK,
                 loc="left")
    ax.legend(fontsize=8, frameon=False, loc="upper right", ncol=2)

    # --- attitude ---------------------------------------------------------
    ax = axes[2]
    ax.axhline(0, color=GRID, lw=1, zorder=1)
    ax.plot(t, d["pitch_deg"], color=C_BLUE, lw=2, label="pitch", zorder=3)
    ax.plot(t, d["yaw_deg"], color=C_ORANGE, lw=2, label="yaw", zorder=3)
    ax.plot(t, d["roll_deg"], color=C_AQUA, lw=2, label="roll", zorder=3)
    ax.set_ylabel("attitude [deg]", color=INK)
    ax.set_title("Attitude", fontsize=10, color=INK, loc="left")
    ax.legend(fontsize=8, frameon=False, loc="upper right", ncol=3)

    # --- gimbal -----------------------------------------------------------
    ax = axes[3]
    glim = GIMBAL_LIMIT_DEG
    for lim in (glim, -glim):
        ax.axhline(lim, color=C_RED, lw=1, ls=":", zorder=1)
    ax.annotate("+/-%g deg nominal envelope" % glim, (t[-1], glim),
                xytext=(-6, -12), textcoords="offset points", ha="right",
                fontsize=8, color=C_RED)
    ax.plot(t, d["gimbal_outer_deg"], color=C_BLUE, lw=2, label="gimbal pitch",
            zorder=3)
    ax.plot(t, d["gimbal_inner_deg"], color=C_ORANGE, lw=2, label="gimbal yaw",
            zorder=3)
    ax.set_ylabel("deflection [deg]", color=INK)
    ax.set_xlabel("time [s]", color=INK)
    ax.set_ylim(-glim * 1.2, glim * 1.2)
    ax.set_title("Gimbal angles", fontsize=10, color=INK, loc="left")
    ax.legend(fontsize=8, frameon=False, loc="upper right", ncol=2)

    for ax in axes:
        ax.ticklabel_format(axis="y", style="plain", useOffset=False)
        ax.grid(color=GRID, lw=0.6, alpha=0.7)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(GRID)
        ax.tick_params(colors=MUTED, labelsize=8)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    return fig


def four_panel(d, out, title="TVC simulation", subtitle="", target_m=None):
    fig = make_figure(d, title, subtitle, target_m)
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130, facecolor="white")
    print("wrote %s" % out)


def save_run(result, path):
    """Save the analytic result in the same CSV format as the ROS controller."""
    data = series_from_run(result)
    data["thrust_N"] = result["thrust_N"]
    data["tau_p_Nm"] = result["tau_p_Nm"]
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as stream:
        stream.write("# axis_convention: rocket_v2\n")
        writer = csv.writer(stream)
        writer.writerow(data)
        writer.writerows(zip(*data.values()))


def main(argv=None):
    """Read a Gazebo flight log and write the four-panel PNG."""
    ap = argparse.ArgumentParser(
        prog="tvc.py plot", description="Plot a Gazebo flight log CSV.")
    ap.add_argument("log", help="CSV written by sim, the GUI, or the ROS controller")
    ap.add_argument("out", nargs="?", default="out/flight.png",
                    help="output PNG (default: out/flight.png)")
    args = ap.parse_args(argv)
    d = read_log(args.log)
    four_panel(d, args.out,
               "Coaxial TVC vehicle",
               "thrust vectoring only: 2-axis gimbal for yaw/pitch, "
               "rotor differential for roll",
               target_m=None)

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
