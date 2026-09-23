"""Build the controller-architecture / demonstrated-task literature map."""

from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Patch

from build_control_validation_plot import (
    ARCHITECTURES, CLOSE_FILL, OTHER_FILL, PAPERS,
)


ROOT = Path(__file__).resolve().parent
STEM = "04_control_task_profile"
TASK_LEVELS = (
    "Attitude regulation",
    "Ascent / hover",
    "Trajectory tracking",
    "Landing / diversion",
)
TASKS = {
    "Denton (2022)": "Ascent / hover",
    "Chen (2024)": "Ascent / hover",
    "Santos (2026)": "Trajectory tracking",
    "Li (2024)": "Trajectory tracking",
    "Torrente (2021)": "Trajectory tracking",
    "Osedo (2023)": "Attitude regulation",
    "Spannagl (2021)": "Landing / diversion",
    "Linsen (2022)": "Landing / diversion",
}


def build(out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "svg.fonttype": "none",
        "font.size": 9,
        "savefig.facecolor": "white",
    })
    fig, ax = plt.subplots(figsize=(11.8, 5.9))
    fig.subplots_adjust(left=0.22, right=0.98, top=0.90, bottom=0.19)
    ax.set_xlim(-0.5, 4.5)
    ax.set_ylim(-0.5, 3.5)
    ax.set_xticks(range(5), ["PID /\ncascade", "Nonlinear\nfeedback",
                             "MPC /\nNMPC", "Residual-model\nlearning",
                             "Direct\nRL policy"])
    ax.set_yticks(range(4), ["Attitude\nregulation", "Ascent /\nhover",
                             "Trajectory\ntracking", "Landing /\ndiversion"])
    ax.set_xlabel("Controller / policy architecture", labelpad=12, fontsize=11)
    ax.set_ylabel("Demonstrated task", labelpad=17, fontsize=11)
    ax.tick_params(axis="both", length=0, pad=8, labelsize=9)

    close_region = Ellipse((0.75, 2.0), width=3.9, height=2.6, angle=30,
                           facecolor=CLOSE_FILL, edgecolor="none", alpha=0.18,
                           zorder=-1)
    close_region.set_gid("region-close-tvc")
    ax.add_patch(close_region)
    other_region = Ellipse((2.9, 1.4), width=3.6, height=2.4, angle=-35,
                           facecolor=OTHER_FILL, edgecolor="none", alpha=0.18,
                           zorder=-1)
    other_region.set_gid("region-other-platforms")
    ax.add_patch(other_region)
    ax.legend(handles=[Patch(facecolor=CLOSE_FILL, alpha=0.35, edgecolor="none",
                             label="Close TVC systems"),
                       Patch(facecolor=OTHER_FILL, alpha=0.35, edgecolor="none",
                             label="Other platforms")],
              loc="upper center", bbox_to_anchor=(0.5, 1.09), ncol=2,
              frameon=False, fontsize=9, handlelength=1.4, columnspacing=2.0)
    for position in (0.5, 1.5, 2.5, 3.5):
        ax.axvline(position, color="0.84", linewidth=0.8, zorder=0)
    for position in (0.5, 1.5, 2.5):
        ax.axhline(position, color="0.84", linewidth=0.8, zorder=0)
    for spine in ax.spines.values():
        spine.set_color("0.72")
        spine.set_linewidth(0.8)

    for paper in PAPERS:
        task = TASKS.get(paper.label)
        if task is None:
            continue
        x = ARCHITECTURES.index(paper.architecture) - 0.39
        y = TASK_LEVELS.index(task) + paper.y_offset
        ax.plot(x, y, "o", markersize=5.5, color="black", zorder=3)
        ax.annotate(paper.label, (x, y), xytext=(6, 0),
                    textcoords="offset points", ha="left", va="center",
                    fontsize=9, color="black", zorder=4)

    for extension in ("png", "svg", "pdf"):
        options = {"dpi": 450} if extension == "png" else {}
        fig.savefig(out / f"{STEM}.{extension}", **options)
    plt.close(fig)


if __name__ == "__main__":
    destination = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "focused_plots"
    build(destination)
    print(f"Created {STEM} in {destination}")
