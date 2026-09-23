"""Build the combined controller-architecture / validation-evidence map."""

from dataclasses import dataclass
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Patch


ROOT = Path(__file__).resolve().parent
STEM = "02_control_validation_profile"
CLOSE_FILL = "#d56d93"
OTHER_FILL = "#75a9c9"

ARCHITECTURES = (
    "PID / cascade",
    "Nonlinear feedback",
    "MPC / NMPC",
    "Residual-model learning",
    "Direct RL policy",
)
VALIDATION = (
    "Constrained hardware rig",
    "Indoor free flight",
    "Outdoor free flight",
)


@dataclass(frozen=True)
class Paper:
    label: str
    architecture: str
    validation: str
    y_offset: float = 0.0


PAPERS = (
    Paper("Denton (2022)", "PID / cascade", "Outdoor free flight"),
    Paper("Chen (2024)", "Nonlinear feedback", "Indoor free flight"),
    Paper("Spannagl (2021)", "MPC / NMPC", "Outdoor free flight", -0.16),
    Paper("Linsen (2022)", "MPC / NMPC", "Outdoor free flight", 0.16),
    Paper("Santos (2026)", "PID / cascade", "Indoor free flight"),
    Paper("Li (2024)", "MPC / NMPC", "Indoor free flight"),
    Paper("Torrente (2021)", "Residual-model learning", "Indoor free flight"),
    Paper("Osedo (2023)", "Direct RL policy", "Constrained hardware rig"),
)


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
    ax.set_ylim(-0.5, 2.5)
    ax.set_xticks(range(5), ["PID /\ncascade", "Nonlinear\nfeedback",
                             "MPC /\nNMPC", "Residual-model\nlearning",
                             "Direct\nRL policy"])
    ax.set_yticks(range(3), ["Constrained\nhardware rig",
                             "Indoor free\nflight", "Outdoor free\nflight"])
    ax.set_xlabel("Controller / policy architecture", labelpad=12, fontsize=11)
    ax.set_ylabel("Validation setting", labelpad=17, fontsize=11)
    ax.tick_params(axis="both", length=0, pad=8, labelsize=9)
    close_region = Ellipse((0.58, 1.55), width=3.5, height=1.8, angle=24,
                           facecolor=CLOSE_FILL, edgecolor="none", alpha=0.18,
                           zorder=-1)
    close_region.set_gid("region-close-tvc")
    ax.add_patch(close_region)
    other_region = Ellipse((2.65, 0.65), width=3.8, height=1.7, angle=-30,
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
    for position in (0.5, 1.5):
        ax.axhline(position, color="0.84", linewidth=0.8, zorder=0)
    for spine in ax.spines.values():
        spine.set_color("0.72")
        spine.set_linewidth(0.8)

    for paper in PAPERS:
        x = ARCHITECTURES.index(paper.architecture) - 0.39
        y = VALIDATION.index(paper.validation) + paper.y_offset
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
