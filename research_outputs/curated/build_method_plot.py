"""Render the 13-study, monochrome control-method evidence plot."""

from pathlib import Path
import json

import matplotlib.pyplot as plt

from paper_labels import first_author_labels


HERE = Path(__file__).resolve().parent
CORPUS = json.loads((HERE / "curated_corpus.json").read_text(encoding="utf-8"))

# The 13 peer-reviewed primary studies in the screened corpus. Architecture
# distance is documented in the reader, not encoded using marker color/shape.
# The identification papers have their own explicit x-axis category.
POINTS = {
    "R06": (0, 3, 0.0),
    "N03": (0, 1, 0.0),
    "R05": (1, 4, -0.08),
    "N01": (1, 4, 0.08),
    "R01": (2, 4, 0.0),
    "R02": (3, 4, 0.0),
    "R10": (3, 3, 0.0),
    "R04": (4, 1, -0.08),
    "N04": (4, 1, 0.08),
    "N02": (4, 3, 0.0),
    "R20": (5, 0, 0.0),
    "R09": (6, 2, 0.0),
    "R13": (6, 3, 0.0),
}

available = {paper["ID"] for paper in CORPUS["main"]}
assert set(POINTS) == available
display_names = first_author_labels()

labels = [
    "System\nidentification",
    "Cascaded\nfeedback / PID",
    "MPC + PID\nloops",
    "Integrated\noptimal / MPC",
    "Robust / nonlinear\ncontrol + allocation",
    "RL-assisted\nrobust control",
    "Direct RL\npolicy",
    "Residual RL",
    "Learned-model\nadaptive control",
]

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "svg.fonttype": "none"})
fig, ax = plt.subplots(figsize=(16.6, 8.0), dpi=180)
fig.patch.set_facecolor("white")
ax.set_facecolor("white")

ax.set_xlim(-0.55, len(labels) - 0.45)
ax.set_ylim(-0.45, 4.5)
ax.set_xticks(range(len(labels)), labels)
ax.set_yticks([0, 1, 2, 3, 4], [
    "Hardware reported;\nsetup unverified*",
    "Simulation only",
    "Constrained rig",
    "Indoor free flight",
    "Outdoor free flight",
])
ax.grid(axis="both", color="0.83", linewidth=0.8)
ax.set_axisbelow(True)
ax.axhline(0.5, color="black", linewidth=1.0, linestyle="--")

for paper_id, (x, y, offset) in POINTS.items():
    ax.plot(x + offset, y, "o", color="black", markersize=6.5)
    ax.annotate(
        display_names[paper_id].replace(" (", "\n("),
        (x + offset, y),
        xytext=(0, -24 if paper_id in {"N01", "N04"} else 21),
        textcoords="offset points",
        ha="center",
        va="center",
        color="black",
        fontsize=9,
    )

ax.set_title("Control method and reported validation: all 13 screened studies", pad=17, fontsize=15)
ax.set_xlabel("Principal contribution / control method", labelpad=13)
ax.set_ylabel("Reported validation category", labelpad=13)
for spine in ("top", "right"):
    ax.spines[spine].set_visible(False)
fig.text(
    0.10,
    0.015,
    "One black dot per paper, labeled by first author and publication year. Small offsets separate shared cells.\n"
    "*The bottom row is unclassified, not an evidence rank. Xie (2023) needs its experimental setup verified.\n"
    "Empty columns describe this screened set only. Scope and source-depth qualifications are in the complete guide.",
    fontsize=9,
    color="black",
)
fig.subplots_adjust(left=0.16, right=0.985, bottom=0.25, top=0.89)

for suffix in ("png", "svg"):
    fig.savefig(HERE / f"tvc_control_method_map.{suffix}", dpi=220, facecolor="white")
plt.close(fig)
