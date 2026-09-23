"""Build a sourced mass–maximum-static-thrust Ashby property chart."""

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from paper_labels import first_author_labels


HERE = Path(__file__).resolve().parent
G = 9.81


def derive_thrust_n(value: float, unit: str, mass_kg: float) -> float:
    """Normalize the force stated in a paper to newtons."""
    if value <= 0 or mass_kg <= 0:
        raise ValueError("Mass and reported thrust must be positive")
    if unit == "N":
        return value
    if unit == "kgf":
        return value * G
    if unit == "thrust_weight_ratio":
        return value * mass_kg * G
    raise ValueError(f"Unsupported thrust unit: {unit}")


def load_plot_points() -> list[dict]:
    """Load the manually source-audited records with normalized thrust."""
    dataset = json.loads((HERE / "mass_thrust_sources.json").read_text(encoding="utf-8"))
    if dataset["gravity_m_s2"] != G:
        raise ValueError("Dataset and builder use different gravity constants")
    points = []
    for record in dataset["records"]:
        point = dict(record)
        point["thrust_n"] = derive_thrust_n(
            record["reported_thrust"], record["reported_unit"], record["mass_kg"]
        )
        point["thrust_weight_ratio"] = point["thrust_n"] / (record["mass_kg"] * G)
        points.append(point)
    return points


def render_chart() -> None:
    points = load_plot_points()
    display_names = first_author_labels()
    plt.rcParams.update({"svg.fonttype": "none"})
    fig, ax = plt.subplots(figsize=(10.6, 7.0), dpi=180)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    x_min, x_max = 0.28, 2.4
    y_min, y_max = 3.0, 35.0
    mass_axis = np.geomspace(x_min, x_max, 300)
    for ratio in (1.0, 1.5, 2.0, 3.0):
        thrust_axis = ratio * G * mass_axis
        ax.plot(mass_axis, thrust_axis, linestyle="--", linewidth=0.9, color="0.66")
        label_x = min(x_max * 0.94, y_max * 0.82 / (ratio * G))
        ax.text(
            label_x,
            ratio * G * label_x,
            f"T/W = {ratio:g}",
            rotation=33,
            va="bottom",
            ha="left",
            fontsize=9,
            color="0.35",
        )

    for point in points:
        ax.plot(point["mass_kg"], point["thrust_n"], "o", color="black", markersize=7)
        y_offset = -19 if point["id"] == "R02" else 11
        ax.annotate(
            display_names[point["id"]],
            (point["mass_kg"], point["thrust_n"]),
            xytext=(0, y_offset),
            textcoords="offset points",
            ha="center",
            fontsize=10,
            color="black",
        )

    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_xticks([0.3, 0.5, 1, 1.5, 2], ["0.3", "0.5", "1", "1.5", "2"])
    ax.set_yticks([3, 5, 10, 20, 30], ["3", "5", "10", "20", "30"])
    ax.minorticks_off()
    ax.grid(color="0.86", linewidth=0.8)
    ax.set_axisbelow(True)
    ax.spines[["top", "right"]].set_visible(False)
    ax.set_xlabel("All-up vehicle mass (kg)", labelpad=10)
    ax.set_ylabel("Maximum static thrust (N)", labelpad=10)
    ax.set_title("Ashby property chart: mass vs maximum static thrust", pad=15, fontsize=14)
    fig.text(
        0.12,
        0.035,
        "Close two-axis coaxial TVC vehicles with published data only. Dashed lines are constant thrust/weight.\n"
        "Thrust values are approximate; Denton (2022) and Linsen (2022) use conversions. See the guide for sources.",
        fontsize=9,
        color="black",
    )
    fig.subplots_adjust(left=0.13, right=0.97, bottom=0.17, top=0.91)
    for suffix in ("png", "svg"):
        fig.savefig(HERE / f"tvc_mass_thrust_ashby.{suffix}", dpi=220, facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    render_chart()
