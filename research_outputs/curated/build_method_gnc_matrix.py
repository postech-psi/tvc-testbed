"""Render the sourced method-by-GNC comparison as a monochrome table."""

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from paper_labels import first_author_labels


HERE = Path(__file__).resolve().parent


def main():
    data = json.loads((HERE / "method_gnc_sources.json").read_text(encoding="utf-8"))
    corpus = json.loads((HERE / "curated_corpus.json").read_text(encoding="utf-8"))
    papers = data["papers"]
    display_names = first_author_labels()
    ids = [paper["id"] for paper in papers]
    expected = {paper["ID"] for paper in corpus["main"]}
    assert len(ids) == len(set(ids)) == 13 and set(ids) == expected
    assert all(len(paper["cells"]) == len(data["columns"]) for paper in papers)
    assert all(paper["url"].startswith("https://") and paper["locator"] for paper in papers)

    plt.rcParams.update({"font.family": "DejaVu Sans", "svg.fonttype": "none"})
    fig, ax = plt.subplots(figsize=(17.4, 11.8), dpi=180)
    fig.patch.set_facecolor("white")
    ax.set_axis_off()
    fig.text(0.035, 0.947, "Control method and its place in GNC", fontsize=22, weight="bold")
    fig.text(0.035, 0.909,
             "All 13 screened papers | Text-only comparison of implemented or described components", fontsize=12)

    labels = [f"{display_names[p['id']]}{'*' if p['provisional'] else ''}\n{p['label']}\n{p['scope'].capitalize()} architecture" for p in papers]
    cells = [[label] + p["cells"] for label, p in zip(labels, papers)]
    table = ax.table(
        cellText=cells,
        colLabels=["Paper / method"] + data["columns"],
        cellLoc="left",
        colLoc="left",
        colWidths=[0.258, 0.126, 0.15, 0.16, 0.15, 0.156],
        bbox=[0, 0, 1, 1],
    )
    table.auto_set_font_size(False)
    table.set_fontsize(10.4)
    for (row, col), cell in table.get_celld().items():
        cell.set_facecolor("white")
        cell.set_edgecolor("black")
        cell.set_linewidth(0.45)
        cell.PAD = 0.045
        if row == 0:
            cell.set_text_props(weight="bold")
            cell.set_linewidth(0.8)
        if row == 9:
            # The first comparator follows the eight close-architecture studies.
            cell.set_linewidth(0.9)

    fig.text(0.035, 0.069,
             "Cells show component roles, not novelty or a GNC completeness score. Allocation is part of control; modeling supports GNC.", fontsize=10)
    fig.text(0.035, 0.047,
             "? = insufficiently audited.  — = no separate guidance contribution established.  * = provisional source depth; see Markdown audit.", fontsize=10)
    fig.text(0.035, 0.025,
             "Given references are inputs to a controller, not evidence of a new planner. Sources: method_gnc_sources.json | 23 September 2026", fontsize=10)
    fig.subplots_adjust(left=0.035, right=0.975, bottom=0.11, top=0.875)
    for suffix in ("png", "svg"):
        fig.savefig(HERE / f"tvc_method_gnc_matrix.{suffix}", dpi=200, facecolor="white")
    plt.close(fig)
    print(f"Rendered matrix: {len(ids)} unique papers; all corpus IDs present.")


if __name__ == "__main__":
    main()
