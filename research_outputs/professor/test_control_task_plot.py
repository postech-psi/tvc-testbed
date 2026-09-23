"""Checks for the controller-versus-demonstrated-task figure."""

from pathlib import Path
import subprocess
import sys

from pypdf import PdfReader


ROOT = Path(__file__).resolve().parent
SCRIPT = ROOT / "build_control_task_plot.py"
STEM = "04_control_task_profile"


def test_only_real_world_papers_are_placed_by_demonstrated_task():
    sys.path.insert(0, str(ROOT))
    try:
        from build_control_task_plot import TASKS
    finally:
        sys.path.pop(0)

    assert TASKS == {
        "Denton (2022)": "Ascent / hover",
        "Chen (2024)": "Ascent / hover",
        "Santos (2026)": "Trajectory tracking",
        "Li (2024)": "Trajectory tracking",
        "Torrente (2021)": "Trajectory tracking",
        "Osedo (2023)": "Attitude regulation",
        "Spannagl (2021)": "Landing / diversion",
        "Linsen (2022)": "Landing / diversion",
    }


def test_task_figure_exports_both_elliptical_groups_without_wilson(tmp_path):
    subprocess.run([sys.executable, str(SCRIPT), str(tmp_path)], check=True)
    svg = (tmp_path / f"{STEM}.svg").read_text(encoding="utf-8")
    assert "Controller / policy architecture" in svg
    assert "Demonstrated task" in svg
    for term in ("Attitude", "regulation", "Ascent /", "hover",
                 "Trajectory", "tracking", "Landing /", "diversion"):
        assert term in svg
    for name in ("Denton", "Chen", "Santos", "Li", "Torrente",
                 "Osedo", "Spannagl", "Linsen"):
        assert name in svg
    assert "Wilson" not in svg
    assert "Our Vehicle" not in svg
    assert svg.count('id="region-close-tvc"') == 1
    assert svg.count('id="region-other-platforms"') == 1
    assert (tmp_path / f"{STEM}.png").stat().st_size > 20_000
    assert len(PdfReader(tmp_path / f"{STEM}.pdf").pages) == 1
