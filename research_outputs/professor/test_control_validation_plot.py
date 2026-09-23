"""Evidence and artifact checks for the combined control/validation figure."""

from pathlib import Path
import subprocess
import sys

from pypdf import PdfReader


ROOT = Path(__file__).resolve().parent
SCRIPT = ROOT / "build_control_validation_plot.py"


def test_combined_plot_preserves_each_papers_validation_setting(tmp_path):
    sys.path.insert(0, str(ROOT))
    try:
        from build_control_validation_plot import PAPERS
    finally:
        sys.path.pop(0)

    points = {paper.label: (paper.architecture, paper.validation) for paper in PAPERS}
    assert len(points) == 8  # Physical-system experimental papers only.
    assert "Wilson (2022)" not in points
    assert points["Osedo (2023)"] == ("Direct RL policy", "Constrained hardware rig")
    assert "Our Vehicle" not in points
    assert points["Spannagl (2021)"] == ("MPC / NMPC", "Outdoor free flight")
    assert points["Linsen (2022)"] == ("MPC / NMPC", "Outdoor free flight")
    assert points["Torrente (2021)"] == ("Residual-model learning", "Indoor free flight")


def test_combined_plot_exports_readable_standalone_formats(tmp_path):
    subprocess.run([sys.executable, str(SCRIPT), str(tmp_path)], check=True)
    stem = "02_control_validation_profile"
    svg = (tmp_path / f"{stem}.svg").read_text(encoding="utf-8")
    assert "Controller / policy architecture" in svg
    assert "Validation setting" in svg
    for name in ("Denton", "Chen", "Spannagl", "Linsen", "Santos",
                 "Li", "Torrente", "Osedo"):
        assert name in svg
    assert "Wilson" not in svg
    assert "Simulation only" not in svg
    assert "Our Vehicle" not in svg
    assert "#1768ac" not in svg
    assert "GNC function demonstrated" not in svg
    assert (tmp_path / f"{stem}.png").stat().st_size > 20_000
    assert len(PdfReader(tmp_path / f"{stem}.pdf").pages) == 1


def test_colored_evidence_regions_show_only_the_two_platform_groups(tmp_path):
    subprocess.run([sys.executable, str(SCRIPT), str(tmp_path)], check=True)
    svg = (tmp_path / "02_control_validation_profile.svg").read_text(encoding="utf-8")
    assert svg.count('id="region-close-tvc"') == 1
    assert svg.count('id="region-other-platforms"') == 1
    assert "Close TVC systems" in svg
    assert "Other platforms" in svg
