"""Output-contract checks for the professor-facing standalone figures."""

from pathlib import Path
import os
import re
import subprocess
import sys
import tempfile
import unittest
from xml.etree import ElementTree as ET

from pypdf import PdfReader


ROOT = Path(__file__).resolve().parent
class FocusedDeckTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        temporary = tempfile.TemporaryDirectory()
        cls.addClassCleanup(temporary.cleanup)
        cls.plots = Path(temporary.name) / "focused_plots"
        cls.deck = Path(temporary.name) / "deck.pdf"
        env = {**os.environ, "TVC_FIGURE_OUT": str(cls.plots),
               "TVC_DECK_OUT": str(cls.deck)}
        subprocess.run(["py", "-3.13", str(ROOT / "build_focused_ashby_plots.py")],
                       check=True, env=env)
        subprocess.run([sys.executable, str(ROOT / "build_ashby_deck.py")],
                       check=True, env=env)

    def test_control_maps_are_separate_standalone_figures(self):
        close = (self.plots / "02a_close_tvc_control.svg").read_text(encoding="utf-8")
        transfer = (self.plots / "02b_transfer_methods.svg").read_text(encoding="utf-8")
        for label in ("Denton (2022)", "Chen (2024)", "Santos (2026)",
                      "Spannagl (2021)", "Linsen (2022)", "Our Vehicle"):
            self.assertIn(label, close)
            self.assertNotIn(label, transfer)
        for label in ("Osedo (2023)", "Torrente (2021)", "Li (2024)"):
            self.assertIn(label, transfer)
            self.assertNotIn(label, close)
        self.assertNotIn("Wilson (2022)", transfer)
        self.assertNotIn("Residual-model", close)
        self.assertNotIn("Direct", close)
        self.assertIn("Residual-model", transfer)
        self.assertIn("Direct", transfer)
        for svg in (close, transfer):
            self.assertIn("Implemented controller / policy architecture", svg)
            self.assertIn("GNC function demonstrated", svg)
            self.assertNotIn("Learning-augmented", svg)
            self.assertNotIn("Neural-Fly", svg)
            self.assertNotIn("Liu (2022)", svg)
        self.assertFalse((self.plots / "02_control_reference.pdf").exists())

    def test_our_vehicle_is_blue_and_yearless_in_every_plot(self):
        for name in ("01_hardware_thrust.svg", "02a_close_tvc_control.svg", "03_validation.svg"):
            svg = (self.plots / name).read_text(encoding="utf-8")
            self.assertIn(">Our Vehicle</text>", svg, name)
            self.assertNotIn(">our vehicle</text>", svg, name)
            self.assertNotIn("POSTECH (2026)", svg, name)
            self.assertGreaterEqual(svg.count("#1768ac"), 2, name)
            for excess in ("fit at 1600 us", "stated capacity", "reported maximum",
                           "from ~2 T/W", "E-Rocket", "POSTECH current*"):
                self.assertNotIn(excess, svg, (name, excess))

    def test_close_system_grid_is_smaller_and_centered(self):
        root = ET.parse(self.plots / "02a_close_tvc_control.svg").getroot()
        ns = {"svg": "http://www.w3.org/2000/svg"}
        grid = root.find(".//svg:g[@id='axes_1']/svg:g[@id='patch_2']/svg:path", ns)
        self.assertIsNotNone(grid)
        coords = [float(value) for value in re.findall(r"-?\d+(?:\.\d+)?", grid.attrib["d"])]
        width = coords[2] - coords[0]
        height = coords[1] - coords[5]
        midpoint = (coords[0] + coords[2]) / 2
        self.assertLess(width, 400)
        self.assertLess(height, 275)
        self.assertAlmostEqual(midpoint, 360, delta=15)

    def test_hardware_labels_use_requested_sides_and_stay_close(self):
        root = ET.parse(self.plots / "01_hardware_thrust.svg").getroot()
        ns = {"svg": "http://www.w3.org/2000/svg"}
        names = ("Denton (2022)", "Chen (2024)", "Spannagl (2021)",
                 "Our Vehicle", "Linsen (2022)", "Santos (2026)")
        labels = {node.text: node for node in root.findall(".//svg:text", ns)
                  if node.text in names}
        markers = root.findall(".//svg:use", ns)[-len(names):]
        self.assertEqual(len(labels), len(names))
        for name, marker in zip(names, markers):
            label = labels[name]
            if name in ("Denton (2022)", "Chen (2024)"):
                horizontal_gap = float(label.attrib["x"]) - float(marker.attrib["x"])
                anchor = "text-anchor: start"
            else:
                horizontal_gap = float(marker.attrib["x"]) - float(label.attrib["x"])
                anchor = "text-anchor: end"
            vertical_gap = float(marker.attrib["y"]) - float(label.attrib["y"])
            self.assertGreater(horizontal_gap, 0, name)
            self.assertLessEqual(horizontal_gap, 10, name)
            self.assertGreater(vertical_gap, 0, name)
            self.assertLessEqual(vertical_gap, 10, name)
            self.assertIn(anchor, label.attrib["style"], name)

    def test_standalone_figures_have_no_slide_chrome(self):
        for stem in ("01_hardware_thrust", "02a_close_tvc_control",
                     "02b_transfer_methods", "03_validation"):
            svg = (self.plots / f"{stem}.svg").read_text(encoding="utf-8")
            for slide_text in ("01  Mass vs thrust property map",
                               "02  Control approach vs GNC function",
                               "03  Validation evidence map",
                               "POSTECH TVC  |", "Evidence details:",
                               "Empty cells are not novelty proof."):
                self.assertNotIn(slide_text, svg, (stem, slide_text))
            page = PdfReader(self.plots / f"{stem}.pdf").pages
            self.assertEqual(len(page), 1)
            self.assertLessEqual(float(page[0].mediabox.width), 750)

    def test_deck_has_four_pages(self):
        pages = PdfReader(self.deck).pages
        self.assertEqual(len(pages), 4)
        text = "\n".join(page.extract_text() for page in pages)
        self.assertIn("Osedo (2023)", text)
        self.assertIn("Spannagl (2021)", text)
        self.assertEqual(text.count("Our Vehicle"), 3)


if __name__ == "__main__":
    unittest.main()
