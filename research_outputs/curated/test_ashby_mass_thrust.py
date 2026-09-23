"""Check the unit conversions behind the published mass–thrust chart."""

import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_mass_thrust_ashby import derive_thrust_n, load_plot_points


class AshbyMassThrustTests(unittest.TestCase):
    def test_kgf_conversion(self):
        self.assertAlmostEqual(derive_thrust_n(2.3, "kgf", 1.7), 22.563)

    def test_reported_ratio_conversion(self):
        self.assertAlmostEqual(derive_thrust_n(2.0, "thrust_weight_ratio", 0.366), 7.18092)

    def test_direct_newtons(self):
        self.assertEqual(derive_thrust_n(20, "N", 0.946), 20)

    def test_unsupported_unit_is_rejected(self):
        with self.assertRaises(ValueError):
            derive_thrust_n(20, "kg", 0.946)

    def test_only_sourced_complete_records_are_plotted(self):
        records = load_plot_points()
        self.assertEqual({point["id"] for point in records}, {"R02", "R05", "N01"})
        for point in records:
            self.assertGreater(point["mass_kg"], 0)
            self.assertGreater(point["thrust_n"], point["mass_kg"] * 9.81)
            self.assertTrue(point["source_url"].startswith("https://"))
            self.assertTrue(point["locator"])


if __name__ == "__main__":
    unittest.main()
