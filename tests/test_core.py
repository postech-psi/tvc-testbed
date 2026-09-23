"""Small checks for the pipeline's essentials; no scenario or baseline framework."""
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/tvc_control"))
from tvc_control.config import load_gains, load_vehicle_params
from tvc_control.control.allocation import allocate
from tvc_control.plotting import read_log, save_run
from tvc_control.simulation import SimConfig, simulate


class CoreChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.params = load_vehicle_params()
        cls.result = simulate(cls.params, load_gains(), SimConfig(t_final=0.2))

    def test_simulation_returns_finite_normalized_state(self):
        self.assertEqual(len(self.result["t"]), 20)
        self.assertTrue(np.isfinite(self.result["pos"]).all())
        np.testing.assert_allclose(np.linalg.norm(self.result["quat"], axis=1), 1.0, atol=1e-9)

    def test_large_request_stays_inside_actuator_limits(self):
        command = allocate((100.0, -100.0, 100.0), 1000.0, self.params)
        self.assertLessEqual(command.T_cmd, self.params.T_max)
        for k, angle in enumerate(command.delta_cmd):
            self.assertGreaterEqual(angle, self.params.delta_min[k])
            self.assertLessEqual(angle, self.params.delta_max[k])
        for motor in (command.u_a, command.u_b):
            self.assertGreaterEqual(motor, 0.0)
            self.assertLessEqual(motor, 1.0)

    def test_saved_run_can_be_read_for_plotting(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "run.csv"
            save_run(self.result, path)
            loaded = read_log(path)
        np.testing.assert_allclose(loaded["z_m"], self.result["pos"][:, 2])
        self.assertEqual(len(loaded["t_s"]), 20)
