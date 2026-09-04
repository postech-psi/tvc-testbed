"""
Shared test setup: the import path, and the vehicle every test measures against.
================================================================================
This replaces the ad-hoc `sys.path.insert` preludes that used to be duplicated
across five executable scripts. One definition, one place to fix.

The fixtures are session-scoped because building the measured thrust/torque
surface sweeps a 161x161 grid. That is startup work the flight code does once,
and it should be once here too -- otherwise the suite spends most of its runtime
rebuilding the same table.
"""
import os
import sys

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, "src", "tvc_control")
if SRC not in sys.path:
    sys.path.insert(0, SRC)


@pytest.fixture(scope="session")
def repo():
    """Absolute path to the repository root."""
    return REPO


@pytest.fixture(scope="session")
def vehicle():
    """The parsed YAML (config.Vehicle) -- has .raw for the solver constants."""
    from tvc_control.config import load
    return load()


@pytest.fixture(scope="session")
def vp():
    """The flight code's VehicleParams, built from the YAML."""
    from tvc_control.config import load_vehicle_params
    return load_vehicle_params()


@pytest.fixture(scope="session")
def gains():
    """The default gain profile from control_gains.yaml."""
    from tvc_control.config import load_gains
    return load_gains()
