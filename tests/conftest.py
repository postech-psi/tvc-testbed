"""
Shared test setup: import paths, and the vehicle every test measures against.
================================================================================
This replaces the ad-hoc `sys.path.insert` preludes that were duplicated across
sim/validate_control.py, sim/capture_golden.py, sim/hover.py, tvc_gui.py and
tools/gen_model_sdf.py. One definition, one place to fix.
"""
import os
import sys

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

for p in (os.path.join(REPO, "src", "tvc_control"), os.path.join(REPO, "sim")):
    if p not in sys.path:
        sys.path.insert(0, p)


@pytest.fixture(scope="session")
def repo():
    return REPO


@pytest.fixture(scope="session")
def vp():
    """The real vehicle, loaded from the YAML.

    Session-scoped because building the measured surface sweeps a 161x161 grid;
    that is startup work the flight code does once, and it should be once here
    too.
    """
    from tvc_control.config import load_vehicle_params
    return load_vehicle_params()


@pytest.fixture(scope="session")
def gains():
    from tvc_control.config import load_gains
    return load_gains()
