"""
The five closed-loop scenarios, run as tests.
================================================================================
The scenarios are DEFINED in tvc_control/verify/scenarios.py and imported here.
They are deliberately not re-written as tests: a scenario definition that exists
twice will eventually disagree with itself, and then neither copy is evidence of
anything.

`python tvc.py validate` remains the human-facing entry point -- it prints the
authority budget and a readable line per scenario. This file is the machine
gate, so CI fails on a regression without anyone having to read the output.

Each scenario is slow (a 6-12 s closed-loop run with a stiff-ish integrator), so
this is the expensive part of the suite. It is worth it: these are the only
tests that exercise the whole stack -- estimator seam, all four loops,
allocation, both actuator lag models and the rigid body -- against each other.
"""
import pytest

from tvc_control.verify import scenarios


@pytest.mark.parametrize("name,fn", scenarios.SCENARIOS,
                         ids=[n.replace(" ", "_") for n, _ in scenarios.SCENARIOS])
def test_scenario(name, fn, vp, gains):
    ok, detail, _ = fn(vp, gains, False)
    assert ok, "%s: %s" % (name, detail)


def test_the_suite_covers_all_three_axes_and_the_altitude_loop():
    """A guard against the suite silently shrinking.

    Each of these catches a specific structural failure that nothing else in the
    test suite would notice: a missing tau_P term leaves body z drifting
    forever, and a sign error in the altitude cascade only shows up in closed
    loop.
    """
    names = [n for n, _ in scenarios.SCENARIOS]
    for expected in ("lateral", "roll", "climb", "tilted", "motor lag"):
        assert any(expected in n for n in names), \
            "no scenario covering %r any more" % expected
