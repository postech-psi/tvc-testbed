"""
Cross-plant validation: does the analytic plant agree with Gazebo?
================================================================================
    python tvc.py cross-plant            # analytic hover vs the frozen Gazebo golden

Two independent plants share ONE controller. Making them agree on a matched
scenario is the cheapest evidence short of flight data (docs/7-CREDIBILITY.md:
Verification), and disagreement is exactly what cross-plant validation exists to
surface -- e.g. the ROS thrust-axis limit cycle.

WHAT THIS MODULE DOES NOW
    - defines a ScenarioSpec both plants can run from the same initial state;
    - runs it on the analytic plant and reduces the trace to PLANT-AGNOSTIC
      metrics (tilt, drift, thrust, gimbal) named to match the Gazebo golden;
    - compares two metric dicts under VERSIONED cross-plant tolerances, which are
      deliberately looser than Gazebo's self-reproducibility tolerances because
      they must absorb the known analytic<->Gazebo differences (no gyroscopic or
      contact terms in the analytic plant, different servo settling) without
      hiding a sign, frame, or allocation error.

    A frozen Gazebo result already exists (reference/golden/hover_baseline.json),
    so the analytic-vs-Gazebo hover comparison runs today with no live sim.

LIVE MULTI-SCENARIO REPLAY (run_gazebo) needs gz-sim, which lives in the
devcontainer; it is stubbed here and wired in that environment.
"""
import json
import os
from dataclasses import dataclass

import numpy as np

from ..harness.mil import SimConfig, simulate

_GOLDEN = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "..", "reference", "golden", "hover_baseline.json"))

# Versioned analytic<->Gazebo tolerances. LOOSER than the Gazebo self-tolerances
# in hover_baseline.json: they must cover the modelling differences between the
# plants (no gyroscopic/contact terms analytically, ~2x faster Gazebo servo)
# while still being tight enough to catch a real error. Ratify these from runs;
# a change here is a change to what "the plants agree" means.
CROSS_PLANT_TOLERANCES = {
    "final_altitude_m": 0.05,
    "final_tilt_deg": 0.5,
    "final_roll_deg": 1.0,
    "peak_tilt_deg": 1.5,
    "tilt_settling_time_s": 0.5,
    "max_thrust_N": 1.5,
    "min_thrust_N": 1.5,
    "peak_gimbal_inner_deg": 1.5,
    "peak_gimbal_outer_deg": 1.5,
    "peak_drift_m": 0.15,
}


@dataclass
class ScenarioSpec:
    """An initial state + setpoints both plants can run identically."""
    name: str
    init_pitch_deg: float = 0.0
    init_yaw_deg: float = 0.0
    init_roll_deg: float = 0.0
    t_final: float = 30.0
    altitude_hold: bool = True
    z_des: float = 2.0
    init_z: float = 2.0
    position_hold: bool = True
    tilt_compensation: bool = True

    def to_simconfig(self):
        return SimConfig(
            t_final=self.t_final, dt_ctrl=0.01,
            att_pitch_des_deg=0.0, att_yaw_des_deg=0.0, att_roll_des_deg=0.0,
            init_att_pitch_deg=self.init_pitch_deg,
            init_att_yaw_deg=self.init_yaw_deg,
            init_att_roll_deg=self.init_roll_deg,
            altitude_hold=self.altitude_hold, z_des=self.z_des, init_z=self.init_z,
            position_hold=self.position_hold, tilt_compensation=self.tilt_compensation)


# The matched hover: the Gazebo world's deliberate 10/-7 deg spawn, held at 2 m.
HOVER = ScenarioSpec(name="hover", init_pitch_deg=10.0, init_yaw_deg=-7.0,
                     t_final=30.0, z_des=2.0, init_z=2.0)


@dataclass
class Discrepancy:
    metric: str
    a: float
    b: float
    diff: float
    tol: float
    within: bool


def _tilt_deg(euler_deg):
    """Total tilt from vertical: the two lateral (pitch-x, yaw-y) angles combined.
    Roll about the thrust axis does not tilt the vehicle, so it is excluded."""
    return np.sqrt(euler_deg[:, 0] ** 2 + euler_deg[:, 1] ** 2)


def run_analytic(spec, vp, gains):
    """Run one ScenarioSpec on the analytic plant; return plant-agnostic metrics
    named to match the Gazebo golden."""
    r = simulate(vp, gains, spec.to_simconfig())
    t = r["t"]
    tilt = _tilt_deg(r["euler_deg"])
    drift = np.sqrt(r["pos"][:, 0] ** 2 + r["pos"][:, 1] ** 2)

    # tilt settling: last time tilt leaves a 2% band of the initial tilt.
    init_tilt = float(np.hypot(spec.init_pitch_deg, spec.init_yaw_deg))
    tol = max(0.02 * init_tilt, 0.05)
    outside = np.where(tilt > tol)[0]
    settle = float(t[outside[-1]]) if len(outside) else 0.0

    return {
        "final_altitude_m": float(r["pos"][-1, 2]),
        "final_tilt_deg": float(tilt[-1]),
        "final_roll_deg": float(r["euler_deg"][-1, 2]),
        "peak_tilt_deg": float(np.max(tilt)),
        "tilt_settling_time_s": settle,
        "max_thrust_N": float(np.max(r["thrust_N"])),
        "min_thrust_N": float(np.min(r["thrust_N"])),
        "peak_gimbal_inner_deg": float(np.max(np.abs(r["delta_deg"][:, 0]))),
        "peak_gimbal_outer_deg": float(np.max(np.abs(r["delta_deg"][:, 1]))),
        "peak_drift_m": float(np.max(drift)),
    }


def compare(metrics_a, metrics_b, tolerances):
    """One Discrepancy per shared metric that has a tolerance. within=|a-b|<=tol."""
    out = []
    for k, tol in tolerances.items():
        if k in metrics_a and k in metrics_b:
            diff = abs(metrics_a[k] - metrics_b[k])
            out.append(Discrepancy(k, metrics_a[k], metrics_b[k], diff, tol,
                                   diff <= tol))
    return out


def run_gazebo(spec):
    """Live Gazebo replay. Needs gz-sim from the devcontainer; wired there."""
    raise NotImplementedError(
        "live Gazebo replay runs in the devcontainer; use load_gazebo_golden() "
        "for the frozen hover comparison outside it")


def load_gazebo_golden(path=None):
    """The frozen Gazebo hover metrics (reference/golden/hover_baseline.json)."""
    with open(path or _GOLDEN, encoding="utf-8") as f:
        return json.load(f)["metrics"]


def main(argv=None):
    """Compare the analytic hover against the frozen Gazebo golden."""
    from ..config import load_gains, load_vehicle_params
    vp, gains = load_vehicle_params(), load_gains()
    a = run_analytic(HOVER, vp, gains)
    g = load_gazebo_golden()
    diffs = compare(a, g, CROSS_PLANT_TOLERANCES)

    print("cross-plant: analytic hover vs frozen Gazebo golden (10/-7 spawn, 2 m)")
    print("  %-24s %10s %10s %8s %6s  %s"
          % ("metric", "analytic", "gazebo", "|diff|", "tol", ""))
    n_fail = 0
    for d in diffs:
        n_fail += not d.within
        print("  %-24s %10.4f %10.4f %8.4f %6.3f  %s"
              % (d.metric, d.a, d.b, d.diff, d.tol,
                 "OK" if d.within else "**OUT**"))
    print("\n%d/%d metrics within cross-plant tolerance"
          % (len(diffs) - n_fail, len(diffs)))
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
