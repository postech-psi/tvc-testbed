"""
capture_golden.py -- freeze the current simulator's numbers as a reference.

WHY THIS EXISTS
---------------
The unification work ahead splits physics.py into flight code and plant, moves
gains, changes the Gazebo mapping, and finally renames every axis in the repo.
The last of those is supposed to be a PURE RENAME, and the only way to make that
claim checkable is to have the numbers written down BEFORE the work starts.

So: this script records what the analytic simulator produces today, exactly, in
a machine-readable form. Phase 6's acceptance criterion is that re-running it
after the rename reproduces this file bit-for-bit.

Determinism: every scenario is a fixed-step ZOH loop over solve_ivp with a fixed
max_step and no randomness, so repeated runs agree to the last bit. If that ever
stops being true, the comparison below will say so rather than drifting quietly.

Usage:
    python sim/capture_golden.py                    # write sim/golden/
    python sim/capture_golden.py --check            # compare, exit 1 on drift

NOT COVERED HERE: the Gazebo flight. `sim/hover.py` needs gz-transport, which
only exists in the devcontainer, so the hover golden is captured separately with
    bash sim/run_hover.sh --duration 30 --altitude 2.0 --log sim/golden/hover_baseline.csv
See sim/golden/README.md for what is and is not frozen.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "tvc_control"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

import validate_control as vc
from tvc_control.physics import (
    VehicleParams, ControlGains, SimConfig, simulate,
    axial_headroom, axial_limits, allocate, thrust_axis,
)

GOLDEN_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "golden")
JSON_PATH = os.path.join(GOLDEN_DIR, "analytic_baseline.json")

# Metrics are rounded before storage. Not to hide drift -- to keep the file
# stable against the last-bit noise of a different BLAS or numpy build, which
# would otherwise make every diff a false alarm. 9 decimals is far tighter than
# any change the upcoming work could legitimately make and still be a rename.
ROUND = 9


def _round(x):
    if isinstance(x, dict):
        return {k: _round(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_round(v) for v in x]
    if isinstance(x, (bool, np.bool_)):
        return bool(x)
    if isinstance(x, (int, np.integer)):
        return int(x)
    if isinstance(x, (float, np.floating)):
        return round(float(x), ROUND)
    return x


def capture():
    vp, gains = VehicleParams(), ControlGains()
    out = {}

    # --- the vehicle the numbers below describe ------------------------------
    # Recorded so a golden mismatch can be attributed: a changed metric with an
    # unchanged vehicle block is a code change; both changing is a re-measured
    # airframe, which is a legitimate reason for the numbers to move.
    T_hov = vp.m * vp.g
    out["vehicle"] = _round({
        "m": vp.m, "Ix": vp.Ix, "Iy": vp.Iy, "Iz": vp.Iz, "L": vp.L,
        "T_max": vp.T_max, "T_min": vp.T_min, "g": vp.g,
        "gimbal_max_deg": vp.gimbal_max_deg,
        "gimbal_deadtime_s": vp.gimbal_deadtime_s,
        "delta_min_deg": np.rad2deg(vp.delta_min).tolist(),
        "delta_max_deg": np.rad2deg(vp.delta_max).tolist(),
        "delta_rate_max_deg": np.rad2deg(vp.delta_rate_max).tolist(),
        "surface_present": vp.surface is not None,
    })

    # --- control authority budget -------------------------------------------
    q_lo, q_hi = axial_limits(T_hov, vp)
    out["authority_at_hover"] = _round({
        "T_hover_N": T_hov,
        "axial_headroom_Nm": axial_headroom(T_hov, vp),
        "axial_limit_lo_Nm": q_lo,
        "axial_limit_hi_Nm": q_hi,
        "axial_limits_vs_thrust": {
            "%.2f" % T: list(axial_limits(T, vp))
            for T in (8.0, 10.0, 12.0, 13.028, 15.0, 17.0)
        },
    })

    # --- allocation round-trip ----------------------------------------------
    # The single most important invariant to freeze: allocate() must realize the
    # moment it was asked for. Today that holds to ~1e-13 N*m; if a refactor
    # breaks it, everything downstream is quietly wrong.
    rng = np.random.default_rng(20260904)
    travel = float(np.min(np.abs(np.concatenate([vp.delta_min, vp.delta_max]))))
    lat = T_hov * vp.L * np.sin(travel)
    worst, n_ok = 0.0, 0
    for _ in range(2000):
        M = np.array([rng.uniform(-1, 1) * lat * 0.7,
                      rng.uniform(-1, 1) * lat * 0.7,
                      rng.uniform(q_lo, q_hi) * 0.95])
        a = allocate(M, T_hov, vp)
        if a.gimbal_saturated or a.axial_saturated:
            continue
        n_ok += 1
        # gnc returns plain tuples (flight code carries no numpy); the
        # harness is where they become arrays.
        nh = np.asarray(thrust_axis(a.delta_cmd), dtype=float)
        tau = np.cross([0, 0, -vp.L], T_hov * nh) + a.tau_p * nh
        worst = max(worst, float(np.max(np.abs(tau - M))))
    out["allocation_roundtrip"] = {
        "cases_unsaturated": n_ok,
        # An absolute bound, not the raw value: the exact last digits of a
        # 1e-13 residual are BLAS-dependent and would make this file flap.
        "worst_moment_error_Nm_under": 1e-9,
        "worst_moment_error_Nm_measured": float("%.3e" % worst),
    }

    # --- the four validation scenarios --------------------------------------
    out["scenarios"] = {}
    for name, fn in vc.SCENARIOS:
        ok, detail, r = fn(vp, gains, False)
        out["scenarios"][name] = {
            "pass": bool(ok),
            "detail": detail,
            "metrics": _round(r["metrics"]),
        }

    # --- the headless smoke test (physics.py __main__ defaults) --------------
    r = simulate(vp, gains, SimConfig())
    out["smoke_test"] = _round(r["metrics"])

    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true",
                    help="compare against the stored golden; exit 1 on drift")
    args = ap.parse_args()

    current = capture()

    if not args.check:
        os.makedirs(GOLDEN_DIR, exist_ok=True)
        with open(JSON_PATH, "w", encoding="utf-8", newline="\n") as f:
            json.dump(current, f, indent=2, sort_keys=True)
            f.write("\n")
        print("wrote %s" % os.path.relpath(JSON_PATH))
        for name, s in current["scenarios"].items():
            print("  [%s] %s" % ("PASS" if s["pass"] else "FAIL", name))
        return 0

    if not os.path.exists(JSON_PATH):
        print("no golden at %s -- run without --check first" % JSON_PATH)
        return 1
    with open(JSON_PATH, encoding="utf-8") as f:
        stored = json.load(f)

    drift = []

    def walk(a, b, path=""):
        if isinstance(a, dict) and isinstance(b, dict):
            for k in sorted(set(a) | set(b)):
                if k not in a:
                    drift.append("%s/%s: ADDED" % (path, k))
                elif k not in b:
                    drift.append("%s/%s: REMOVED" % (path, k))
                else:
                    walk(a[k], b[k], "%s/%s" % (path, k))
        elif isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
            for i, (x, y) in enumerate(zip(a, b)):
                walk(x, y, "%s[%d]" % (path, i))
        elif a != b:
            drift.append("%s: %r -> %r" % (path, a, b))

    # The measured residual is expected to wobble in its last digits; only its
    # bound is a golden. Compare the bound, report the value.
    stored_meas = stored.get("allocation_roundtrip", {}).pop(
        "worst_moment_error_Nm_measured", None)
    cur_meas = current["allocation_roundtrip"].pop(
        "worst_moment_error_Nm_measured", None)

    walk(stored, current)

    bound = current["allocation_roundtrip"]["worst_moment_error_Nm_under"]
    if cur_meas is not None and float(cur_meas) > bound:
        drift.append("/allocation_roundtrip: residual %s exceeds bound %g"
                     % (cur_meas, bound))

    if drift:
        print("GOLDEN DRIFT (%d):" % len(drift))
        for d in drift:
            print("  " + d)
        print("\nA pure rename moves NO numbers. If this fired during a rename,")
        print("the rename is wrong. If it fired during a physics change, update")
        print("the golden deliberately and say why in the commit message.")
        return 1

    print("golden matches (allocation residual %s, bound %g)" % (cur_meas, bound))
    return 0


if __name__ == "__main__":
    sys.exit(main())
