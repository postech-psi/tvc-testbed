"""
The frozen numerical baseline: what this simulator produced, written down.
================================================================================
    python tvc.py golden            # write reference/golden/analytic_baseline.json
    python tvc.py golden --check    # compare; exit 1 and name every drifted field

WHY A BASELINE EXISTS AT ALL
    Some changes are supposed to move no numbers -- a rename, a file move, a
    refactor that only relocates code. That claim is untestable without a record
    taken beforehand, and an untestable claim is exactly how a rename quietly
    becomes a physics change. This file is the record.

    It earned its keep once already: the axis rename (roll = thrust axis) was
    checked against a baseline captured before it, 137 numeric values under the
    key relabelling, zero differences.

WHAT IS FROZEN
    the vehicle constants ......... so a mismatch can be attributed. A changed
                                    metric with an unchanged vehicle block is a
                                    code change; both changing is a re-measured
                                    airframe, which is a legitimate reason.
    the authority budget .......... roll headroom and the signed feasible set
    the allocation round-trip ..... allocate() must realize the moment it was
                                    asked for; today to ~4e-13 N*m
    the five scenarios ............ pass/fail plus every metric
    the smoke test ................ the default SimConfig run

TWO DELIBERATE EXCEPTIONS TO EXACT COMPARISON
    Metrics are rounded to 9 decimals -- far tighter than any legitimate
    refactor-era change, loose enough to survive a different BLAS or numpy
    build. And the allocation round-trip stores a BOUND (< 1e-9 N*m), not the
    residual: the residual's last digits are platform-dependent, and the bound
    is the actual invariant. The measured value is still printed.

DETERMINISM
    Every scenario is a fixed-step zero-order-hold loop over solve_ivp with a
    fixed max_step and no randomness, so repeated runs agree to the last bit.
    If that stops being true, --check says so rather than drifting quietly.

NOT COVERED HERE: the Gazebo flight, which needs gz-transport. See
reference/golden/README.md for that gap and how to close it.

NEVER regenerate the golden to make a failing check pass.
"""
import argparse
import json
import os

import numpy as np

from . import scenarios as vc
from ..config import load_gains, load_vehicle_params
from ..gnc.allocation import allocate, roll_headroom, roll_limits
from ..gnc.mathx import thrust_axis
from ..harness.mil import SimConfig, simulate

# repo/src/tvc_control/tvc_control/verify -> repo
_REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     "..", "..", "..", ".."))
GOLDEN_DIR = os.path.join(_REPO, "reference", "golden")
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
    vp, gains = load_vehicle_params(), load_gains()
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
    q_lo, q_hi = roll_limits(T_hov, vp)
    out["authority_at_hover"] = _round({
        "T_hover_N": T_hov,
        "roll_headroom_Nm": roll_headroom(T_hov, vp),
        "roll_limit_lo_Nm": q_lo,
        "roll_limit_hi_Nm": q_hi,
        "roll_limits_vs_thrust": {
            "%.2f" % T: list(roll_limits(T, vp))
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
        if a.gimbal_saturated or a.roll_saturated:
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

    # --- the smoke test: the default SimConfig, unchanged since before the
    # --- restructure, so it is the longest-lived number in this file ---------
    r = simulate(vp, gains, SimConfig())
    out["smoke_test"] = _round(r["metrics"])

    return out


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="tvc.py golden",
        description="Capture or check the frozen numerical baseline.")
    ap.add_argument("--check", action="store_true",
                    help="compare against the stored golden; exit 1 on drift")
    args = ap.parse_args(argv)

    current = capture()

    if not args.check:
        os.makedirs(GOLDEN_DIR, exist_ok=True)
        with open(JSON_PATH, "w", encoding="utf-8", newline="\n") as f:
            json.dump(current, f, indent=2, sort_keys=True)
            f.write("\n")
        print("wrote %s" % os.path.relpath(JSON_PATH, _REPO))
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
        print("\nA refactor that claims to move no numbers moves none. If this")
        print("fired during one, the refactor is wrong -- not the baseline.")
        print("If it fired during a deliberate physics or parameter change,")
        print("re-run `python tvc.py golden` in its own commit and say which")
        print("numbers moved and why. See reference/golden/README.md.")
        return 1

    print("golden matches (allocation residual %s, bound %g)" % (cur_meas, bound))
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
