"""
validate_control.py -- the four scenarios that exercise the three-axis
allocation, the axial (tau_P) channel, and the altitude loop.

Run:  python sim/validate_control.py [--verbose]

This is a REGRESSION HARNESS, not a tuning tool. Each scenario asserts a
threshold chosen to catch a specific structural mistake, and the thresholds are
loose enough that ordinary gain retuning does not trip them:

  lateral    a sign error in the gimbal allocation, or a gimbal limit so tight
             the vehicle cannot recover from a routine upset at all.
  axial      the tau_P*n_hat term missing from the moment sum, or the axial
             loop not closed -- both leave body z drifting forever.
  climb      altitude cascade sign / windup errors.
  tilted     the 1/cos(theta) feedforward missing. This one is an A/B: it runs
             the SAME scenario twice and asserts the compensated run sags less,
             which is a claim about the feedforward rather than about a gain
             set, so it stays meaningful after retuning.

Every number here comes from sim/vehicle_params.yaml, so re-running after a
mass-properties change re-validates against the new airframe.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "tvc_control"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

from tvc_control.physics import (
    VehicleParams, ControlGains, SimConfig, simulate, axial_headroom,
)


def _fmt(ok):
    return "PASS" if ok else "FAIL"


def scenario_lateral(vp, gains, verbose):
    """Lateral upset recovery: released at 8 deg on both lateral axes, hold 0."""
    cfg = SimConfig(t_final=6.0,
                    roll_des_deg=0.0, pitch_des_deg=0.0,
                    init_roll_deg=8.0, init_pitch_deg=-8.0)
    r = simulate(vp, gains, cfg)
    m = r["metrics"]
    ok = abs(m["final_roll_deg"]) < 1.0 and abs(m["final_pitch_deg"]) < 1.0
    detail = ("final lateral = (%+.3f, %+.3f) deg, peak gimbal %.2f/%.2f deg "
              "of %.1f, saturated %.0f%% of the run"
              % (m["final_roll_deg"], m["final_pitch_deg"],
                 m["max_delta1_deg"], m["max_delta2_deg"],
                 vp.gimbal_max_deg, 100 * m["gimbal_sat_frac"]))
    return ok, detail, r


def scenario_axial(vp, gains, verbose):
    """Axial (thrust-axis) upset: released at 20 deg about body z, hold 0.

    This is the scenario that did not exist before tau_P became a control
    input -- with the gimbal alone, body z is uncontrollable and this test
    can only fail.
    """
    cfg = SimConfig(t_final=6.0,
                    roll_des_deg=0.0, pitch_des_deg=0.0,
                    init_roll_deg=0.0, init_pitch_deg=0.0,
                    init_axial_deg=20.0)
    r = simulate(vp, gains, cfg)
    m = r["metrics"]
    ok = abs(m["final_axial_deg"]) < 2.0
    cap = axial_headroom(vp.m * vp.g, vp)
    detail = ("final axial = %+.3f deg, peak |tau_P| = %.4f N.m of %.4f "
              "available at hover (%.0f%%), capped %.0f%% of the run"
              % (m["final_axial_deg"], m["max_tau_p_Nm"], cap,
                 100 * m["max_tau_p_Nm"] / cap if cap > 0 else 0,
                 100 * m["axial_sat_frac"]))
    return ok, detail, r


def scenario_climb(vp, gains, verbose):
    """Climb to 2 m and hold, starting from the ground, attitude level."""
    cfg = SimConfig(t_final=12.0,
                    roll_des_deg=0.0, pitch_des_deg=0.0,
                    init_roll_deg=0.0, init_pitch_deg=0.0,
                    altitude_hold=True, z_des=2.0, init_z=0.0)
    r = simulate(vp, gains, cfg)
    m = r["metrics"]
    ok = abs(m["altitude_error_m"]) < 0.10
    detail = ("final z = %.3f m (err %+.3f), thrust %.2f-%.2f N, "
              "saturated %.0f%% of the run"
              % (m["final_z_m"], m["altitude_error_m"],
                 r["thrust_N"].min(), r["thrust_N"].max(),
                 100 * m["thrust_sat_frac"]))
    return ok, detail, r


def scenario_tilted_hover(vp, gains, verbose):
    """Hold 2 m while commanded to a 6 deg lateral tilt, with and without the
    1/cos(theta) feedforward. The compensated run must sag less."""
    base = dict(t_final=12.0, roll_des_deg=0.0, pitch_des_deg=6.0,
                init_roll_deg=0.0, init_pitch_deg=0.0,
                altitude_hold=True, z_des=2.0, init_z=2.0)
    on = simulate(vp, gains, SimConfig(tilt_compensation=True, **base))
    off = simulate(vp, gains, SimConfig(tilt_compensation=False, **base))
    sag_on = on["metrics"]["max_alt_sag_m"]
    sag_off = off["metrics"]["max_alt_sag_m"]
    ok = sag_on <= sag_off
    detail = ("peak altitude sag %.4f m with compensation vs %.4f m without "
              "(%.0f%% less); final tilt %+.2f deg"
              % (sag_on, sag_off,
                 100 * (1 - sag_on / sag_off) if sag_off > 0 else 0.0,
                 on["metrics"]["final_pitch_deg"]))
    return ok, detail, on


SCENARIOS = [
    ("lateral upset recovery", scenario_lateral),
    ("axial upset recovery", scenario_axial),
    ("climb and hold", scenario_climb),
    ("tilted hover (1/cos compensation)", scenario_tilted_hover),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--verbose", action="store_true",
                    help="also print the vehicle's control-authority budget")
    args = ap.parse_args()

    vp = VehicleParams()
    gains = ControlGains()
    T_hov = vp.m * vp.g

    if args.verbose:
        cap = axial_headroom(T_hov, vp)
        # Worst-case travel, not the nominal +/-7: the stops are asymmetric and
        # differ per axis, and the binding one is what sets guaranteed authority.
        travel = float(np.min(np.abs(np.concatenate([vp.delta_min, vp.delta_max]))))
        lat = T_hov * vp.L * np.sin(travel)
        t_max = vp.T_max
        print("control authority at hover (T = %.2f N, %.0f%% of the %.2f N ceiling):"
              % (T_hov, 100 * T_hov / t_max, t_max))
        print("  lateral  %.4f N.m -> %6.2f rad/s^2   gimbal %+.2f..%+.2f / %+.2f..%+.2f deg, L = %.4f m"
              % (lat, lat / vp.Ix,
                 np.degrees(vp.delta_min[0]), np.degrees(vp.delta_max[0]),
                 np.degrees(vp.delta_min[1]), np.degrees(vp.delta_max[1]), vp.L))
        print("  axial    %.4f N.m -> %6.2f rad/s^2   Iz is %.1fx smaller than Ix"
              % (cap, cap / vp.Iz, vp.Ix / vp.Iz))
        print("  cross-coupling tau_P/(T*L) = %.1f%%, allocation rotated %.2f deg "
              "(%.0f%% of the tighter axis's travel)"
              % (100 * cap / (T_hov * vp.L),
                 np.degrees(np.arctan(cap / (T_hov * vp.L))),
                 100 * np.arctan(cap / (T_hov * vp.L)) / travel))
        if vp.surface is not None:
            print("  axial headroom vs thrust (measured surface):")
            for T in (10.0, 11.5, T_hov, 15.0, 17.0):
                print("    T = %5.2f N (%3.0f%%)  ->  |tau_P| <= %.4f N.m"
                      % (T, 100 * T / t_max, axial_headroom(T, vp)))
            print("  NOTE authority is not the whole story: the axial channel has")
            print("       %.1fx the angular acceleration of the lateral pair but its"
                  % ((cap / vp.Iz) / (lat / vp.Ix)))
            print("       actuator is ~3x slower (motor ~100 ms vs gimbal ~30 ms).")
        print()

    failures = 0
    for name, fn in SCENARIOS:
        ok, detail, _ = fn(vp, gains, args.verbose)
        failures += not ok
        print("[%s] %-36s %s" % (_fmt(ok), name, detail))

    print()
    print("%d/%d scenarios passed" % (len(SCENARIOS) - failures, len(SCENARIOS)))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
