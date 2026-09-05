"""
The five closed-loop scenarios. VERIFICATION, not validation.
================================================================================
    python tvc.py validate [--verbose]

This is a REGRESSION HARNESS, not a tuning tool. Each scenario asserts one
threshold chosen to catch one specific STRUCTURAL mistake, and every threshold
is loose enough that ordinary gain retuning does not trip it. A test that fails
when someone changes a gain teaches people to ignore it.

    lateral     a sign error in the gimbal allocation, or a gimbal limit so
                tight the vehicle cannot recover from a routine upset at all.
    roll        the tau_P*n_hat term missing from the moment sum, or the roll
                loop not closed -- both leave body z drifting forever.
    climb       altitude cascade sign or windup errors.
    tilted      the 1/cos(theta) feedforward missing. An A/B: the SAME scenario
                twice, asserting the compensated run sags less. That is a claim
                about the feedforward rather than about a gain set, so it stays
                meaningful after retuning.
    motor lag   the open question about the measured 100 ms motor response,
                printed every run so it cannot quietly stop being mentioned.

Note the name. `validate` in the command line is the historical spelling; what
happens here is verification -- internal consistency, never agreement with the
real vehicle. Nothing in this project has been compared against flight data.
See docs/7-CREDIBILITY.md.

Every number comes from vehicle_params.yaml, so re-running after a
mass-properties change re-verifies against the new airframe.
"""
import argparse

import numpy as np

from ..config import load_gains, load_vehicle_params
from ..gnc.allocation import roll_headroom
from ..harness.mil import SimConfig, simulate


def _fmt(ok):
    return "PASS" if ok else "FAIL"


def scenario_lateral(vp, gains, verbose):
    """Lateral upset recovery: released at 8 deg on both lateral axes, hold 0."""
    cfg = SimConfig(t_final=6.0,
                    att_pitch_des_deg=0.0, att_yaw_des_deg=0.0,
                    init_att_pitch_deg=8.0, init_att_yaw_deg=-8.0)
    r = simulate(vp, gains, cfg)
    m = r["metrics"]
    ok = abs(m["final_pitch_deg"]) < 1.0 and abs(m["final_yaw_deg"]) < 1.0
    detail = ("final lateral = (%+.3f, %+.3f) deg, peak gimbal %.2f/%.2f deg "
              "of %.1f, saturated %.0f%% of the run"
              % (m["final_pitch_deg"], m["final_yaw_deg"],
                 m["max_gimbal_inner_deg"], m["max_gimbal_outer_deg"],
                 vp.gimbal_max_deg, 100 * m["gimbal_sat_frac"]))
    return ok, detail, r


def scenario_roll(vp, gains, verbose):
    """Roll (thrust-axis) upset: released at 20 deg about body z, hold 0.

    This is the scenario that did not exist before tau_P became a control
    input -- with the gimbal alone, body z is uncontrollable and this test
    can only fail.
    """
    cfg = SimConfig(t_final=6.0,
                    att_pitch_des_deg=0.0, att_yaw_des_deg=0.0,
                    init_att_pitch_deg=0.0, init_att_yaw_deg=0.0,
                    init_att_roll_deg=20.0)
    r = simulate(vp, gains, cfg)
    m = r["metrics"]
    ok = abs(m["final_roll_deg"]) < 2.0
    cap = roll_headroom(vp.m * vp.g, vp)
    detail = ("final roll = %+.3f deg, peak |tau_P| = %.4f N.m of %.4f "
              "available at hover (%.0f%%), capped %.0f%% of the run"
              % (m["final_roll_deg"], m["max_tau_p_Nm"], cap,
                 100 * m["max_tau_p_Nm"] / cap if cap > 0 else 0,
                 100 * m["roll_sat_frac"]))
    return ok, detail, r


def scenario_climb(vp, gains, verbose):
    """Climb to 2 m and hold, starting from the ground, attitude level."""
    cfg = SimConfig(t_final=12.0,
                    att_pitch_des_deg=0.0, att_yaw_des_deg=0.0,
                    init_att_pitch_deg=0.0, init_att_yaw_deg=0.0,
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
    base = dict(t_final=12.0, att_pitch_des_deg=0.0, att_yaw_des_deg=6.0,
                init_att_pitch_deg=0.0, init_att_yaw_deg=0.0,
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
                 on["metrics"]["final_yaw_deg"]))
    return ok, detail, on


def scenario_motor_lag_model(vp, gains, verbose):
    """The roll channel under both readings of the measured 100 ms motor lag.

    THE OPEN QUESTION THIS GUARDS. The bench recorded "command -> thrust
    response delay ~0.10 s" and did not say whether that is a transport delay or
    a first-order time constant. It decides whether the thrust-axis channel is
    controllable at the flight-validated gains:

        as a LAG    the roll channel recovers cleanly and never saturates
        as a DELAY  it winds up to ~70 deg and saturates ~94% of the run

    The pitch/yaw pair is unaffected either way -- this is entirely a roll-channel
    result, which follows from Izz being 11.6x smaller than Ixx while carrying
    the stiffest normalized gain.

    The check asserts only the optimistic reading (the current default), because
    asserting the pessimistic one would make the suite red over a measurement
    nobody has taken yet. What it does instead is print the pessimistic number
    every run, so the risk cannot quietly stop being mentioned.
    """
    base = dict(t_final=6.0, att_pitch_des_deg=0.0, att_yaw_des_deg=0.0,
                init_att_pitch_deg=0.0, init_att_yaw_deg=0.0, init_att_roll_deg=20.0)
    lag = simulate(vp, gains, SimConfig(motor_model="first_order",
                                        motor_tau_s=0.10, motor_deadtime_s=0.0,
                                        **base))
    dly = simulate(vp, gains, SimConfig(motor_model="delay",
                                        motor_tau_s=0.0, motor_deadtime_s=0.10,
                                        **base))
    lag_final = abs(lag["metrics"]["final_roll_deg"])
    dly_peak = float(np.max(np.abs(dly["euler_deg"][:, 2])))
    ok = lag_final < 2.0
    detail = ("as a LAG: final roll %.3f deg (stable). AS A DELAY: peak roll "
              "%.1f deg, %.0f%% saturated -- UNRESOLVED, needs one bench run"
              % (lag_final, dly_peak,
                 100 * float(np.mean(dly["saturated"][:, 1]))))
    return ok, detail, lag


SCENARIOS = [
    ("lateral upset recovery", scenario_lateral),
    ("roll upset recovery", scenario_roll),
    ("climb and hold", scenario_climb),
    ("tilted hover (1/cos compensation)", scenario_tilted_hover),
    ("motor lag model (roll channel)", scenario_motor_lag_model),
]


def main(argv=None):
    """Run all five scenarios. Returns 1 if any failed, which is the CI gate."""
    ap = argparse.ArgumentParser(
        prog="tvc.py validate",
        description="Run the five closed-loop verification scenarios.")
    ap.add_argument("--verbose", action="store_true",
                    help="also print the vehicle's control-authority budget")
    ap.add_argument("--gains", default=None,
                    help="gain profile from control_gains.yaml "
                         "(default: the file's own default_profile)")
    ap.add_argument("--plot", metavar="DIR", default=None,
                    help="also write one four-panel PNG per scenario into DIR "
                         "(needs matplotlib)")
    args = ap.parse_args(argv)

    vp = load_vehicle_params()
    gains = load_gains(args.gains)
    T_hov = vp.m * vp.g

    if args.verbose:
        cap = roll_headroom(T_hov, vp)
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
        print("  roll     %.4f N.m -> %6.2f rad/s^2   Izz is %.1fx smaller than Ixx"
              % (cap, cap / vp.Iz, vp.Ix / vp.Iz))
        print("  cross-coupling tau_P/(T*L) = %.1f%%, allocation rotated %.2f deg "
              "(%.0f%% of the tighter axis's travel)"
              % (100 * cap / (T_hov * vp.L),
                 np.degrees(np.arctan(cap / (T_hov * vp.L))),
                 100 * np.arctan(cap / (T_hov * vp.L)) / travel))
        if vp.surface is not None:
            print("  roll headroom vs thrust (measured surface):")
            for T in (10.0, 11.5, T_hov, 15.0, 17.0):
                print("    T = %5.2f N (%3.0f%%)  ->  |tau_P| <= %.4f N.m"
                      % (T, 100 * T / t_max, roll_headroom(T, vp)))
            print("  NOTE authority is not the whole story: the roll channel has")
            print("       %.1fx the angular acceleration of the lateral pair but its"
                  % ((cap / vp.Iz) / (lat / vp.Ix)))
            print("       actuator is ~3x slower (motor ~100 ms vs gimbal ~30 ms).")
        print()

    # Imported here, not at module scope: this command is the fast inner loop
    # and the CI gate, and it must keep running on a machine with no matplotlib.
    if args.plot:
        import os
        from ..apps.plot import four_panel, series_from_run
        os.makedirs(args.plot, exist_ok=True)

    failures = 0
    for name, fn in SCENARIOS:
        ok, detail, r = fn(vp, gains, args.verbose)
        failures += not ok
        print("[%s] %-36s %s" % (_fmt(ok), name, detail))
        if args.plot:
            slug = name.split(" (")[0].replace(" ", "_")
            four_panel(series_from_run(r),
                       os.path.join(args.plot, "%s.png" % slug),
                       "Analytic plant - %s" % name,
                       "same flight code as the Gazebo run; "
                       "solve_ivp instead of gz-sim",
                       # The altitude this run was supposed to hold: z_des when
                       # the altitude loop is closed, the release height when it
                       # is not. metrics carries both ends of that subtraction,
                       # so the line is the scenario's own reference and not a
                       # guess read back off the curve.
                       target_m=(r["metrics"]["final_z_m"]
                                 - r["metrics"]["altitude_error_m"]))

    print()
    print("%d/%d scenarios passed" % (len(SCENARIOS) - failures, len(SCENARIOS)))
    return 1 if failures else 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
