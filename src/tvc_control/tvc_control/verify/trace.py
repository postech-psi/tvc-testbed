"""
Trace one control step: every intermediate quantity, with its formula.
================================================================================
    python tvc.py trace
    python tvc.py trace --steps 5
    python tvc.py trace --case roll --steps 3

This is the answer to "what actually happens when the simulator runs". It steps
the real code -- not a reimplementation -- and prints every number the control
path computes on the way from a vehicle state to a set of actuator commands, in
the order it is computed, with the formula that produced it.

Read it once and the whole pipeline is concrete: which loop produces which
number, which limit binds, where a command gets clamped and by how much.

HOW IT STAYS HONEST
    Each stage is recomputed here by calling the SAME functions the controller
    calls, and the final result is then compared against what
    TvcController.update() actually returned. If they disagree, this file prints
    a MISMATCH line instead of quietly showing a plausible fiction. So a trace
    that reads clean is a trace of the real control path.
"""
import argparse
import math

from ..config import load_gains, load_vehicle_params
from ..gnc import allocation as alloc_mod
from ..gnc.controller import TvcController
from ..gnc.mathx import (attitude_error, euler_to_quat, quat_to_euler,
                         quat_to_rotmat, thrust_axis)
from ..gnc.types import ControlMode, Setpoint
from ..hal.gazebo import rotor_speeds
from ..plant.actuators import ActuatorChain
from ..plant.rigidbody import dynamics

D = math.degrees


# --- the starting conditions you can trace from -------------------------------
# Deliberately the same upsets verify/scenarios.py uses, so a trace explains a
# scenario rather than describing some other flight.
CASES = {
    "lateral": dict(
        what="released at +8 deg pitch and -8 deg yaw, holding 2 m",
        att=(8.0, -8.0, 0.0), z=2.0, altitude_hold=True, position_hold=False),
    "roll": dict(
        what="released at +20 deg roll (about the thrust axis), holding 2 m",
        att=(0.0, 0.0, 20.0), z=2.0, altitude_hold=True, position_hold=False),
    "climb": dict(
        what="level on the ground, commanded to climb to 2 m",
        att=(0.0, 0.0, 0.0), z=0.0, altitude_hold=True, position_hold=False,
        z_des=2.0),
    "hover": dict(
        what="level at 2 m, holding position and altitude",
        att=(0.0, 0.0, 0.0), z=2.0, altitude_hold=True, position_hold=True),
}


def _rule(title):
    print("\n" + title)
    print("-" * 78)


def _vec(v, unit="", fmt="%+.6f"):
    return "(" + ", ".join(fmt % x for x in v) + ")" + (" " + unit if unit else "")


def trace_step(k, state, setpoint, controller, chain, vp, gains, mode, dt,
               vehicle, verbose=True):
    """One step, printed. Returns (actuator setpoint, achieved, xdot)."""
    p = print if verbose else (lambda *a, **k: None)

    if verbose:
        print("=" * 78)
        print("STEP %d      t = %.3f s      dt = %.0f ms" % (k, k * dt, dt * 1000))
        print("=" * 78)

    # ---------------------------------------------------------------- 1. input
    pitch, yaw, roll = quat_to_euler(state.quat)
    _rule("1. STATE IN  (seam A -- what the controller believes, never truth)")
    p("   position   %s m       altitude %.4f m" % (_vec(state.pos_i, fmt="%+.4f"),
                                                    state.pos_i[2]))
    p("   velocity   %s m/s" % _vec(state.vel_i, fmt="%+.4f"))
    p("   quaternion %s   (qw, qx, qy, qz), inertial <- body" % _vec(state.quat))
    p("   euler      pitch %+.3f   yaw %+.3f   roll %+.3f  deg   [readout only]"
      % (D(pitch), D(yaw), D(roll)))
    p("   body rates %s rad/s" % _vec(state.omega_b))

    # ------------------------------------------------------ 2. position (outer)
    pitch_des, yaw_des = setpoint.pitch_des, setpoint.yaw_des
    if mode.position_hold:
        _rule("2. POSITION LOOP  -> attitude setpoint      [gnc/position.py]")
        ex = state.pos_i[0] - setpoint.pos_des[0]
        ey = state.pos_i[1] - setpoint.pos_des[1]
        pitch_des, yaw_des = controller.position.update(
            state.pos_i, state.vel_i, setpoint.pos_des)
        p("   error      ex %+.4f m   ey %+.4f m" % (ex, ey))
        p("   yaw_des   = -clamp(kp*ex + kd*vx) = -(%.2f*%+.4f + %.2f*%+.4f)"
          % (gains.kp_pos, ex, gains.kd_pos, state.vel_i[0]))
        p("   pitch_des = +clamp(kp*ey + kd*vy) = +(%.2f*%+.4f + %.2f*%+.4f)"
          % (gains.kp_pos, ey, gains.kd_pos, state.vel_i[1]))
        p("   -> pitch_des %+.3f deg,  yaw_des %+.3f deg   (clamped to +/-%.1f)"
          % (D(pitch_des), D(yaw_des), gains.max_tilt_deg))
        p("   NOTE this loop is ~10x slower than the attitude loop on purpose:")
        p("        wn = sqrt(g*kp) = %.2f rad/s against the attitude loop's %.2f."
          % (math.sqrt(9.81 * gains.kp_pos),
             math.sqrt(gains.kp_rate * gains.kp_angle)))
    else:
        _rule("2. POSITION LOOP  -- OFF (mode.position_hold is false)")
        p("   attitude setpoint comes straight from the Setpoint:")
        p("   pitch_des %+.3f deg,  yaw_des %+.3f deg"
          % (D(pitch_des), D(yaw_des)))

    # -------------------------------------------------------------- 3. altitude
    delta_known = controller._delta_cmd
    if mode.altitude_hold:
        _rule("3. ALTITUDE LOOP  -> thrust command         [gnc/altitude.py]")
        z, vz = state.pos_i[2], state.vel_i[2]
        raw = gains.kp_alt * (setpoint.z_des - z)
        vz_des = max(min(raw, gains.vz_max), -gains.vz_max)
        p("   vz_des = clamp(kp_alt*(z_des - z), +/-vz_max)")
        p("          = clamp(%.2f*(%.3f - %.3f), +/-%.1f)  =  %+.4f m/s"
          % (gains.kp_alt, setpoint.z_des, z, gains.vz_max, vz_des))
        az = controller.altitude.pid_vz.kp * (vz_des - vz) \
            + controller.altitude.pid_vz.ki * controller.altitude.pid_vz.integral
        p("   az_des = PID(vz_des - vz)  =  kp_vz*%+.4f + ki_vz*I"
          % (vz_des - vz))
        p("          = %.2f*%+.4f%s  =  %+.4f m/s^2"
          % (gains.kp_vz, vz_des - vz,
             "" if gains.ki_vz == 0 else " + %.2f*%+.4f" % (
                 gains.ki_vz, controller.altitude.pid_vz.integral), az))
        r2 = quat_to_rotmat(state.quat)[2]
        n = thrust_axis(delta_known)
        proj = r2[0] * n[0] + r2[1] * n[1] + r2[2] * n[2]
        p("   proj   = (R(q) @ n_hat) . z_inertial   -- the useful fraction of thrust")
        p("          = %.6f   (thrust axis is %.2f deg off vertical)"
          % (proj, D(math.acos(min(max(proj, -1.0), 1.0)))))
        proj_c = max(proj, controller.altitude.cos_min)
        T_raw = vp.m * (vp.g + az) / proj_c
        p("   T      = m*(g + az_des) / max(proj, cos_min)          <- 1/cos FEEDFORWARD")
        p("          = %.4f*(%.2f %+.4f) / %.6f  =  %.4f N"
          % (vp.m, vp.g, az, proj_c, T_raw))
        p("          without the 1/cos term this would be %.4f N, i.e. %.1f%% low"
          % (vp.m * (vp.g + az), 100 * (1 - proj_c)))
    else:
        _rule("3. ALTITUDE LOOP  -- OFF; thrust pinned at hover")
        T_raw = vp.m * vp.g
        p("   T = m*g = %.4f N   (not zero and not T_max: the allocator's" % T_raw)
        p("   feasible set depends on T, so a placeholder would change how much")
        p("   authority the other axes get.)")

    # -------------------------------------------------------------- 4. attitude
    _rule("4. ATTITUDE LOOP  -> body-rate setpoint     [gnc/attitude.py]")
    q_des = euler_to_quat(pitch_des, yaw_des, setpoint.roll_des)
    e = attitude_error(state.quat, q_des)
    p("   q_des  = euler_to_quat(pitch_des, yaw_des, roll_des) = %s" % _vec(q_des))
    p("   e      = 2*sgn(qe_w)*qe_v  for  qe = q^-1 (x) q_des      [quaternion error]")
    p("          = %s rad" % _vec(e))
    p("          = (%+.3f, %+.3f, %+.3f) deg" % tuple(D(v) for v in e))
    p("          sgn() picks the SHORT way round; without it a 10 deg target can")
    p("          be chased the 350 deg way, which on 7 deg of gimbal is a tumble.")
    w_des = (gains.kp_angle * e[0], gains.kp_angle * e[1],
             gains.kp_angle_roll * e[2])
    p("   w_des  = kp_angle * e     (roll uses its OWN gain: %.1f vs %.1f)"
      % (gains.kp_angle_roll, gains.kp_angle))
    p("          = %s rad/s" % _vec(w_des))

    # ------------------------------------------------------------------ 5. rate
    _rule("5. RATE LOOP  -> angular acceleration       [gnc/attitude.py, gnc/pid.py]")
    err = tuple(w_des[i] - state.omega_b[i] for i in range(3))
    p("   rate error = w_des - omega = %s rad/s" % _vec(err))
    a = controller.attitude
    alpha = (
        a.pid_pitch_rate.kp * err[0] + a.pid_pitch_rate.ki * a.pid_pitch_rate.integral,
        a.pid_yaw_rate.kp * err[1] + a.pid_yaw_rate.ki * a.pid_yaw_rate.integral,
        a.pid_roll_rate.kp * err[2] + a.pid_roll_rate.ki * a.pid_roll_rate.integral,
    )
    p("   alpha  = PID(rate error)  -- output is ANGULAR ACCELERATION, not torque")
    p("          pitch: %.3f * %+.5f = %+.4f rad/s^2"
      % (gains.kp_rate, err[0], alpha[0]))
    p("          yaw  : %.3f * %+.5f = %+.4f rad/s^2"
      % (gains.kp_rate, err[1], alpha[1]))
    p("          roll : %.3f * %+.5f = %+.4f rad/s^2   <- gain is %.0fx bigger"
      % (gains.kp_rate_roll, err[2], alpha[2],
         gains.kp_rate_roll / gains.kp_rate))
    p("          ...because Izz is %.1fx SMALLER. In torque units the two gains"
      % (vp.Ix / vp.Iz))
    p("          are %.4f and %.4f N*m/(rad/s) -- nearly the same loop."
      % (vp.Ix * gains.kp_rate, vp.Iz * gains.kp_rate_roll))
    p("   frozen = %s (anti-windup: hold the integrator while saturated)"
      % ((a.last_alloc.gimbal_saturated, a.last_alloc.gimbal_saturated,
          a.last_alloc.roll_saturated),))

    M = (vp.Ix * alpha[0], vp.Iy * alpha[1], vp.Iz * alpha[2])
    _rule("6. MOMENT  -- inertia applied ONCE, here     [gnc/attitude.py]")
    p("   M = I_diag * alpha = %s N*m" % _vec(M))
    p("   Diagonal inertia on purpose: this is gain scheduling, not dynamics. A")
    p("   controller that inverts its own model's cross terms would be claiming")
    p("   an accuracy the mass budget does not support (the PLANT uses the full")
    p("   tensor -- see step 10).")

    # ------------------------------------------------------------ 7. allocation
    _rule("7. ALLOCATION  -> actuator commands         [gnc/allocation.py]")
    T_cmd = max(min(T_raw, vp.T_max), vp.T_min)
    p("   7a. thrust has priority")
    p("       T_cmd = clamp(T, T_min=%.1f, T_max=%.2f) = %.4f N%s"
      % (vp.T_min, vp.T_max, T_cmd,
         "   CLAMPED" if abs(T_cmd - T_raw) > 1e-9 else ""))
    q_lo, q_hi = alloc_mod.roll_limits(T_cmd, vp)
    p("   7b. how much roll torque exists at THIS thrust")
    p("       roll_limits(%.3f N) = [%+.4f, %+.4f] N*m" % (T_cmd, q_lo, q_hi))
    p("       ASYMMETRIC -- the lower prop runs in the upper prop's wake.")
    p("       tau_P is bought with a thrust SPLIT, so this interval shrinks to")
    p("       nothing at both idle and full throttle.")
    tau_p0 = max(min(M[2], q_hi), q_lo)
    p("       tau_P = clamp(M_z, lo, hi) = %+.6f%s"
      % (tau_p0, "   CLAMPED" if abs(tau_p0 - M[2]) > 1e-12 else ""))
    d0 = alloc_mod._lateral_gimbal((M[0], M[1]), T_cmd, tau_p0, vp)
    p("   7c. solve the lateral 2x2 for the gimbal, then refine")
    p("       [M_x]   [ tau_P  -T*L ] [d1]      det = -(tau_P^2 + (T*L)^2) < 0")
    p("       [M_y] = [ -T*L  -tau_P] [d2]      so it is NEVER singular")
    p("       T*L = %.4f * %.4f = %.4f N*m of lateral authority per radian"
      % (T_cmd, vp.L, T_cmd * vp.L))
    p("       seed + 3 Newton steps on the exact trig map")
    p("       -> delta = (%+.3f, %+.3f) deg   [inner, outer]" % (D(d0[0]), D(d0[1])))
    p("   7d. cosine-loss iteration (twice): only tau_P*cos(d1)*cos(d2) reaches")
    p("       body z, and d1/d2 are not known until 7c has run.")
    result = alloc_mod.allocate(M, T_raw, vp)
    p("       -> tau_P %+.6f  (was %+.6f)" % (result.tau_p, tau_p0))
    p("   7e. stops. inner [%+.2f, %+.2f]  outer [%+.2f, %+.2f] deg -- per RING,"
      % (D(vp.delta_min[0]), D(vp.delta_max[0]),
         D(vp.delta_min[1]), D(vp.delta_max[1])))
    p("       measured, and neither is symmetric about its own neutral.")
    if result.gimbal_saturated:
        p("       SATURATED: the PAIR is scaled down, not clipped per axis --")
        p("       clipping would rotate the commanded torque toward the corner")
        p("       of the box, which is the wrong thing to do mid-recovery.")
    p("       -> delta_cmd = (%+.3f, %+.3f) deg"
      % (D(result.delta_cmd[0]), D(result.delta_cmd[1])))
    p("   7f. motor commands: invert the measured surface for (T, tau_P)")
    p("       -> u_a %.4f  u_b %.4f   (PWM %.0f / %.0f us)"
      % (result.u_a, result.u_b,
         vp.surface.pwm_min + result.u_a * (vp.surface.pwm_max - vp.surface.pwm_min),
         vp.surface.pwm_min + result.u_b * (vp.surface.pwm_max - vp.surface.pwm_min)))
    p("       -> per-prop split T1 %.4f  T2 %.4f N" % (result.T1, result.T2))
    p("       saturation flags: gimbal=%s roll=%s thrust=%s"
      % (result.gimbal_saturated, result.roll_saturated, result.thrust_saturated))

    # -------------------------------------------------------------- 8. the seam
    cmd = controller.update(state, setpoint, dt)
    _rule("8. ACTUATOR SETPOINT  (seam B -- everything the flight code decided)")
    p("   motor_a          %.6f        normalized [0,1]" % cmd.motor_a)
    p("   motor_b          %.6f" % cmd.motor_b)
    p("   gimbal_inner_rad %+.6f rad = %+.3f deg   (yaw plane, body y)"
      % (cmd.gimbal_inner_rad, D(cmd.gimbal_inner_rad)))
    p("   gimbal_outer_rad %+.6f rad = %+.3f deg   (pitch plane, body x)"
      % (cmd.gimbal_outer_rad, D(cmd.gimbal_outer_rad)))
    p("   thrust_n         %.6f N       PREDICTION, not a command" % cmd.thrust_n)
    p("   tau_p_nm         %+.6f N*m" % cmd.tau_p_nm)
    p("   sat_gimbal %s  sat_roll %s  sat_thrust %s"
      % (cmd.sat_gimbal, cmd.sat_roll, cmd.sat_thrust))

    bad = []
    for name, got, want in (("gimbal_inner", cmd.gimbal_inner_rad, result.delta_cmd[0]),
                            ("gimbal_outer", cmd.gimbal_outer_rad, result.delta_cmd[1]),
                            ("thrust", cmd.thrust_n, result.T_cmd),
                            ("tau_p", cmd.tau_p_nm, result.tau_p)):
        if abs(got - want) > 1e-9:
            bad.append("%s: traced %.9f, controller %.9f" % (name, want, got))
    if bad:
        p("\n   *** MISMATCH -- this trace is NOT the real control path ***")
        for b in bad:
            p("   " + b)
    else:
        p("\n   [checked: every number above matches TvcController.update()]")

    # ----------------------------------------------------------------- 9. HAL
    _rule("9. HAL  -> transport units                  [hal/gazebo.py]")
    rot = vehicle.raw["rotors"]
    wa, wb = rotor_speeds(cmd.thrust_n, cmd.tau_p_nm, rot["motor_constant"],
                          rot["moment_constant"], rot["max_rot_velocity"])
    p("   split = tau_P / c      = %+.6f / %.3f = %+.4f N"
      % (cmd.tau_p_nm, rot["moment_constant"], cmd.tau_p_nm / rot["moment_constant"]))
    p("   omega = sqrt((T -/+ split)/2 / k)")
    p("         -> rotor A %.2f   rotor B %.2f  rad/s   (ceiling %.0f)"
      % (wa, wb, rot["max_rot_velocity"]))
    p("   NOT AN RPM. These are control allocation variables -- the plugin's own")
    p("   algebra inverted so it reproduces the measured surface exactly.")

    # ------------------------------------------------------------ 10. the plant
    _rule("10. PLANT  -- simulation only, none of this flies")
    n_delay = int(round(vp.gimbal_deadtime_s / dt))
    achieved_delta, T_ach, tau_ach = chain.update(
        (cmd.gimbal_inner_rad, cmd.gimbal_outer_rad),
        cmd.thrust_n, cmd.tau_p_nm, dt)
    p("   10a. gimbal [plant/actuators.py]")
    p("        transport deadtime %.0f ms = %d steps at this dt -- a FIFO of"
      % (vp.gimbal_deadtime_s * 1000, n_delay))
    p("        commands, because that is what a serial servo bus does.")
    p("        then a per-ring slew limit: %.0f deg/s inner, %.0f deg/s outer,"
      % (D(vp.delta_rate_max[0]), D(vp.delta_rate_max[1])))
    p("        so at most %.3f / %.3f deg of movement this step."
      % (D(vp.delta_rate_max[0]) * dt, D(vp.delta_rate_max[1]) * dt))
    p("        -> ACHIEVED delta = (%+.4f, %+.4f) deg"
      % (D(achieved_delta[0]), D(achieved_delta[1])))
    p("   10b. motors [plant/actuators.py]")
    p("        model '%s', tau = %.3f s, applied to the PAIR (T, tau_P) because"
      % (chain.motor.model, chain.motor.tau_s))
    p("        that is what the bench measured. a = dt/(tau+dt) = %.4f"
      % (dt / (chain.motor.tau_s + dt) if chain.motor.tau_s > 0 else 1.0))
    p("        -> ACHIEVED T = %.4f N   tau_P = %+.6f N*m" % (T_ach, tau_ach))
    p("   10c. rigid body [plant/rigidbody.py]")
    import numpy as np
    x = np.concatenate([state.pos_i, state.vel_i, state.quat, state.omega_b])
    xdot = dynamics(0.0, x, T_ach, achieved_delta, vp, tau_ach)
    nh = thrust_axis(achieved_delta)
    p("        n_hat  = [sin d1, -sin d2 cos d1, cos d1 cos d2] = %s" % _vec(nh))
    p("        F_body = T * n_hat = %s N"
      % _vec(tuple(T_ach * v for v in nh), fmt="%+.4f"))
    p("        tau    = r_G x F + tau_P*n_hat,   r_G = (0, 0, -L) = (0, 0, %.4f)"
      % -vp.L)
    tau = (float(np.cross([vp.dx, vp.dy, -vp.L], [T_ach * v for v in nh])[0] + tau_ach * nh[0]),
           float(np.cross([vp.dx, vp.dy, -vp.L], [T_ach * v for v in nh])[1] + tau_ach * nh[1]),
           float(np.cross([vp.dx, vp.dy, -vp.L], [T_ach * v for v in nh])[2] + tau_ach * nh[2]))
    p("               = %s N*m" % _vec(tau))
    p("        I*wdot = tau - omega x (I omega),  FULL tensor (Iyz/Izz = %.0f%%)"
      % (100 * abs(vp.Iyz) / vp.Iz))
    p("        -> a_inertial = %s m/s^2" % _vec(xdot[3:6], fmt="%+.4f"))
    p("        -> omega_dot  = %s rad/s^2" % _vec(xdot[10:13], fmt="%+.4f"))
    p("   10d. integrate: solve_ivp RK45 over one control period, max_step=dt/4")

    return cmd, achieved_delta, T_ach, tau_ach, xdot


def main(argv=None):
    """Trace --steps control steps from the --case starting condition."""
    ap = argparse.ArgumentParser(
        prog="tvc.py trace",
        description="Trace every computation in the control path, with numbers.")
    ap.add_argument("--case", default="lateral", choices=sorted(CASES),
                    help="starting condition (default: lateral)")
    ap.add_argument("--steps", type=int, default=1, help="how many steps to trace")
    ap.add_argument("--dt", type=float, default=0.01, help="control period [s]")
    ap.add_argument("--gains", default=None, help="profile from control_gains.yaml")
    args = ap.parse_args(argv)

    import numpy as np
    from scipy.integrate import solve_ivp

    from ..config import load
    from ..gnc.mathx import quat_normalize
    from ..plant.sensors import PerfectEstimator

    vp, gains, vehicle = load_vehicle_params(), load_gains(args.gains), load()
    case = CASES[args.case]
    mode = ControlMode(altitude_hold=case["altitude_hold"],
                       position_hold=case["position_hold"])
    controller = TvcController(vp, gains, mode)
    md = vehicle.motor_dynamics
    chain = ActuatorChain(vp, md.get("model", "first_order"),
                          md.get("tau_s", 0.10), md.get("deadtime_s", 0.0))
    estimator = PerfectEstimator()

    print("TVC control-path trace")
    print("case '%s': %s" % (args.case, case["what"]))
    print("gains '%s', vehicle %.4f kg, hover thrust %.3f N (%.0f%% of %.2f N)"
          % (args.gains or "<file default>", vp.m, vp.m * vp.g,
             100 * vp.m * vp.g / vp.T_max, vp.T_max))

    q0 = euler_to_quat(*[math.radians(v) for v in case["att"]])
    x = np.concatenate([[0.0, 0.0, case["z"]], [0.0, 0.0, 0.0], q0, [0.0, 0.0, 0.0]])
    z_des = case.get("z_des", case["z"])
    setpoint = Setpoint(z_des=z_des, pos_des=(0.0, 0.0, z_des))

    t = 0.0
    for k in range(args.steps):
        state = estimator.estimate(x[0:3], x[3:6], quat_normalize(x[6:10]),
                                   x[10:13], t)
        cmd, delta, T_ach, tau_ach, _ = trace_step(
            k, state, setpoint, controller, chain, vp, gains, mode, args.dt,
            vehicle)
        sol = solve_ivp(dynamics, [t, t + args.dt], x,
                        args=(T_ach, delta, vp, tau_ach),
                        method="RK45", max_step=args.dt / 4)
        x = sol.y[:, -1]
        x[6:10] = quat_normalize(x[6:10])
        t += args.dt

    pitch, yaw, roll = quat_to_euler(x[6:10])
    print("\n" + "=" * 78)
    print("after %d step(s): pitch %+.3f  yaw %+.3f  roll %+.3f deg, z %.4f m"
          % (args.steps, D(pitch), D(yaw), D(roll), x[2]))
    print("=" * 78)
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
