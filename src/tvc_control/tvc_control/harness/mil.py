"""
Model-in-the-loop harness: analytic plant + flight code, fixed step.
================================================================================
    python tvc.py validate      runs the scenarios built on this
    python tvc.py gui           drives it from a form

The fastest pipeline and the only one whose output can be frozen as a numerical
baseline, because it is the only one that is bit-reproducible: a fixed-rate
zero-order-hold loop over solve_ivp with a fixed max_step and no randomness
anywhere.

One iteration is estimator -> controller -> actuator chain -> integrator, and
each of those is a seam that a different harness swaps for something else. What
this file owns, and what gnc/ therefore does not, is the CLOCK: dt is passed in,
never read.
"""

import numpy as np
from scipy.integrate import solve_ivp
from dataclasses import dataclass

from ..gnc.params import VehicleParams, ControlGains
from ..gnc.mathx import quat_normalize, quat_to_euler, euler_to_quat
from ..gnc.types import ControlMode, Setpoint
from ..gnc.controller import TvcController
from ..plant.rigidbody import dynamics
from ..plant.actuators import ActuatorChain
from ..plant.sensors import PerfectEstimator


@dataclass
class SimConfig:
    """Simulation run configuration: horizon, control period, targets, disturbance.

    att_pitch_des_deg and att_yaw_des_deg are the LATERAL setpoints -- rotations
    about body x and body y, the two the gimbal drives. att_roll_des_deg is the
    thrust-axis channel, driven by tau_P alone. docs/4-CONVENTIONS.md.
    """

    t_final: float = 5.0
    dt_ctrl: float = 0.01

    att_pitch_des_deg: float = 0.0
    att_yaw_des_deg: float = 5.0
    att_roll_des_deg: float = 0.0      # body-z (thrust-axis) attitude setpoint

    init_att_pitch_deg: float = 3.0
    init_att_yaw_deg: float = -4.0
    init_att_roll_deg: float = 0.0

    # Altitude loop. Disabled by default so the historical attitude-only smoke
    # test, the GUI, and every existing plot keep producing the same numbers:
    # with altitude off the thrust command is pinned at hover (T = mg) exactly
    # as before. Turn it on for the climb / tilted-hover scenarios.
    altitude_hold: bool = False
    z_des: float = 1.0              # m, altitude setpoint when altitude_hold
    init_z: float = 0.0             # m
    tilt_compensation: bool = True  # 1/cos(theta) feedforward on the thrust cmd

    # Position hold, off by default so every pre-existing run is unchanged.
    # Explicitly a mode rather than inferred from a non-zero target: "hold the
    # origin" and "do not run the position loop" are different commands.
    position_hold: bool = False
    x_des: float = 0.0              # m, inertial
    y_des: float = 0.0

    # Motor lag model. Defaults mirror vehicle_params.yaml's motor_dynamics; the
    # bench record does not say whether the measured 100 ms is a delay or a time
    # constant, so both are runnable and the choice is explicit at the call site
    # rather than buried. See plant/actuators.py::MotorLag.
    motor_model: str = "first_order"
    motor_tau_s: float = 0.10
    motor_deadtime_s: float = 0.0

    # Battery sag. OFF by default: with it off the actuator chain is bit-identical
    # to before and every frozen baseline reproduces. On, the thrust surface's
    # fresh-pack authority derates as the pack drains (plant/battery.py); the
    # numbers come from vehicle_params.yaml's motor_dynamics.battery block.
    battery_sag: bool = False


def simulate(vparams: VehicleParams, gains: ControlGains, cfg: SimConfig):
    """
    Run a closed-loop simulation with zero-order-hold control.

    Three-axis attitude always; altitude hold only when cfg.altitude_hold is
    set (otherwise thrust is pinned at hover, T = mg, so every pre-existing
    run reproduces its old numbers exactly).

    Returns a dict of numpy arrays: t, euler_deg, delta_deg, omega_deg, quat,
    pos, thrust_N, tau_p_Nm, motor_N, plus scalar metrics under 'metrics'.
    """
    battery, current_per_n = None, 0.0
    if cfg.battery_sag:
        # Reading the YAML here (not in gnc/) is fine: this is the harness, and
        # the battery block lives with the rest of the measured vehicle data.
        from ..config import load
        from ..plant.battery import BatteryState
        md = load().motor_dynamics
        bc = md["battery"]
        battery = BatteryState(
            v_full=bc["v_full_v"], v_per_mah=bc["v_per_mah"],
            capacity_mah=bc["capacity_mah"],
            thrust_sensitivity_n_per_v=bc["thrust_sensitivity_n_per_v"])
        # Approximate pack current from thrust: linear, peak_current_a at T_max.
        current_per_n = md.get("peak_current_a", 34.7) / vparams.T_max

    chain = ActuatorChain(vparams,
                          motor_model=cfg.motor_model,
                          motor_tau_s=cfg.motor_tau_s,
                          motor_deadtime_s=cfg.motor_deadtime_s,
                          battery=battery, current_per_n=current_per_n)
    estimator = PerfectEstimator()
    controller = TvcController(vparams, gains, ControlMode(
        altitude_hold=cfg.altitude_hold,
        position_hold=cfg.position_hold,
        tilt_compensation=cfg.tilt_compensation,
    ))

    pitch_des = np.deg2rad(cfg.att_pitch_des_deg)
    yaw_des = np.deg2rad(cfg.att_yaw_des_deg)
    roll_des = np.deg2rad(cfg.att_roll_des_deg)

    q0 = euler_to_quat(np.deg2rad(cfg.init_att_pitch_deg),
                       np.deg2rad(cfg.init_att_yaw_deg),
                       np.deg2rad(cfg.init_att_roll_deg))
    x = np.concatenate([[0, 0, cfg.init_z], [0, 0, 0], q0, [0, 0, 0]])

    n_steps = int(np.ceil(cfg.t_final / cfg.dt_ctrl))
    t_arr = np.zeros(n_steps)
    euler_arr = np.zeros((n_steps, 3))
    delta_arr = np.zeros((n_steps, 2))
    omega_arr = np.zeros((n_steps, 3))
    # Quaternion + position histories: not needed by the 2D plots, but they are
    # what the 3D viewer animates (Euler angles would reintroduce gimbal lock).
    quat_arr = np.zeros((n_steps, 4))
    pos_arr = np.zeros((n_steps, 3))
    # Actuator histories, for the authority/saturation story the plots tell.
    thrust_arr = np.zeros(n_steps)
    tau_p_arr = np.zeros(n_steps)
    motor_arr = np.zeros((n_steps, 2))
    sat_arr = np.zeros((n_steps, 3), dtype=bool)   # gimbal, roll, thrust

    setpoint = Setpoint(pitch_des=pitch_des, yaw_des=yaw_des,
                        roll_des=roll_des, z_des=cfg.z_des,
                        pos_des=(cfg.x_des, cfg.y_des, cfg.z_des))

    t = 0.0
    delta = np.zeros(2)
    for k in range(n_steps):
        q = quat_normalize(x[6:10])
        omega = x[10:13]

        # Seam A: the controller is handed an ESTIMATE, never the plant's state,
        # even though today's estimator is the identity. See plant/sensors.py.
        est = estimator.estimate(x[0:3], x[3:6], q, omega, t)

        # gimbal_rad is the ACHIEVED deflection. A simulation knows it; the
        # vehicle does not (the servos give no position feedback), so flight
        # code falls back to its own last command. Passing it here keeps this
        # harness bit-identical to its pre-facade behaviour; the difference
        # between the two is one servo lag and is worth measuring later.
        cmd = controller.update(est, setpoint, cfg.dt_ctrl, gimbal_rad=delta)
        alloc = controller.attitude.last_alloc
        delta_cmd = np.array([cmd.gimbal_inner_rad, cmd.gimbal_outer_rad])
        # Both actuator lags in one call: the gimbal's 30 ms transport delay and
        # slew, and the motors' ~100 ms thrust response. What reaches the rigid
        # body is what the actuators ACHIEVED, not what was commanded.
        delta, T_ach, tau_p_ach = chain.update(
            delta_cmd, alloc.T_cmd, alloc.tau_p, cfg.dt_ctrl)

        sol = solve_ivp(dynamics, [t, t + cfg.dt_ctrl], x,
                         args=(T_ach, delta, vparams, tau_p_ach),
                         method='RK45', max_step=cfg.dt_ctrl / 4)
        if not sol.success:
            raise RuntimeError("rigid-body integration failed at t=%.6f s: %s"
                               % (t, sol.message))
        x = sol.y[:, -1]
        x[6:10] = quat_normalize(x[6:10])

        t += cfg.dt_ctrl
        t_arr[k] = t
        euler_arr[k] = np.rad2deg(quat_to_euler(x[6:10]))
        delta_arr[k] = np.rad2deg(delta)
        # Log the POST-step rate (x after the solve), not the pre-step `omega`,
        # so the rate trace lines up in time with the euler/attitude trace.
        omega_arr[k] = np.rad2deg(x[10:13])
        quat_arr[k] = x[6:10]
        pos_arr[k] = x[0:3]
        thrust_arr[k] = T_ach
        tau_p_arr[k] = tau_p_ach
        motor_arr[k] = (alloc.T1, alloc.T2)
        sat_arr[k] = (cmd.sat_gimbal, cmd.sat_roll, cmd.sat_thrust)

    metrics = _compute_metrics(t_arr, euler_arr, delta_arr, cfg,
                               pos_arr=pos_arr, tau_p_arr=tau_p_arr,
                               sat_arr=sat_arr)

    return {
        "t": t_arr,
        "euler_deg": euler_arr,
        "delta_deg": delta_arr,
        "omega_deg": omega_arr,
        "quat": quat_arr,
        "pos": pos_arr,
        "thrust_N": thrust_arr,
        "tau_p_Nm": tau_p_arr,
        "motor_N": motor_arr,
        "saturated": sat_arr,
        "metrics": metrics,
    }


def _compute_metrics(t_arr, euler_arr, delta_arr, cfg: SimConfig, band=0.02,
                     pos_arr=None, tau_p_arr=None, sat_arr=None):
    """
    Compute summary metrics:
      - final pitch/yaw error
      - max |delta1|, max |delta2|
      - 2%-band settling time for yaw (first time after which the response
        stays within +/- band*|step size| of the final value)
    """
    final_pitch = euler_arr[-1, 0]
    final_yaw = euler_arr[-1, 1]

    pitch_err = final_pitch - cfg.att_pitch_des_deg
    yaw_err = final_yaw - cfg.att_yaw_des_deg

    max_d1 = np.max(np.abs(delta_arr[:, 0]))
    max_d2 = np.max(np.abs(delta_arr[:, 1]))

    # settling time on yaw (typically the dominant commanded motion)
    step_size = max(abs(cfg.att_yaw_des_deg - cfg.init_att_yaw_deg), 1e-6)
    # A 2% band on a zero-size step is not a band. When yaw is commanded from
    # 0 to 0, step_size collapses to the 1e-6 floor and any cross-axis coupling
    # -- which the full inertia tensor now produces, ~0.2 deg of it -- reads as
    # "never settled" for the whole horizon. Below the angle this vehicle can
    # actually hold, settling is not a meaningful measurement, so an absolute
    # floor applies.
    tol = max(band * step_size, 0.05)
    err_series = np.abs(euler_arr[:, 1] - cfg.att_yaw_des_deg)
    outside = np.where(err_series > tol)[0]
    settling_time = t_arr[outside[-1]] if len(outside) > 0 else 0.0

    m = {
        "final_pitch_deg": final_pitch,
        "final_yaw_deg": final_yaw,
        "pitch_error_deg": pitch_err,
        "yaw_error_deg": yaw_err,
        "max_gimbal_inner_deg": max_d1,
        "max_gimbal_outer_deg": max_d2,
        "settling_time_s": settling_time,
        # Roll channel: rotation about the thrust axis, driven by tau_P.
        "final_roll_deg": euler_arr[-1, 2],
        "roll_error_deg": euler_arr[-1, 2] - cfg.att_roll_des_deg,
    }
    if tau_p_arr is not None:
        m["max_tau_p_Nm"] = float(np.max(np.abs(tau_p_arr)))
    if pos_arr is not None:
        m["final_z_m"] = float(pos_arr[-1, 2])
        # Altitude sag is the headline number for the tilt-compensation test:
        # how far the vehicle dropped below where it started/was asked to be.
        ref = cfg.z_des if cfg.altitude_hold else cfg.init_z
        m["max_alt_sag_m"] = float(np.max(np.maximum(ref - pos_arr[:, 2], 0.0)))
        m["altitude_error_m"] = float(pos_arr[-1, 2] - ref)
    if sat_arr is not None:
        m["gimbal_sat_frac"] = float(np.mean(sat_arr[:, 0]))
        m["roll_sat_frac"] = float(np.mean(sat_arr[:, 1]))
        m["thrust_sat_frac"] = float(np.mean(sat_arr[:, 2]))
    return m
