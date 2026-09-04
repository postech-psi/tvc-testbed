"""
Model-in-the-loop harness: analytic plant + flight code, fixed step.
================================================================================
Moved verbatim from physics.py.
"""

import numpy as np
from scipy.integrate import solve_ivp
from dataclasses import dataclass, field

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

    roll_des_deg / pitch_des_deg are the LATERAL setpoints (body x / body y);
    axial_des_deg is the thrust-axis channel the paper calls roll. See the
    axis-naming note in the module docstring.
    """

    t_final: float = 5.0
    dt_ctrl: float = 0.01

    roll_des_deg: float = 0.0
    pitch_des_deg: float = 5.0
    axial_des_deg: float = 0.0      # body-z (thrust-axis) attitude setpoint

    init_roll_deg: float = 3.0
    init_pitch_deg: float = -4.0
    init_axial_deg: float = 0.0

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


def simulate(vparams: VehicleParams, gains: ControlGains, cfg: SimConfig):
    """
    Run a closed-loop simulation with zero-order-hold control.

    Three-axis attitude always; altitude hold only when cfg.altitude_hold is
    set (otherwise thrust is pinned at hover, T = mg, so every pre-existing
    run reproduces its old numbers exactly).

    Returns a dict of numpy arrays: t, euler_deg, delta_deg, omega_deg, quat,
    pos, thrust_N, tau_p_Nm, motor_N, plus scalar metrics under 'metrics'.
    """
    chain = ActuatorChain(vparams,
                          motor_model=cfg.motor_model,
                          motor_tau_s=cfg.motor_tau_s,
                          motor_deadtime_s=cfg.motor_deadtime_s)
    estimator = PerfectEstimator()
    controller = TvcController(vparams, gains, ControlMode(
        altitude_hold=cfg.altitude_hold,
        position_hold=cfg.position_hold,
        tilt_compensation=cfg.tilt_compensation,
    ))

    roll_des = np.deg2rad(cfg.roll_des_deg)
    pitch_des = np.deg2rad(cfg.pitch_des_deg)
    axial_des = np.deg2rad(cfg.axial_des_deg)

    q0 = euler_to_quat(np.deg2rad(cfg.init_roll_deg),
                       np.deg2rad(cfg.init_pitch_deg),
                       np.deg2rad(cfg.init_axial_deg))
    x = np.concatenate([[0, 0, cfg.init_z], [0, 0, 0], q0, [0, 0, 0]])

    T_hover = vparams.m * vparams.g

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
    sat_arr = np.zeros((n_steps, 3), dtype=bool)   # gimbal, axial, thrust

    setpoint = Setpoint(roll_des=roll_des, pitch_des=pitch_des,
                        axial_des=axial_des, z_des=cfg.z_des,
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
        delta_cmd = np.array([cmd.gimbal_delta1_rad, cmd.gimbal_delta2_rad])
        # Both actuator lags in one call: the gimbal's 30 ms transport delay and
        # slew, and the motors' ~100 ms thrust response. What reaches the rigid
        # body is what the actuators ACHIEVED, not what was commanded.
        delta, T_ach, tau_p_ach = chain.update(
            delta_cmd, alloc.T_cmd, alloc.tau_p, cfg.dt_ctrl)

        sol = solve_ivp(dynamics, [t, t + cfg.dt_ctrl], x,
                         args=(T_ach, delta, vparams, tau_p_ach),
                         method='RK45', max_step=cfg.dt_ctrl / 4)
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
        sat_arr[k] = (cmd.sat_gimbal, cmd.sat_axial, cmd.sat_thrust)

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
      - final roll/pitch error
      - max |delta1|, max |delta2|
      - 2%-band settling time for pitch (first time after which the response
        stays within +/- band*|step size| of the final value)
    """
    final_roll = euler_arr[-1, 0]
    final_pitch = euler_arr[-1, 1]

    roll_err = final_roll - cfg.roll_des_deg
    pitch_err = final_pitch - cfg.pitch_des_deg

    max_d1 = np.max(np.abs(delta_arr[:, 0]))
    max_d2 = np.max(np.abs(delta_arr[:, 1]))

    # settling time on pitch (typically the dominant commanded motion)
    step_size = max(abs(cfg.pitch_des_deg - cfg.init_pitch_deg), 1e-6)
    # A 2% band on a zero-size step is not a band. When pitch is commanded from
    # 0 to 0, step_size collapses to the 1e-6 floor and any cross-axis coupling
    # -- which the full inertia tensor now produces, ~0.2 deg of it -- reads as
    # "never settled" for the whole horizon. Below the angle this vehicle can
    # actually hold, settling is not a meaningful measurement, so an absolute
    # floor applies.
    tol = max(band * step_size, 0.05)
    err_series = np.abs(euler_arr[:, 1] - cfg.pitch_des_deg)
    outside = np.where(err_series > tol)[0]
    settling_time = t_arr[outside[-1]] if len(outside) > 0 else 0.0

    m = {
        "final_roll_deg": final_roll,
        "final_pitch_deg": final_pitch,
        "roll_error_deg": roll_err,
        "pitch_error_deg": pitch_err,
        "max_delta1_deg": max_d1,
        "max_delta2_deg": max_d2,
        "gimbal_max_deg": np.rad2deg(0.0),  # filled in by caller if needed
        "settling_time_s": settling_time,
        # Axial channel (the paper's roll axis).
        "final_axial_deg": euler_arr[-1, 2],
        "axial_error_deg": euler_arr[-1, 2] - cfg.axial_des_deg,
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
        m["axial_sat_frac"] = float(np.mean(sat_arr[:, 1]))
        m["thrust_sat_frac"] = float(np.mean(sat_arr[:, 2]))
    return m
