"""
tvc_physics.py
==============
Core 6-DOF rigid-body dynamics, quaternion kinematics, actuator model, and
cascaded PID attitude controller for a coax-motor + 2-axis-gimbal VTVL vehicle.

This module is GUI-agnostic and display-agnostic -- it is imported by both the
interactive GUI (tvc_gui.py) and can be run/tested headlessly from the command
line or a test script.

Physics corrections and conventions carried over from Step 1 / Step 2 notes:
  - The TVC moment arm is the AXIAL distance L from the gimbal pivot to the
    vehicle center of mass (along body +z, the nominal thrust axis) -- NOT a
    lateral offset (dx, dy). A purely lateral offset crossed with an on-axis
    thrust vector produces zero torque to leading order; dx/dy are kept only
    as optional CM-misalignment DISTURBANCE terms (default 0).
  - Roll / yaw about the thrust axis (body z) is NOT controllable by the
    2-axis gimbal -- torque about z from d_cm x F is zero when d_cm = (0,0,-L).
    Physical roll authority comes from differential RPM between the two
    counter-rotating coax propellers (reaction-torque coupling), which is
    modeled here only as an external tau_z input, not derived from the gimbal.
  - Attitude is represented with a unit quaternion throughout (never Euler
    angles internally) specifically to avoid the +/-90 deg gimbal-lock
    singularity documented in the Step 1 guide and the TVC/6-DOF lecture notes.

References:
  - de Lajarte (2021), "Guidance, Navigation and Control of a Sounding Rocket",
    Ch. 3 (rigid-body sim) and Ch. 4 (Thrust Vectoring Control)
  - Spannagl et al. (2021), EmboRockETH gimbal geometry & cascaded PID
  - Linsen et al. (2022), Optimal TVC of an electric small-scale rocket
"""

import numpy as np
from scipy.integrate import solve_ivp
from dataclasses import dataclass, field


# =============================================================================
# 1. Parameter containers
# =============================================================================

@dataclass
class VehicleParams:
    """Physical vehicle parameters. Angles are stored in DEGREES for the GUI's
    convenience and converted to radians via properties where dynamics need them."""

    m: float = 1.8                  # kg, total mass
    Ix: float = 0.010               # kg*m^2, roll inertia (body x)
    Iy: float = 0.010               # kg*m^2, pitch inertia (body y)
    Iz: float = 0.004               # kg*m^2, yaw/spin inertia (body z)

    L: float = 0.15                 # m, AXIAL lever arm: gimbal pivot -> CM
    dx: float = 0.0                 # m, lateral CM misalignment (disturbance, default 0)
    dy: float = 0.0                 # m, lateral CM misalignment (disturbance, default 0)

    T_max: float = 30.0             # N, max thrust (per combined coax unit)
    T_min: float = 5.0              # N, min thrust (idle; must stay > 0 for allocation)

    gimbal_max_deg: float = 15.0        # deg, mechanical gimbal limit (each axis)
    gimbal_rate_max_deg: float = 180.0  # deg/s, servo slew rate

    g: float = 9.81                 # m/s^2

    @property
    def gimbal_max(self):
        return np.deg2rad(self.gimbal_max_deg)

    @property
    def gimbal_rate_max(self):
        return np.deg2rad(self.gimbal_rate_max_deg)


@dataclass
class ControlGains:
    """Cascaded PID gains: outer angle loop + inner rate loop.
    Same gains are shared between roll and pitch (symmetric vehicle assumption)."""

    kp_angle: float = 4.0      # outer loop: angle error -> rate setpoint
    kp_rate: float = 0.02      # inner loop: rate error -> torque
    ki_rate: float = 0.002
    kd_rate: float = 0.004
    i_limit: float = 0.5       # integrator clamp (anti-windup)


@dataclass
class SimConfig:
    """Simulation run configuration: horizon, control period, targets, disturbance."""

    t_final: float = 5.0
    dt_ctrl: float = 0.01

    roll_des_deg: float = 0.0
    pitch_des_deg: float = 5.0

    init_roll_deg: float = 3.0
    init_pitch_deg: float = -4.0


# =============================================================================
# 2. Quaternion utilities
# =============================================================================

def quat_normalize(q):
    """Renormalize to unit quaternion (mandatory after every integration step;
    truncation error accumulates otherwise -- see Step 1 guide, Prop. on renorm)."""
    return q / np.linalg.norm(q)


def quat_to_rotmat(q):
    """Body -> inertial rotation matrix R(q), from unit quaternion q=(qw,qx,qy,qz)."""
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),     1 - 2*(qx**2 + qy**2)],
    ])


def quat_kinematics(q, omega):
    """dq/dt = 0.5 * Omega(omega) @ q  -- linear, singularity-free kinematics."""
    wx, wy, wz = omega
    Omega = np.array([
        [0, -wx, -wy, -wz],
        [wx,  0,  wz, -wy],
        [wy, -wz,  0,  wx],
        [wz,  wy, -wx,  0],
    ])
    return 0.5 * Omega @ q


def quat_to_euler(q):
    """ZYX (yaw-pitch-roll) Euler angles, for readout/plotting ONLY -- never
    used internally for kinematics propagation (that would reintroduce gimbal lock)."""
    qw, qx, qy, qz = q
    roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
    pitch = np.arcsin(np.clip(2*(qw*qy - qz*qx), -1, 1))
    yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
    return np.array([roll, pitch, yaw])


def euler_to_quat(roll, pitch, yaw):
    """Construct a quaternion from ZYX Euler angles (used only to set up initial
    conditions / disturbances in a human-friendly way)."""
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return np.array([qw, qx, qy, qz])


# =============================================================================
# 3. Actuator model: rate-limited gimbal servo
# =============================================================================

class GimbalActuator:
    """First-order rate-limited servo model for the 2-axis gimbal."""

    def __init__(self, params: VehicleParams):
        self.p = params
        self.delta = np.zeros(2)   # current [delta1, delta2], rad

    def reset(self):
        self.delta = np.zeros(2)

    def update(self, delta_cmd, dt):
        delta_cmd = np.clip(delta_cmd, -self.p.gimbal_max, self.p.gimbal_max)
        max_step = self.p.gimbal_rate_max * dt
        step = np.clip(delta_cmd - self.delta, -max_step, max_step)
        self.delta = self.delta + step
        return self.delta.copy()


# =============================================================================
# 4. Rigid-body dynamics (Newton-Euler), held control input over dt (ZOH)
# =============================================================================

def dynamics(t, x, T, delta, params: VehicleParams):
    """
    x = [r(3), v(3), q(4), omega(3)]  (13 states)
    T:      scalar thrust magnitude [N]
    delta:  [delta1, delta2] gimbal deflection angles [rad]
    """
    v = x[3:6]
    q = quat_normalize(x[6:10])
    omega = x[10:13]

    d1, d2 = delta
    # Exact (not small-angle) gimballed thrust vector in body frame.
    f_body = T * np.array([
        np.sin(d1),
        np.sin(d2) * np.cos(d1),
        np.cos(d1) * np.cos(d2),
    ])

    R = quat_to_rotmat(q)
    a_inertial = (R @ f_body) / params.m + np.array([0, 0, -params.g])

    # AXIAL lever arm (gimbal pivot -> CM along body z), plus optional lateral
    # misalignment disturbance terms dx, dy (normally ~0 for a balanced vehicle).
    d_cm = np.array([params.dx, params.dy, -params.L])
    tau = np.cross(d_cm, f_body)

    I = np.diag([params.Ix, params.Iy, params.Iz])
    I_inv = np.diag([1/params.Ix, 1/params.Iy, 1/params.Iz])
    omega_dot = I_inv @ (tau - np.cross(omega, I @ omega))

    q_dot = quat_kinematics(q, omega)

    return np.concatenate([v, a_inertial, q_dot, omega_dot])


# =============================================================================
# 5. Cascaded PID controller
# =============================================================================

class PID:
    def __init__(self, kp, ki, kd, i_limit=1e9):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = i_limit
        self.integral = 0.0
        self.prev_err = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0

    def update(self, err, dt):
        self.integral = np.clip(self.integral + err*dt, -self.i_limit, self.i_limit)
        deriv = (err - self.prev_err) / dt if dt > 0 else 0.0
        self.prev_err = err
        return self.kp*err + self.ki*self.integral + self.kd*deriv


class AttitudeController:
    """
    Cascade: attitude angle error -> desired body rate
             -> body rate error   -> desired torque
             -> control allocation -> gimbal angle command

    tau_z (spin/yaw about the thrust axis) is NOT controllable by the gimbal
    alone -- it is left at 0 here. Real yaw control on this vehicle comes
    from differential RPM between the two coax motors (not modeled in this
    attitude-only simulator; see Step 3 notes for the full allocation layer).
    """

    def __init__(self, params: VehicleParams, gains: ControlGains):
        self.p = params
        self.gains = gains
        self.pid_roll_angle = PID(kp=gains.kp_angle, ki=0.0, kd=0.0)
        self.pid_pitch_angle = PID(kp=gains.kp_angle, ki=0.0, kd=0.0)
        self.pid_roll_rate = PID(kp=gains.kp_rate, ki=gains.ki_rate, kd=gains.kd_rate, i_limit=gains.i_limit)
        self.pid_pitch_rate = PID(kp=gains.kp_rate, ki=gains.ki_rate, kd=gains.kd_rate, i_limit=gains.i_limit)

    def reset(self):
        for pid in (self.pid_roll_angle, self.pid_pitch_angle, self.pid_roll_rate, self.pid_pitch_rate):
            pid.reset()

    def update(self, q, omega, roll_des, pitch_des, T_des, dt):
        roll, pitch, _ = quat_to_euler(q)

        roll_rate_des = self.pid_roll_angle.update(roll_des - roll, dt)
        pitch_rate_des = self.pid_pitch_angle.update(pitch_des - pitch, dt)

        tau_x = self.pid_roll_rate.update(roll_rate_des - omega[0], dt)
        tau_y = self.pid_pitch_rate.update(pitch_rate_des - omega[1], dt)

        # Control allocation: invert tau_x = L*T*sin(delta2), tau_y = -L*T*sin(delta1)
        T_safe = max(T_des, self.p.T_min)
        arg1 = np.clip(-tau_y / (self.p.L * T_safe), -1.0, 1.0)
        arg2 = np.clip(tau_x / (self.p.L * T_safe), -1.0, 1.0)
        delta1 = np.arcsin(arg1)
        delta2 = np.arcsin(arg2)

        return np.clip([delta1, delta2], -self.p.gimbal_max, self.p.gimbal_max)


# =============================================================================
# 6. Simulation driver
# =============================================================================

def simulate(vparams: VehicleParams, gains: ControlGains, cfg: SimConfig):
    """
    Run a closed-loop simulation with zero-order-hold control.

    Returns a dict of numpy arrays: t, euler_deg, delta_deg, omega_deg, plus
    scalar summary metrics under the 'metrics' key.
    """
    gimbal = GimbalActuator(vparams)
    controller = AttitudeController(vparams, gains)

    roll_des = np.deg2rad(cfg.roll_des_deg)
    pitch_des = np.deg2rad(cfg.pitch_des_deg)

    q0 = euler_to_quat(np.deg2rad(cfg.init_roll_deg), np.deg2rad(cfg.init_pitch_deg), 0.0)
    x = np.concatenate([[0, 0, 0], [0, 0, 0], q0, [0, 0, 0]])

    T_hover = vparams.m * vparams.g

    n_steps = int(np.ceil(cfg.t_final / cfg.dt_ctrl))
    t_arr = np.zeros(n_steps)
    euler_arr = np.zeros((n_steps, 3))
    delta_arr = np.zeros((n_steps, 2))
    omega_arr = np.zeros((n_steps, 3))

    t = 0.0
    for k in range(n_steps):
        q = quat_normalize(x[6:10])
        omega = x[10:13]

        delta_cmd = controller.update(q, omega, roll_des, pitch_des, T_hover, cfg.dt_ctrl)
        delta = gimbal.update(delta_cmd, cfg.dt_ctrl)

        sol = solve_ivp(dynamics, [t, t + cfg.dt_ctrl], x,
                         args=(T_hover, delta, vparams),
                         method='RK45', max_step=cfg.dt_ctrl / 4)
        x = sol.y[:, -1]
        x[6:10] = quat_normalize(x[6:10])

        t += cfg.dt_ctrl
        t_arr[k] = t
        euler_arr[k] = np.rad2deg(quat_to_euler(x[6:10]))
        delta_arr[k] = np.rad2deg(delta)
        omega_arr[k] = np.rad2deg(omega)

    metrics = _compute_metrics(t_arr, euler_arr, delta_arr, cfg)

    return {
        "t": t_arr,
        "euler_deg": euler_arr,
        "delta_deg": delta_arr,
        "omega_deg": omega_arr,
        "metrics": metrics,
    }


def _compute_metrics(t_arr, euler_arr, delta_arr, cfg: SimConfig, band=0.02):
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
    tol = band * step_size
    err_series = np.abs(euler_arr[:, 1] - cfg.pitch_des_deg)
    outside = np.where(err_series > tol)[0]
    settling_time = t_arr[outside[-1]] if len(outside) > 0 else 0.0

    return {
        "final_roll_deg": final_roll,
        "final_pitch_deg": final_pitch,
        "roll_error_deg": roll_err,
        "pitch_error_deg": pitch_err,
        "max_delta1_deg": max_d1,
        "max_delta2_deg": max_d2,
        "gimbal_max_deg": np.rad2deg(0.0),  # filled in by caller if needed
        "settling_time_s": settling_time,
    }


if __name__ == "__main__":
    # Headless smoke test -- verifies the physics/controller integrate cleanly
    # without requiring a display. Run: python3 tvc_physics.py
    vp = VehicleParams()
    gains = ControlGains()
    cfg = SimConfig()

    result = simulate(vp, gains, cfg)
    m = result["metrics"]
    print("Headless smoke test:")
    print(f"  final roll:  {m['final_roll_deg']:+.3f} deg (target {cfg.roll_des_deg:.1f})")
    print(f"  final pitch: {m['final_pitch_deg']:+.3f} deg (target {cfg.pitch_des_deg:.1f})")
    print(f"  max |delta1|: {m['max_delta1_deg']:.2f} deg")
    print(f"  max |delta2|: {m['max_delta2_deg']:.2f} deg")
    print(f"  settling time (pitch, 2% band): {m['settling_time_s']:.2f} s")
