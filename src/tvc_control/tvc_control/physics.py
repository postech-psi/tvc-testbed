"""
physics.py (tvc_control package)
=================================
Core 6-DOF rigid-body dynamics, quaternion kinematics, actuator model, and
cascaded PID attitude controller for a coax-motor + 2-axis-gimbal VTVL vehicle.

This module is GUI-agnostic and display-agnostic -- it is the single source of
truth imported by both the standalone GUI (tvc_gui.py, at the repo root) and
the ROS2 simulator_node/controller_node in this package, and can also be
run/tested headlessly from the command line or a test script.

Physics corrections and conventions carried over from Step 1 / Step 2 notes:
  - The TVC moment arm is the AXIAL distance L from the gimbal pivot to the
    vehicle center of mass (along body +z, the nominal thrust axis) -- NOT a
    lateral offset (dx, dy). A purely lateral offset crossed with an on-axis
    thrust vector produces zero torque to leading order; dx/dy are kept only
    as optional CM-misalignment DISTURBANCE terms (default 0).
  - AXIS NAMING. Body x and y are the LATERAL axes: the two the gimbal can
    tilt thrust about. Body z is the AXIAL (thrust) axis. The controller-design
    note calls the axial channel "roll" and the lateral pair "pitch/yaw";
    the code predates that and calls the lateral pair roll/pitch (roll_des_deg
    is body x, pitch_des_deg is body y). Rather than rename five files and
    break tvc_gui / controller_node, NEW code here says lateral/axial, which
    collides with neither. Paper term -> code term: roll -> axial (M_z, tau_P),
    pitch/yaw -> lateral (M_x, M_y).
  - The 2-axis gimbal has NO authority about the thrust axis: d_cm x F has zero
    z-component when d_cm = (0,0,-L). Axial authority comes entirely from
    tau_P, the net propeller reaction torque left over when the two
    counter-rotating coax props are deliberately unbalanced. tau_P is a
    CONTROL INPUT here, not an external disturbance, and it acts along the
    gimballed thrust axis n_hat -- so tilting the gimbal leaks axial torque
    into the lateral axes (the tau_P*sin(delta) cross terms in `dynamics`).
    With the measured +/-7 deg gimbal and hover thrust that cross term is
    ~6.5% of lateral authority, and it rotates the allocation matrix by
    atan(tau_P/(T*L)) = 3.7 deg -- over half the gimbal's travel. Small, but
    not ignorable, which is why `allocate` inverts the full 2x2 rather than
    the diagonal-only special case.
  - Attitude is represented with a unit quaternion throughout (never Euler
    angles internally) specifically to avoid the +/-90 deg gimbal-lock
    singularity documented in the Step 1 guide and the TVC/6-DOF lecture notes.

References:
  - de Lajarte (2021), "Guidance, Navigation and Control of a Sounding Rocket",
    Ch. 3 (rigid-body sim) and Ch. 4 (Thrust Vectoring Control)
  - Spannagl et al. (2021), EmboRockETH gimbal geometry & cascaded PID
  - Linsen et al. (2022), Optimal TVC of an electric small-scale rocket
"""

import os
import sys

import numpy as np
from scipy.integrate import solve_ivp
from dataclasses import dataclass, field


# =============================================================================
# 0. Single-source-of-truth defaults
# =============================================================================
# The authoritative vehicle numbers live in sim/vehicle_params.yaml (see its
# header). Both this module and the Gazebo controller read them, so mass / CG /
# inertia / lever arm cannot drift apart. When that file is not reachable -- an
# installed ROS2 context without the sim/ tree alongside -- we fall back to the
# literals below so physics.py stays importable and self-contained.

def _load_vehicle_defaults():
    """Search upward from this file for sim/vehicle_params.py and load it.
    Returns a dict of defaults, or None if the sim tree is not present."""
    here = os.path.dirname(os.path.abspath(__file__))
    d = here
    for _ in range(6):
        sim_dir = os.path.join(d, "sim")
        if os.path.isfile(os.path.join(sim_dir, "vehicle_params.py")):
            if sim_dir not in sys.path:
                sys.path.insert(0, sim_dir)
            try:
                import vehicle_params as _vp
                v = _vp.load()
            except Exception:
                return None
            try:
                import actuator_maps as _am
                surface = _am.ThrustTorqueSurface(v.raw.get("thrust_torque_surface"))
                if not surface.ok:
                    surface = None
                axes = _am.GimbalAxisMap.all_from_params() or None
            except Exception:
                surface, axes = None, None
            return {
                "m": v.mass, "Ix": v.Ix, "Iy": v.Iy, "Iz": v.Iz,
                "L": v.L,
                # dx/dy stay 0 by design: per this module's convention they are
                # OPT-IN CM-misalignment disturbance terms, not the vehicle's
                # nominal state. The real <2 mm lateral CG offset lives in the
                # SDF (link pose + products of inertia); forcing it here as a
                # constant bias torque only muddies the attitude smoke test.
                "dx": 0.0, "dy": 0.0,
                "T_max": v.thrust_at_max_n,
                "gimbal_max_deg": v.gimbal_max_deg,
                "gimbal_rate_max_deg": v.gimbal_rate_max_deg,
                "gimbal_deadtime_s": v.gimbal_deadtime_s,
                "k_moment": v.moment_constant,
                "tau_p_max": v.tau_p_max_nm,
                "surface": surface,
                "gimbal_axes": axes,
                "g": v.g,
            }
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return None


# Literal fallbacks used only when sim/vehicle_params.yaml cannot be found.
_D = _load_vehicle_defaults() or {
    "m": 1.328, "Ix": 0.022616, "Iy": 0.022581, "Iz": 0.001957,
    "L": 0.2111, "dx": 0.0, "dy": 0.0,
    "T_max": 20.0, "gimbal_max_deg": 7.0, "gimbal_rate_max_deg": 180.0,
    "gimbal_deadtime_s": 0.030, "k_moment": 0.016, "tau_p_max": None,
    "surface": None, "gimbal_axes": None,
    "g": 9.81,
}
_D.setdefault("surface", None)
_D.setdefault("gimbal_axes", None)


# =============================================================================
# 1. Parameter containers
# =============================================================================

@dataclass
class VehicleParams:
    """Physical vehicle parameters. Angles are stored in DEGREES for the GUI's
    convenience and converted to radians via properties where dynamics need them.

    Defaults come from sim/vehicle_params.yaml (the single source of truth), so
    editing that file re-parameterizes this sim without touching code here."""

    m: float = _D["m"]              # kg, total mass
    Ix: float = _D["Ix"]            # kg*m^2, roll inertia (body x)
    Iy: float = _D["Iy"]            # kg*m^2, pitch inertia (body y)
    Iz: float = _D["Iz"]            # kg*m^2, yaw/spin inertia (body z)

    L: float = _D["L"]              # m, AXIAL lever arm: gimbal pivot -> CM
    dx: float = _D["dx"]            # m, lateral CM misalignment (disturbance)
    dy: float = _D["dy"]            # m, lateral CM misalignment (disturbance)

    T_max: float = _D["T_max"]      # N, max thrust (combined coax unit)
    T_min: float = 5.0              # N, min thrust (idle; must stay > 0 for allocation)

    gimbal_max_deg: float = _D["gimbal_max_deg"]        # deg, mechanical gimbal limit (each axis)
    gimbal_rate_max_deg: float = _D["gimbal_rate_max_deg"]  # deg/s, servo slew rate
    gimbal_deadtime_s: float = _D["gimbal_deadtime_s"]  # s, servo transport delay

    # Axial (thrust-axis) reaction torque authority. k_moment is the coax
    # drag-torque / thrust ratio, so tau_P = k_moment * (T2 - T1) for a thrust
    # split of (T2 - T1) newtons. tau_p_max is an optional MEASURED cap; when
    # None the allocator uses only the split headroom the thrust command leaves.
    k_moment: float = _D["k_moment"]        # m, drag-torque / thrust
    tau_p_max: float = _D["tau_p_max"]      # N*m or None

    # Bench-measured (PWM A, PWM B) -> (thrust, tau_P) surface, or None. When
    # present it REPLACES the k_moment model everywhere authority is computed;
    # k_moment survives only for the Gazebo plugin, which cannot take a surface.
    surface: object = _D["surface"]
    # Per-axis measured gimbal maps {'inner': .., 'outer': ..}, or None.
    # default_factory because a dict default is mutable and dataclasses reject
    # it -- the factory hands out the same shared, read-only spec object.
    gimbal_axes: object = field(default_factory=lambda: _D["gimbal_axes"])

    g: float = _D["g"]              # m/s^2

    @property
    def gimbal_max(self):
        return np.deg2rad(self.gimbal_max_deg)

    @property
    def gimbal_rate_max(self):
        return np.deg2rad(self.gimbal_rate_max_deg)

    # --- per-axis travel ------------------------------------------------------
    # delta1 is the pitch-plane deflection, carried by the INNER ring; delta2 is
    # the roll-plane one, carried by the OUTER ring (base_link -> roll joint ->
    # outer ring -> pitch joint -> inner ring -> rotors). Neither axis is
    # symmetric about its own neutral, and they differ from each other, so the
    # limits are vectors rather than one scalar. Falls back to the symmetric
    # +/-gimbal_max_deg when the measured block is absent.

    def _axis(self, name):
        return (self.gimbal_axes or {}).get(name)

    @property
    def delta_min(self):
        inner, outer = self._axis("inner"), self._axis("outer")
        return np.deg2rad([inner.min_deg if inner else -self.gimbal_max_deg,
                           outer.min_deg if outer else -self.gimbal_max_deg])

    @property
    def delta_max(self):
        inner, outer = self._axis("inner"), self._axis("outer")
        return np.deg2rad([inner.max_deg if inner else self.gimbal_max_deg,
                           outer.max_deg if outer else self.gimbal_max_deg])

    @property
    def delta_rate_max(self):
        inner, outer = self._axis("inner"), self._axis("outer")
        return np.deg2rad([inner.rate_max_deg if inner else self.gimbal_rate_max_deg,
                           outer.rate_max_deg if outer else self.gimbal_rate_max_deg])


@dataclass
class ControlGains:
    """Cascaded PID gains: outer angle loop + inner rate loop.

    LATERAL (body x, y) gains are shared between the two axes -- Ix and Iy
    differ by 0.15% on this airframe, so treating them as symmetric is exact
    to within the mass-budget uncertainty.

    AXIAL (body z) gains are SEPARATE and much smaller, and this is not a
    tuning preference -- it is forced by the airframe. Iz = 0.00196 kg*m^2 is
    11.6x smaller than Ix, so the same torque produces 11.6x the angular
    acceleration about z. Sharing one gain set makes the axial channel
    violently underdamped (or the lateral channel uselessly slow). Also the
    two channels have different actuators and therefore different lags: the
    lateral axes go through a 30 ms servo deadtime, the axial channel through
    the motor time constant.
    """

    kp_angle: float = 4.0      # outer loop: angle error -> rate setpoint
    kp_rate: float = 0.02      # inner loop: rate error -> torque
    ki_rate: float = 0.002
    kd_rate: float = 0.004
    i_limit: float = 0.5       # integrator clamp (anti-windup)

    # --- axial channel (body z / tau_P), scaled by Iz/Ix ~= 1/11.6 ---
    kp_angle_axial: float = 4.0
    kp_rate_axial: float = 0.0018
    ki_rate_axial: float = 0.00018
    kd_rate_axial: float = 0.00035
    i_limit_axial: float = 0.05

    # --- altitude cascade: outer P (m -> m/s), inner PID (m/s -> m/s^2) ---
    kp_alt: float = 1.5        # m/s of climb demand per m of altitude error
    kp_vz: float = 4.0         # m/s^2 per m/s
    ki_vz: float = 1.0
    kd_vz: float = 0.2
    i_limit_vz: float = 4.0    # m/s^2 of integral authority
    vz_max: float = 2.0        # m/s, climb/descent rate limit


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
    """Rate-limited servo model for the 2-axis gimbal, with transport delay.

    Two separate lags, often confused:
      RATE LIMIT (gimbal_rate_max) -- how fast the servo can slew once moving.
      DEADTIME   (gimbal_deadtime) -- how long after a command arrives before
                                      the servo moves at all.
    Deadtime is the one that costs phase margin: it is pure delay, so it eats
    gimbal_deadtime * omega radians of phase at every frequency and cannot be
    compensated by a lead term the way a first-order lag can. At 30 ms it is
    worth 17 deg of phase at the 10 rad/s lateral crossover, which is why the
    lateral gains cannot simply be raised until the response looks fast.

    The delay is modeled as a FIFO of commands rather than a filter, because
    that is what a serial servo bus actually does -- the command sits in a
    queue, then executes in full.
    """

    def __init__(self, params: VehicleParams):
        self.p = params
        self.delta = np.zeros(2)   # current [delta1, delta2], rad
        self._pending = []         # command FIFO, oldest first

    def reset(self):
        self.delta = np.zeros(2)
        self._pending = []

    def update(self, delta_cmd, dt):
        # Per-axis, asymmetric stops and per-axis slew: the inner ring reaches
        # 403 deg/s and the outer only 235, so a symmetric shared limit either
        # slows the inner axis or lets the outer one move faster than it can.
        delta_cmd = np.clip(delta_cmd, self.p.delta_min, self.p.delta_max)

        # Delay by round(deadtime/dt) control steps. After appending, the FIFO
        # holds the last n+1 commands, so popping the oldest yields the command
        # issued n steps ago. Before the queue has filled (first n steps) there
        # is no command to execute yet, so the servo holds position.
        n_delay = int(round(self.p.gimbal_deadtime_s / dt)) if dt > 0 else 0
        self._pending.append(np.asarray(delta_cmd, dtype=float))
        if len(self._pending) > n_delay:
            target = self._pending.pop(0)
        else:
            target = self.delta.copy()

        max_step = self.p.delta_rate_max * dt
        step = np.clip(target - self.delta, -max_step, max_step)
        self.delta = self.delta + step
        return self.delta.copy()


# =============================================================================
# 4. Rigid-body dynamics (Newton-Euler), held control input over dt (ZOH)
# =============================================================================

def thrust_axis(delta):
    """Unit vector n_hat along the gimballed thrust axis, in the body frame.

    Exact (not small-angle). Sign convention matches Gazebo / sim/hover.py --
    n_hat = R_x(d2) R_y(d1) [0,0,1] -- so gimbal signs and gains are portable
    between the two sims. d1 deflects in the pitch plane (about body y),
    d2 in the roll plane (about body x).
    """
    d1, d2 = delta
    return np.array([
        np.sin(d1),
        -np.sin(d2) * np.cos(d1),
        np.cos(d1) * np.cos(d2),
    ])


def dynamics(t, x, T, delta, params: VehicleParams, tau_p=0.0):
    """
    x = [r(3), v(3), q(4), omega(3)]  (13 states)
    T:      scalar thrust magnitude [N]
    delta:  [delta1, delta2] gimbal deflection angles [rad]
    tau_p:  net propeller reaction torque about the THRUST AXIS [N*m], from the
            deliberate imbalance between the two counter-rotating coax props.
            Defaults to 0 so every existing caller integrates exactly as before.
    """
    v = x[3:6]
    q = quat_normalize(x[6:10])
    omega = x[10:13]

    n_hat = thrust_axis(delta)
    f_body = T * n_hat

    R = quat_to_rotmat(q)
    a_inertial = (R @ f_body) / params.m + np.array([0, 0, -params.g])

    # AXIAL lever arm (gimbal pivot -> CM along body z), plus optional lateral
    # misalignment disturbance terms dx, dy (normally ~0 for a balanced vehicle).
    d_cm = np.array([params.dx, params.dy, -params.L])
    # tau_P acts along the PROP AXIS, not along body z: the props are mounted on
    # the gimbal, so their reaction torque tilts with the thrust. That is what
    # couples the axial channel into the lateral axes -- expanding gives
    #     M_x = -T*L*sin(d2)cos(d1) + tau_P*sin(d1)
    #     M_y = -T*L*sin(d1)        - tau_P*sin(d2)cos(d1)
    #     M_z =                       tau_P*cos(d1)cos(d2)
    # Dropping the tau_P terms (as this module did before) is exact only while
    # the axial channel is unused; it under-predicts lateral torque by ~6.5% of
    # full authority once tau_P is commanded.
    tau = np.cross(d_cm, f_body) + tau_p * n_hat

    I = np.diag([params.Ix, params.Iy, params.Iz])
    I_inv = np.diag([1/params.Ix, 1/params.Iy, 1/params.Iz])
    omega_dot = I_inv @ (tau - np.cross(omega, I @ omega))

    q_dot = quat_kinematics(q, omega)

    return np.concatenate([v, a_inertial, q_dot, omega_dot])


# =============================================================================
# 4b. Control allocation: (M_x, M_y, M_z, T) -> (delta1, delta2, T1, T2)
# =============================================================================
# Inverting the moment map from `dynamics`. Three stages, in this order, because
# the stages are not independent:
#
#   1. AXIAL.   M_z comes only from tau_P (the gimbal has no authority about
#               the thrust axis), so tau_P* = M_z* directly -- then saturate it
#               against what the thrust command actually leaves available.
#   2. LATERAL. With tau_P known, solve the 2x2 for the gimbal angles. The
#               off-diagonal tau_P terms rotate the effective torque axis by
#               atan(tau_P/(T*L)), so ignoring them aims the gimbal in the
#               wrong direction by that angle -- 3.7 deg at hover with a
#               0.18 N.m axial command, against 7 deg of travel.
#   3. MOTORS.  (T, tau_P) -> per-rotor thrusts.
#
# PRIORITY when the command is outside the feasible set: total thrust wins.
# Losing altitude ends the test; losing some axial authority does not. tau_P is
# clipped to the headroom the thrust command leaves, and the gimbal pair is
# scaled down as a PAIR (preserving torque DIRECTION) rather than clipped per
# axis, which would swing the resulting torque vector sideways.


@dataclass
class Allocation:
    """What the allocator produced, plus which limits it hit.

    The saturation flags exist for anti-windup: an integrator that keeps
    charging while its actuator is pinned is the classic way to turn a brief
    saturation into a long overshoot, so the controllers freeze the relevant
    integrator whenever the matching flag is set."""

    delta_cmd: np.ndarray = field(default_factory=lambda: np.zeros(2))
    tau_p: float = 0.0
    T_cmd: float = 0.0
    T1: float = 0.0                 # per-rotor thrust, N (upper prop)
    T2: float = 0.0                 # per-rotor thrust, N (lower prop)
    gimbal_saturated: bool = False
    axial_saturated: bool = False
    thrust_saturated: bool = False


def axial_headroom(T, params: VehicleParams):
    """Max |tau_P| available at total thrust T [N] -> N*m.

    tau_P is bought with a thrust SPLIT between the props, and the split has to
    fit inside the per-rotor limits: with T1 = (T-s)/2 and T2 = (T+s)/2 both in
    [0, T_max/2], the split s is bounded by min(T, T_max - T). So axial
    authority is largest at half throttle and vanishes at both idle and full
    throttle -- at hover (T = 13.03 N of 20 N) the binding side is the top one,
    leaving 6.97 N of split, i.e. 0.112 N*m.

    A measured cap (params.tau_p_max) is applied on top when available; see the
    tau_p_max_nm note in sim/vehicle_params.yaml for why the bench number and
    this analytic number currently disagree.
    """
    if params.surface is not None:
        tau = params.surface.max_torque_at(T)
    else:
        split_max = max(min(T, params.T_max - T), 0.0)
        tau = params.k_moment * split_max
    if params.tau_p_max is not None:
        tau = min(tau, params.tau_p_max)
    return tau


def axial_limits(T, params: VehicleParams):
    """Signed (min, max) tau_P [N*m] at total thrust T -- the asymmetric truth.

    axial_headroom() returns the symmetric figure guaranteed in both directions,
    which is the right number for sizing gains and for quoting authority. This
    one returns the actual interval, which is what the allocator should clamp
    against: the coax wake makes one direction stronger than the other, and
    throwing that away costs real control authority in the strong direction.
    """
    if params.surface is not None:
        lo, hi = params.surface.torque_limits_at(T)
    else:
        cap = axial_headroom(T, params)
        lo, hi = -cap, cap
    if params.tau_p_max is not None:
        lo = max(lo, -params.tau_p_max)
        hi = min(hi, params.tau_p_max)
    return lo, hi


def mix_motors(T, tau_p, params: VehicleParams):
    """(total thrust, axial torque) -> (T1, T2) per-rotor thrusts [N].

    tau_P = k_moment * (T2 - T1): equal and opposite prop drag torques cancel,
    and what survives is proportional to the imbalance.

    This is the ANALYTIC path. The real vehicle needs the measured two-input
    surfaces T = f_T(u1, u2), tau_P = f_Q(u1, u2), because the lower prop runs
    inside the upper prop's wake and the two commands are therefore coupled --
    a separable per-motor model does not hold. When that bench data lands in
    sim/vehicle_params.yaml's PWM maps, this function is the seam to replace
    (a 2-D Newton solve or a precomputed inverse lookup); nothing above it
    changes, because everything above speaks in (T, tau_P).
    """
    if params.surface is not None:
        # Measured path: solve the real surface for the two PWM commands, then
        # report what those commands actually produce per prop. The split is no
        # longer proportional to tau_P -- the surface's own gradients are
        # unequal (dQ/db = +0.087 against dQ/da = -0.0795), which is the wake
        # asymmetry, not fit noise.
        pwm_a, pwm_b, T_ach, _ = params.surface.inverse(T, tau_p)
        # Per-prop thrust is not separately measured (the load cell reads the
        # pair), so split the ACHIEVED total by the commands' share of it. This
        # is only used for logging and for Gazebo's two-rotor plugin; nothing in
        # the moment model depends on the individual values.
        share = (pwm_b - params.surface.pwm_min) + 1e-9
        share_a = (pwm_a - params.surface.pwm_min) + 1e-9
        f = share_a / (share_a + share)
        return f * T_ach, (1.0 - f) * T_ach

    T = float(np.clip(T, 0.0, params.T_max))
    split_max = max(min(T, params.T_max - T), 0.0)
    split = float(np.clip(tau_p / params.k_moment, -split_max, split_max))
    return 0.5 * (T - split), 0.5 * (T + split)


def _lateral_gimbal(M_xy, T, tau_p, params: VehicleParams, iters=3):
    """Solve M_x, M_y for (delta1, delta2) -- the 2x2 inverse of §5, refined.

    Seed: the small-angle inverse. With s1~d1, s2*c1~d2 the map is

        [M_x]   [ tau_P   -T*L ] [d1]
        [M_y] = [ -T*L   -tau_P] [d2]

    whose determinant is -(tau_P^2 + (T*L)^2) < 0 always, so the inverse exists
    for any thrust and any axial command -- there is no allocation singularity
    to guard against, only actuator limits.

    Refine: three Newton steps on the exact trigonometric map. At 7 deg the
    small-angle error is only ~0.3%, so this is cheap insurance rather than a
    necessity -- but it costs microseconds and removes the approximation from
    the argument entirely, which matters if the gimbal travel is ever revised
    upward.
    """
    TL = T * params.L
    D = tau_p ** 2 + TL ** 2
    Mx, My = float(M_xy[0]), float(M_xy[1])
    d = np.array([(tau_p * Mx - TL * My) / D,
                  (-TL * Mx - tau_p * My) / D])

    for _ in range(iters):
        d1, d2 = d
        s1, c1, s2, c2 = np.sin(d1), np.cos(d1), np.sin(d2), np.cos(d2)
        g = np.array([-TL * s2 * c1 + tau_p * s1 - Mx,
                      -TL * s1 - tau_p * s2 * c1 - My])
        J = np.array([[TL * s2 * s1 + tau_p * c1, -TL * c2 * c1],
                      [-TL * c1 + tau_p * s2 * s1, -tau_p * c2 * c1]])
        try:
            step = np.linalg.solve(J, g)
        except np.linalg.LinAlgError:
            break                       # keep the small-angle seed
        if not np.all(np.isfinite(step)):
            break
        d = d - step
    return d


def allocate(M_des, T_des, params: VehicleParams):
    """Desired body moment (3,) and total thrust -> Allocation.

    M_des is (M_x, M_y, M_z) in N*m: the first two lateral, the third axial.
    """
    M_des = np.asarray(M_des, dtype=float)

    # --- stage 0: thrust has priority; everything else works with what's left
    T_cmd = float(np.clip(T_des, params.T_min, params.T_max))
    thrust_sat = not np.isclose(T_cmd, T_des)

    # --- stages 1 & 2: axial, then lateral, iterated twice.
    # M_z can only come from tau_P, but what reaches body z is
    # tau_P*cos(d1)*cos(d2), not tau_P -- and d1, d2 are not known until the
    # lateral stage has run, which in turn needs tau_P. One extra pass closes
    # that loop: solve the gimbal against the first tau_P guess, then divide
    # the guess by the cosine loss those angles imply and re-solve. Skipping it
    # leaves M_z short by tau_P*(1 - cos*cos), which at the 7 deg stops is 1.5%
    # of the axial command -- small, but it is a systematic bias, not noise, so
    # the axial integrator would otherwise spend the whole flight paying it off.
    q_lo, q_hi = axial_limits(T_cmd, params)
    axial_sat = M_des[2] < q_lo - 1e-12 or M_des[2] > q_hi + 1e-12
    tau_p = float(np.clip(M_des[2], q_lo, q_hi))
    delta = _lateral_gimbal(M_des[:2], T_cmd, tau_p, params)
    for _ in range(2):
        loss = np.cos(delta[0]) * np.cos(delta[1])
        if loss < 1e-6:
            break
        tau_p = float(np.clip(M_des[2] / loss, q_lo, q_hi))
        delta = _lateral_gimbal(M_des[:2], T_cmd, tau_p, params)

    # Scale the PAIR down on saturation so the torque direction survives.
    # Clipping each axis independently would rotate the commanded torque vector
    # toward the corner of the square, which is exactly the wrong thing to do
    # while recovering from an off-axis upset.
    # With asymmetric stops the scale factor is per-axis-per-sign: find the
    # largest s in (0, 1] that keeps BOTH axes inside their own travel.
    lo, hi = params.delta_min, params.delta_max
    scale = 1.0
    for k in range(2):
        if delta[k] > hi[k] > 0:
            scale = min(scale, hi[k] / delta[k])
        elif delta[k] < lo[k] < 0:
            scale = min(scale, lo[k] / delta[k])
    gimbal_sat = scale < 1.0 - 1e-12
    if gimbal_sat:
        delta = delta * scale
    delta = np.clip(delta, lo, hi)

    T1, T2 = mix_motors(T_cmd, tau_p, params)

    return Allocation(delta_cmd=delta, tau_p=tau_p, T_cmd=T_cmd, T1=T1, T2=T2,
                      gimbal_saturated=gimbal_sat, axial_saturated=axial_sat,
                      thrust_saturated=thrust_sat)


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

    def update(self, err, dt, freeze=False):
        """freeze=True holds the integrator (conditional anti-windup).

        Set it whenever the actuator this PID drives is saturated: the output
        cannot follow anyway, so continuing to integrate only builds a charge
        that has to be paid back as overshoot once the limit clears. The
        proportional and derivative terms still respond."""
        if not freeze:
            self.integral = np.clip(self.integral + err*dt,
                                    -self.i_limit, self.i_limit)
        deriv = (err - self.prev_err) / dt if dt > 0 else 0.0
        self.prev_err = err
        return self.kp*err + self.ki*self.integral + self.kd*deriv


class AttitudeController:
    """
    Three-axis cascade:
        attitude error -> desired body rate  (outer, P)
        rate error     -> desired moment     (inner, PID)
        desired moment -> actuator commands  (allocation, §4b)

    All three axes are closed here. The LATERAL pair (body x, y) is driven by
    the gimbal; the AXIAL channel (body z) is driven by tau_P, the differential
    prop reaction torque. See the axis-naming note in the module docstring:
    roll_des/pitch_des are the lateral setpoints, axial_des is the one the
    controller-design note calls roll.

    The axial channel used to be left at zero here on the grounds that the
    gimbal cannot produce M_z. That premise is right and the conclusion was
    wrong: the gimbal cannot, but the props can, and on this airframe they
    can do it hard -- Iz is 11.6x smaller than Ix, so 0.11 N*m of tau_P buys
    56 rad/s^2 about z against 15 rad/s^2 about x at full lateral authority.
    """

    def __init__(self, params: VehicleParams, gains: ControlGains):
        self.p = params
        self.gains = gains
        self.pid_roll_angle = PID(kp=gains.kp_angle, ki=0.0, kd=0.0)
        self.pid_pitch_angle = PID(kp=gains.kp_angle, ki=0.0, kd=0.0)
        self.pid_axial_angle = PID(kp=gains.kp_angle_axial, ki=0.0, kd=0.0)
        self.pid_roll_rate = PID(kp=gains.kp_rate, ki=gains.ki_rate, kd=gains.kd_rate, i_limit=gains.i_limit)
        self.pid_pitch_rate = PID(kp=gains.kp_rate, ki=gains.ki_rate, kd=gains.kd_rate, i_limit=gains.i_limit)
        self.pid_axial_rate = PID(kp=gains.kp_rate_axial, ki=gains.ki_rate_axial,
                                  kd=gains.kd_rate_axial, i_limit=gains.i_limit_axial)
        # Last allocation, exposed so callers that need the motor commands (or
        # the saturation flags, for an outer loop's own anti-windup) can read
        # them without changing update()'s return type.
        self.last_alloc = Allocation()

    def reset(self):
        for pid in (self.pid_roll_angle, self.pid_pitch_angle, self.pid_axial_angle,
                    self.pid_roll_rate, self.pid_pitch_rate, self.pid_axial_rate):
            pid.reset()
        self.last_alloc = Allocation()

    def desired_moment(self, q, omega, roll_des, pitch_des, axial_des, dt):
        """Attitude + rate cascade -> desired body moment (M_x, M_y, M_z) [N*m].

        Euler angles are used for the OUTER loop error only. That is safe here
        and is not the gimbal-lock trap the module docstring warns about: the
        trap is propagating kinematics through Euler angles (this module never
        does -- see quat_kinematics), whereas an error measured at the few
        degrees this vehicle actually flies is nowhere near the +/-90 deg
        singularity. If the envelope ever grows, swap in the quaternion error
        2*sgn(qe_w)*qe_v; it agrees with this to first order.
        """
        roll, pitch, axial = quat_to_euler(q)
        sat = self.last_alloc

        roll_rate_des = self.pid_roll_angle.update(roll_des - roll, dt)
        pitch_rate_des = self.pid_pitch_angle.update(pitch_des - pitch, dt)
        axial_rate_des = self.pid_axial_angle.update(axial_des - axial, dt)

        # Freeze the lateral integrators when the gimbal is on its stops, and
        # the axial one when tau_P is capped -- see PID.update / Allocation.
        tau_x = self.pid_roll_rate.update(roll_rate_des - omega[0], dt,
                                          freeze=sat.gimbal_saturated)
        tau_y = self.pid_pitch_rate.update(pitch_rate_des - omega[1], dt,
                                           freeze=sat.gimbal_saturated)
        tau_z = self.pid_axial_rate.update(axial_rate_des - omega[2], dt,
                                           freeze=sat.axial_saturated)
        return np.array([tau_x, tau_y, tau_z])

    def update(self, q, omega, roll_des, pitch_des, T_des, dt, axial_des=0.0):
        """Backward-compatible entry point: returns the gimbal command only.

        The full allocation -- tau_P, per-rotor thrusts, saturation flags --
        lands on self.last_alloc. Kept this shape because controller_node and
        the GUI call update() and use the return value directly as the servo
        command; adding a tuple return would have broken both.
        """
        M = self.desired_moment(q, omega, roll_des, pitch_des, axial_des, dt)
        self.last_alloc = allocate(M, T_des, self.p)
        return self.last_alloc.delta_cmd.copy()


class AltitudeController:
    """Cascade P (altitude -> climb rate) + PID (climb rate -> accel) -> thrust.

    The tilt compensation is the part that matters. Vertical dynamics are

        m*z_ddot = T * (R(q) n_hat) . z_inertial - m*g

    so the useful fraction of thrust is cos(theta), theta being the angle
    between the thrust axis and vertical. Dividing the thrust command by that
    same projection is FEEDFORWARD: it cancels the altitude sag from a tilt at
    the instant the tilt happens, instead of waiting for the altitude error to
    grow large enough for feedback to notice. Without it, every attitude
    maneuver shows up as an altitude dip.

    The divisor is floored (cos_min) so a large or briefly bad attitude
    estimate cannot demand unbounded thrust; at the floor the vehicle simply
    accepts the sag, which is the safe failure.
    """

    def __init__(self, params: VehicleParams, gains: ControlGains, cos_min=0.7,
                 tilt_compensation=True):
        self.p = params
        self.gains = gains
        self.cos_min = cos_min
        # tilt_compensation=False divides by 1 instead of the projection. Only
        # useful for the A/B in sim/validate_control.py that shows what the
        # feedforward is actually buying; flying without it is strictly worse.
        self.tilt_compensation = tilt_compensation
        self.pid_vz = PID(kp=gains.kp_vz, ki=gains.ki_vz, kd=gains.kd_vz,
                          i_limit=gains.i_limit_vz)
        self.thrust_saturated = False

    def reset(self):
        self.pid_vz.reset()
        self.thrust_saturated = False

    def update(self, z, vz, q, delta, z_des, dt):
        """Altitude state + current gimbal -> total thrust command [N]."""
        vz_des = np.clip(self.gains.kp_alt * (z_des - z),
                         -self.gains.vz_max, self.gains.vz_max)
        # Anti-windup: hold the integrator whenever the previous command was
        # clipped by the feasible set (§4b priority: thrust first, but "first"
        # still ends at T_max).
        az_des = self.pid_vz.update(vz_des - vz, dt, freeze=self.thrust_saturated)

        # Projection of the (gimballed, then rotated) thrust axis onto vertical.
        if self.tilt_compensation:
            proj = float(quat_to_rotmat(q) @ thrust_axis(delta) @ np.array([0, 0, 1.0]))
            proj = max(proj, self.cos_min)
        else:
            proj = 1.0

        T_cmd = self.p.m * (self.p.g + az_des) / proj
        T_clipped = float(np.clip(T_cmd, self.p.T_min, self.p.T_max))
        self.thrust_saturated = not np.isclose(T_clipped, T_cmd)
        return T_clipped


# =============================================================================
# 6. Simulation driver
# =============================================================================

def simulate(vparams: VehicleParams, gains: ControlGains, cfg: SimConfig):
    """
    Run a closed-loop simulation with zero-order-hold control.

    Three-axis attitude always; altitude hold only when cfg.altitude_hold is
    set (otherwise thrust is pinned at hover, T = mg, so every pre-existing
    run reproduces its old numbers exactly).

    Returns a dict of numpy arrays: t, euler_deg, delta_deg, omega_deg, quat,
    pos, thrust_N, tau_p_Nm, motor_N, plus scalar metrics under 'metrics'.
    """
    gimbal = GimbalActuator(vparams)
    controller = AttitudeController(vparams, gains)
    alt_ctl = (AltitudeController(vparams, gains,
                                 tilt_compensation=cfg.tilt_compensation)
               if cfg.altitude_hold else None)

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

    t = 0.0
    delta = np.zeros(2)
    for k in range(n_steps):
        q = quat_normalize(x[6:10])
        omega = x[10:13]

        # Altitude first: the allocator needs T before it can size the axial
        # headroom or the gimbal angles (§4b priority). Compensation uses the
        # gimbal position actually reached, not the one just commanded.
        if alt_ctl is not None:
            T_cmd = alt_ctl.update(x[2], x[5], q, delta, cfg.z_des, cfg.dt_ctrl)
            # The altitude loop clips to [T_min, T_max] itself, so by the time
            # allocate() sees the command it is already in range and its own
            # thrust_saturated flag can never fire. Carry the upstream flag
            # forward or the logs claim the vehicle never hit its thrust limit.
            alt_thrust_sat = alt_ctl.thrust_saturated
        else:
            T_cmd = T_hover
            alt_thrust_sat = False

        delta_cmd = controller.update(q, omega, roll_des, pitch_des,
                                      T_cmd, cfg.dt_ctrl, axial_des=axial_des)
        alloc = controller.last_alloc
        delta = gimbal.update(delta_cmd, cfg.dt_ctrl)

        sol = solve_ivp(dynamics, [t, t + cfg.dt_ctrl], x,
                         args=(alloc.T_cmd, delta, vparams, alloc.tau_p),
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
        thrust_arr[k] = alloc.T_cmd
        tau_p_arr[k] = alloc.tau_p
        motor_arr[k] = (alloc.T1, alloc.T2)
        sat_arr[k] = (alloc.gimbal_saturated, alloc.axial_saturated,
                      alloc.thrust_saturated or alt_thrust_sat)

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
    tol = band * step_size
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


if __name__ == "__main__":
    # Headless smoke test -- verifies the physics/controller integrate cleanly
    # without requiring a display. Run: python3 src/tvc_control/tvc_control/physics.py
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
