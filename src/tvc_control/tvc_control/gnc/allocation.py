"""
Control allocation: desired moment + thrust -> actuator commands.
================================================================================
Moved verbatim from physics.py. The layer exists because this vehicle is
over-actuated in the sense that matters: the high-level loops ask for a virtual
control effort (a body moment and a total thrust) and something has to decide
which effectors produce it, subject to a feasible set that is neither a box nor
symmetric. Keeping that decision in its own layer -- rather than folded into the
attitude controller -- is the standard hierarchy (Johansen & Fossen 2013) and is
what lets the same controller drive a different effector suite later.
"""

import numpy as np
from dataclasses import dataclass, field

from .params import VehicleParams
from .mathx import thrust_axis


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
