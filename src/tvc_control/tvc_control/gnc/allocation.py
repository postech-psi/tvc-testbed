"""
Control allocation: desired moment + thrust -> actuator commands.
================================================================================
The layer exists because this vehicle is over-actuated in the sense that
matters: the high-level loops ask for a virtual control effort (a body moment
and a total thrust) and something has to decide which effectors produce it,
subject to a feasible set that is neither a box nor symmetric. Keeping that
decision in its own layer -- rather than folding it into the attitude
controller -- is the standard hierarchy (Johansen & Fossen 2013) and is what
lets the same controller drive a different effector suite later.

    motion control  ->  virtual control effort  ->  ALLOCATION  ->  effectors
    attitude.py         (M_x, M_y, M_z) and T                      delta, u_a, u_b

WHAT MAKES THIS ONE INTERESTING
    The feasible set is not a box. tau_P is bought with a thrust SPLIT between
    the two props, so roll authority is a function of total thrust: it peaks
    near half throttle and vanishes at both idle and the ceiling. It is also
    ASYMMETRIC, because the lower prop runs in the upper prop's wake -- at hover
    the reachable interval is [-0.089, +0.147] N*m, not +/- anything.

    And the axes are coupled through the gimbal. The props ride ON the gimbal,
    so their reaction torque tilts with the thrust and leaks into the lateral
    axes; conversely only tau_P*cos(d1)*cos(d2) of the commanded roll torque
    reaches body z. Both couplings are handled by the two-pass solve in
    allocate().

Derivation: docs/3-THEORY.md, section 4.
"""

import math
from dataclasses import dataclass, field

from .params import VehicleParams
from .mathx import clamp, isclose


@dataclass
class Allocation:
    """What the allocator produced, plus which limits it hit.

    The saturation flags exist for anti-windup: an integrator that keeps
    charging while its actuator is pinned is the classic way to turn a brief
    saturation into a long overshoot, so the controllers freeze the relevant
    integrator whenever the matching flag is set."""

    delta_cmd: tuple = field(default_factory=lambda: (0.0, 0.0))
    tau_p: float = 0.0
    T_cmd: float = 0.0
    T1: float = 0.0                 # per-rotor thrust, N (upper prop)
    T2: float = 0.0                 # per-rotor thrust, N (lower prop)
    # Normalized [0,1] motor commands against the measured surface's own
    # pwm_min/pwm_max. THESE are what a HAL sends; T1/T2 are a derived split
    # kept for diagnostic logging. Zero when no surface
    # is loaded (the analytic fallback has no command coordinate to normalize).
    u_a: float = 0.0
    u_b: float = 0.0
    gimbal_saturated: bool = False
    roll_saturated: bool = False
    thrust_saturated: bool = False


def roll_headroom(T, params: VehicleParams):
    """Max |tau_P| available at total thrust T [N] -> N*m.

    This is the ANALYTIC fallback, used only when no measured surface is
    loaded. tau_P is bought with a thrust SPLIT between the props, and the split
    must fit the per-rotor limits: with T1 = (T-s)/2 and T2 = (T+s)/2 both in
    [0, T_max/2], the split s is bounded by min(T, T_max - T). So roll authority
    is largest at half throttle and vanishes at both idle and full throttle.

    With the measured surface loaded -- the normal case -- the figure comes off
    the surface instead, and at hover it is 0.0888 N*m.

    A measured cap (params.tau_p_max) is applied on top when available; see the
    tau_p_max_nm note in vehicle_params.yaml for why the bench headline figure
    and what is actually available at hover are different numbers.
    """
    if params.surface is not None:
        tau = params.surface.max_torque_at(T)
    else:
        split_max = max(min(T, params.T_max - T), 0.0)
        tau = params.k_moment * split_max
    if params.tau_p_max is not None:
        tau = min(tau, params.tau_p_max)
    return tau


def roll_limits(T, params: VehicleParams):
    """Signed (min, max) tau_P [N*m] at total thrust T -- the asymmetric truth.

    roll_headroom() returns the symmetric figure guaranteed in both directions,
    which is the right number for sizing gains and for quoting authority. This
    one returns the actual interval, which is what the allocator should clamp
    against: the coax wake makes one direction stronger than the other, and
    throwing that away costs real control authority in the strong direction.
    """
    if params.surface is not None:
        lo, hi = params.surface.torque_limits_at(T)
    else:
        cap = roll_headroom(T, params)
        lo, hi = -cap, cap
    if params.tau_p_max is not None:
        lo = max(lo, -params.tau_p_max)
        hi = min(hi, params.tau_p_max)
    return lo, hi


def motor_setpoint(T, tau_p, params: VehicleParams):
    """(T, tau_P) -> (u_a, u_b, T1, T2).

    u_a/u_b are the NORMALIZED [0,1] motor commands -- the actual actuator
    output, and the coordinate the bench surface is defined on. T1/T2 are the
    per-prop thrust split derived from them.

    Callers want the COMMANDS. T1/T2 are derived from them and are used only
    for diagnostic logging; nothing in the moment model
    depends on the individual values, because the load cell measured the pair.
    """
    if params.surface is not None:
        # Measured path: solve the real surface for the two PWM commands, then
        # report what those commands actually produce per prop. The split is no
        # longer proportional to tau_P -- the surface's own gradients are
        # unequal (dQ/db = +0.087 against dQ/da = -0.0795), which is the wake
        # asymmetry, not fit noise.
        pwm_a, pwm_b, T_ach, _ = params.surface.inverse(T, tau_p)
        span = params.surface.pwm_max - params.surface.pwm_min
        u_a = (pwm_a - params.surface.pwm_min) / span
        u_b = (pwm_b - params.surface.pwm_min) / span
        # Per-prop thrust is not separately measured (the load cell reads the
        # pair), so split the ACHIEVED total by the commands' share of it. This
        # is only used for diagnostic logging; nothing in
        # the moment model depends on the individual values.
        share = (pwm_b - params.surface.pwm_min) + 1e-9
        share_a = (pwm_a - params.surface.pwm_min) + 1e-9
        f = share_a / (share_a + share)
        return u_a, u_b, f * T_ach, (1.0 - f) * T_ach

    T = clamp(T, 0.0, params.T_max)
    split_max = max(min(T, params.T_max - T), 0.0)
    split = clamp(tau_p / params.k_moment, -split_max, split_max)
    # No surface means no command coordinate to normalize against; the analytic
    # fallback speaks only in thrust. Reported as 0 rather than guessed.
    return 0.0, 0.0, 0.5 * (T - split), 0.5 * (T + split)


def _lateral_gimbal(M_xy, T, tau_p, params: VehicleParams, iters=3):
    """Solve M_x, M_y for (delta1, delta2): the lateral 2x2 inverse, refined.

    Seed: the small-angle inverse. With s1~d1, s2*c1~d2 the map is

        [M_x]   [ tau_P   -T*L ] [d1]
        [M_y] = [ -T*L   -tau_P] [d2]

    whose determinant is -(tau_P^2 + (T*L)^2) < 0 always, so the inverse exists
    for any thrust and any roll command -- there is no allocation singularity
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
    d1 = (tau_p * Mx - TL * My) / D
    d2 = (-TL * Mx - tau_p * My) / D

    # The 2x2 is solved by Cramer's rule rather than a library call. At this
    # size it is the same arithmetic a solver would do, minus the dependency and
    # minus the unbounded-work worry: the iteration count is fixed, so worst-case
    # execution time is stated rather than hoped for.
    for _ in range(iters):
        s1, c1 = math.sin(d1), math.cos(d1)
        s2, c2 = math.sin(d2), math.cos(d2)
        g1 = -TL * s2 * c1 + tau_p * s1 - Mx
        g2 = -TL * s1 - tau_p * s2 * c1 - My
        j11 = TL * s2 * s1 + tau_p * c1
        j12 = -TL * c2 * c1
        j21 = -TL * c1 + tau_p * s2 * s1
        j22 = -tau_p * c2 * c1
        det = j11 * j22 - j12 * j21
        if det == 0.0:
            break                       # keep the small-angle seed
        step1 = (g1 * j22 - j12 * g2) / det
        step2 = (j11 * g2 - g1 * j21) / det
        if not (math.isfinite(step1) and math.isfinite(step2)):
            break
        d1 -= step1
        d2 -= step2
    return (d1, d2)


def allocate(M_des, T_des, params: VehicleParams):
    """Desired body moment (3,) and total thrust -> Allocation.

    M_des is (M_x, M_y, M_z) in N*m: the first two lateral, the third roll.
    """
    Mx, My, Mz = float(M_des[0]), float(M_des[1]), float(M_des[2])

    # --- stage 0: thrust has priority; everything else works with what's left
    T_cmd = clamp(T_des, params.T_min, params.T_max)
    thrust_sat = not isclose(T_cmd, T_des)

    # --- stages 1 & 2: roll, then lateral, iterated twice.
    # M_z can only come from tau_P, but what reaches body z is
    # tau_P*cos(d1)*cos(d2), not tau_P -- and d1, d2 are not known until the
    # lateral stage has run, which in turn needs tau_P. One extra pass closes
    # that loop: solve the gimbal against the first tau_P guess, then divide
    # the guess by the cosine loss those angles imply and re-solve. Skipping it
    # leaves M_z short by tau_P*(1 - cos*cos), which at the 7 deg stops is 1.5%
    # of the roll command -- small, but it is a systematic bias, not noise, so
    # the roll integrator would otherwise spend the whole flight paying it off.
    q_lo, q_hi = roll_limits(T_cmd, params)
    axial_sat = Mz < q_lo - 1e-12 or Mz > q_hi + 1e-12
    tau_p = clamp(Mz, q_lo, q_hi)
    delta = _lateral_gimbal((Mx, My), T_cmd, tau_p, params)
    for _ in range(2):
        loss = math.cos(delta[0]) * math.cos(delta[1])
        if loss < 1e-6:
            break
        tau_p = clamp(Mz / loss, q_lo, q_hi)
        delta = _lateral_gimbal((Mx, My), T_cmd, tau_p, params)

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
        delta = (delta[0] * scale, delta[1] * scale)
    delta = (clamp(delta[0], lo[0], hi[0]), clamp(delta[1], lo[1], hi[1]))

    u_a, u_b, T1, T2 = motor_setpoint(T_cmd, tau_p, params)

    return Allocation(delta_cmd=delta, tau_p=tau_p, T_cmd=T_cmd, T1=T1, T2=T2,
                      u_a=u_a, u_b=u_b,
                      gimbal_saturated=gimbal_sat, roll_saturated=axial_sat,
                      thrust_saturated=thrust_sat)
