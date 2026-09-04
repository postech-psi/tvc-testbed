"""
The Gazebo command-side inversion.
================================================================================
gz-sim's MulticopterMotorModel computes, per rotor, T_i = k*omega_i^2 and
tau_i = +/- c*T_i, hence

    T = k(omega_a^2 + omega_b^2)        tau_P = c(T_b - T_a)

That model is separable, symmetric and linear in the thrust split. The measured
coax surface is none of the three, and the gap is not small: at hover the plugin
with the original c = 0.016 reached only ~50% of the measured roll torque, and
being ODD in the split it cannot represent the sign asymmetry at all.

The fix is to invert the plugin's own algebra on the command side: take the
(T, tau_P) the allocator chose off the measured surface, and solve for the omega
pair that makes the plugin produce exactly that. Two equations, two unknowns,
closed form. The asymmetry survives because it lives in the commanded tau_P --
which came from the surface -- and not in the plugin.

This file asserts that the round trip is exact over the vehicle's whole feasible
set, rather than leaving it argued from algebra in a comment. The price of the
trick -- simulated rotor speed stops being a physical RPM -- is checked in
tests/test_consistency.py, along with the sign chain it depends on.
"""
import numpy as np


def _consts(vehicle):
    r = vehicle.raw["rotors"]
    return r["motor_constant"], r["moment_constant"], r["max_rot_velocity"]


def test_inversion_is_exact_over_the_measured_envelope(vp, vehicle):
    from tvc_control.hal.gazebo import plugin_forward, rotor_speeds
    k, c, wmax = _consts(vehicle)

    worst_T = worst_Q = 0.0
    n = 0
    for T in np.linspace(1.0, vp.T_max, 40):
        lo, hi = vp.surface.torque_limits_at(T)
        for tau in np.linspace(lo, hi, 15):
            wa, wb = rotor_speeds(T, tau, k, c, wmax)
            assert 0.0 <= wa <= wmax and 0.0 <= wb <= wmax
            # A clamp would mean the request exceeded SOLVER headroom, which is
            # a different and much rarer thing than the vehicle being out of
            # authority -- and it must not happen inside the vehicle's own
            # feasible set, because there the allocator has already guaranteed
            # the request is physically reachable.
            assert max(wa, wb) < wmax - 1e-9, (
                "solver ceiling hit at T=%.2f tau=%.4f -- raise max_rot_velocity"
                % (T, tau))
            T_b, Q_b = plugin_forward(wa, wb, k, c)
            worst_T = max(worst_T, abs(T_b - T))
            worst_Q = max(worst_Q, abs(Q_b - tau))
            n += 1
    assert n > 500
    assert worst_T < 1e-9, "thrust round-trip error %.2e N" % worst_T
    assert worst_Q < 1e-9, "torque round-trip error %.2e N*m" % worst_Q


def test_a_torque_request_never_costs_thrust(vp, vehicle):
    """Roll authority is paid for in headroom, not in lift.

    The split is symmetric about the commanded total: t_a = (T - s)/2 and
    t_b = (T + s)/2 sum to T for any s. If that ever stopped holding, every roll
    correction would show up as an altitude dip and be debugged as an altitude
    problem.
    """
    from tvc_control.hal.gazebo import plugin_forward, rotor_speeds
    k, c, wmax = _consts(vehicle)

    T = vp.m * vp.g
    lo, hi = vp.surface.torque_limits_at(T)
    for tau in np.linspace(lo, hi, 21):
        T_b, _ = plugin_forward(*rotor_speeds(T, tau, k, c, wmax), k, c)
        assert abs(T_b - T) < 1e-9


def test_zero_torque_gives_equal_rotor_speeds(vehicle):
    """The sanity check the whole inversion has to pass first."""
    from tvc_control.hal.gazebo import rotor_speeds
    k, c, wmax = _consts(vehicle)
    wa, wb = rotor_speeds(13.028, 0.0, k, c, wmax)
    assert abs(wa - wb) < 1e-9
    assert 0.0 < wa < wmax
