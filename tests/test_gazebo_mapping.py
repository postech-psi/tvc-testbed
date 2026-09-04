"""
The Gazebo command-side inversion, and the sign chain it depends on.
================================================================================
The inversion is the whole reason Gazebo can reproduce the measured surface, so
the claim "exact over the measured envelope" is asserted here rather than argued
in a comment.

The sign test is the more valuable one. Three independent choices have to agree
about which direction positive axial torque points -- the fitted surface's
coefficients, the allocator's convention, and the Gazebo plugin's
tau = c*(T_b - T_a). Nothing forced them to, and they currently do by
coincidence. If one flips, the vehicle spins up instead of correcting and it
reads as a control bug.
"""
import numpy as np


def _consts(vehicle):
    r = vehicle.raw["rotors"]
    return r["motor_constant"], r["moment_constant"], r["max_rot_velocity"]


def test_inversion_is_exact_over_the_measured_envelope(vp):
    from tvc_control.config import load
    from tvc_control.hal.gazebo import rotor_speeds, plugin_forward
    k, c, wmax = _consts(load())

    worst_T = worst_Q = 0.0
    n = 0
    for T in np.linspace(1.0, vp.T_max, 40):
        lo, hi = vp.surface.torque_limits_at(T)
        for tau in np.linspace(lo, hi, 15):
            wa, wb = rotor_speeds(T, tau, k, c, wmax)
            assert 0.0 <= wa <= wmax and 0.0 <= wb <= wmax
            # A clamp would mean the request exceeded solver headroom, which
            # must not happen inside the vehicle's own feasible set.
            assert max(wa, wb) < wmax - 1e-9, (
                "solver ceiling hit at T=%.2f tau=%.4f -- raise max_rot_velocity"
                % (T, tau))
            T_b, Q_b = plugin_forward(wa, wb, k, c)
            worst_T = max(worst_T, abs(T_b - T))
            worst_Q = max(worst_Q, abs(Q_b - tau))
            n += 1
    assert n > 500
    assert worst_T < 1e-9, "thrust round-trip error %.2e N" % worst_T
    assert worst_Q < 1e-9, "torque round-trip error %.2e N.m" % worst_Q


def test_tau_p_sign_chain_agrees_end_to_end(vp):
    """Surface, allocator and Gazebo plugin must mean the same thing by +tau_P.

    Commanding MORE on rotor B than rotor A must give POSITIVE axial torque in
    all three. The surface says so through its gradients (dTz/db > 0 > dTz/da);
    the plugin says so through tau = c*(T_b - T_a); the allocator inherits it.
    """
    from tvc_control.config import load
    from tvc_control.hal.gazebo import rotor_speeds, plugin_forward
    k, c, wmax = _consts(load())

    # 1. the measured surface: more B than A -> positive
    _, q_more_b = vp.surface.forward_norm(-1.0, +1.0)
    _, q_more_a = vp.surface.forward_norm(+1.0, -1.0)
    assert q_more_b > 0.0 > q_more_a

    # 2. the plugin, driven by the inversion: a positive tau_P request must put
    #    the faster rotor on B
    wa, wb = rotor_speeds(13.0, +0.05, k, c, wmax)
    assert wb > wa
    _, q = plugin_forward(wa, wb, k, c)
    assert q > 0.0

    # 3. and the allocator agrees about which way M_z points
    from tvc_control.gnc.allocation import allocate
    a = allocate((0.0, 0.0, +0.05), 13.0, vp)
    assert a.tau_p > 0.0


def test_solver_constants_are_headroom_not_physics(vp):
    """max_rot_velocity must EXCEED what the real vehicle can produce.

    It used to equal it -- 2*k*wmax^2 was 17.787 N against a measured 17.79 --
    and sim/hover.py calibrated thrust->omega from that coincidence while the
    plugin used motor_constant. Now the two are deliberately different, so the
    old equality would be a bug and the inequality is the invariant.
    """
    from tvc_control.config import load
    k, c, wmax = _consts(load())
    plugin_ceiling = 2.0 * k * wmax * wmax
    assert plugin_ceiling > vp.T_max * 1.1, (
        "no solver headroom: plugin ceiling %.2f N vs vehicle %.2f N"
        % (plugin_ceiling, vp.T_max))
    # And the vehicle's own limit is still enforced -- by the allocator, since
    # the plugin no longer does it.
    a = allocate_max(vp)
    assert a <= vp.T_max + 1e-9


def allocate_max(vp):
    from tvc_control.gnc.allocation import allocate
    return allocate((0.0, 0.0, 0.0), 1e6, vp).T_cmd
