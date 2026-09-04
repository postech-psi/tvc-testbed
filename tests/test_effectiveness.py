"""
The bench-measured thrust/torque surface, and its inverse.
================================================================================
Everything the vehicle can and cannot do about the thrust axis is read off this
surface, so an error in it is an error in every authority claim the project
makes. Three kinds of check:

  ANCHORS. Two points whose values were quoted independently on the bench
  slides. They confirm the coefficients were transcribed correctly, which is not
  something the fit statistics can tell you.

  ROUND-TRIP. inverse() then forward() must return what was asked for, over the
  whole feasible set -- including at its boundary, which is exactly where
  Newton's method has the most trouble.

  FEASIBILITY. The inverse must never hand back a command outside the PWM
  square, and torque_limits_at() must never promise a torque the inverse cannot
  actually hit.
"""
import numpy as np
import pytest


# --- anchors -----------------------------------------------------------------

def test_anchor_peak_thrust(vp):
    """f_T(1, 1) = 17.79 N, the quoted peak combined thrust at full throttle."""
    T, _ = vp.surface.forward_norm(1.0, 1.0)
    assert T == pytest.approx(17.79, abs=0.01)


def test_anchor_peak_reaction_torque(vp):
    """f_Q(1, -1) = -0.173 N*m, matching the quoted ~0.18 N*m peak."""
    _, Q = vp.surface.forward_norm(1.0, -1.0)
    assert Q == pytest.approx(-0.173, abs=0.002)


def test_the_surface_is_not_separable(vp):
    """T(a,b) != T1(a) + T2(b). If it were, two per-motor curves would do and
    this whole class would be unnecessary.

    The lower rotor runs inside the upper rotor's wake, so both outputs depend
    on both commands. Mirrored commands give measurably unmirrored results.
    """
    t_pm, q_pm = vp.surface.forward_norm(+1.0, -1.0)
    t_mp, q_mp = vp.surface.forward_norm(-1.0, +1.0)
    assert abs(t_mp - t_pm) > 0.5, "thrust is suspiciously symmetric in (a, b)"
    assert abs(abs(q_mp) - abs(q_pm)) > 0.003, "torque is symmetric; wake missing"


def test_the_surface_is_not_proportional_to_the_thrust_split(vp):
    """The reason a single Gazebo momentConstant cannot represent this vehicle.

    c = tau_P / (T_b - T_a) is not a constant: the value needed to cover the
    measured envelope spans roughly 3x. Fitting the middle over-promises the
    weak direction, the roll integrator winds up, and it looks like a tuning
    problem.
    """
    ratios = []
    for T in np.linspace(0.3 * vp.T_max, 0.9 * vp.T_max, 12):
        lo, hi = vp.surface.torque_limits_at(T)
        if hi <= 0:
            continue
        split = max(min(T, vp.T_max - T), 1e-6)
        ratios.append(hi / split)
    assert max(ratios) / min(ratios) > 2.0, \
        "the torque/split ratio looks constant, which contradicts the bench data"


# --- round-trip --------------------------------------------------------------

def test_thrust_is_always_achieved_exactly(vp):
    """The invariant that must hold everywhere, including on the boundary.

    Thrust has priority over torque, and the diagonal bisection at the end of
    inverse()'s ladder is what makes that unconditional. Before it existed the
    solve lost 0.32 N -- 1.8% of the vehicle's lift -- within 0.05 N of the
    ceiling, at the one moment there is none to spare.
    """
    worst, n = 0.0, 0
    for T in np.linspace(1.0, vp.T_max, 40):
        lo, hi = vp.surface.torque_limits_at(T)
        for Q in np.linspace(lo, hi, 15):
            _, _, T_a, _ = vp.surface.inverse(T, Q)
            worst = max(worst, abs(T_a - T))
            n += 1
    assert n > 500
    assert worst < 1e-6, "worst thrust error %.3e N" % worst


def test_torque_is_exact_except_at_the_extremes_of_the_throttle(vp):
    """And where it is not, the shortfall is bounded and its cause is known.

    torque_limits_at() reads a 200-bin table, so where the reachable set changes
    fast in thrust it over-promises by up to a bin's worth. inverse() then trades
    that excess away to keep thrust exact. The affected band is below ~8% and
    above ~95% throttle -- precisely where roll authority is collapsing anyway --
    and hover sits at 73%.
    """
    worst_mid, worst_end = 0.0, 0.0
    for T in np.linspace(1.0, vp.T_max, 60):
        frac = T / vp.T_max
        lo, hi = vp.surface.torque_limits_at(T)
        for Q in np.linspace(lo, hi, 15):
            _, _, _, Q_a = vp.surface.inverse(T, Q)
            err = abs(Q_a - Q)
            if 0.10 < frac < 0.94:
                worst_mid = max(worst_mid, err)
            else:
                worst_end = max(worst_end, err)
    assert worst_mid < 1e-6, \
        "torque error %.3e N*m in the usable throttle band" % worst_mid
    assert worst_end < 0.01, \
        "torque shortfall at the extremes grew to %.4f N*m" % worst_end


def test_inverse_stays_inside_the_command_square(vp):
    """A PWM outside [pwm_min, pwm_max] is a command no ESC will honour, so a
    solution that leaves the square is not a solution."""
    for T in np.linspace(1.0, vp.T_max, 25):
        lo, hi = vp.surface.torque_limits_at(T)
        for Q in (lo, 0.0, hi):
            a, b, _, _ = vp.surface.inverse(T, Q)
            assert vp.surface.pwm_min - 1e-6 <= a <= vp.surface.pwm_max + 1e-6
            assert vp.surface.pwm_min - 1e-6 <= b <= vp.surface.pwm_max + 1e-6


def test_the_promised_limits_are_reachable_in_the_usable_band(vp):
    """torque_limits_at() must not promise what inverse() cannot hit.

    The tabulated extremes sit ON the reachable set's boundary, which is where
    the Jacobian degenerates -- Newton can approach it but not land on it.
    SOLVER_MARGIN backs the promise 2% inside for exactly this reason, and this
    is the test that says the margin is enough across the throttle band the
    vehicle actually flies in.
    """
    for T in np.linspace(0.12 * vp.T_max, 0.93 * vp.T_max, 30):
        lo, hi = vp.surface.torque_limits_at(T)
        for Q in (lo, hi):
            _, _, T_a, Q_a = vp.surface.inverse(T, Q)
            assert abs(T_a - T) < 1e-6
            assert abs(Q_a - Q) < 1e-6


def test_iteration_count_is_bounded(vp):
    """Flight code may not run an unbounded search. inverse() takes `iters` and
    a fixed backoff ladder, so worst-case work is a number someone can write
    down: 6 ladder rungs x iters Newton steps x one cubic evaluation each."""
    import inspect
    src = inspect.getsource(type(vp.surface).inverse)
    assert "while" not in src
    assert "for shrink in" in src


# --- the gimbal maps ---------------------------------------------------------

def test_gimbal_maps_are_per_ring_and_asymmetric(vp):
    """The two rings are different hardware and neither is symmetric about its
    own neutral. One symmetric +/-7 deg actuator overstates negative travel on
    both."""
    inner, outer = vp.gimbal_axes["inner"], vp.gimbal_axes["outer"]
    assert inner.gain != outer.gain
    assert inner.rate_max_deg != outer.rate_max_deg
    for ax in (inner, outer):
        assert abs(abs(ax.min_deg) - abs(ax.max_deg)) > 0.05


def test_gimbal_pwm_round_trip(vp):
    for name, ax in vp.gimbal_axes.items():
        for deg in (ax.min_deg, 0.0, ax.max_deg):
            assert ax.pwm_to_deg(ax.deg_to_pwm(deg)) == pytest.approx(deg, abs=1e-9)


def test_gimbal_normalization_saturates_at_its_own_travel(vp):
    """PX4's ActuatorServos wants [-1, 1] against the servo's own range, and the
    two ends of that range are different numbers here."""
    import math
    for name, ax in vp.gimbal_axes.items():
        assert ax.normalized(math.radians(ax.max_deg)) == pytest.approx(1.0)
        assert ax.normalized(math.radians(ax.min_deg)) == pytest.approx(-1.0)
        assert abs(ax.normalized(math.radians(10.0))) <= 1.0
