"""
Control allocation: does it realize the moment it was asked for?
================================================================================
Everything downstream of the allocator assumes the answer is yes. If it is not,
the attitude loop is closed around a plant it does not have, and the symptom is
a tuning problem that no amount of tuning fixes.

Three claims are checked:

  1. UNSATURATED COMMANDS ARE EXACT. Feed a random moment inside the feasible
     set, run the allocator, then compute the moment its output actually
     produces through the same r x F + tau_P*n_hat the plant integrates.
  2. SATURATED COMMANDS PRESERVE TORQUE DIRECTION. When the gimbal is on its
     stops the pair is scaled down, not clipped per axis. Clipping rotates the
     commanded torque vector toward the corner of the box, which is exactly the
     wrong thing to do while recovering from an off-axis upset.
  3. SATURATION IS REPORTED. The flags drive anti-windup, so a flag that fires
     late leaves an integrator charging against a pinned actuator.
"""
import math

import numpy as np


def realized_moment(alloc, T, vp):
    """The moment the plant will actually produce from this allocation.

    Deliberately recomputed here from the geometry rather than imported, so the
    test is an independent statement of what r x F + tau_P*n_hat means and not a
    restatement of the code under test.
    """
    from tvc_control.gnc.mathx import thrust_axis
    n = np.asarray(thrust_axis(alloc.delta_cmd), float)
    r = np.array([vp.dx, vp.dy, -vp.L])
    return np.cross(r, T * n) + alloc.tau_p * n


def test_unsaturated_allocation_is_exact(vp):
    from tvc_control.gnc.allocation import allocate, roll_limits

    T = vp.m * vp.g
    q_lo, q_hi = roll_limits(T, vp)
    travel = float(np.min(np.abs(np.concatenate([vp.delta_min, vp.delta_max]))))
    lat = T * vp.L * math.sin(travel)

    rng = np.random.default_rng(20260905)
    worst, n_ok = 0.0, 0
    for _ in range(2000):
        M = np.array([rng.uniform(-1, 1) * lat * 0.7,
                      rng.uniform(-1, 1) * lat * 0.7,
                      rng.uniform(q_lo, q_hi) * 0.95])
        a = allocate(M, T, vp)
        if a.gimbal_saturated or a.roll_saturated:
            continue
        n_ok += 1
        worst = max(worst, float(np.max(np.abs(realized_moment(a, T, vp) - M))))

    assert n_ok > 1500, "too few unsaturated samples (%d) to mean anything" % n_ok
    assert worst < 1e-9, "worst moment error %.3e N*m" % worst


def test_saturation_preserves_the_torque_direction(vp):
    """Ask for twice the lateral authority the vehicle has, on a diagonal.

    The realized lateral torque must point the same way as the request. Per-axis
    clipping would rotate it toward 45 degrees regardless of what was asked.
    """
    from tvc_control.gnc.allocation import allocate

    T = vp.m * vp.g
    lat = T * vp.L * math.sin(min(abs(min(vp.delta_min)), abs(max(vp.delta_max))))
    # Deliberately lopsided so a corner-seeking clip is visible as a rotation.
    request = np.array([2.0 * lat, 0.5 * lat, 0.0])

    a = allocate(request, T, vp)
    assert a.gimbal_saturated

    got = realized_moment(a, T, vp)[:2]
    want = request[:2]
    cos = float(np.dot(got, want) / (np.linalg.norm(got) * np.linalg.norm(want)))
    # 0.5 deg of rotation. Not zero: the exact trigonometric map is mildly
    # nonlinear, so a uniform scale of the ANGLES is not a uniform scale of the
    # torques. Per-axis clipping of this request would rotate it by ~19 deg.
    assert math.degrees(math.acos(min(cos, 1.0))) < 0.5


def test_thrust_has_priority_over_torque(vp):
    """Roll authority is given up; lift never is.

    The boundary of the reachable set is where the surface's Jacobian
    degenerates, so a target sitting on it is the one case Newton cannot solve.
    The inverse backs the torque off in stages rather than returning a command
    that silently produces over a newton less lift.
    """
    from tvc_control.gnc.allocation import allocate

    for frac in (0.4, 0.6, 0.73, 0.85, 0.95):
        T = frac * vp.T_max
        a = allocate((0.0, 0.0, 10.0), T, vp)      # an impossible roll demand
        assert a.roll_saturated
        # inverse() returns what the commands it chose will ACTUALLY produce.
        _, _, achieved_T, _ = vp.surface.inverse(T, a.tau_p)
        assert abs(achieved_T - T) < 1e-3, \
            "lost %.3f N of thrust at %.0f%% throttle" \
            % (abs(achieved_T - T), 100 * frac)


def test_thrust_is_clamped_to_the_vehicles_own_ceiling(vp):
    """The Gazebo plugin no longer enforces this -- maxRotVelocity is solver
    headroom now -- so the allocator is the only thing that does."""
    from tvc_control.gnc.allocation import allocate
    a = allocate((0.0, 0.0, 0.0), 1e6, vp)
    assert a.T_cmd <= vp.T_max + 1e-9
    assert a.thrust_saturated
    b = allocate((0.0, 0.0, 0.0), -1e6, vp)
    assert b.T_cmd >= vp.T_min - 1e-9


def test_roll_authority_collapses_at_both_ends_of_the_throttle(vp):
    """tau_P is bought with a thrust split, so there is none to buy at idle or
    at the ceiling. A model that offered constant roll authority would promise
    the most exactly where the vehicle has the least."""
    from tvc_control.gnc.allocation import roll_headroom
    lo = roll_headroom(0.02 * vp.T_max, vp)
    mid = roll_headroom(0.60 * vp.T_max, vp)
    hi = roll_headroom(0.999 * vp.T_max, vp)
    assert mid > lo and mid > hi
    assert hi < 0.2 * mid


def test_the_feasible_roll_interval_is_asymmetric(vp):
    """The coax wake makes one direction stronger. A symmetric cap either throws
    away authority on the strong side or promises what the weak side cannot
    deliver; earlier code did the latter and lost up to 0.09 N*m."""
    from tvc_control.gnc.allocation import roll_limits
    lo, hi = roll_limits(vp.m * vp.g, vp)
    assert lo < 0.0 < hi
    assert abs(hi) > 1.3 * abs(lo), \
        "expected a materially asymmetric interval, got [%.4f, %.4f]" % (lo, hi)


def test_headroom_is_zero_once_the_feasible_set_stops_containing_zero(vp):
    """Above ~17 N the reachable tau_P interval is entirely POSITIVE.

    Both props are near their stops and the coax asymmetry means every
    reachable torque is a positive bias the vehicle has to wear. There is then
    no authority in either direction, and the old min(|lo|,|hi|) reported the
    smaller end of that interval as if it were headroom -- 0.0090 N.m at 17.4 N,
    which reads as authority and is the exact opposite of the truth.

    This is not academic. The climb scenario commands full thrust, the allocator
    clamps tau_P to the nearest feasible value, and the vehicle takes 53 deg of
    thrust-axis roll before the throttle comes off the stop. See
    docs/7-CREDIBILITY.md.
    """
    from tvc_control.gnc.allocation import roll_headroom, roll_limits
    saw_excluded = False
    for i in range(101):
        T = vp.T_max * i / 100.0
        lo, hi = roll_limits(T, vp)
        cap = roll_headroom(T, vp)
        if lo > 0.0 or hi < 0.0:
            saw_excluded = True
            assert cap == 0.0, (
                "at T = %.2f N the interval is [%+.4f, %+.4f], which excludes "
                "zero, but headroom reports %.4f" % (T, lo, hi, cap))
        else:
            assert cap <= min(abs(lo), abs(hi)) + 1e-12
    assert saw_excluded, ("no thrust in the envelope had a one-sided feasible "
                          "set -- either the surface changed or this test is "
                          "no longer scanning it")
