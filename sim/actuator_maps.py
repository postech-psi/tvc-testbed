"""
actuator_maps.py -- PWM <-> physical actuator conversions.
================================================================================
This is where the user's bench data lands: motor PWM -> thrust, and servo PWM ->
gimbal angle. Until that data is filled into the `pwm_thrust_map` /
`pwm_servo_map` blocks of sim/vehicle_params.yaml, every map falls back to the
simple analytic model the sims already use, so nothing breaks before the data
arrives.

REPRESENTATIONS (choose one per map in the YAML)
    type: poly    coeffs: [c0, c1, ...]         value = sum_i c_i * pwm**i
    type: table   pwm: [...]   value: [...]      monotonic, linearly interpolated
    type: null    (or omitted)                   -> analytic fallback

THRUST map returns NEWTONS (combined) for a PWM in microseconds.
SERVO  map returns RADIANS of gimbal deflection for a PWM in microseconds.
Both provide the forward map and its inverse, because the controllers think in
physical units (thrust, radians) and Gazebo/hardware want PWM (or, for the
current Gazebo motor model, rotor rad/s).

Usage:
    from actuator_maps import ThrustMap, ServoMap
    tm = ThrustMap.from_params()          # reads sim/vehicle_params.yaml
    omega = tm.thrust_to_rotor_omega(13.0)   # what the multicopter model wants
    sm = ServoMap.from_params()
    pwm = sm.rad_to_pwm(0.1)
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vehicle_params as _vp


def _poly(coeffs, x):
    return sum(c * x ** i for i, c in enumerate(coeffs))


def _interp(xs, ys, x):
    """Linear interpolation, clamped at the ends. xs must be ascending."""
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            t = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + t * (ys[i] - ys[i - 1])
    return ys[-1]


class _Map:
    """Shared forward/inverse machinery for a scalar PWM map."""

    def __init__(self, spec):
        self.spec = spec or {}
        self.type = self.spec.get("type")

    def forward(self, pwm):
        """PWM (us) -> physical value, or None if no data (caller uses fallback)."""
        if self.type == "poly":
            return _poly(self.spec["coeffs"], pwm)
        if self.type == "table":
            return _interp(self.spec["pwm"], self.spec["value"], pwm)
        return None

    def inverse(self, value):
        """Physical value -> PWM (us), or None if no data. Table maps invert by
        swapping the axes (requires the value column to be monotonic too)."""
        if self.type == "table":
            return _interp(self.spec["value"], self.spec["pwm"], value)
        if self.type == "poly":
            raise NotImplementedError(
                "inverse of a poly PWM map needs a numeric root solve; add it "
                "when the data is a polynomial rather than a table")
        return None


class ThrustMap(_Map):
    """PWM <-> combined thrust (N), plus the rotor-omega the Gazebo multicopter
    motor model consumes. Falls back to thrust = k*omega^2 when no PWM data."""

    def __init__(self, spec, v):
        super().__init__(spec)
        self.v = v

    @classmethod
    def from_params(cls, path=None):
        v = _vp.load(path)
        return cls(v.raw.get("pwm_thrust_map"), v)

    def thrust_to_rotor_omega(self, thrust_n):
        """Combined thrust -> per-rotor omega (rad/s) for the motor model. Uses
        the analytic thrust = motor_constant * omega^2 per rotor (two rotors),
        which is exactly what the SDF motor model integrates; the PWM map, when
        present, is for hardware/PWM-domain work, not this conversion."""
        half = max(self.v.thrust_at_max_n / 2.0, 1e-9)
        per_rotor = max(min(thrust_n / 2.0, half), 0.0)
        return self.v.max_rot_velocity * math.sqrt(per_rotor / half)


class ServoMap(_Map):
    """PWM <-> gimbal deflection (rad). Falls back to command == radians."""

    @classmethod
    def from_params(cls, path=None):
        v = _vp.load(path)
        return cls(v.raw.get("pwm_servo_map"))

    def pwm_to_rad(self, pwm):
        r = self.forward(pwm)
        return r if r is not None else 0.0

    def rad_to_pwm(self, rad):
        p = self.inverse(rad)
        if p is not None:
            return p
        raise ValueError("no pwm_servo_map data; fill it in "
                         "sim/vehicle_params.yaml before converting rad->PWM")


# =============================================================================
# Measured two-input surface: (PWM A, PWM B) -> (thrust, axial torque)
# =============================================================================

class ThrustTorqueSurface:
    """The bench-measured coax actuator surface, and its inverse.

    Forward is a cubic in the normalized commands a = (A-1500)/500 and
    b = (B-1500)/500. Inverse is a 2-D Newton solve, because a bivariate cubic
    has no closed-form inverse -- the alternative, a precomputed lookup, costs
    memory and interpolation error for no gain at these speeds.

    The inverse's real work is not the root find but the FEASIBILITY question.
    (T, tau_P) cannot be commanded independently: the reachable set is the image
    of the command square, and it is a curved lens, not a rectangle. Axial
    authority peaks near 11 N of thrust and collapses at both ends -- 0.151 N.m
    at hover, 0.028 N.m at 98% throttle. So the inverse clamps in the documented
    priority order (thrust first, then torque into whatever is left) BEFORE
    solving, and the solve is then guaranteed a target it can actually reach.
    """

    # Fraction of the true boundary the inverse promises; see max_torque_at.
    SOLVER_MARGIN = 0.98

    # Monomial exponents, matching the YAML coefficient order.
    _TERMS = ((0, 0), (1, 0), (0, 1), (2, 0), (1, 1), (0, 2),
              (3, 0), (2, 1), (1, 2), (0, 3))

    def __init__(self, spec, n_grid=161):
        self.spec = spec or {}
        self.ok = self.spec.get("type") == "cubic_ab"
        if not self.ok:
            return
        self.c_T = list(self.spec["thrust_n"])
        self.c_Q = list(self.spec["torque_nm"])
        self.offset = float(self.spec.get("pwm_offset", 1500.0))
        self.scale = float(self.spec.get("pwm_scale", 500.0))
        self.pwm_min = float(self.spec.get("pwm_min", 1000.0))
        self.pwm_max = float(self.spec.get("pwm_max", 2000.0))
        self._n_grid = n_grid
        self._headroom = None       # built lazily; see _build_headroom
        self._warm = None           # last solution, reused as the next seed

    @classmethod
    def from_params(cls, path=None):
        v = _vp.load(path)
        return cls(v.raw.get("thrust_torque_surface"))

    # --- forward -------------------------------------------------------------
    def _norm(self, pwm):
        return (pwm - self.offset) / self.scale

    def _denorm(self, x):
        return x * self.scale + self.offset

    def _eval(self, coeffs, a, b):
        return sum(c * a ** i * b ** j
                   for c, (i, j) in zip(coeffs, self._TERMS))

    def _grad(self, coeffs, a, b):
        da = sum(c * i * a ** (i - 1) * b ** j
                 for c, (i, j) in zip(coeffs, self._TERMS) if i)
        db = sum(c * j * a ** i * b ** (j - 1)
                 for c, (i, j) in zip(coeffs, self._TERMS) if j)
        return da, db

    def forward_norm(self, a, b):
        """Normalized commands -> (thrust N, axial torque N*m)."""
        return self._eval(self.c_T, a, b), self._eval(self.c_Q, a, b)

    def forward(self, pwm_a, pwm_b):
        """PWM microseconds -> (thrust N, axial torque N*m)."""
        return self.forward_norm(self._norm(pwm_a), self._norm(pwm_b))

    # --- feasible set --------------------------------------------------------
    def _build_headroom(self):
        """Tabulate max |tau_P| against thrust by sweeping the command square.

        Done once, on a grid, rather than solved analytically: the boundary of
        the reachable set is where the Jacobian degenerates or a command hits a
        stop, and enumerating is both simpler and more honest than assuming
        which of those binds where.
        """
        n = self._n_grid
        lo = self._norm(self.pwm_min)
        hi = self._norm(self.pwm_max)
        pts, ts, qs = [], [], []
        for i in range(n):
            a = lo + (hi - lo) * i / (n - 1)
            for j in range(n):
                b = lo + (hi - lo) * j / (n - 1)
                T, Q = self.forward_norm(a, b)
                pts.append((a, b, T, Q))
                ts.append(T)
                qs.append(Q)
        # Kept as numpy columns, not a list of tuples: the inverse seeds itself
        # by scanning this every time it cannot warm-start, and a 26k-point
        # Python loop there costs more than the Newton solve it is seeding.
        g = np.asarray(pts, dtype=float)
        self._grid = (g[:, 0], g[:, 1], g[:, 2], g[:, 3])
        t_lo, t_hi = min(ts), max(ts)
        # SIGNED tables, one per direction. The two are not mirror images: the
        # surface reaches +0.1797 N.m (A=1000, B=2000) against -0.1733 N.m at
        # the mirrored command, and at different thrusts (11.18 vs 10.37 N).
        # That asymmetry is the coax wake -- the prop in front sees clean air
        # and the one behind does not -- so a single symmetric cap either throws
        # away authority on the strong side or promises authority the weak side
        # cannot deliver. Earlier code did the latter and lost up to 0.09 N.m.
        nb = 200
        hi_q = [None] * (nb + 1)
        lo_q = [None] * (nb + 1)
        for T, Q in zip(ts, qs):
            k = int(round((T - t_lo) / (t_hi - t_lo) * nb)) if t_hi > t_lo else 0
            if hi_q[k] is None or Q > hi_q[k]:
                hi_q[k] = Q
            if lo_q[k] is None or Q < lo_q[k]:
                lo_q[k] = Q

        def _fill(tab):
            for k in range(1, nb + 1):
                if tab[k] is None:
                    tab[k] = tab[k - 1]
            for k in range(nb - 1, -1, -1):
                if tab[k] is None:
                    tab[k] = tab[k + 1]
            return [0.0 if v is None else v for v in tab]

        self._headroom = (t_lo, t_hi, _fill(lo_q), _fill(hi_q))
        self.thrust_min, self.thrust_max = t_lo, t_hi

    def thrust_limits(self):
        if self._headroom is None:
            self._build_headroom()
        return self.thrust_min, self.thrust_max

    def torque_limits_at(self, thrust_n):
        """Signed (min, max) tau_P [N*m] reachable at a given total thrust.

        Both ends already carry SOLVER_MARGIN, so every value in the returned
        interval is one the inverse can actually hit.
        """
        if self._headroom is None:
            self._build_headroom()
        t_lo, t_hi, lo_q, hi_q = self._headroom
        if thrust_n <= t_lo or thrust_n >= t_hi:
            return 0.0, 0.0
        x = (thrust_n - t_lo) / (t_hi - t_lo) * (len(hi_q) - 1)
        k = int(x)
        f = x - k
        k2 = min(k + 1, len(hi_q) - 1)
        q_hi = hi_q[k] * (1 - f) + hi_q[k2] * f
        q_lo = lo_q[k] * (1 - f) + lo_q[k2] * f
        # SOLVER MARGIN. The tabulated extremes sit exactly on the boundary of
        # the reachable set, which is precisely where the surface's Jacobian
        # degenerates -- Newton cannot land on it, only near it. Promising a
        # headroom the inverse cannot deliver makes the allocator ask for the
        # one target that fails, so what is handed out is 2% inside the true
        # edge. The cost is 2% of axial authority; the alternative was a silent
        # thrust shortfall at full axial command.
        return q_lo * self.SOLVER_MARGIN, q_hi * self.SOLVER_MARGIN

    def max_torque_at(self, thrust_n):
        """Symmetric |tau_P| [N*m] guaranteed in BOTH directions at this thrust.

        The conservative summary, for sizing gains and quoting authority. Use
        torque_limits_at() where the extra authority on the strong side matters.
        """
        q_lo, q_hi = self.torque_limits_at(thrust_n)
        return min(abs(q_lo), abs(q_hi))

    # --- inverse -------------------------------------------------------------
    def _solve(self, T_t, Q_t, iters, tol):
        """One Newton solve for an exact (T, Q) target. Returns (a, b, T, Q)."""
        if self._headroom is None:
            self._build_headroom()
        lo = self._norm(self.pwm_min)
        hi = self._norm(self.pwm_max)

        # Two seeds, tried in order. The warm start is what makes this usable in
        # a control loop: consecutive commands differ by a fraction of a newton,
        # so the previous solution is already within Newton's basin and converges
        # in two or three steps with no search at all.
        seeds = []
        if self._warm is not None:
            seeds.append(self._warm)
        seeds.append(None)          # None = fall back to the grid scan

        for seed in seeds:
            if seed is None:
                # Newton on a cubic is only locally convergent and this surface
                # is not monotone in b, so a cold solve needs a real seed: scan
                # the cached grid for the closest reachable point. Scaled
                # residual -- 1 N and 0.01 N.m are comparable errors here.
                ga, gb, gT, gQ = self._grid
                r = (gT - T_t) ** 2 + ((gQ - Q_t) / 0.01) ** 2
                k = int(np.argmin(r))
                seed = (float(ga[k]), float(gb[k]))
            a, b = self._newton(seed, T_t, Q_t, lo, hi, iters, tol)
            T, Q = self.forward_norm(a, b)
            if abs(T - T_t) < 1e-6 and abs(Q - Q_t) < 1e-6:
                break

        self._warm = (a, b)
        return a, b, T, Q

    def _newton(self, seed, T_t, Q_t, lo, hi, iters, tol):
        a, b = seed
        for _ in range(iters):
            T, Q = self.forward_norm(a, b)
            r1, r2 = T - T_t, Q - Q_t
            if abs(r1) < tol and abs(r2) < tol:
                break
            dTa, dTb = self._grad(self.c_T, a, b)
            dQa, dQb = self._grad(self.c_Q, a, b)
            det = dTa * dQb - dTb * dQa
            if abs(det) < 1e-12:
                break
            da = (r1 * dQb - r2 * dTb) / det
            db = (dTa * r2 - dQa * r1) / det
            a = min(max(a - da, lo), hi)
            b = min(max(b - db, lo), hi)
        return a, b

    def inverse(self, thrust_n, torque_nm, iters=40, tol=1e-9, thrust_tol=1e-4):
        """(thrust, axial torque) -> (PWM A, PWM B), clamped to the feasible set.

        Returns (pwm_a, pwm_b, achieved_thrust, achieved_torque). The achieved
        pair is returned rather than assumed, because the clamping is part of
        the answer: a caller that logs the request instead of the result reports
        authority the vehicle never had.

        THRUST HAS PRIORITY, and enforcing that takes more than clamping the
        torque to max_torque_at(T). The boundary of the reachable set is exactly
        where the surface's Jacobian degenerates, so a target sitting ON that
        boundary is the one case Newton cannot solve -- it stalls with a large
        thrust residual and would silently return a command producing over a
        newton less lift than asked. When that happens, back the torque off in
        stages and re-solve: the vehicle gives up axial authority, never lift.
        """
        t_lo, t_hi = self.thrust_limits()
        T_t = min(max(thrust_n, t_lo), t_hi)
        q_lo, q_hi = self.torque_limits_at(T_t)
        Q_t = min(max(torque_nm, q_lo), q_hi)

        for shrink in (1.0, 0.97, 0.9, 0.7, 0.4, 0.0):
            a, b, T, Q = self._solve(T_t, Q_t * shrink, iters, tol)
            if abs(T - T_t) <= thrust_tol:
                break
        return self._denorm(a), self._denorm(b), T, Q


# =============================================================================
# Measured gimbal axis maps
# =============================================================================

class GimbalAxisMap:
    """One gimbal axis: PWM <-> angle, with its own measured travel limits.

    Separate per axis on purpose. The two rings are different hardware -- the
    outer has 1.6x the gain per microsecond, 0.6x the slew rate and 2/3 the
    bandwidth of the inner -- and neither one's travel is symmetric about its
    own neutral. Treating them as one symmetric +/-7 deg actuator overstates
    negative travel on both.
    """

    def __init__(self, spec):
        self.gain = float(spec["gain_deg_per_us"])      # deg per us
        self.neutral = float(spec["neutral_pwm"])       # us
        self.min_deg = float(spec["min_deg"])
        self.max_deg = float(spec["max_deg"])
        self.rate_max_deg = float(spec.get("rate_max_deg", 180.0))
        self.bandwidth_hz = float(spec.get("bandwidth_hz", 0.0))

    @classmethod
    def all_from_params(cls, path=None):
        v = _vp.load(path)
        return {k: cls(spec) for k, spec in (v.gimbal_axes or {}).items()}

    def pwm_to_deg(self, pwm):
        return self.gain * (pwm - self.neutral)

    def deg_to_pwm(self, deg):
        return self.neutral + self.clamp_deg(deg) / self.gain

    def clamp_deg(self, deg):
        return min(max(deg, self.min_deg), self.max_deg)


if __name__ == "__main__":
    tm = ThrustMap.from_params()
    sm = ServoMap.from_params()
    v = _vp.load()
    print("thrust map type: %s   servo map type: %s"
          % (tm.type, sm.type))
    print("hover thrust %.2f N -> rotor omega %.1f rad/s (%.0f%% of max)"
          % (v.weight_n, tm.thrust_to_rotor_omega(v.weight_n),
             100 * tm.thrust_to_rotor_omega(v.weight_n) / v.max_rot_velocity))

    surf = ThrustTorqueSurface.from_params()
    if surf.ok:
        t_lo, t_hi = surf.thrust_limits()
        print("")
        print("measured surface: thrust %.2f .. %.2f N" % (t_lo, t_hi))
        print("  axial torque available vs thrust:")
        for frac in (0.50, 0.65, 0.75, 0.85, 0.95):
            T = t_hi * frac
            print("    T = %5.2f N (%3.0f%% throttle)  ->  |tau_P| <= %.4f N.m"
                  % (T, 100 * frac, surf.max_torque_at(T)))
        print("  hover  T = %.2f N (%3.0f%% throttle)  ->  |tau_P| <= %.4f N.m"
              % (v.weight_n, 100 * v.weight_n / t_hi,
                 surf.max_torque_at(v.weight_n)))
        a, b, T, Q = surf.inverse(v.weight_n, 0.10)
        print("  inverse(T=%.2f, tau_P=0.100) -> A=%.0f us, B=%.0f us "
              "(achieved %.4f N, %.4f N.m)" % (v.weight_n, a, b, T, Q))

    print("")
    for name, ax in GimbalAxisMap.all_from_params().items():
        print("gimbal %-5s: %+.2f..%+.2f deg, neutral %.1f us, "
              "1600 us -> %+.2f deg, %.0f deg/s, BW %.0f Hz"
              % (name, ax.min_deg, ax.max_deg, ax.neutral,
                 ax.pwm_to_deg(1600.0), ax.rate_max_deg, ax.bandwidth_hz))
