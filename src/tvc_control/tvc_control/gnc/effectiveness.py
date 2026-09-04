"""
Actuator effectiveness: what a command actually produces, and how to invert it.
================================================================================
Flight code. Pure Python -- no numpy, no file I/O, bounded iteration counts.

WHY THE COAX PAIR NEEDS A SURFACE AND NOT TWO CURVES
    The lower rotor runs inside the upper rotor's wake, so its thrust and drag
    torque depend on BOTH commands. T = T1(u_a) + T2(u_b) does not hold, and the
    bench data says so plainly: the mixed terms are large (-0.978*a*b in thrust,
    -0.479*a^2*b), and mirrored commands are not mirrored results -- +0.180 N*m
    at (a=-1, b=+1) against -0.173 N*m at (a=+1, b=-1), with 11.18 N of thrust
    against 10.37 N. That asymmetry IS the wake, and it is the reason a single
    "moment constant" cannot represent this vehicle.

WHY THE INVERSE IS MOSTLY ABOUT FEASIBILITY
    (T, tau_P) cannot be commanded independently. The reachable set is the image
    of the command square -- a curved lens, not a rectangle -- and axial
    authority peaks near 11 N of thrust and collapses at both ends: +0.147 /
    -0.089 N*m at hover, essentially nothing at 96% throttle. So the inverse
    clamps to the feasible set in the documented priority order (thrust first,
    torque into whatever is left) BEFORE solving, and the solve is then
    guaranteed a target it can reach.

PORTING NOTES
    Two tables are built once at construction and then only read:
      - the signed headroom table, which defines the feasible set
      - a coarse seed table for the inverse's cold path
    Construction sweeps a 161x161 grid, which is startup work, not control-path
    work. The seed table is deliberately much coarser (33x33) because it has to
    live in flash: the full sweep would be ~830 KB, the seed table is ~35 KB.
    The control path itself does at most `iters` Newton steps from a warm start,
    so its worst case is a fixed number of cubic evaluations.
"""

import math

# Monomial exponents, matching the YAML coefficient order:
# [1, a, b, a^2, ab, b^2, a^3, a^2*b, a*b^2, b^3]
_TERMS = ((0, 0), (1, 0), (0, 1), (2, 0), (1, 1), (0, 2),
          (3, 0), (2, 1), (1, 2), (0, 3))


class ThrustTorqueSurface:
    """Bench-measured (PWM A, PWM B) -> (thrust, axial torque), and its inverse."""

    # Fraction of the true boundary the inverse promises. The tabulated extremes
    # sit exactly on the edge of the reachable set, which is precisely where the
    # surface's Jacobian degenerates -- Newton can approach it but not land on
    # it. Promising headroom the inverse cannot deliver makes the allocator ask
    # for the one target that fails, so what is handed out is 2% inside. The
    # cost is 2% of axial authority; the alternative was a silent thrust
    # shortfall at full axial command.
    SOLVER_MARGIN = 0.98

    def __init__(self, coeffs_thrust, coeffs_torque, pwm_offset=1500.0,
                 pwm_scale=500.0, pwm_min=1000.0, pwm_max=2000.0,
                 n_grid=161, n_seed=33, n_bins=200):
        self.c_T = tuple(coeffs_thrust)
        self.c_Q = tuple(coeffs_torque)
        self.offset = float(pwm_offset)
        self.scale = float(pwm_scale)
        self.pwm_min = float(pwm_min)
        self.pwm_max = float(pwm_max)
        self.ok = True
        self._n_grid = n_grid
        self._n_seed = n_seed
        self._n_bins = n_bins
        self._warm = None
        self._built = False

    # --- forward -------------------------------------------------------------
    def _norm(self, pwm):
        return (pwm - self.offset) / self.scale

    def _denorm(self, x):
        return x * self.scale + self.offset

    @staticmethod
    def _eval(c, a, b):
        return sum(c[i] * a ** e[0] * b ** e[1] for i, e in enumerate(_TERMS))

    @staticmethod
    def _grad(c, a, b):
        da = sum(c[i] * e[0] * a ** (e[0] - 1) * b ** e[1]
                 for i, e in enumerate(_TERMS) if e[0])
        db = sum(c[i] * e[1] * a ** e[0] * b ** (e[1] - 1)
                 for i, e in enumerate(_TERMS) if e[1])
        return da, db

    def forward_norm(self, a, b):
        """Normalized commands -> (thrust N, axial torque N*m)."""
        return self._eval(self.c_T, a, b), self._eval(self.c_Q, a, b)

    def forward(self, pwm_a, pwm_b):
        """PWM microseconds -> (thrust N, axial torque N*m)."""
        return self.forward_norm(self._norm(pwm_a), self._norm(pwm_b))

    # --- feasible set --------------------------------------------------------
    def _build(self):
        """Sweep the command square once: signed headroom + a coarse seed table.

        The boundary of the reachable set is where the Jacobian degenerates or a
        command hits a stop. Enumerating is simpler and more honest than
        assuming which of those binds where.
        """
        n = self._n_grid
        lo = self._norm(self.pwm_min)
        hi = self._norm(self.pwm_max)
        step = n // self._n_seed if n >= self._n_seed else 1

        ts, qs = [], []
        seeds = []
        for i in range(n):
            a = lo + (hi - lo) * i / (n - 1)
            for j in range(n):
                b = lo + (hi - lo) * j / (n - 1)
                T, Q = self.forward_norm(a, b)
                ts.append(T)
                qs.append(Q)
                if i % step == 0 and j % step == 0:
                    seeds.append((a, b, T, Q))
        self._seeds = tuple(seeds)

        t_lo, t_hi = min(ts), max(ts)
        nb = self._n_bins
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
            return tuple(0.0 if v is None else v for v in tab)

        self._lo_q = _fill(lo_q)
        self._hi_q = _fill(hi_q)
        self.thrust_min, self.thrust_max = t_lo, t_hi
        self._built = True

    def thrust_limits(self):
        if not self._built:
            self._build()
        return self.thrust_min, self.thrust_max

    def torque_limits_at(self, thrust_n):
        """Signed (min, max) tau_P [N*m] reachable at a given total thrust.

        The two ends are not mirror images: the surface reaches +0.180 N*m and
        -0.173 N*m at different thrusts, so a single symmetric cap either throws
        away authority on the strong side or promises what the weak side cannot
        deliver. Earlier code did the latter and lost up to 0.09 N*m.

        Both ends already carry SOLVER_MARGIN, so every value in the returned
        interval is one the inverse can actually hit.
        """
        if not self._built:
            self._build()
        t_lo, t_hi = self.thrust_min, self.thrust_max
        if thrust_n <= t_lo or thrust_n >= t_hi:
            return 0.0, 0.0
        x = (thrust_n - t_lo) / (t_hi - t_lo) * self._n_bins
        k = int(x)
        f = x - k
        k2 = min(k + 1, self._n_bins)
        q_hi = self._hi_q[k] * (1 - f) + self._hi_q[k2] * f
        q_lo = self._lo_q[k] * (1 - f) + self._lo_q[k2] * f
        return q_lo * self.SOLVER_MARGIN, q_hi * self.SOLVER_MARGIN

    def max_torque_at(self, thrust_n):
        """Symmetric |tau_P| guaranteed in BOTH directions at this thrust.

        The conservative summary, for sizing gains and quoting authority. Use
        torque_limits_at() where the extra authority on the strong side matters.
        """
        q_lo, q_hi = self.torque_limits_at(thrust_n)
        return min(abs(q_lo), abs(q_hi))

    # --- inverse -------------------------------------------------------------
    def _newton(self, a, b, T_t, Q_t, lo, hi, iters, tol):
        for _ in range(iters):
            T, Q = self.forward_norm(a, b)
            r1, r2 = T - T_t, Q - Q_t
            if abs(r1) < tol and abs(r2) < tol:
                break
            dTa, dTb = self._grad(self.c_T, a, b)
            dQa, dQb = self._grad(self.c_Q, a, b)
            det = dTa * dQb - dTb * dQa
            if det == 0.0:
                break
            da = (r1 * dQb - r2 * dTb) / det
            db = (dTa * r2 - dQa * r1) / det
            a = min(max(a - da, lo), hi)
            b = min(max(b - db, lo), hi)
        return a, b

    def _solve(self, T_t, Q_t, iters, tol):
        if not self._built:
            self._build()
        lo = self._norm(self.pwm_min)
        hi = self._norm(self.pwm_max)

        # Warm start first. Consecutive control commands differ by a fraction of
        # a newton, so the previous solution is already inside Newton's basin
        # and converges in two or three steps with no search at all. The seed
        # table is the cold path: Newton on a cubic is only locally convergent
        # and this surface is not monotone in b, so a fixed seed can converge
        # onto the wrong branch near the edges of the reachable set.
        seeds = []
        if self._warm is not None:
            seeds.append(self._warm)
        seeds.append(None)

        a = b = 0.0
        T = Q = 0.0
        for seed in seeds:
            if seed is None:
                best = None
                for sa, sb, sT, sQ in self._seeds:
                    # Scaled residual: 1 N and 0.01 N*m are comparable errors.
                    r = (sT - T_t) ** 2 + ((sQ - Q_t) / 0.01) ** 2
                    if best is None or r < best:
                        best, seed = r, (sa, sb)
            a, b = self._newton(seed[0], seed[1], T_t, Q_t, lo, hi, iters, tol)
            T, Q = self.forward_norm(a, b)
            if abs(T - T_t) < 1e-6 and abs(Q - Q_t) < 1e-6:
                break

        self._warm = (a, b)
        return a, b, T, Q

    def inverse(self, thrust_n, torque_nm, iters=40, tol=1e-9, thrust_tol=1e-4):
        """(thrust, axial torque) -> (PWM A, PWM B, achieved T, achieved tau_P).

        The achieved pair is returned rather than assumed because the clamping
        is part of the answer: a caller that logs the request instead of the
        result reports authority the vehicle never had.

        THRUST HAS PRIORITY, and enforcing that takes more than clamping the
        torque. The boundary of the reachable set is exactly where the Jacobian
        degenerates, so a target sitting ON it is the one case Newton cannot
        solve -- it stalls with a large thrust residual and would silently
        return a command producing over a newton less lift than asked. When that
        happens, back the torque off in stages and re-solve: the vehicle gives
        up axial authority, never lift.
        """
        t_lo, t_hi = self.thrust_limits()
        T_t = min(max(thrust_n, t_lo), t_hi)
        q_lo, q_hi = self.torque_limits_at(T_t)
        Q_t = min(max(torque_nm, q_lo), q_hi)

        a = b = T = Q = 0.0
        for shrink in (1.0, 0.97, 0.9, 0.7, 0.4, 0.0):
            a, b, T, Q = self._solve(T_t, Q_t * shrink, iters, tol)
            if abs(T - T_t) <= thrust_tol:
                break
        return self._denorm(a), self._denorm(b), T, Q


class GimbalAxisMap:
    """One gimbal ring: PWM <-> angle, with its own measured travel limits.

    Separate per ring on purpose. The two are different hardware -- the outer
    has 1.6x the gain per microsecond, 0.6x the slew rate and 2/3 the bandwidth
    of the inner -- and neither one's travel is symmetric about its own neutral.
    Treating them as one symmetric +/-7 deg actuator overstates negative travel
    on both.
    """

    def __init__(self, gain_deg_per_us, neutral_pwm, min_deg, max_deg,
                 rate_max_deg=180.0, bandwidth_hz=0.0):
        self.gain = float(gain_deg_per_us)
        self.neutral = float(neutral_pwm)
        self.min_deg = float(min_deg)
        self.max_deg = float(max_deg)
        self.rate_max_deg = float(rate_max_deg)
        self.bandwidth_hz = float(bandwidth_hz)

    def pwm_to_deg(self, pwm):
        return self.gain * (pwm - self.neutral)

    def deg_to_pwm(self, deg):
        return self.neutral + self.clamp_deg(deg) / self.gain

    def clamp_deg(self, deg):
        return min(max(deg, self.min_deg), self.max_deg)

    def normalized(self, rad):
        """Angle -> [-1, 1] against this ring's own travel, for PX4 servo output."""
        deg = self.clamp_deg(math.degrees(rad))
        span = self.max_deg if deg >= 0.0 else -self.min_deg
        return deg / span if span else 0.0
