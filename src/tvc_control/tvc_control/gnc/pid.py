"""
The one PID used by every loop in the cascade.
================================================================================
Position, altitude, attitude and rate all use this class; there is no second
implementation anywhere in the repository. Parallel form,

    u = kp*e + ki*integral(e) + kd*de/dt

with two deliberate features and nothing else:

  * `dt` is an ARGUMENT. Nothing here reads a clock, which is what lets the
    whole controller be stepped deterministically by a harness, and what makes
    a run reproducible.
  * `freeze` is conditional anti-windup. The caller sets it when the actuator
    this PID drives is saturated: continuing to integrate against a pinned
    actuator only builds a charge that must be paid back as overshoot once the
    limit clears. The proportional and derivative terms still respond.
"""

from .mathx import clamp


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
            self.integral = clamp(self.integral + err*dt,
                                  -self.i_limit, self.i_limit)
        deriv = (err - self.prev_err) / dt if dt > 0 else 0.0
        self.prev_err = err
        return self.kp*err + self.ki*self.integral + self.kd*deriv
