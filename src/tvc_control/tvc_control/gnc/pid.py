"""
The one PID used by every loop in the cascade.
================================================================================
Moved verbatim from physics.py.
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
