"""PID controller with integral limiting."""

from .mathx import clamp


class PID:
    """Parallel-form PID with conditional anti-windup. `dt` is an argument, never
    a clock read, which is what makes every loop reproducible.
    """
    def __init__(self, kp, ki, kd, i_limit=1e9):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = i_limit
        self.integral = 0.0
        self.prev_err = 0.0

    def reset(self):
        """Clear the integrator and the previous error. Call between runs."""
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
