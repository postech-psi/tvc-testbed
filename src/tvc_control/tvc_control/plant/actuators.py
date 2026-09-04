"""
Actuator dynamics: what the commanded deflection actually does. Simulation only.
================================================================================
Moved verbatim from physics.py.
"""

import numpy as np

from ..gnc.params import VehicleParams


class GimbalActuator:
    """Rate-limited servo model for the 2-axis gimbal, with transport delay.

    Two separate lags, often confused:
      RATE LIMIT (gimbal_rate_max) -- how fast the servo can slew once moving.
      DEADTIME   (gimbal_deadtime) -- how long after a command arrives before
                                      the servo moves at all.
    Deadtime is the one that costs phase margin: it is pure delay, so it eats
    gimbal_deadtime * omega radians of phase at every frequency and cannot be
    compensated by a lead term the way a first-order lag can. At 30 ms it is
    worth 17 deg of phase at the 10 rad/s lateral crossover, which is why the
    lateral gains cannot simply be raised until the response looks fast.

    The delay is modeled as a FIFO of commands rather than a filter, because
    that is what a serial servo bus actually does -- the command sits in a
    queue, then executes in full.
    """

    def __init__(self, params: VehicleParams):
        self.p = params
        self.delta = np.zeros(2)   # current [delta1, delta2], rad
        self._pending = []         # command FIFO, oldest first

    def reset(self):
        self.delta = np.zeros(2)
        self._pending = []

    def update(self, delta_cmd, dt):
        # Per-axis, asymmetric stops and per-axis slew: the inner ring reaches
        # 403 deg/s and the outer only 235, so a symmetric shared limit either
        # slows the inner axis or lets the outer one move faster than it can.
        delta_cmd = np.clip(np.asarray(delta_cmd, dtype=float),
                            self.p.delta_min, self.p.delta_max)

        # Delay by round(deadtime/dt) control steps. After appending, the FIFO
        # holds the last n+1 commands, so popping the oldest yields the command
        # issued n steps ago. Before the queue has filled (first n steps) there
        # is no command to execute yet, so the servo holds position.
        n_delay = int(round(self.p.gimbal_deadtime_s / dt)) if dt > 0 else 0
        self._pending.append(np.asarray(delta_cmd, dtype=float))
        if len(self._pending) > n_delay:
            target = self._pending.pop(0)
        else:
            target = self.delta.copy()

        # gnc hands out plain tuples now; the plant is where they become arrays.
        max_step = np.asarray(self.p.delta_rate_max, dtype=float) * dt
        step = np.clip(target - self.delta, -max_step, max_step)
        self.delta = self.delta + step
        return self.delta.copy()
