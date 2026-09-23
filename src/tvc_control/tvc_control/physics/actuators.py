"""Gimbal delay and slew limits, motor response and optional battery derating."""

import numpy as np

from ..control.params import VehicleParams


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
        """Return the servo to centre and empty the command FIFO."""
        self.delta = np.zeros(2)
        self._pending = []

    def update(self, delta_cmd, dt):
        """Commanded deflection -> ACHIEVED deflection after deadtime and slew."""
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


class MotorLag:
    """Command -> achieved (thrust, roll torque), with the measured lag.

    Acts on the PHYSICAL PAIR, not on rotor speed, because (T, tau_P) is what
    the bench measured. Filtering a rotor speed instead would put the lag on a
    quantity nobody characterised and, since T ~ omega^2, would not even be a
    first-order lag in thrust.

    The model is switchable (see motor_dynamics in settings/vehicle.yaml) because
    the bench record does not say whether the 100 ms is a transport delay or a
    time constant, and the roll channel's usable gain depends on which. Guessing
    silently would bury that in the code; the switch keeps it a stated
    assumption with a measurable answer.
    """

    def __init__(self, model="first_order", tau_s=0.10, deadtime_s=0.0):
        if model not in ("first_order", "delay", "delay_plus_lag"):
            raise ValueError("unknown motor lag model %r" % model)
        self.model = model
        self.tau_s = float(tau_s)
        self.deadtime_s = float(deadtime_s)
        self.reset()

    def reset(self):
        """Un-initialise the lag: the next command starts settled."""
        self.T = None            # None = not yet initialised; see update()
        self.tau_p = 0.0
        self._pending = []

    def update(self, T_cmd, tau_p_cmd, dt):
        """Commanded (thrust, roll torque) -> what the motors actually produce."""
        # Transport delay first, as a FIFO of commands: that is what an ESC
        # queue does, and a filter would smear an effect that is actually a
        # clean shift in time.
        if self.model in ("delay", "delay_plus_lag") and self.deadtime_s > 0.0:
            n = int(round(self.deadtime_s / dt)) if dt > 0 else 0
            self._pending.append((T_cmd, tau_p_cmd))
            if len(self._pending) > n:
                T_cmd, tau_p_cmd = self._pending.pop(0)
            else:
                # Nothing has arrived yet. Hold the current state rather than
                # commanding zero, which would be a thrust dropout at t=0.
                T_cmd = T_cmd if self.T is None else self.T
                tau_p_cmd = self.tau_p

        if self.T is None:
            # Start settled on the first command. Starting from zero thrust
            # would make every run begin with an unmodelled free-fall transient
            # that has nothing to do with the controller under test.
            self.T, self.tau_p = T_cmd, tau_p_cmd
            return self.T, self.tau_p

        if self.model in ("first_order", "delay_plus_lag") and self.tau_s > 0.0:
            a = dt / (self.tau_s + dt)
            self.T += a * (T_cmd - self.T)
            self.tau_p += a * (tau_p_cmd - self.tau_p)
        else:
            self.T, self.tau_p = T_cmd, tau_p_cmd
        return self.T, self.tau_p


class ActuatorChain:
    """Every analytic actuator dynamic in one object, so analytic paths agree.

    The gimbal and the motors have different lags (30 ms transport on the
    servos, ~100 ms on thrust) and the difference is the reason the roll channel
    is authority-rich and bandwidth-poor despite its small inertia. Bundling
    them keeps analytic simulation runs from having
    accidentally model different subsets.
    """

    def __init__(self, params, motor_model="first_order", motor_tau_s=0.10,
                 motor_deadtime_s=0.0, battery=None, current_per_n=0.0):
        self.gimbal = GimbalActuator(params)
        self.motor = MotorLag(motor_model, motor_tau_s, motor_deadtime_s)
        # OPTIONAL battery-sag stage. None (the default) leaves update() doing
        # exactly what it did before, so the toggle-off path is bit-identical.
        # current_per_n turns achieved thrust into an approximate pack current
        # (linear: peak_current_a / T_max) for the coulomb count.
        self.battery = battery
        self.current_per_n = float(current_per_n)

    def reset(self):
        """Reset every stage -- gimbal FIFO, motor lag, and the battery if present."""
        self.gimbal.reset()
        self.motor.reset()
        if self.battery is not None:
            self.battery.reset()

    def update(self, delta_cmd, T_cmd, tau_p_cmd, dt):
        """-> (achieved delta, achieved T, achieved tau_P).

        With a battery attached, the achieved thrust is derated for the pack's
        state of charge: the motor produces its commanded thrust, then a sagged
        pack cannot sustain it. tau_P is left as the motor produced it -- the
        sag model is a thrust-only derate (see physics/battery.py)."""
        delta = self.gimbal.update(delta_cmd, dt)
        T, tau_p = self.motor.update(T_cmd, tau_p_cmd, dt)
        if self.battery is not None:
            self.battery.update(self.current_per_n * T, dt)
            T = self.battery.derate(T)
        return delta, T, tau_p
