"""Run position, altitude, attitude and allocation calculations in that order."""

from .types import ActuatorSetpoint, ControlMode, EstimatedState, Setpoint
from .attitude import AttitudeController
from .altitude import AltitudeController
from .position import PositionController


class TvcController:
    """Flight code. Owns no clock, no I/O, and no plant."""

    def __init__(self, params, gains, mode=None):
        self.p = params
        self.gains = gains
        self.mode = mode or ControlMode()

        self.attitude = AttitudeController(params, gains)
        self.altitude = AltitudeController(params, gains,
                                           tilt_compensation=self.mode.tilt_compensation)
        self.position = PositionController(gains)

        # Last commanded gimbal deflection. This is the flight-code estimate of
        # where the gimbal is: the PTK 8515 servos give no position feedback, so
        # on the vehicle there is nothing better available, and the tilt
        # feedforward has to be computed against the command.
        self._delta_cmd = (0.0, 0.0)

    def reset(self):
        """Return every loop to its start-of-run state."""
        self.attitude.reset()
        self.altitude.reset()
        self._delta_cmd = (0.0, 0.0)

    def update(self, state: EstimatedState, setpoint: Setpoint, dt,
               gimbal_rad=None) -> ActuatorSetpoint:
        """One control step. Returns what to send to the actuators.

        gimbal_rad -- the ACHIEVED gimbal deflection, if the caller happens to
        know it. Simulation does; hardware does not. Passing it keeps the
        analytic harness bit-identical to its pre-facade behaviour, and leaving
        it None (the flight case) falls back to the last command, which differs
        only by the servo's own lag.
        """
        pitch_des, yaw_des = setpoint.pitch_des, setpoint.yaw_des

        # --- 1. position -> attitude setpoint --------------------------------
        if self.mode.position_hold:
            pitch_des, yaw_des = self.position.update(
                state.pos_i, state.vel_i, setpoint.pos_des)

        # --- 2. altitude -> thrust -------------------------------------------
        delta = self._delta_cmd if gimbal_rad is None else gimbal_rad
        if self.mode.altitude_hold:
            T_cmd = self.altitude.update(state.pos_i[2], state.vel_i[2],
                                         state.quat, delta, setpoint.z_des, dt)
        else:
            # No altitude loop: hold hover thrust. Not zero and not T_max --
            # the allocator's feasible set depends on T, so a placeholder here
            # would silently change how much authority the other axes get.
            T_cmd = self.p.m * self.p.g

        # --- 3 & 4. attitude cascade and allocation --------------------------
        delta_cmd = self.attitude.update(state.quat, state.omega_b,
                                         pitch_des, yaw_des, T_cmd, dt,
                                         roll_des=setpoint.roll_des)
        alloc = self.attitude.last_alloc
        self._delta_cmd = (float(delta_cmd[0]), float(delta_cmd[1]))

        return ActuatorSetpoint(
            motor_a=alloc.u_a,
            motor_b=alloc.u_b,
            gimbal_inner_rad=float(delta_cmd[0]),
            gimbal_outer_rad=float(delta_cmd[1]),
            thrust_n=alloc.T_cmd,
            tau_p_nm=alloc.tau_p,
            sat_gimbal=alloc.gimbal_saturated,
            sat_roll=alloc.roll_saturated,
            sat_thrust=alloc.thrust_saturated or self.altitude.thrust_saturated,
        )
