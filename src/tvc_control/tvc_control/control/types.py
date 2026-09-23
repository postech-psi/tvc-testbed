"""Controller inputs and outputs: estimated state, targets, modes and actuator commands."""

from dataclasses import dataclass, field


def _v3():
    return (0.0, 0.0, 0.0)


@dataclass
class EstimatedState:
    """Vehicle state as the controller believes it to be.

    Frames per docs/GUIDE.md: body +z is the thrust axis; the quaternion
    is (qw, qx, qy, qz), inertial <- body; rates are body-frame.
    """

    pos_i: tuple = field(default_factory=_v3)      # m, inertial
    vel_i: tuple = field(default_factory=_v3)      # m/s, inertial
    quat: tuple = field(default_factory=lambda: (1.0, 0.0, 0.0, 0.0))
    omega_b: tuple = field(default_factory=_v3)    # rad/s, body
    stamp_s: float = 0.0                           # when this estimate was valid


@dataclass
class Setpoint:
    """What the vehicle is being asked to do.

    Which fields matter is decided by ControlMode, not by leaving fields unset:
    a mode that ignores a field should be visibly ignoring it rather than
    silently reading a default.
    """

    # Attitude targets, radians, rocket convention: pitch about body x, yaw
    # about body y, roll about body z (the thrust axis). docs/GUIDE.md.
    pitch_des: float = 0.0
    yaw_des: float = 0.0
    roll_des: float = 0.0

    z_des: float = 0.0                             # m, inertial altitude
    pos_des: tuple = field(default_factory=_v3)    # m, inertial x/y (z unused)


@dataclass
class ControlMode:
    """Which outer loops are closed this run.

    Explicit rather than inferred from whether a setpoint is non-zero: "hold
    position at the origin" and "do not run the position loop" are different
    commands and must not share a representation.
    """

    altitude_hold: bool = False
    position_hold: bool = False
    tilt_compensation: bool = True   # 1/cos(theta) feedforward on thrust


@dataclass
class ActuatorSetpoint:
    """What the flight code commands. A HAL converts this to transport units."""

    motor_a: float = 0.0             # [0,1], normalized against the surface
    motor_b: float = 0.0
    gimbal_inner_rad: float = 0.0    # inner ring -> yaw plane   (about body y)
    gimbal_outer_rad: float = 0.0    # outer ring -> pitch plane (about body x)

    thrust_n: float = 0.0            # expected achievement, not a command
    tau_p_nm: float = 0.0

    sat_gimbal: bool = False
    sat_roll: bool = False
    sat_thrust: bool = False
