"""
The two seams: what the flight code consumes, and what it produces.
================================================================================
These are the entire interface between the controller and everything else. A
harness fills an EstimatedState and a Setpoint, calls TvcController.update, and
hands the resulting ActuatorSetpoint to a HAL. Nothing else crosses.

WHY EstimatedState AND NOT THE TRUE STATE
    On the vehicle the controller will be fed an estimator's output, not truth,
    and an estimator brings noise, bias and latency. If the controller is
    written against truth today, adding those later is not a new module -- it is
    a rewrite of every loop that assumed clean measurements.

    So the seam exists from the start and today's estimator is the identity
    (PerfectEstimator, in plant/sensors.py). Sensor models and an EKF slot in
    behind it later without the controller changing at all. `stamp_s` is here
    for the same reason: estimator latency is certain to appear, and adding the
    field once code depends on the struct means touching every call site.

WHY ActuatorSetpoint LOOKS THE WAY IT DOES
    The motor commands are normalized [0,1] against the measured surface's own
    pwm_min/pwm_max, and the gimbal commands are in radians. That asymmetry is
    deliberate: each is the natural coordinate of its own calibration -- the
    thrust/torque surface is a function of normalized commands, and the
    allocator solves for angles. Converting either one earlier would push a
    calibration into the controller or a control decision into a HAL.

    The shape also matches where this is going. With
    OffboardControlMode.direct_actuator, PX4 disables its own allocator and the
    external controller supplies ActuatorMotors/ActuatorServos directly --
    normalized, and in the FRD body frame. Both the near-term offboard path and
    a future in-tree PX4 module converge on that same uORB interface, so the
    frame conversion (docs/4-CONVENTIONS.md, PX4 boundary) belongs in a PX4 HAL
    and appears nowhere in this package.

    thrust_n and tau_p_nm are the values the allocation EXPECTS to achieve, not
    commands. They are carried for logging and cross-plant comparison; a caller
    that logs the request instead of the achievement reports authority the
    vehicle never had.
"""

from dataclasses import dataclass, field


def _v3():
    return (0.0, 0.0, 0.0)


@dataclass
class EstimatedState:
    """Vehicle state as the controller believes it to be.

    Frames per docs/4-CONVENTIONS.md: body +z is the thrust axis; the quaternion
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
    # about body y, roll about body z (the thrust axis). docs/4-CONVENTIONS.md.
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
