# Everything the flight code decided in one control step, in one message.
#
# Replaces GimbalCommand, which carried only the two gimbal angles. The thrust
# and roll-torque commands were computed and then DISCARDED by controller_node,
# so the ROS2 path had no altitude loop and no roll authority at all -- both
# rotors were held at the same speed by the bridge.
#
# ONE MESSAGE, NOT SEVERAL TOPICS. Thrust, gimbal and tau_P must be time-aligned:
# the 1/cos(theta) feedforward and the tau_P cross-terms in the allocation are
# computed against each other, so splitting them lets a subscriber pair values
# from different control steps and quietly get a different vehicle.
#
# Frames and axis names follow docs/CONVENTIONS.md. The convention token travels
# with the data so a stale node talking to a fresh one fails loudly instead of
# reinterpreting numbers under the wrong names.

std_msgs/Header header

uint8 AXIS_CONVENTION_ROCKET_V2 = 1
uint8 AXIS_CONVENTION_ROCKET_V2 = 2
uint8 axis_convention

# --- gimbal, radians ---------------------------------------------------------
# delta1 is the inner ring (yaw plane), delta2 the outer ring (pitch plane).
# Named by index rather than by axis until the rename lands, so the field names
# do not have to change meaning when it does.
float64 gimbal_inner_rad
float64 gimbal_outer_rad

# --- motors, normalized [0, 1] ----------------------------------------------
# Against the measured surface's own pwm_min/pwm_max, the coordinate the bench
# characterisation is defined on. Also the shape PX4's ActuatorMotors wants, so
# the HAL between here and either transport is affine.
float64 motor_a
float64 motor_b

# --- what those commands are expected to achieve -----------------------------
# Predictions, not commands. Carried for logging and cross-plant comparison: a
# consumer that records the request instead of the achievement reports authority
# the vehicle never had.
float64 thrust_n
float64 tau_p_nm

# Rotor speeds for gz-sim's MulticopterMotorModel. A CONTROL ALLOCATION
# VARIABLE, not a physical RPM -- see tvc_control/hal/gazebo.py. Zero on any
# non-Gazebo transport.
float64 rotor_a_speed_rads
float64 rotor_b_speed_rads

# --- saturation, for the subscriber's own anti-windup ------------------------
bool sat_gimbal
bool sat_roll
bool sat_thrust
