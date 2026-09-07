# Everything the flight code decided in one control step, in one message.
#
# ONE MESSAGE, NOT SEVERAL TOPICS. Thrust, gimbal and tau_P must be
# time-aligned: the 1/cos(theta) thrust feedforward and the tau_P cross-terms in
# the allocation are computed against each other, so splitting them across
# topics lets a subscriber pair values from different control steps and quietly
# fly a different vehicle.
#
# Frames, axis names and signs: docs/4-CONVENTIONS.md. The convention token
# travels WITH the data so a stale node talking to a fresh one fails loudly
# instead of reinterpreting numbers under the wrong names -- the failure mode
# that makes an axis rename dangerous.

std_msgs/Header header

# 0 is not a valid convention. A default-constructed message therefore fails the
# receiver's check instead of being accepted as whatever the enum's first entry
# happens to be.
uint8 AXIS_CONVENTION_UNSET = 0
uint8 AXIS_CONVENTION_ROCKET_V2 = 2
uint8 axis_convention

# --- gimbal, radians ---------------------------------------------------------
# Named after the mechanical RING, not the axis. A ring is a fact about the
# hardware and is invariant under any naming convention, so a future revision of
# the convention does not have to touch this message, the SDF, or the gz topics.
#   inner ring -> yaw plane   (rotates about body y)
#   outer ring -> pitch plane (rotates about body x)
float64 gimbal_inner_rad
float64 gimbal_outer_rad

# --- motors, normalized [0, 1] ----------------------------------------------
# Against the measured surface's own pwm_min/pwm_max, the coordinate the bench
# characterisation is defined on. Also the shape PX4's ActuatorMotors wants, so
# the HAL between here and either transport is affine.
float64 motor_a
float64 motor_b

# --- what those commands are expected to achieve -----------------------------
# PREDICTIONS, NOT COMMANDS. The allocator clamps to the vehicle's feasible set,
# so what was asked for and what will be produced differ whenever a limit binds.
# Carried for logging and cross-plant comparison: a consumer that records the
# request instead of the achievement reports authority the vehicle never had.
float64 thrust_n
float64 tau_p_nm

# --- saturation diagnostics --------------------------------------------------
# The controller already applies anti-windup internally. These flags tell logs,
# monitors, and future hardware safety logic which authority limit was active.
bool sat_gimbal
bool sat_roll
bool sat_thrust
