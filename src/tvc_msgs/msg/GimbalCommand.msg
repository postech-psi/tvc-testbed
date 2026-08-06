# Commanded 2-axis gimbal deflection, in radians.
# Matches the (delta1, delta2) convention in tvc_control/physics.py:
# delta1 = pitch-plane deflection, delta2 = roll-plane deflection.

std_msgs/Header header
float64 delta1
float64 delta2
