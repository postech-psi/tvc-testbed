"""
hal -- transport adapters. Neither flight code nor plant.
================================================================================
A HAL converts an ActuatorSetpoint into one transport's units and does no
control of its own. Keeping them here rather than inside the nodes means the
same conversion is used by every path that talks to that transport: the ROS2
bridge and the standalone gz-transport controller cannot drift apart, because
there is only one function.

  gazebo.py   normalized commands -> gz.msgs.Actuators rotor speeds
  (later) px4.py     -> ActuatorMotors / ActuatorServos, plus the FRD frame
                        conversion (docs/CONVENTIONS.md). That conversion lives
                        here and nowhere else; FRD never appears inside gnc/.
  (later) bench.py   -> raw PWM for the ESCs and servos
"""
