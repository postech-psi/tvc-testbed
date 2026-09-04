"""
tvc_plant -- SIMULATION ONLY. None of this ever flies.
================================================================================
Rigid-body dynamics, actuator dynamics (servo slew, transport delay, motor lag),
and sensor models. The vehicle itself supplies all of this in hardware; here it
is modelled so the flight code in `gnc/` can be exercised without one.

The hard rule that gives the split its value: `gnc/` must never import from
here. A controller that reaches into the plant is a controller that cannot fly,
and the dependency is one-way precisely so that violating it fails to import
rather than passing review.
"""
