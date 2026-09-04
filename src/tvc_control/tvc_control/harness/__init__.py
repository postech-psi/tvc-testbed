"""
harness -- owns the clock and the transport. Neither flight code nor plant.
================================================================================
A harness picks a plant, picks a clock, wires the two to the flight code in
`gnc/`, and records what happened. `mil.py` is the model-in-the-loop harness:
analytic plant, fixed step, fully deterministic. The Gazebo (software-in-the-
loop) harnesses live in `sim/hover.py` and the ROS2 nodes.

The clock lives here and nowhere else. The controller takes `dt` as an argument
and never reads a clock of its own, which is what makes a run reproducible and
therefore what makes comparing two plants mean anything.
"""
