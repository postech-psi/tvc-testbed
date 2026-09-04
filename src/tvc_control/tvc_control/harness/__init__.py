"""
harness -- owns the clock and the transport. Neither flight code nor plant.
================================================================================
A harness picks a plant, picks a clock, wires the two to the flight code in
`gnc/`, and records what happened.

    mil.py   MODEL-IN-THE-LOOP. Analytic plant, fixed step, fully deterministic.
             No ROS, no Gazebo, no display. The fastest loop and the only one
             whose output can be frozen as a numerical baseline.
    gz.py    SOFTWARE-IN-THE-LOOP. gz-sim plant over gz-transport, no ROS.
             Steps on odometry arrival, so it runs in lockstep with the plant.

The ROS 2 pipelines are a third harness whose clock and transport are ROS's;
they live in `nodes/` because their shape is dictated by rclpy.

THE CLOCK LIVES HERE AND NOWHERE ELSE. Every function in `gnc/` takes `dt` as an
argument and never reads a clock of its own. That is what makes a run
reproducible, and reproducibility is what makes comparing two plants mean
anything: a tolerance against a run that cannot be repeated is not a tolerance.
"""
