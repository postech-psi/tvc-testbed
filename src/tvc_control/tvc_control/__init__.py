"""
tvc_control -- everything that flies the vehicle, and everything that pretends to.

The package is split into six layers with a strictly one-way dependency. Reading
them in this order is reading the simulator in order:

    gnc/        FLIGHT CODE. Controller, allocation, actuator effectiveness.
                Runs on the vehicle. Pure Python by enforced rule.
    plant/      SIMULATION ONLY. Rigid body, actuator lag, sensor models.
                Never flies. gnc/ may not import from here.
    hal/        Transport adapters. Converts one ActuatorSetpoint into one
                transport's units. No control.
    harness/    Owns the clock and the I/O. Picks a plant, wires it to gnc/,
                records what happened.
    nodes/      ROS 2 wrappers. Thin: no control math lives here.
    apps/       Desktop tools -- GUI, 3D viewer, plotter, GIF recorder.
    verify/     The scenario suite and the frozen numerical baseline.

    config.py   The ONLY place vehicle numbers enter the program.

Everything runnable is reachable from one entry point:

    python tvc.py --help

See docs/1-ARCHITECTURE.md for the file-by-file map and the two seams.
"""
