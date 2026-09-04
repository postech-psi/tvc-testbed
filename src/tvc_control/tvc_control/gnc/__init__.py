"""
tvc_gnc -- FLIGHT CODE. This is what runs on the vehicle.
================================================================================
(The restructure plan calls this package `tvc_gnc`; inside the ROS package
`tvc_control` it is `gnc`, so imports read `tvc_control.gnc` rather than the
stuttering `tvc_control.tvc_gnc`.)

WHAT BELONGS HERE
    The controller, the control allocation, the actuator effectiveness models,
    and the parameter struct they read. Anything that must execute on the
    vehicle to make it fly.

WHAT DOES NOT
    Rigid-body dynamics, actuator lag models, sensor models, integrators,
    plotting, ROS, Gazebo, file I/O. Those live in `plant/` and `harness/`.
    `dynamics()` never flies, so it is not in here -- that separation is the
    whole point of the split, and keeping it honest is what makes "the code we
    tested is the code that flies" a checkable claim rather than a hope.

WHY THE SEPARATION IS PHYSICAL AND NOT JUST TIDINESS
    PX4's flight code is unaware that a simulator exists; what a simulation
    swaps is the sensor/actuator boundary, not the controller. Mirroring that
    here means the eventual PX4 module is a port of this directory and nothing
    else, and it means a test that exercises this directory is testing flight
    behaviour rather than a simulation-flavoured cousin of it.

PORTING DISCIPLINE (enforced by tests/test_gnc_purity.py, arriving with the
purity conversion): no numpy/scipy/yaml/ROS imports, no dynamic allocation in
the control path, no exceptions as control flow, no clock reads -- every
function takes dt -- and every numerical iteration carries a fixed upper bound
so worst-case execution time is stated rather than hoped for.
"""
