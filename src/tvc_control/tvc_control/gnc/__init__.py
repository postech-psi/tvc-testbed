"""
gnc -- FLIGHT CODE. This is what runs on the vehicle.
================================================================================
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

PORTING DISCIPLINE, to review explicitly whenever this directory changes:

    imports          math, dataclasses and typing ONLY. No numpy, no scipy, no
                     yaml, no ROS. An ndarray has no fixed-size C++ counterpart,
                     so every array in the control path would become a design
                     decision during the port instead of a mechanical
                     translation; a scipy call is an algorithm someone would
                     have to reimplement under time pressure without the tests
                     that covered the original.
    no file I/O      parameters are injected once at startup, never looked up.
                     That is the shape PX4's parameter system already has, and
                     it makes a control step's worst case independent of a
                     filesystem. The loader is tvc_control/config.py, outside
                     this package, on purpose.
    no global state  a module-level list is a shared buffer waiting to be found
                     by the second controller instance.
    no `while`       every iteration count comes from a `range`, so worst-case
                     execution time is a number someone can write down. The two
                     numerical iterations here -- the allocation's Newton
                     refinement and the surface inverse -- carry their bound at
                     the call site.
    dt is an argument  nothing here reads a clock. That is what makes a run
                     reproducible, and reproducibility is what makes comparing
                     two plants mean anything.

Read in this order: params -> mathx -> pid -> allocation -> attitude ->
altitude -> position -> controller. types.py is the two seams; effectiveness.py
is the measured hardware model everything else asks about feasibility.
docs/3-THEORY.md derives every equation in here.
"""
