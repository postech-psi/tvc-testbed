"""
nodes -- ROS 2 wrappers. Thin by rule: no control math lives here.
================================================================================
Each node subscribes, calls into gnc/ or plant/, and publishes. If a formula
appears in this directory it is in the wrong place -- the point of the split is
that the ROS 2 path and the analytic path run identical control code, and a
formula written here would be a fourth copy of something.

    controller.py     state in -> TvcController -> ActuatorCommand out
    simulator.py      ActuatorCommand in -> analytic plant -> Odometry out
    gazebo_bridge.py  ActuatorCommand in -> the three gz-sim plugin topics

controller.py + simulator.py is the ROS-only pipeline (no physics engine).
controller.py + gazebo_bridge.py + gz-sim is the software-in-the-loop pipeline.
The two launch files differ only in which plant process starts.
"""

from rcl_interfaces.msg import ParameterDescriptor      # noqa: E402
from rclpy.parameter import Parameter                   # noqa: E402


def reject_retired_parameters(node, retired):
    """Refuse to start if a launch file still sets a parameter that was retired.

    A parameter that looks live while being ignored is worse than one that is
    gone: it makes a launch file document a control decision that is not
    happening. So setting one is a hard stop, not a warning.

    THE DESCRIPTOR IS NOT OPTIONAL. This was written as
        node.declare_parameter(name, Parameter.Type.NOT_SET)
    in both nodes, and rclpy on Jazzy rejects that outright --
    "Cannot declare parameter as statically typed of type NOT_SET" -- so the
    guard against a retired parameter killed every node whether one was set or
    not. `ros2 launch` had never been run, so nothing caught it. Declaring with
    dynamic_typing is the supported way to ask "did anyone pass this?".

    It lives here rather than in each node because there were two copies and
    they had already drifted apart in which names they listed.

    `retired` maps each name to what to do instead. The advice is per-name
    because "this is gone" without "use that" just moves the search to the git
    log.
    """
    for name, advice in sorted(retired.items()):
        node.declare_parameter(name, None,
                               ParameterDescriptor(dynamic_typing=True))
        if node.get_parameter(name).type_ != Parameter.Type.NOT_SET:
            raise SystemExit(
                "parameter '%s' was retired and is being ignored: %s "
                "See docs/4-CONVENTIONS.md." % (name, advice))
