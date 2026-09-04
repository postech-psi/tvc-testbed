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
