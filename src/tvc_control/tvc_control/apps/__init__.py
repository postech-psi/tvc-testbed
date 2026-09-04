"""
apps -- desktop tools. They read the simulator; they are not part of it.
================================================================================
    gui.py     Tkinter form over the analytic harness: edit parameters and gains,
               run, see the response.
    view3d.py  Animates a run's quaternion history on the real CAD mesh.
    plot.py    Renders a Gazebo flight log (CSV) as four stacked plots.
    record.py  Captures the Gazebo chase camera into an animated GIF.

None of these needs ROS or Gazebo except record.py, which reads a gz topic.
All are reachable from `python tvc.py <name>`.
"""
