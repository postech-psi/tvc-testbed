"""
sim/vehicle_params.py -- COMPATIBILITY SHIM.
================================================================================
The loader and vehicle_params.yaml moved into the ROS package
(tvc_control/config.py, tvc_control/vehicle_params.yaml) so that an installed
node can reach them. Before the move, physics.py and gazebo_bridge_node.py each
carried literal fallback constants for the case where sim/ was not on the path,
and those fallbacks went stale -- which is the bug the move removes.

This shim keeps tools/, sim/hover.py and sim/plot_flight.py importing
`vehicle_params` unchanged. New code should import tvc_control.config directly.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "tvc_control"))

from tvc_control.config import (  # noqa: E402,F401
    Vehicle, load, load_gains, load_vehicle_params, _DEFAULT_PATH,
    SUPPORTED_AXIS_CONVENTIONS,
)

if __name__ == "__main__":
    from tvc_control import config
    config.main() if hasattr(config, "main") else None
