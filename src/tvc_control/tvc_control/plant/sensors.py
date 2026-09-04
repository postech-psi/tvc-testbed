"""
Sensor models -- simulation only.
================================================================================
Today this is one identity function, and that is the point.

The controller consumes an EstimatedState, never the plant's true state, even
while the estimator is a passthrough. Writing the loops against truth and adding
noise later is not an extra module -- it is a rewrite of every loop that assumed
clean, instantaneous, unbiased measurements. Putting the seam in while it costs
nothing is what makes the later addition a drop-in.

What arrives here later, in roughly this order:
  - IMU noise, bias and the 250 Hz sample rate the SDF already publishes at
  - odometry latency (the estimate is about the past; EstimatedState.stamp_s
    exists so the controller can eventually know by how much)
  - an attitude/position estimator, at which point this file stops being a
    passthrough and the difference between truth and estimate becomes a thing
    the simulation can actually show
"""

from ..gnc.types import EstimatedState


class PerfectEstimator:
    """Ground truth, relabelled as an estimate.

    Deliberately not "no estimator at all": the type conversion is where the
    honesty lives. A controller that takes an EstimatedState cannot quietly
    start depending on something only a simulator knows.
    """

    def estimate(self, pos_i, vel_i, quat, omega_b, t):
        """Plant truth -> EstimatedState. Today the identity; the type is the point."""
        return EstimatedState(
            pos_i=tuple(float(v) for v in pos_i),
            vel_i=tuple(float(v) for v in vel_i),
            quat=tuple(float(v) for v in quat),
            omega_b=tuple(float(v) for v in omega_b),
            stamp_s=float(t),
        )
