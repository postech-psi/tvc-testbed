"""Convert simulator truth to EstimatedState. A real sensor estimator is not implemented."""

from ..control.types import EstimatedState


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
