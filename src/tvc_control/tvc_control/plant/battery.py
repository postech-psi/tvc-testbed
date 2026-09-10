"""
Battery-sag model -- simulation only.
================================================================================
The measured thrust/torque surface is a FRESH-PACK fit. On a real flight the
pack sags and thrust falls with it -- 13.8 -> 12.0 N over a sustained run, ~13%.
This model reproduces that fade so a long-hover or climb scenario sees the
authority it will actually have late in a flight, not the fresh-pack optimum.

HOW IT WORKS
    Coulomb counting: integrate the drawn current into charge (mAh), map charge
    to voltage on the measured discharge slope, and derate thrust by the measured
    sensitivity relative to a reference voltage. At the reference voltage the
    derate is the identity, so a fresh pack reproduces the surface exactly.

    voltage(mAh)   = v_full + v_per_mah * mAh            (v_per_mah < 0)
    T_actual       = T_nominal - k * (v_ref - voltage)   (clamped >= 0)

All four numbers come from tvc-data/motor/identify_battery.py, which fits them
from the seven sustained A1850/B1850 runs; they enter vehicle_params.yaml tagged
measured. This is OFF by default (SimConfig.battery_sag) -- with it off nothing
in the plant changes and the frozen baselines are untouched.
"""


class BatteryState:
    """Pack state under coulomb counting; derates thrust as it drains.

    v_ref defaults to v_full: a fresh pack (0 mAh drawn) sits at v_full and its
    derate is exactly zero, so the surface is reproduced unchanged until charge
    is drawn.
    """

    def __init__(self, v_full, v_per_mah, capacity_mah,
                 thrust_sensitivity_n_per_v, v_ref=None):
        self.v_full = float(v_full)
        self.v_per_mah = float(v_per_mah)        # negative: volts lost per mAh
        self.capacity_mah = float(capacity_mah)
        self.k = float(thrust_sensitivity_n_per_v)
        self.v_ref = float(v_ref) if v_ref is not None else float(v_full)
        self.reset()

    def reset(self):
        """Fresh pack: no charge drawn, voltage at v_full."""
        self.mah = 0.0
        self.voltage = self.v_full

    def update(self, current_a, dt):
        """Draw `current_a` for `dt` seconds; return the resulting pack voltage.

        Charge is clamped at the pack capacity so voltage does not run off the
        fitted line past empty -- the fit has no data there.
        """
        self.mah = min(self.mah + max(current_a, 0.0) * dt / 3.6,
                       self.capacity_mah)
        self.voltage = self.v_full + self.v_per_mah * self.mah
        return self.voltage

    def derate(self, thrust_n):
        """Fresh-pack thrust command -> what the sagged pack can actually make."""
        return max(0.0, thrust_n - self.k * (self.v_ref - self.voltage))
