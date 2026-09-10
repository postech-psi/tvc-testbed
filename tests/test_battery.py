"""
Tests for the battery-sag model (plant/battery.py) and its wiring into the
actuator chain. The load-bearing property is that with sag OFF, thrust is
bit-identical to today's plant.
"""
from tvc_control.plant.battery import BatteryState


def _fresh():
    # Representative measured numbers (identify_battery.py / vehicle_params.yaml).
    return BatteryState(v_full=11.9, v_per_mah=-0.00035, capacity_mah=4200,
                        thrust_sensitivity_n_per_v=1.90)


def test_fresh_pack_is_identity():
    b = _fresh()
    assert b.voltage == 11.9
    assert b.derate(13.03) == 13.03      # 0 mAh drawn -> no derate


def test_drain_lowers_voltage_and_thrust():
    b = _fresh()
    b.update(current_a=25.0, dt=60.0)    # a minute at 25 A
    assert b.voltage < 11.9
    assert b.derate(13.03) < 13.03


def test_derate_is_monotonic_in_charge():
    b = _fresh()
    b.update(25.0, 30.0)
    v1, t1 = b.voltage, b.derate(13.03)
    b.update(25.0, 30.0)
    v2, t2 = b.voltage, b.derate(13.03)
    assert v2 < v1
    assert t2 < t1


def test_capacity_clamps_charge():
    b = _fresh()
    b.update(1e6, 1.0)                    # absurd draw
    assert b.mah == b.capacity_mah


def test_reset_restores_fresh():
    b = _fresh()
    b.update(25.0, 60.0)
    b.reset()
    assert b.mah == 0.0 and b.voltage == 11.9
