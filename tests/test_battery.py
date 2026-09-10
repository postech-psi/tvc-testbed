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


# --- wiring into the actuator chain ------------------------------------------
def test_chain_without_battery_is_unchanged():
    """battery=None must leave the achieved thrust exactly as the motor made it."""
    from tvc_control.config import load_vehicle_params
    from tvc_control.plant.actuators import ActuatorChain
    vp = load_vehicle_params()
    chain = ActuatorChain(vp)                    # no battery -> old behaviour
    for _ in range(50):
        _, T, _ = chain.update([0.0, 0.0], 13.0, 0.0, 0.01)
    assert abs(T - 13.0) < 1e-9                  # first_order lag settles to command


def test_chain_with_battery_derates_achieved_thrust_over_time():
    from tvc_control.config import load_vehicle_params
    from tvc_control.plant.actuators import ActuatorChain
    vp = load_vehicle_params()
    chain = ActuatorChain(vp, battery=_fresh(), current_per_n=34.7 / vp.T_max)
    _, T_early, _ = chain.update([0.0, 0.0], 13.0, 0.0, 0.01)
    for _ in range(2000):                        # 20 s of steady draw
        _, T_late, _ = chain.update([0.0, 0.0], 13.0, 0.0, 0.01)
    assert T_late < T_early                       # pack sag reduces achieved thrust


def test_closedloop_battery_raises_commanded_thrust():
    """Altitude hold compensates the sag, so ACHIEVED thrust stays ~mg but the
    COMMANDED per-rotor thrust must rise to overcome the derate."""
    from tvc_control.config import load_gains, load_vehicle_params
    from tvc_control.harness.mil import SimConfig, simulate
    vp, gains = load_vehicle_params(), load_gains()
    base = dict(t_final=15.0, altitude_hold=True, z_des=2.0, init_z=2.0)
    off = simulate(vp, gains, SimConfig(battery_sag=False, **base))
    on = simulate(vp, gains, SimConfig(battery_sag=True, **base))
    assert on["motor_N"][-1].sum() > off["motor_N"][-1].sum()
