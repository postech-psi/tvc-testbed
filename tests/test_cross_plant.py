"""
Tests for the cross-plant validation harness. The comparator is tested
hermetically; run_analytic is checked for sane, matched-scenario output.
"""
from tvc_control.verify import cross_plant as X


def test_compare_flags_out_of_tolerance():
    a = {"final_altitude_m": 2.00, "peak_tilt_deg": 12.2}
    b = {"final_altitude_m": 2.01, "peak_tilt_deg": 20.0}
    tol = {"final_altitude_m": 0.05, "peak_tilt_deg": 1.5}
    d = {x.metric: x for x in X.compare(a, b, tol)}
    assert d["final_altitude_m"].within          # 0.01 <= 0.05
    assert not d["peak_tilt_deg"].within         # 7.8 > 1.5


def test_compare_only_shared_metrics_with_tol():
    a = {"final_altitude_m": 2.0, "unmatched": 1.0}
    b = {"final_altitude_m": 2.0}
    out = X.compare(a, b, X.CROSS_PLANT_TOLERANCES)
    names = {x.metric for x in out}
    assert "final_altitude_m" in names
    assert "unmatched" not in names


def test_run_analytic_hover_is_sane():
    from tvc_control.config import load_gains, load_vehicle_params
    vp, gains = load_vehicle_params(), load_gains()
    # A short matched hover keeps the test fast.
    spec = X.ScenarioSpec(name="hover", init_pitch_deg=10.0, init_yaw_deg=-7.0,
                          t_final=6.0, z_des=2.0, init_z=2.0)
    m = X.run_analytic(spec, vp, gains)
    # Spawned tilted (~12.2 deg), recovers toward level, holds ~2 m.
    assert abs(m["peak_tilt_deg"] - 12.2) < 1.0
    assert m["final_tilt_deg"] < 2.0
    assert abs(m["final_altitude_m"] - 2.0) < 0.1


def test_run_gazebo_is_stubbed_outside_container():
    import pytest
    with pytest.raises(NotImplementedError):
        X.run_gazebo(X.HOVER)


def test_golden_loads():
    g = X.load_gazebo_golden()
    assert "final_altitude_m" in g and "tilt_settling_time_s" in g
