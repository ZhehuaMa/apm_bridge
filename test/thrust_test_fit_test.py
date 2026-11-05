"""fpv_test.md §2/§3/§4 pipeline tests: SOP schedule, normalization, fit and closed loop.

Pure Python (numpy only): exercises scripts/propulsion_model.py and the SOP helpers of
scripts/thrust_test_node.py against the synthetic physical plant in propulsion_plant.py.
"""

import importlib.util
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))
sys.path.insert(0, HERE)
import propulsion_model as pm  # noqa: E402
import propulsion_plant as plant  # noqa: E402

_spec = importlib.util.spec_from_file_location('thrust_test_node',
                                               os.path.join(HERE, '..', 'scripts', 'thrust_test_node.py'))
tool = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(tool)

LEVELS = [round(0.05 * i, 2) for i in range(1, 21)]


# ---------------------------------------------------------------- §2.2 SOP

def test_sop_default_levels_are_twenty_steps_of_005():
    assert tool.sop_levels(0.05, 0.05, 1.00) == pytest.approx(LEVELS)
    assert tool.sop_levels(0.1, 0.25, 0.9)[-1] == pytest.approx(0.9)
    with pytest.raises(ValueError):
        tool.sop_levels(0.0, 0.05, 1.0)


def test_sop_timeline_settle_sample_and_cooldown():
    timeline = tool.sop_timeline(LEVELS, 2.5, 0.5, 0.70, 5.0, 0.05)
    samples = [(u, d) for action, u, d in timeline if action == 'sample']
    assert [u for u, _ in samples] == pytest.approx(LEVELS)
    assert all(d == pytest.approx(2.0) for _, d in samples)
    assert all(d == pytest.approx(0.5) for action, _, d in timeline if action == 'settle')
    cooldowns = [i for i, (action, _, _) in enumerate(timeline) if action == 'cooldown']
    # one idle cooldown right after each of the u >= 0.70 steps (0.70 .. 1.00)
    assert len(cooldowns) == 7
    for i in cooldowns:
        assert timeline[i - 1][0] == 'sample' and timeline[i - 1][1] >= 0.70 - 1e-9
        assert timeline[i][1:] == (0.05, 5.0)
    total = sum(d for _, _, d in timeline)
    assert total == pytest.approx(20 * 2.5 + 7 * 5.0)
    with pytest.raises(ValueError):
        tool.sop_timeline(LEVELS, 2.5, 2.5, 0.7, 5.0, 0.05)


def test_average_window_synchronous_mean():
    window = tool.new_window(0.4, 0.365, 4)
    window['weight'] = [4.0, 3.9, 4.1]
    window['V'] = [22.0, 22.2, float('nan')]
    window['I'] = [10.0, 12.0]
    window['rpm'] = [[1000.0, 1100.0], [float('nan')], [900.0], [1000.0]]
    window['P'] = [100000.0]
    point = tool.average_window(window, 5.0, 10.0)
    assert point['F'] == pytest.approx(10.0)
    assert point['V'] == pytest.approx(22.1)
    assert point['I'] == pytest.approx(11.0)
    assert point['rpm'][0] == pytest.approx(1050.0) and math.isnan(point['rpm'][1])
    assert math.isnan(point['t']) and point['n_weight'] == 3


def test_duty_collective_mapping():
    assert pm.collective_to_duty(0.0, 0.055) == pytest.approx(0.055)
    assert pm.collective_to_duty(1.0, 0.055) == pytest.approx(1.0)
    for c in (0.0, 0.3, 1.0):
        assert pm.duty_to_collective(pm.collective_to_duty(c, 0.055), 0.055) == pytest.approx(c)
    assert pm.duty_to_collective(0.05, 0.055) == 0.0  # below FC idle -> collective 0
    for removed in ('--u-idle', '--motor-idle'):
        with pytest.raises(SystemExit):
            tool.parse_args([removed, '0.05'])


# ---------------------------------------------------------------- §3.1 normalization

def test_environment_factor_matches_document_formula():
    for p, t in ((101325.0, 0.0), (85000.0, 30.0), (95000.0, -10.0)):
        assert pm.environment_factor(p, t) == pytest.approx((101325.0 / p) * ((t + 273.15) / 273.15))
    assert pm.environment_factor(101325.0, 0.0) == 1.0
    for bad in ((0.0, 10.0), (101325.0, -300.0), (math.nan, 0.0)):
        with pytest.raises(ValueError):
            pm.environment_factor(*bad)


def test_rpm_rms():
    assert pm.rpm_rms([3, 4, 3, 4]) == pytest.approx(math.sqrt(12.5))
    assert pm.rpm_rms([1000.0, math.nan, 1000.0]) == pytest.approx(1000.0)
    assert math.isnan(pm.rpm_rms([math.nan] * 4))
    assert pm.rpm_rms(plant.motor_rpms(5000.0)) == pytest.approx(5000.0)


def test_electrical_identification_recovers_req_and_kv():
    pts = [plant.bench_point(u, 85000.0, 30.0) for u in LEVELS]
    ident, err = pm.identify_electrical([p['u'] for p in pts], [p['V'] for p in pts],
                                        [p['I'] for p in pts], [pm.rpm_rms(p['rpm']) for p in pts])
    assert err is None
    assert ident['r_eq'] == pytest.approx(plant.R_EQ, rel=1e-9)
    assert ident['ke'] == pytest.approx(plant.KE, rel=1e-9)
    assert ident['kv'] == pytest.approx(1.0 / plant.KE, rel=1e-9)
    ident, err = pm.identify_electrical([0.5] * 4, [22.0] * 4, [math.nan] * 4, [1e4] * 4)
    assert ident is None and err


def test_standard_voltage_thrust_formula():
    f_std, i_std = pm.standard_voltage_thrust(0.5, 21.0, 30.0, 20.0, 0.03, 22.2)
    i_expected = 30.0 * (22.2 / 21.0) ** 2
    assert i_std == pytest.approx(i_expected)
    assert f_std == pytest.approx(20.0 * ((0.5 * 22.2 - i_expected * 0.03) / (0.5 * 21.0 - 30.0 * 0.03)) ** 2)
    # first-order compact formula without R_eq / current
    f_std, i_std = pm.standard_voltage_thrust(0.5, 21.0, math.nan, 20.0, None, 22.2)
    assert f_std == pytest.approx(20.0 * (22.2 / 21.0) ** 2) and math.isnan(i_std)


# ---------------------------------------------------------------- §3.2 fit

def test_zero_intercept_cubic_normal_equations_exact():
    u = np.linspace(0.05, 1.0, 20)
    truth = (-15.0, 70.0, 6.0)
    f = truth[0] * u ** 3 + truth[1] * u ** 2 + truth[2] * u
    coef, stats = pm.fit_zero_intercept_cubic(u, f)
    assert coef == pytest.approx(truth, rel=1e-9)
    assert stats['max_error'] < 1e-9 and stats['r_squared'] == pytest.approx(1.0)
    x = np.stack([u ** 3, u ** 2, u], axis=1)
    assert coef == pytest.approx(tuple(np.linalg.inv(x.T @ x) @ x.T @ f), rel=1e-9)


def test_monotonic_check():
    assert pm.monotonic_check(-15.0, 70.0, 6.0)[0]
    assert not pm.monotonic_check(0.0, 50.0, -1.0)[0]           # a1 <= 0
    ok, slope = pm.monotonic_check(-100.0, 50.0, 10.0)          # dF/du(1) = -190
    assert not ok and slope == pytest.approx(-190.0)
    assert pm.min_slope_on_unit_interval(10.0, -10.0, 5.0) == pytest.approx(5.0 - 100.0 / 30.0)


def test_calibrate_recovers_standard_curve_and_closed_loop():
    pts = [plant.bench_point(u, 85000.0, 30.0) for u in LEVELS]
    res = pm.calibrate(pts, plant.V_REF)
    assert res['monotonic'] and not res['warnings']
    assert res['r_eq'] == pytest.approx(plant.R_EQ, rel=1e-9)
    assert res['k_i'] > 0.0
    model = pm.PropulsionModel(res['a3'], res['a2'], res['a1'], r_eq=res['r_eq'], k_i=res['k_i'])
    assert model.validate() == ''
    # §4 in closed loop on the physical plant: other altitude/temperature/battery states.
    for p, t, v_oc in ((101325.0, 0.0, 22.2), (85000.0, 30.0, 24.0), (95000.0, -10.0, 21.0)):
        for f_req in (10.0, 25.0, 45.0):
            realized, _ = plant.closed_loop_thrust(model, f_req, p, t, v_oc)
            assert realized == pytest.approx(f_req, rel=0.01), (p, t, v_oc, f_req)


def test_calibrate_airsim_mode_without_baro_sag_or_rpm():
    pts = [{'u': u, 'V': math.nan, 'I': math.nan, 'rpm': [math.nan] * 4, 'F': plant.AIRSIM_A1 * u,
            'P': math.nan, 't': math.nan} for u in LEVELS]
    res = pm.calibrate(pts, plant.V_REF, voltage_normalization=False, environment_normalization=False)
    assert (res['a3'], res['a2'], res['a1']) == pytest.approx((0.0, 0.0, plant.AIRSIM_A1), abs=1e-9)
    assert res['r_eq'] == 0.0 and res['k_i'] == 0.0 and res['monotonic']
    model = pm.PropulsionModel(res['a3'], res['a2'], res['a1'])
    assert model.throttle(45.0) == pytest.approx(0.5, abs=1e-6)  # no voltage, env = 1


def test_calibrate_warns_below_twenty_points():
    pts = [plant.bench_point(u, 101325.0, 0.0) for u in (0.2, 0.4, 0.6, 0.8)]
    res = pm.calibrate(pts, plant.V_REF)
    assert any('N >= 20' in w for w in res['warnings'])


def test_render_param_yaml_is_pasteable():
    import yaml
    args = tool.parse_args(['--yes', '--no-env-comp'])
    text = tool.render_param_yaml({'a3': -1.0, 'a2': 2.0, 'a1': 80.0, 'r_eq': 0.03, 'k_i': 1.5}, args)
    p = yaml.safe_load(text)['custom_link_bridge']['ros__parameters']
    # u_idle is not a parameter: the node reads the FC motor_idle at link-up
    assert p['motor_parameters'] == {'a3': -1.0, 'a2': 2.0, 'a1': 80.0, 'r_eq': 0.03, 'k_i': 1.5}
    assert p['baro_compensation'] is False and p['voltage_compensation'] is True
    assert len(args.levels) == 20
