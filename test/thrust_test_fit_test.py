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
    # u_idle=0 (默认) 时窗口 [0,1] 退化为旧版: slope(0) = a1 <= 0 拒绝
    assert not pm.monotonic_check(0.0, 50.0, -1.0)[0]
    # 凸数据拟合的典型形态: 负斜率区只到 u≈0.012, 完全落在 u_idle=0.055 之下 -> 放行
    ok, slope = pm.monotonic_check(5.50, 57.87, -1.344, 0.055)
    assert ok and slope == pytest.approx(3 * 5.50 * 0.055 ** 2 + 2 * 57.87 * 0.055 - 1.344)
    # 负斜率区侵入可操作域 / 窗口内非单调 -> 仍然拒绝
    assert not pm.monotonic_check(0.0, -50.0, 0.5, 0.05)[0]     # slope(u_idle) < 0
    ok, slope = pm.monotonic_check(-100.0, 50.0, 10.0)          # dF/du(1) = -190
    assert not ok and slope == pytest.approx(-190.0)
    # F_std(u_idle) <= 0 护栏的隔离用例: 窗口斜率为正 (slope(0.9)=+1.4) 但
    # F(0.9) = -1.17 < 0 -> 仅因怠速点推力非正被拒 (单调+F(1)>0 不蕴含窗口恒正)
    ok, slope = pm.monotonic_check(0.0, 3.0, -4.0, 0.9)
    assert not ok and slope == pytest.approx(1.4)
    assert pm.min_slope_on_unit_interval(10.0, -10.0, 5.0) == pytest.approx(5.0 - 100.0 / 30.0)
    assert pm.min_slope_on_unit_interval(10.0, -10.0, 5.0, 0.5) == pytest.approx(2.5)  # 驻点 1/3 在窗外


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


def test_verification_weights_resolvable_and_bounded():
    # tare 6 kg (58.84 N) -> 25/50/75% 静重推力, 都在标定域内
    w = tool.verification_weights(6.0, 9.80665, 200.0)
    assert w == pytest.approx([14.7101, 29.4202, 44.1299], rel=1e-4)
    # 标定域上界截断 (F_std(1) = 20 N 只覆盖 25% 档)
    assert tool.verification_weights(6.0, 9.80665, 20.0) == pytest.approx([14.7101], rel=1e-4)
    # 太轻的静重连最高档都小于台秤 10 g 分辨率 -> 安全跳过 (0.01 kg: 100% < 0.098 N)
    assert tool.verification_weights(0.01, 9.80665, 200.0) == []
    assert tool.verification_verify_pass([{'rel': 0.01}, {'rel': -0.02}])
    assert not tool.verification_verify_pass([{'rel': 0.03}])
    assert not tool.verification_verify_pass([])


def test_tool_streams_body_rates_by_default():
    args = tool.parse_args(['--yes'])
    assert args.attitude is False and args.no_verify is False and args.verify_sample_time > 0.0
    assert tool.parse_args(['--attitude', '--yes']).attitude is True
    assert tool.parse_args(['--no-verify', '--yes']).no_verify is True


def test_render_param_yaml_is_pasteable():
    import yaml
    args = tool.parse_args(['--yes', '--no-env-comp'])
    text = tool.render_param_yaml({'a3': -1.0, 'a2': 2.0, 'a1': 80.0, 'r_eq': 0.03, 'k_i': 1.5}, args)
    p = yaml.safe_load(text)['custom_link_bridge']['ros__parameters']
    # u_idle is not a parameter: the node reads the FC motor_idle at link-up
    assert p['motor_parameters'] == {'a3': -1.0, 'a2': 2.0, 'a1': 80.0, 'r_eq': 0.03, 'k_i': 1.5}
    assert p['baro_compensation'] is False and p['voltage_compensation'] is True
    assert len(args.levels) == 20


# ---------------------------------------------------------------- plot

def _agg_pyplot():
    matplotlib = pytest.importorskip('matplotlib')
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    return plt


def test_plot_results_dashes_extrapolation_beyond_calibrated_domain(tmp_path):
    plt = _agg_pyplot()
    # 截短扫描 (u = 0.1..0.5): 拟合曲线在标定域内实线, 域外虚线并从 u_max 起
    pts = [plant.bench_point(u, 85000.0, 30.0) for u in [round(0.1 * i, 1) for i in range(1, 6)]]
    res = pm.calibrate(pts, plant.V_REF)
    assert res['monotonic']
    png = str(tmp_path / 'plot.png')
    fig = tool.plot_results(res, png)
    assert fig is not None and os.path.exists(png)
    lines = {line.get_label(): line for line in fig.axes[0].get_lines()}
    assert lines['F_std(u) fit'].get_xdata()[-1] <= 0.5 + 1e-9
    assert lines['extrapolated fit'].get_xdata()[0] == pytest.approx(0.5)
    assert 'REJECTED' not in fig.axes[0].get_title()
    plt.close(fig)


def test_calibrate_accepts_convex_fit_with_sub_idle_dip(tmp_path):
    """最小放宽 (spec 3.2, 2026-11-10 修订): 凸推力数据 (F ~ u^2.1, 真实桨典型) 的
    无截距三次拟合 a1 < 0, 但负斜率区 (u <~ 0.012) 完全落在 u_idle=0.055 之下 --
    运行时 clamp 不可达, u_idle 传入后应整体放行 (参数片段可出、图不盖章)。"""
    plt = _agg_pyplot()
    pts = [{'u': u, 'V': math.nan, 'I': math.nan, 'rpm': [math.nan] * 4,
            'F': 62.0 * u ** 2.1, 'P': math.nan, 't': math.nan} for u in LEVELS]
    res = pm.calibrate(pts, plant.V_REF, voltage_normalization=False,
                       environment_normalization=False, u_idle=0.055)
    assert res['a1'] < 0.0                      # 拟合形态不变, 只是判据窗口收紧
    assert res['monotonic'] and res['min_slope'] > 0.0
    assert not pm.monotonic_check(res['a3'], res['a2'], res['a1'])[0]  # 旧版 [0,1] 仍会拒
    model = pm.PropulsionModel(res['a3'], res['a2'], res['a1'], u_idle=0.055)
    assert model.validate() == ''               # 运行时同口径放行
    fig = tool.plot_results(res, str(tmp_path / 'accepted.png'))
    assert fig is not None and 'REJECTED' not in fig.axes[0].get_title()
    plt.close(fig)


def test_plot_results_stamps_rejected_deadband_fit(tmp_path):
    plt = _agg_pyplot()
    # 负斜率区侵入可操作域的实例: 推力起始死区 (u <~ 0.09 无推力) 的拟合 dip 到
    # u≈0.068 > u_idle=0.055 -> 仍拒绝; 被拒拟合出图必须盖章 REJECTED。
    onset = [62.0 * max(u - 0.09, 0.0) ** 2 / (1 - 0.09) ** 2 for u in LEVELS]
    pts = [{'u': u, 'V': math.nan, 'I': math.nan, 'rpm': [math.nan] * 4,
            'F': f, 'P': math.nan, 't': math.nan} for u, f in zip(LEVELS, onset)]
    res = pm.calibrate(pts, plant.V_REF, voltage_normalization=False,
                       environment_normalization=False, u_idle=0.055)
    assert not res['monotonic'] and res['a1'] < 0.0
    fig = tool.plot_results(res, str(tmp_path / 'rejected.png'))
    assert fig is not None and 'REJECTED' in fig.axes[0].get_title()
    plt.close(fig)


def test_calibrate_rejects_deadband_with_sub_idle_negative_thrust():
    """F_std(u_idle) > 0 护栏的数据级用例: 推力起始死区略高于 u_idle (onset 0.065
    vs u_idle 0.055) 时, 窗口斜率仍为正但 F(u_idle) < 0 —— 放行会让反解初值
    sqrt(负数) 产出 NaN 油门, 必须拒绝且原因指向 F_std(u_idle)。"""
    onset = [62.0 * max(u - 0.065, 0.0) ** 2 / (1 - 0.065) ** 2 for u in LEVELS]
    pts = [{'u': u, 'V': math.nan, 'I': math.nan, 'rpm': [math.nan] * 4,
            'F': f, 'P': math.nan, 't': math.nan} for u, f in zip(LEVELS, onset)]
    res = pm.calibrate(pts, plant.V_REF, voltage_normalization=False,
                       environment_normalization=False, u_idle=0.055)
    assert not res['monotonic']
    assert res['min_slope'] > 0.0                          # 拒绝不是因为斜率
    assert 'F_std(u_idle) <= 0' in res['monotonic_reason']  # 而是怠速点推力非正
    model = pm.PropulsionModel(res['a3'], res['a2'], res['a1'], u_idle=0.055)
    assert model.validate() and model.inverse is None       # 运行时同口径拒绝, 反函数不安装
