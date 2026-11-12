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
    # 电流模型: 管道系 I_std/F_std ≈ 1.50 带轻微曲率 (I_std ≈ I·(V_ref/V)² 近似 +
    # 电压归一所致, 并非裸工厂的 K_LOAD/K_AERO = 2.0); 无噪数据精确恢复 LS 解
    assert res['k_i'] == pytest.approx(1.5001621, rel=1e-6)
    assert res['k_i2'] == pytest.approx(-1.7950294e-4, rel=1e-5, abs=1e-12)
    assert res['k_i3'] == pytest.approx(6.2577736e-6, rel=1e-5, abs=1e-12)
    model = pm.PropulsionModel(res['a3'], res['a2'], res['a1'], r_eq=res['r_eq'], k_i=res['k_i'],
                               k_i2=res['k_i2'], k_i3=res['k_i3'])
    assert model.validate() == ''
    # §4 in closed loop on the physical plant: other altitude/temperature/battery states.
    for p, t, v_oc in ((101325.0, 0.0, 22.2), (85000.0, 30.0, 24.0), (95000.0, -10.0, 21.0)):
        for f_req in (10.0, 25.0, 45.0):
            realized, _ = plant.closed_loop_thrust(model, f_req, p, t, v_oc)
            assert realized == pytest.approx(f_req, rel=0.01), (p, t, v_oc, f_req)


def test_fit_current_coefficient_cubic_model():
    # U 形 I/F 数据 (2026-11-12 台架形态): 三项拟合在验证/飞行工况段 (F>=5) 覆盖
    # 良好, 单一线性 k_i 在该段平均偏 15%
    f = np.array([0.8, 2.4, 5.0, 8.2, 12.2, 16.6, 22.0, 27.4, 34.8, 42.0, 49.0, 56.6, 63.8, 71.8, 79.5, 87.5])
    ratio = np.array([2.75, 1.9, 1.34, 1.23, 1.21, 1.24, 1.24, 1.30, 1.31, 1.34, 1.41, 1.47, 1.58, 1.67, 1.76, 1.85])
    # 合成数据精确恢复系数
    truth = (1.2, 0.012, -0.00005)
    i_syn = ((truth[2] * f + truth[1]) * f + truth[0]) * f
    assert pm.fit_current_coefficient(f, i_syn) == pytest.approx(truth, rel=1e-6)
    # U 形实测形态: F>=5 段平均相对残差立方 << 纯线性模型 (旧 §1.4 行为), F>=10 段最大 < 5%
    i = ratio * f
    k_i, k_i2, k_i3 = pm.fit_current_coefficient(f, i)
    k_pure = float(np.sum(f * i) / np.sum(f * f))   # 旧版: 过原点纯线性
    pred = ((k_i3 * f + k_i2) * f + k_i) * f
    m5, m10 = f >= 5.0, f >= 10.0
    assert np.mean(np.abs(pred[m5] - i[m5]) / i[m5]) < 0.2 * np.mean(np.abs(k_pure * f[m5] - i[m5]) / i[m5])
    assert np.max(np.abs(pred[m10] - i[m10]) / i[m10]) < 0.05
    # 数据不足回退线性; 无数据回退零
    assert pm.fit_current_coefficient([1.0, 2.0], [1.0, 2.0]) == pytest.approx((1.0, 0.0, 0.0))
    assert pm.fit_current_coefficient([], []) == (0.0, 0.0, 0.0)


def test_fit_zero_intercept_cubic_weights():
    u = np.linspace(0.05, 1.0, 20)
    f = 60.0 * u ** 2.1
    w = 1.0 / f
    (a3w, a2w, a1w), stats = pm.fit_zero_intercept_cubic(u, f, weights=w)
    # 手工加权正规方程一致 (该数据无约束解 a1 < 0 -> 返回的是 a1=0 边界解, 与手工
    # [u³ u²] 两参加权解一致; 手工三参解用于对照 a1 的符号)
    x2 = np.stack([u ** 3, u ** 2], axis=1)
    sw = np.sqrt(w)[:, None]
    expected2 = np.linalg.solve((x2 * sw).T @ (x2 * sw), (x2 * sw).T @ (f * np.sqrt(w)))
    assert a1w == 0.0
    assert (a3w, a2w) == pytest.approx(tuple(expected2), rel=1e-9)
    x3 = np.stack([u ** 3, u ** 2, u], axis=1)
    unconstrained = np.linalg.solve((x3 * sw).T @ (x3 * sw), (x3 * sw).T @ (f * np.sqrt(w)))
    assert unconstrained[2] < 0.0  # 钳位确因无约束解 a1 < 0 而触发
    # 无约束解为正 a1 的数据: 三参精确一致
    f_pos = 60.0 * u ** 2.1 + 8.0 * u
    w_pos = 1.0 / f_pos
    (a3p, a2p, a1p), _ = pm.fit_zero_intercept_cubic(u, f_pos, weights=w_pos)
    swp = np.sqrt(w_pos)[:, None]
    expected3 = np.linalg.solve((x3 * swp).T @ (x3 * swp), (x3 * swp).T @ (f_pos * np.sqrt(w_pos)))
    assert (a3p, a2p, a1p) == pytest.approx(tuple(expected3), rel=1e-9) and a1p > 0.0
    # 加权把验证/悬停工况段 (u = 0.15..0.30, 即闭环验证档所在的区域) 的相对残差
    # 压平约一半; 极端低油门点 (u=0.05, ~1% 量程) 两种拟合都差 (幂律数据本就超出
    # 三次表达力), 不是本修订的目标区。
    def rel_resid(coef, uu):
        return abs((coef[0] * uu ** 3 + coef[1] * uu ** 2 + coef[2] * uu - 60.0 * uu ** 2.1)
                   / (60.0 * uu ** 2.1))
    plain, _ = pm.fit_zero_intercept_cubic(u, f)
    for uu in (0.15, 0.20, 0.25, 0.30):
        assert rel_resid((a3w, a2w, a1w), uu) < 0.75 * rel_resid(plain, uu), uu
    assert rel_resid((a3w, a2w, a1w), 0.30) < 0.015
    # stats 恒为无权物理残差
    assert stats['n'] == 20 and math.isfinite(stats['rms'])
    with pytest.raises(ValueError):
        pm.fit_zero_intercept_cubic(u, f, weights=np.ones(5))


def test_calibrate_airsim_mode_without_baro_sag_or_rpm():
    pts = [{'u': u, 'V': math.nan, 'I': math.nan, 'rpm': [math.nan] * 4, 'F': plant.AIRSIM_A1 * u,
            'P': math.nan, 't': math.nan} for u in LEVELS]
    res = pm.calibrate(pts, plant.V_REF, voltage_normalization=False, environment_normalization=False)
    assert (res['a3'], res['a2'], res['a1']) == pytest.approx((0.0, 0.0, plant.AIRSIM_A1), abs=1e-9)
    assert res['r_eq'] == 0.0 and res['k_i'] == 0.0 and res['monotonic']
    model = pm.PropulsionModel(res['a3'], res['a2'], res['a1'])
    assert model.throttle(45.0) == pytest.approx(0.5, abs=1e-6)  # no voltage, env = 1


def test_calibrate_warns_below_min_points():
    pts = [plant.bench_point(u, 101325.0, 0.0) for u in (0.2, 0.4, 0.6, 0.8)]
    res = pm.calibrate(pts, plant.V_REF)
    assert any(f'N >= {pm.MIN_POINTS_RECOMMENDED}' in w for w in res['warnings'])


def test_verification_levels_domain_safety_and_resolution():
    g = 9.80665
    frac = tool.VERIFICATION_FRACTIONS
    safe = (6.0 - 1.0 - 0.3) * g  # (tare - critical - margin)·g
    # 宽域 + 富余 tare: 档位 = 安全上限的 fractions 比例
    lv = tool.verification_levels(200.0, 1.0, g, 6.0, 1.0)
    assert lv == pytest.approx([f * safe for f in frac], rel=1e-6)
    # 标定域上界折回请求系: f_max/env (2026-11-11 17:58 越界档即漏乘 env)
    lv = tool.verification_levels(48.44, 1.083, g, 6.0, 1.0)
    cap = min(safe, 48.44 / 1.083)
    assert lv == pytest.approx([f * cap for f in frac], rel=1e-6)
    assert max(lv) * 1.083 <= 48.44 + 1e-9
    # 域不够覆盖高档 -> 档位被域截断而非静重截断
    lv = tool.verification_levels(10.0, 1.0, g, 6.0, 1.0)
    assert lv == pytest.approx([f * 10.0 for f in frac], rel=1e-6)
    # 轻夹具: 安全上限 (tare-critical-margin)*g 连最低档 0.5 N 都不可分辨 -> 安全跳过
    assert tool.verification_levels(200.0, 1.0, g, 1.29, 1.0) == []
    # env 非法按 1 处理
    assert tool.verification_levels(200.0, float('nan'), g, 6.0, 1.0) == \
        pytest.approx(tool.verification_levels(200.0, 1.0, g, 6.0, 1.0))
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
    res = {'a3': -1.0, 'a2': 2.0, 'a1': 80.0, 'r_eq': 0.03, 'k_i': 1.5, 'k_i2': 0.01, 'k_i3': -1e-5}
    text = tool.render_param_yaml(res, args)
    p = yaml.safe_load(text)['custom_link_bridge']['ros__parameters']
    # u_idle is not a parameter: the node reads the FC motor_idle at link-up
    assert p['motor_parameters'] == {'a3': -1.0, 'a2': 2.0, 'a1': 80.0, 'r_eq': 0.03,
                                     'k_i': 1.5, 'k_i2': 0.01, 'k_i3': -1e-5}
    # rclcpp 对 double 参数收到 int 会在启动时抛 InvalidParameterTypeException:
    # 每个值都必须被 YAML 解析为 float (整数值 0/100000 与无小数点科学计数是陷阱)
    assert all(isinstance(v, float) for v in p['motor_parameters'].values())
    # 旧式 result dict (无 k_i2/k_i3 键) 渲染为显式 0.0, 不缺键; a1=0 (钳位拟合) 同样安全
    legacy = tool.render_param_yaml({'a3': -1.0, 'a2': 2.0, 'a1': 0.0, 'r_eq': 0.03,
                                     'k_i': 1.5, 'k_i2': 0.0, 'k_i3': 0.0}, args)
    leg = yaml.safe_load(legacy)['custom_link_bridge']['ros__parameters']['motor_parameters']
    assert leg['k_i2'] == 0.0 and leg['k_i3'] == 0.0 and leg['a1'] == 0.0
    assert all(isinstance(v, float) for v in leg.values())
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
    """最小放宽 (spec 3.2, 2026-11-10 修订) + 2026-11-12 加权/a1 钳 0: 凸推力数据
    (F ~ u^2.1, 真实桨典型) 的相对误差加权拟合把原先的负 a1 钳到 0 (逼近伪影,
    见 fit_zero_intercept_cubic), u_idle=0.055 传入后整体放行 (参数片段可出、
    图不盖章)。"""
    plt = _agg_pyplot()
    pts = [{'u': u, 'V': math.nan, 'I': math.nan, 'rpm': [math.nan] * 4,
            'F': 62.0 * u ** 2.1, 'P': math.nan, 't': math.nan} for u in LEVELS]
    res = pm.calibrate(pts, plant.V_REF, voltage_normalization=False,
                       environment_normalization=False, u_idle=0.055)
    assert res['a1'] == 0.0                     # 2026-11-12: 负 a1 钳 0 (原为 -0.7 量级)
    assert res['monotonic'] and res['min_slope'] > 0.0
    assert not pm.monotonic_check(res['a3'], res['a2'], res['a1'])[0]  # 旧版 [0,1] 仍会拒 (slope(0)=0)
    model = pm.PropulsionModel(res['a3'], res['a2'], res['a1'], u_idle=0.055)
    assert model.validate() == ''               # 运行时同口径放行
    fig = tool.plot_results(res, str(tmp_path / 'accepted.png'))
    assert fig is not None and 'REJECTED' not in fig.axes[0].get_title()
    plt.close(fig)


def test_plot_results_stamps_rejected_deadband_fit(tmp_path):
    plt = _agg_pyplot()
    # 推力起始死区 (u <~ 0.09 无推力): 数据级死区判据 (2026-11-12, 正油门点实测
    # F_std <= 0) 拒绝 —— 与拟合形态无关, 被拒拟合出图必须盖章 REJECTED。
    onset = [62.0 * max(u - 0.09, 0.0) ** 2 / (1 - 0.09) ** 2 for u in LEVELS]
    pts = [{'u': u, 'V': math.nan, 'I': math.nan, 'rpm': [math.nan] * 4,
            'F': f, 'P': math.nan, 't': math.nan} for u, f in zip(LEVELS, onset)]
    res = pm.calibrate(pts, plant.V_REF, voltage_normalization=False,
                       environment_normalization=False, u_idle=0.055)
    assert not res['monotonic'] and 'deadband' in res['monotonic_reason']
    fig = tool.plot_results(res, str(tmp_path / 'rejected.png'))
    assert fig is not None and 'REJECTED' in fig.axes[0].get_title()
    plt.close(fig)


def test_calibrate_rejects_high_throttle_thrust_collapse():
    """高油门推力塌陷 (桨失速) 的数据级用例 (2026-11-12 评审): 无权拟合靠负 a1 下穿
    拒绝这类数据, a1 钳 0 后该路径消失 —— 钳位拟合可能通过单调门并以虚高的外推
    推力放行。判据: 相邻采样点实测 F_std 下降超过 max(0.2 N, 2%) 即拒绝。"""
    collapse = [62.0 * (u / 0.85) ** 2.1 * (1.0 - 0.35 * max(u - 0.85, 0.0) / 0.15) for u in LEVELS]
    pts = [{'u': u, 'V': math.nan, 'I': math.nan, 'rpm': [math.nan] * 4,
            'F': f, 'P': math.nan, 't': math.nan} for u, f in zip(LEVELS, collapse)]
    res = pm.calibrate(pts, plant.V_REF, voltage_normalization=False,
                       environment_normalization=False, u_idle=0.055)
    assert not res['monotonic']
    assert 'collapse' in res['monotonic_reason']
    # 2% 以内的秤噪声级波动不触发
    noisy = [62.0 * u ** 2.1 * (1.0 + 0.01 * math.sin(i)) for i, u in enumerate(LEVELS)]
    pts2 = [{'u': u, 'V': math.nan, 'I': math.nan, 'rpm': [math.nan] * 4,
             'F': f, 'P': math.nan, 't': math.nan} for u, f in zip(LEVELS, noisy)]
    res2 = pm.calibrate(pts2, plant.V_REF, voltage_normalization=False,
                        environment_normalization=False, u_idle=0.055)
    assert res2['monotonic']


def test_calibrate_rejects_deadband_with_sub_idle_negative_thrust():
    """死区护栏的数据级用例 (2026-11-12 修订): 推力起始略高于 u_idle (onset 0.065
    vs u_idle 0.055) 时, 加权拟合的 a1 已钳 0、F_std(u_idle) > 0 恒成立, 原拟合
    伪影判据不再触发 —— 拒绝改由数据级判据承担: 正油门采样点实测 F_std <= 0 即
    模型形态无效 (死区内模型凭空预测推力, 低推力指令系统性失准)。"""
    onset = [62.0 * max(u - 0.065, 0.0) ** 2 / (1 - 0.065) ** 2 for u in LEVELS]
    pts = [{'u': u, 'V': math.nan, 'I': math.nan, 'rpm': [math.nan] * 4,
            'F': f, 'P': math.nan, 't': math.nan} for u, f in zip(LEVELS, onset)]
    res = pm.calibrate(pts, plant.V_REF, voltage_normalization=False,
                       environment_normalization=False, u_idle=0.055)
    assert not res['monotonic']
    assert res['min_slope'] > 0.0                      # 拒绝不是因为斜率
    assert 'deadband' in res['monotonic_reason']       # 而是数据级死区判据
    model = pm.PropulsionModel(res['a3'], res['a2'], res['a1'], u_idle=0.055)
    assert model.validate() == ''                      # 系数本身合法 (a1 钳 0); 拒绝发生在标定数据级
