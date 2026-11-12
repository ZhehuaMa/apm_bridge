"""fpv_test.md 推力-油门建模 (§1/§3) 与在线反解 (§4) 的 Python 实现。

与 include/quadratic_thrust_model.h (节点运行时) 同口径, 供台架标定工具
(thrust_test_node)、SITL 标定、硬件测试镜像与单元测试共用。

油门域: 文档的 u 是整机归一化油门开度 (PWM/DShot 占空比), F(0)=0。Betaflight 的
集体油门 c (custom link 0x20) 映射到 DShot 输出 u = u_idle + (1-u_idle)·c
(dshot.c outputLow), 其中 u_idle 就是飞控 motor_idle (默认 550 -> 5.5%), 也即 §4
步骤 4 的怠速下限 (对应 c = 0)。u_idle 不作参数配置, 运行时经 /fpv/cli
"get motor_idle" 从飞控读取 (fetch_u_idle)。模型全部在占空比 u 上建立。

基准状态 (§1.1): V_ref = cells·3.7 V (6S = 22.20 V), T0 = 273.15 K, P0 = 101325 Pa。
"""

import math
from dataclasses import dataclass, field

import numpy as np

P0 = 101325.0                 # Pa
T0 = 273.15                   # K
V_REF_PER_CELL = 3.7          # 22.20 V for 6S
V_MIN_PER_CELL = 2.0          # 12.0 V for 6S: §5 硬件欠压保护下限
DEFAULT_U_IDLE = 0.055        # Betaflight motor_idle 默认 550; 仅用于无法询问飞控的离线场景
MOTOR_IDLE_SETTING = 'motor_idle'  # Betaflight CLI 设置名 (0.01 % 单位, 0..2000)
F_REQ_EPS = 0.001             # §5 零推力保护阈值 (N)
NEWTON_MAX_ITER = 4           # §4 步骤 2: 3~4 次
NEWTON_TOL = 1e-5             # §4 步骤 2: 1e-5 精度
U_PROJECT_MAX = 1.2           # §5 迭代投影区间 [0, 1.2]
MIN_POINTS_RECOMMENDED = 10   # §3.2 推荐的最少采样点数


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


# ======================================================================
# 油门域换算 (Betaflight DShot 输出映射)
# ======================================================================

def collective_to_duty(collective, u_idle):
    return u_idle + (1.0 - u_idle) * _clamp(collective, 0.0, 1.0)


def duty_to_collective(duty, u_idle):
    return _clamp((duty - u_idle) / (1.0 - u_idle), 0.0, 1.0)


def idle_duty_from_cli(text):
    """'motor_idle = 550' (CLI GET 回读, 0.01 % 单位) -> 0.055; 不合法返回 NaN。"""
    name, sep, value = (text or '').partition('=')
    value = value.strip()
    if not sep or name.strip() != MOTOR_IDLE_SETTING or not value.isdigit() or len(value) > 5:
        return float('nan')
    raw = int(value)
    return raw / 10000.0 if raw <= 2000 else float('nan')


def fetch_u_idle(node, timeout=5.0, spin=False):
    """经 custom_link_bridge 的 /fpv/cli 读取飞控 motor_idle, 返回 u_idle 占空比。

    spin=True: 调用方未在后台 spin 该节点 (rclpy.spin_once 驱动); 否则仅等待 future。
    失败抛 RuntimeError。
    """
    text = cli_command(node, 0, MOTOR_IDLE_SETTING, timeout=timeout, spin=spin)  # 0 = CMD_GET
    u_idle = idle_duty_from_cli(text)
    if not math.isfinite(u_idle):
        raise RuntimeError(f'unexpected motor_idle reply: {text!r}')
    return u_idle


def cli_command(node, cmd, args, timeout=5.0, spin=False, attempts=5):
    """经 custom_link_bridge 的 /fpv/cli 执行一条 CLI 命令, 返回回读文本。

    cmd 取 CliCommand.Request.CMD_*; spin=True 时用 rclpy.spin_once 驱动 (调用方
    无后台 executor)。失败抛 RuntimeError。注意 MODE_LOOKUP 类设置 (ON/OFF 表)
    的 SET 只认字符串值 ("ON"/"OFF"), 数字会被 CLI 引擎拒绝。

    attempts>1 时对超时类失败重试, 退避间隔递增 0.5/2/5 s (真机实测 CLI 通道存在
    与近期 0x20 活动相关的时间窗性 SET 阻塞, 数分钟后自愈 -- 超时的 SET 在 FC 侧
    未被应用, 幂等的 GET/SET 重试安全; 非超时类拒绝立即上抛)。
    """
    import time as _time
    last_error = None
    backoffs = (0.5, 2.0, 5.0, 5.0, 5.0)
    for attempt in range(attempts):
        try:
            return _cli_command_once(node, cmd, args, timeout, spin)
        except RuntimeError as e:
            last_error = e
            if 'timed out' not in str(e) or attempt == attempts - 1:
                raise
            _time.sleep(backoffs[min(attempt, len(backoffs) - 1)])
    raise last_error


def _cli_command_once(node, cmd, args, timeout, spin):
    import time
    import rclpy
    from quadrotor_msgs.srv import CliCommand

    client = node.create_client(CliCommand, '/fpv/cli')
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('/fpv/cli service unavailable (is custom_link_bridge_node running?)')
        request = CliCommand.Request(cmd=cmd, args=args)
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            if spin:
                rclpy.spin_once(node, timeout_sec=0.05)
            else:
                time.sleep(0.02)
        if not future.done():
            future.cancel()
            raise RuntimeError(f'cli {args!r} timed out')
        response = future.result()
        if not response.ok or response.status != 0:
            raise RuntimeError(f'cli {args!r} rejected: {response.error or response.text}')
        return response.text
    finally:
        node.destroy_client(client)


# ======================================================================
# §1.2 / §3.1 步骤 1: 环境推力归一化
# ======================================================================

def environment_factor(pressure_pa, temperature_c):
    """(P0/P)·(T/T0): 实测/期望推力乘以此系数即折算到标准大气 (R_spec 对消)。"""
    t_kelvin = temperature_c + T0
    if not (math.isfinite(pressure_pa) and pressure_pa > 0.0 and
            math.isfinite(t_kelvin) and t_kelvin > 0.0):
        raise ValueError('pressure/temperature must be finite with P > 0 Pa and T > 0 K')
    return (P0 / pressure_pa) * (t_kelvin / T0)


# ======================================================================
# §1.3 / §3.1 步骤 2: 四电机转速均方根
# ======================================================================

def rpm_rms(rpms):
    """RPM_rms = sqrt(mean(RPM_k^2)); 只用有效 (有限) 转速, 全无效返回 NaN。"""
    valid = [float(r) for r in rpms if r is not None and math.isfinite(r)]
    if not valid:
        return float('nan')
    return math.sqrt(sum(r * r for r in valid) / len(valid))


# ======================================================================
# §3.1 步骤 3: R_eq / K_v* 辨识
# ======================================================================

def identify_electrical(u, voltage, current, rpm):
    """多元线性最小二乘: u·V = (1/K_v*)·RPM_rms + R_eq·I (无截距)。

    返回 {'ke', 'kv', 'r_eq', 'rms', 'n'}; 数据不足/退化/非物理 (K_e<=0 或 R_eq<0)
    时返回 (None, 原因)。
    """
    rows = [(ui * vi, ri, ii) for ui, vi, ii, ri in zip(u, voltage, current, rpm)
            if all(math.isfinite(x) for x in (ui, vi, ii, ri)) and vi > 0.0 and ri > 0.0 and ii >= 0.0]
    if len(rows) < 3:
        return None, 'need >= 3 points with valid voltage, current and RPM'
    y = np.array([r[0] for r in rows])
    x = np.array([[r[1], r[2]] for r in rows])
    xtx = x.T @ x
    if abs(np.linalg.det(xtx)) < 1e-12 * max(1.0, float(np.trace(xtx))) ** 2:
        return None, 'RPM and current are collinear; cannot separate K_e and R_eq'
    ke, r_eq = np.linalg.solve(xtx, x.T @ y)
    if not (ke > 0.0) or r_eq < 0.0:
        return None, f'non-physical identification (K_e={ke:.4g}, R_eq={r_eq:.4g})'
    rms = float(np.sqrt(np.mean((x @ np.array([ke, r_eq]) - y) ** 2)))
    return {'ke': float(ke), 'kv': float(1.0 / ke), 'r_eq': float(r_eq),
            'rms': rms, 'n': len(rows)}, None


# ======================================================================
# §3.1 步骤 4: 折算到标准电压
# ======================================================================

def standard_voltage_thrust(u, voltage, current, f0, r_eq, v_ref):
    """F_std = F0·((u·V_ref − I_std·R_eq)/(u·V − I·R_eq))², I_std ≈ I·(V_ref/V)²。

    r_eq 为 None 或电流无效时用一阶紧缩公式 F0·(V_ref/V)²。
    返回 (F_std, I_std); 反电动势非正时返回 (NaN, NaN)。
    """
    if not (math.isfinite(voltage) and voltage > 0.0):
        return float('nan'), float('nan')
    if r_eq is not None and math.isfinite(current) and current >= 0.0:
        i_std = current * (v_ref / voltage) ** 2
        num = u * v_ref - i_std * r_eq
        den = u * voltage - current * r_eq
        if num <= 0.0 or den <= 0.0:
            return float('nan'), float('nan')
        return f0 * (num / den) ** 2, i_std
    return f0 * (v_ref / voltage) ** 2, float('nan')


# ======================================================================
# §3.2: 无截距三次多项式最小二乘 + 单调性检验
# ======================================================================

def fit_zero_intercept_cubic(u, force, weights=None):
    """Θ = (XᵀWX)⁻¹XᵀWF, X = [u³ u² u], 返回 ((a3, a2, a1), stats)。

    weights=None 为等权正规方程 (§3.2 原式); calibrate 传相对误差权重
    w_i = 1/max(F_std_i, floor) (2026-11-12 修订: 绝对牛顿残差的等权拟合被大推力
    端主导, 低油门段 u≲0.3 系统性下凸 +4~24%, §4 反解在该区解出偏高的 u_std ——
    台架闭环验证随请求推力增大而负向漂移的主因之一)。无约束解 a1 < 0 时钳
    a1 = 0 重拟合 (约束最优即边界上的 2 参解): 负 a1 是凸桨数据的逼近伪影,
    F ≈ a1·u 在原点上方下穿非物理, 且 §3.2 门判要求 F_std(u_idle) > 0 ——
    加权下负 a1 的下穿幅度更深 (2026-11-12 实测 a1 可至 −5.5), 会把本可接受的
    数据误拒。stats 恒按无权物理残差 (N) 计算, 与历史结果同口径。
    """
    u = np.asarray(u, dtype=float)
    f = np.asarray(force, dtype=float)
    if u.shape != f.shape or len(u) < 3 or len(np.unique(u)) < 3:
        raise ValueError('need at least 3 distinct throttle levels')
    if not (np.isfinite(u).all() and np.isfinite(f).all()):
        raise ValueError('non-finite calibration samples')
    x = np.stack([u ** 3, u ** 2, u], axis=1)
    if weights is not None:
        w = np.asarray(weights, dtype=float)
        if w.shape != u.shape or not np.isfinite(w).all() or np.any(w <= 0.0):
            raise ValueError('weights must be positive, finite and match the sample count')
        sw = np.sqrt(w)
    else:
        sw = None

    def _solve(xc):  # 加权/无权正规方程 (w 经 sqrt 分摊到两侧)
        if sw is None:
            return np.linalg.solve(xc.T @ xc, xc.T @ f)
        xcw = xc * sw[:, None]
        return np.linalg.solve(xcw.T @ xcw, xcw.T @ (f * sw))

    theta = _solve(x)
    if theta[2] < 0.0:  # a1 < 0: 钳 0 的约束最优 = [u³ u²] 两参解
        theta = np.array([*_solve(x[:, :2]), 0.0])
        if not np.isfinite(theta).all():
            raise ValueError('degenerate calibration samples (a1-clamped refit)')
    residual = x @ theta - f
    ss_tot = float(np.sum((f - f.mean()) ** 2))
    stats = {'rms': float(np.sqrt(np.mean(residual ** 2))),
             'max_error': float(np.max(np.abs(residual))),
             'r_squared': float(1.0 - np.sum(residual ** 2) / ss_tot) if ss_tot > 0 else float('nan'),
             'n': int(len(u))}
    return tuple(float(v) for v in theta), stats


def min_slope_on_unit_interval(a3, a2, a1, u_lo=0.0):
    """min_{u∈[u_lo,1]} dF/du, dF/du = 3a3u² + 2a2u + a1 (端点与驻点取最小)。"""
    u_lo = _clamp(u_lo, 0.0, 1.0)
    candidates = [u_lo, 1.0]
    if a3 != 0.0:
        vertex = -a2 / (3.0 * a3)
        if u_lo < vertex < 1.0:
            candidates.append(vertex)
    return min(3.0 * a3 * u * u + 2.0 * a2 * u + a1 for u in candidates)


def monotonic_check(a3, a2, a1, u_idle=0.0):
    """§3.2 物理合法性 (2026-11-10 修订): dF/du > 0 ∀u∈[u_idle,1] 且 F_std(u_idle) > 0。

    原版要求 a1 > 0 (原点斜率)。但真实螺旋桨静推力对占空比呈凸性 (F ∝ u² 量级),
    无截距三次最小二乘逼近凸曲线会系统性地以微小负 a1 换取更优的 a2——负斜率区
    往往完全落在 u < u_idle (运行时 clamp(u, u_idle, 1) 永远命令不到)。判据窗口
    因此从 [0,1] 收紧到可操作域 [u_idle,1]: 放行这类无害拟合, 仍拒绝负斜率区
    侵入可操作域或谷底落入 [u_idle,1] 的危险拟合 (那会破坏 §4 牛顿反解的根唯一
    性与收敛性)。正推力条件取 F_std(u_idle) > 0: 斜率>0 使 F 在窗口上单调递增,
    min F = F(u_idle), 故它蕴含窗口内恒正 (含 F(1) > 0), 反解初值 w = sqrt(F/F(1))
    亦良定义 —— 注意单调 + F(1) > 0 并不蕴含窗口恒正 (曲线可在窗口左端为负、
    中途穿零, 初值拟合将 sqrt 负数产出 NaN 油门)。u_idle=0 时退化为原版
    (slope(0) = a1 > 0)。返回 (ok, min_slope on [u_idle,1])。
    """
    slope = min_slope_on_unit_interval(a3, a2, a1, u_idle)
    # standardThrust 求值序 ((a3*u + a2)*u + a1)*u, 与 C++ 逐位一致
    f_idle = ((a3 * u_idle + a2) * u_idle + a1) * u_idle
    positive = f_idle > 0.0 if u_idle > 0.0 else True  # u_idle=0: slope(0)=a1>0 已蕴含
    return (slope > 0.0 and positive), slope


def fit_current_coefficient(f_std, i_std):
    """I_std(F_std) = ((k_i3·F + k_i2)·F + k_i)·F 的无截距最小二乘, 返回 (k_i, k_i2, k_i3)。

    §1.4 原先的过原点线性假设 "I ∝ RPM² ∝ F" 被 2026-11 台架数据否定: 实测 I/F
    呈 U 形 (怠速 ~2.7 → F≈12 N 谷底 ~1.2 → 满油门 ~1.9 A/N), 单一线性 k_i 在
    验证工况点带 20~30% 电流偏置, 经 §4 步骤 3 的 (I_meas − I_std)·R_eq 项放大成
    随请求推力增长的系统性欠推力。三次三项式覆盖全段 (数据内残差 ±1.5%)。正规
    方程在 F/F_max 缩放域上求解 (避免 F³ 列的病态)。数据不足 (<4 点)/退化/数据段
    内出现负预测电流 (非物理) 时逐级回退: 三次 → 线性 → (0, 0, 0)。
    """
    pairs = [(f, i) for f, i in zip(f_std, i_std)
             if math.isfinite(f) and math.isfinite(i) and f > 0.0]
    if not pairs:
        return 0.0, 0.0, 0.0
    f_arr = np.array([p[0] for p in pairs])
    i_arr = np.array([p[1] for p in pairs])
    den = float(np.sum(f_arr * f_arr))
    linear = float(np.sum(f_arr * i_arr)) / den if den > 0.0 else 0.0
    if len(pairs) < 4:
        return linear, 0.0, 0.0
    x = f_arr / float(f_arr.max())          # 缩放域, 各列 O(1)
    xs = np.stack([x, x ** 2, x ** 3], axis=1)
    xtx = xs.T @ xs
    if abs(np.linalg.det(xtx)) < 1e-12 * max(1.0, float(np.trace(xtx))) ** 2:
        return linear, 0.0, 0.0
    try:
        c = np.linalg.solve(xtx, xs.T @ i_arr)
    except np.linalg.LinAlgError:
        return linear, 0.0, 0.0
    if not np.isfinite(c).all() or np.any(xs @ c < 0.0):
        return linear, 0.0, 0.0
    scale = float(f_arr.max())
    return float(c[0]) / scale, float(c[1]) / scale ** 2, float(c[2]) / scale ** 3


def calibrate(points, v_ref, voltage_normalization=True, environment_normalization=True, u_idle=0.0):
    """§3.1 + §3.2 完整流水线。

    points: dict 列表, 键 u(占空比) V I rpm(四电机列表) F(N) P(Pa) t(°C); 缺失量用 NaN。
    voltage_normalization=False (AirSim 无压降) 跳过步骤 3/4; 无气压/温度时跳过步骤 1。
    u_idle: 飞控 motor_idle 占空比, 单调性判据的窗口下沿 (dF/du > 0 on [u_idle,1],
    2026-11-10 修订; 0 = 严格旧版 [0,1] 且 a1 > 0)。调用方 (thrust_test_node) 从
    飞控 CLI 读回后传入。
    §3.2 拟合用相对误差权重 1/max(F_std, 1%·F_max) (2026-11-12 修订); 电流模型
    I_std(F_std) 为无截距三次三项式 (k_i, k_i2, k_i3, 2026-11-12 修订)。
    返回 dict: a3 a2 a1 r_eq ke kv k_i k_i2 k_i3 stats monotonic min_slope rows warnings。
    """
    warnings = []
    rows = []
    missing_env = 0
    for p in points:
        env = 1.0
        if environment_normalization:
            if math.isfinite(p.get('P', float('nan'))) and math.isfinite(p.get('t', float('nan'))):
                env = environment_factor(p['P'], p['t'])
            else:
                missing_env += 1
        rows.append({**p, 'env': env, 'F0': p['F'] * env, 'rpm_rms': rpm_rms(p.get('rpm', []))})
    if missing_env:
        warnings.append(f'{missing_env} point(s) without pressure/temperature: environment factor = 1')

    electrical = None
    if voltage_normalization:
        electrical, reason = identify_electrical([r['u'] for r in rows], [r['V'] for r in rows],
                                                 [r['I'] for r in rows], [r['rpm_rms'] for r in rows])
        if electrical is None:
            warnings.append(f'R_eq/K_v identification skipped ({reason}); '
                            f'using first-order (V_ref/V)^2 normalization')
    r_eq = electrical['r_eq'] if electrical else None
    for r in rows:
        if voltage_normalization:
            r['F_std'], r['I_std'] = standard_voltage_thrust(r['u'], r['V'], r['I'], r['F0'], r_eq, v_ref)
        else:
            r['F_std'], r['I_std'] = r['F0'], float('nan')
    if voltage_normalization and r_eq is not None:
        no_current = [r for r in rows if not (math.isfinite(r['I']) and r['I'] >= 0.0)]
        if no_current:
            warnings.append(f'{len(no_current)} point(s) without current normalized with the '
                            f'first-order (V_ref/V)^2 formula while the rest use R_eq')

    used = [r for r in rows if r['u'] > 0.0 and math.isfinite(r['F_std'])]
    if len(used) < len(rows):
        warnings.append(f'{len(rows) - len(used)} point(s) excluded (no voltage or non-positive back-EMF)')
    if len(used) < MIN_POINTS_RECOMMENDED:
        warnings.append(f'only {len(used)} points; fpv_test.md recommends N >= {MIN_POINTS_RECOMMENDED}')
    # 推力死区判据 (2026-11-12 修订, 数据级): 正油门采样点实测 F_std <= 0 说明推力
    # 起始高于采样下限, 无截距三次模型形态无效 —— 拒绝, 不依赖拟合伪影 (原判据
    # 靠无权拟合的负 a1 下穿 F_std(u_idle) < 0; 相对误差加权后 a1 已钳 0, 该伪影
    # 不复存在, 见 fit_zero_intercept_cubic)。
    deadband = None
    zero_pts = [r['u'] for r in used if r['F_std'] <= 0.0]
    if zero_pts:
        deadband = (f'thrust deadband: measured F_std <= 0 at u={min(zero_pts):.3f} '
                    f'(thrust onset above the sampled range; zero-intercept cubic invalid)')
    # 高油门非单调防护 (2026-11-12 评审, 数据级): 桨失速/推力塌陷数据本身随 u 递减,
    # 原"无权拟合负 a1 下穿"的拒绝路径被 a1 钳 0 消除, 钳位拟合反而可能通过单调门,
    # 以外推推力虚高放行。判据: 相邻采样点 (按 u 排序) 实测 F_std 下降超过
    # max(0.2 N, 2%) 即拒绝 —— 吸收台秤 0.1 N 量级噪声, 只抓真实塌陷。
    stall = None
    by_u = sorted(used, key=lambda r: r['u'])
    for prev, cur in zip(by_u, by_u[1:]):
        drop = prev['F_std'] - cur['F_std']
        if drop > max(0.2, 0.02 * prev['F_std']):
            stall = (f'thrust collapse: measured F_std decreases with u '
                     f'({prev["F_std"]:.2f} N at u={prev["u"]:.3f} -> '
                     f'{cur["F_std"]:.2f} N at u={cur["u"]:.3f}); prop stall / bad data')
            break
    used_f = [r['F_std'] for r in used]
    # 相对误差权重 (2026-11-12 修订, 见 fit_zero_intercept_cubic): floor 取数据最大
    # 推力的 1%, 兼防 F=0 (推力死区数据) 产生 inf。
    f_floor = max(0.01 * max(used_f), 1e-9)
    rel_weights = [1.0 / max(f, f_floor) for f in used_f]
    (a3, a2, a1), stats = fit_zero_intercept_cubic([r['u'] for r in used], used_f, weights=rel_weights)
    ok, slope = monotonic_check(a3, a2, a1, u_idle)
    if ok and (deadband or stall):
        ok = False
    if deadband:
        reason = deadband
    elif stall:
        reason = stall
    elif ok:
        reason = ''
    elif slope <= 0.0:
        reason = f'dF/du <= 0 on [u_idle={u_idle:.4f}, 1] (min slope {slope:.4g} N)'
    else:
        f_idle = ((a3 * u_idle + a2) * u_idle + a1) * u_idle
        reason = f'F_std(u_idle) <= 0 (F_std({u_idle:.4f}) = {f_idle:.4g} N)'
    if r_eq:
        k_i, k_i2, k_i3 = fit_current_coefficient(used_f, [r['I_std'] for r in used])
    else:
        k_i, k_i2, k_i3 = 0.0, 0.0, 0.0
    return {'a3': a3, 'a2': a2, 'a1': a1,
            'r_eq': r_eq if r_eq is not None else 0.0,
            'ke': electrical['ke'] if electrical else float('nan'),
            'kv': electrical['kv'] if electrical else float('nan'),
            'electrical_rms': electrical['rms'] if electrical else float('nan'),
            'k_i': k_i, 'k_i2': k_i2, 'k_i3': k_i3,
            'stats': stats, 'monotonic': ok, 'min_slope': slope,
            'monotonic_reason': reason,
            'rows': rows, 'warnings': warnings}


# ======================================================================
# §4: 在线实时反解 (quadratic_thrust_model.h 的 Python 镜像)
# ======================================================================

class LowPassFilter:
    """一阶低通 y += α(x − y), α = 1 − exp(−2π·f_c·dt) (时间步长无关的截止频率)。"""

    def __init__(self, cutoff_hz, initial=float('nan')):
        self.cutoff_hz = cutoff_hz
        self.value = initial

    def update(self, sample, dt):
        if not math.isfinite(sample):
            return self.value
        if not math.isfinite(self.value) or not (dt > 0.0):
            self.value = sample
        else:
            alpha = 1.0 - math.exp(-2.0 * math.pi * self.cutoff_hz * dt)
            self.value += alpha * (sample - self.value)
        return self.value


@dataclass
class PropulsionModel:
    a3: float
    a2: float
    a1: float
    v_ref: float = 6 * V_REF_PER_CELL
    u_idle: float = DEFAULT_U_IDLE
    r_eq: float = 0.0
    # I_std(F_std) = (k_i3·F + k_i2)·F + k_i)·F (A)。单一线性 k_i 的 §1.4 假设被
    # 2026-11 台架数据否定 (I/F 呈 U 形), k_i2/k_i3 为二次/三次修正项, 全 0 = 线性。
    k_i: float = 0.0
    k_i2: float = 0.0
    k_i3: float = 0.0
    v_min: float = 6 * V_MIN_PER_CELL
    inverse: tuple = field(default=None, init=False, repr=False)
    inverse_max_error: float = field(default=float('nan'), init=False, repr=False)

    @classmethod
    def from_parameters(cls, motor_parameters, lipo_cells, u_idle):
        """motor_parameters (YAML) + 飞控读回的 u_idle (fetch_u_idle) -> 模型。"""
        mp = motor_parameters
        return cls(a3=float(mp.get('a3', 0.0)), a2=float(mp.get('a2', 0.0)), a1=float(mp.get('a1', 0.0)),
                   v_ref=lipo_cells * V_REF_PER_CELL, u_idle=float(u_idle),
                   r_eq=float(mp.get('r_eq', 0.0)), k_i=float(mp.get('k_i', 0.0)),
                   k_i2=float(mp.get('k_i2', 0.0)), k_i3=float(mp.get('k_i3', 0.0)),
                   v_min=lipo_cells * V_MIN_PER_CELL)

    def __post_init__(self):
        if not self.validate():
            self._init_inverse()

    def set_u_idle(self, u_idle):
        """飞控 motor_idle 变化后更新 u_idle 并重拟合反函数初值 (同节点 queryMotorIdle)。"""
        self.u_idle = float(u_idle)
        self.inverse = None
        if not self.validate():
            self._init_inverse()

    @property
    def configured(self):
        return self.a3 != 0.0 or self.a2 != 0.0 or self.a1 != 0.0

    def thrust(self, u):
        """标准状态推力 F_std(u) = a3u³ + a2u² + a1u (N)。"""
        return ((self.a3 * u + self.a2) * u + self.a1) * u

    def slope(self, u):
        return (3.0 * self.a3 * u + 2.0 * self.a2) * u + self.a1

    def current_at_thrust(self, f_std):
        """电流模型 I_std(F_std) = ((k_i3·F + k_i2)·F + k_i)·F (A); 三项全 0 即关闭。"""
        return ((self.k_i3 * f_std + self.k_i2) * f_std + self.k_i) * f_std

    def current_model_active(self):
        return self.k_i != 0.0 or self.k_i2 != 0.0 or self.k_i3 != 0.0

    def validate(self):
        """空串 = 合法; 否则为原因 (同 C++ validateModel)。"""
        if not all(math.isfinite(v) for v in (self.a3, self.a2, self.a1, self.v_ref, self.u_idle,
                                               self.r_eq, self.k_i, self.k_i2, self.k_i3, self.v_min)):
            return 'parameters must be finite'
        if not 0.0 <= self.u_idle < 1.0:
            return 'u_idle must be in [0, 1)'
        ok, slope = monotonic_check(self.a3, self.a2, self.a1, self.u_idle)
        if not ok:
            return (f'thrust curve must satisfy dF/du > 0 on [u_idle, 1] and F_std(u_idle) > 0 '
                    f'(min slope {slope:.4g})')
        if self.v_ref <= 0.0 or not 0.0 <= self.v_min < self.v_ref:
            return 'require v_ref > 0 and 0 <= v_min < v_ref'
        if self.r_eq < 0.0:
            return 'r_eq must be non-negative'
        # 电流模型在标定推力段 [0, F(1)] 上须非负 (三项拟合的线性项可以为负, 只看
        # 合成电流; 负 I_std 会经 (I_meas − I_std)·R_eq 抬高油门, 非物理)。
        f_max = self.thrust(1.0)
        if f_max > 0.0 and any(self.current_at_thrust(s * f_max) < 0.0
                               for s in (0.2, 0.4, 0.6, 0.8, 1.0)):
            return 'current model I_std(F) must stay non-negative on the calibrated thrust range'
        return ''

    def _init_inverse(self, samples=256):
        """§4 步骤 2 的初值: 在有效油门区间 [u_idle, 1] 上把反函数拟合为
        u ≈ c0 + c1·w + c2·w² + c3·w³, w = sqrt(F/F(1))。纯二次/线性推力时精确。"""
        lo = max(self.u_idle, 1e-3)
        u = np.linspace(lo, 1.0, samples)
        f_max = self.thrust(1.0)
        w = np.sqrt(np.array([self.thrust(x) for x in u]) / f_max)
        x = np.stack([np.ones_like(w), w, w ** 2, w ** 3], axis=1)
        c = np.linalg.solve(x.T @ x, x.T @ u)
        max_error = float(np.max(np.abs(x @ c - u)))
        if not math.isfinite(max_error):
            return  # 病态/含 NaN (如窗口左端 F<0 使 sqrt 出 NaN): 不安装反函数
        self.inverse = (float(f_max), tuple(float(v) for v in c))
        self.inverse_max_error = max_error

    def initial_guess(self, f_std):
        if self.inverse is not None:
            f_max, c = self.inverse
            w = math.sqrt(max(f_std, 0.0) / f_max)
            u = ((c[3] * w + c[2]) * w + c[1]) * w + c[0]
        else:
            u = math.sqrt(max(f_std, 0.0) / (self.a3 + self.a2 + self.a1 + 1e-4))
        return _clamp(u, 0.01, 1.0)

    def solve_standard_throttle(self, f_std):
        """§4 步骤 2: 牛顿-拉夫逊解 a3u³ + a2u² + a1u − F_std = 0 (投影 [0, 1.2])。"""
        u = self.initial_guess(f_std)
        for _ in range(NEWTON_MAX_ITER):
            slope = self.slope(u)
            if abs(slope) < 1e-6:
                break
            delta = (self.thrust(u) - f_std) / slope
            u = _clamp(u - delta, 0.0, U_PROJECT_MAX)
            if abs(delta) < NEWTON_TOL:
                break
        return u

    def compensate(self, u_std, f_std, voltage, current=float('nan')):
        """§4 步骤 3: 母线电压与电流负载补偿; 无电压遥测时不补偿。

        完整式 u_cmd = (u_std·V_ref + (I_meas − I_std)·R_eq)/V, 其中 I_std 取自
        三项电流模型 current_at_thrust (2026-11-12 修订, 原为 k_i·F_std 线性)。
        """
        if not (math.isfinite(voltage) and voltage > 0.0):
            return u_std
        v = max(voltage, self.v_min)
        if self.r_eq > 0.0 and self.current_model_active() and \
                math.isfinite(current) and current >= 0.0:
            i_std = self.current_at_thrust(f_std)
            return (u_std * self.v_ref + (current - i_std) * self.r_eq) / v
        return u_std * (self.v_ref / v)

    def throttle(self, f_req, env_factor=1.0, voltage=float('nan'), current=float('nan')):
        """§4 步骤 1~4: 期望总推力 (N) -> 占空比 u ∈ [u_idle, 1]。"""
        if not (f_req > F_REQ_EPS) or not self.configured:
            return self.u_idle
        if not (math.isfinite(env_factor) and env_factor > 0.0):
            raise ValueError('environment factor must be finite and positive')
        f_std = f_req * env_factor
        u_std = self.solve_standard_throttle(f_std)
        u_cmd = self.compensate(u_std, f_std, voltage, current)
        return _clamp(u_cmd, self.u_idle, 1.0)

    def collective(self, f_req, env_factor=1.0, voltage=float('nan'), current=float('nan')):
        """throttle() 换算到 Betaflight 集体油门 (custom link 0x20); u_idle 对应 c = 0。"""
        return duty_to_collective(self.throttle(f_req, env_factor, voltage, current), self.u_idle)
