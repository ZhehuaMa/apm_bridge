"""fpv_test.md 推力-油门建模 (§1/§3) 与在线反解 (§4) 的 Python 实现。

与 include/quadratic_thrust_model.h (节点运行时) 同口径, 供台架标定工具
(thrust_test_node)、SITL 标定、硬件测试镜像与单元测试共用。

油门域: 文档的 u 是整机归一化油门开度 (PWM/DShot 占空比), F(0)=0。Betaflight 的
集体油门 c (custom link 0x20) 先映射到 DShot 输出 u = motor_idle + (1-motor_idle)·c
(dshot.c outputLow, 默认 dshot_idle_value=550 -> 5.5%), 因此模型全部在占空比 u 上
建立, 下发前再换算回 c。

基准状态 (§1.1): V_ref = cells·3.7 V (6S = 22.20 V), T0 = 273.15 K, P0 = 101325 Pa。
"""

import math
from dataclasses import dataclass, field

import numpy as np

P0 = 101325.0                 # Pa
T0 = 273.15                   # K
V_REF_PER_CELL = 3.7          # 22.20 V for 6S
V_MIN_PER_CELL = 2.0          # 12.0 V for 6S: §5 硬件欠压保护下限
DEFAULT_U_IDLE = 0.05         # §5 怠速死区下限
DEFAULT_MOTOR_IDLE = 0.055    # Betaflight dshot_idle_value 550
F_REQ_EPS = 0.001             # §5 零推力保护阈值 (N)
NEWTON_MAX_ITER = 4           # §4 步骤 2: 3~4 次
NEWTON_TOL = 1e-5             # §4 步骤 2: 1e-5 精度
U_PROJECT_MAX = 1.2           # §5 迭代投影区间 [0, 1.2]
MIN_POINTS_RECOMMENDED = 20   # §3.2 N >= 20


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


# ======================================================================
# 油门域换算 (Betaflight DShot 输出映射)
# ======================================================================

def collective_to_duty(collective, motor_idle=DEFAULT_MOTOR_IDLE):
    return motor_idle + (1.0 - motor_idle) * _clamp(collective, 0.0, 1.0)


def duty_to_collective(duty, motor_idle=DEFAULT_MOTOR_IDLE):
    return _clamp((duty - motor_idle) / (1.0 - motor_idle), 0.0, 1.0)


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

def fit_zero_intercept_cubic(u, force):
    """Θ = (XᵀX)⁻¹XᵀY, X = [u³ u² u], 返回 ((a3, a2, a1), stats)。"""
    u = np.asarray(u, dtype=float)
    f = np.asarray(force, dtype=float)
    if u.shape != f.shape or len(u) < 3 or len(np.unique(u)) < 3:
        raise ValueError('need at least 3 distinct throttle levels')
    if not (np.isfinite(u).all() and np.isfinite(f).all()):
        raise ValueError('non-finite calibration samples')
    x = np.stack([u ** 3, u ** 2, u], axis=1)
    theta = np.linalg.solve(x.T @ x, x.T @ f)
    residual = x @ theta - f
    ss_tot = float(np.sum((f - f.mean()) ** 2))
    stats = {'rms': float(np.sqrt(np.mean(residual ** 2))),
             'max_error': float(np.max(np.abs(residual))),
             'r_squared': float(1.0 - np.sum(residual ** 2) / ss_tot) if ss_tot > 0 else float('nan'),
             'n': int(len(u))}
    return tuple(float(v) for v in theta), stats


def min_slope_on_unit_interval(a3, a2, a1):
    """min_{u∈[0,1]} dF/du, dF/du = 3a3u² + 2a2u + a1 (端点与驻点取最小)。"""
    candidates = [0.0, 1.0]
    if a3 != 0.0:
        vertex = -a2 / (3.0 * a3)
        if 0.0 < vertex < 1.0:
            candidates.append(vertex)
    return min(3.0 * a3 * u * u + 2.0 * a2 * u + a1 for u in candidates)


def monotonic_check(a3, a2, a1):
    """§3.2 物理合法性: a1 > 0 且 dF/du > 0 ∀u∈[0,1]。返回 (ok, min_slope)。"""
    slope = min_slope_on_unit_interval(a3, a2, a1)
    return (a1 > 0.0 and slope > 0.0), slope


def fit_current_coefficient(f_std, i_std):
    """I_std = k_i·F_std (I ∝ RPM² ∝ F, §1.4), 无截距最小二乘; 无数据返回 0。"""
    pairs = [(f, i) for f, i in zip(f_std, i_std) if math.isfinite(f) and math.isfinite(i) and f > 0.0]
    if len(pairs) < 2:
        return 0.0
    num = sum(f * i for f, i in pairs)
    den = sum(f * f for f, _ in pairs)
    return num / den if den > 0.0 else 0.0


def calibrate(points, v_ref, voltage_normalization=True, environment_normalization=True):
    """§3.1 + §3.2 完整流水线。

    points: dict 列表, 键 u(占空比) V I rpm(四电机列表) F(N) P(Pa) t(°C); 缺失量用 NaN。
    voltage_normalization=False (AirSim 无压降) 跳过步骤 3/4; 无气压/温度时跳过步骤 1。
    返回 dict: a3 a2 a1 r_eq ke kv k_i stats monotonic min_slope rows warnings。
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

    used = [r for r in rows if r['u'] > 0.0 and math.isfinite(r['F_std'])]
    if len(used) < len(rows):
        warnings.append(f'{len(rows) - len(used)} point(s) excluded (no voltage or non-positive back-EMF)')
    if len(used) < MIN_POINTS_RECOMMENDED:
        warnings.append(f'only {len(used)} points; fpv_test.md recommends N >= {MIN_POINTS_RECOMMENDED}')
    (a3, a2, a1), stats = fit_zero_intercept_cubic([r['u'] for r in used], [r['F_std'] for r in used])
    ok, slope = monotonic_check(a3, a2, a1)
    k_i = fit_current_coefficient([r['F_std'] for r in used], [r['I_std'] for r in used]) if r_eq else 0.0
    return {'a3': a3, 'a2': a2, 'a1': a1,
            'r_eq': r_eq if r_eq is not None else 0.0,
            'ke': electrical['ke'] if electrical else float('nan'),
            'kv': electrical['kv'] if electrical else float('nan'),
            'electrical_rms': electrical['rms'] if electrical else float('nan'),
            'k_i': k_i, 'stats': stats, 'monotonic': ok, 'min_slope': slope,
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
    k_i: float = 0.0
    v_min: float = 6 * V_MIN_PER_CELL
    motor_idle: float = DEFAULT_MOTOR_IDLE
    inverse: tuple = field(default=None, init=False, repr=False)
    inverse_max_error: float = field(default=float('nan'), init=False, repr=False)

    @classmethod
    def from_parameters(cls, motor_parameters, lipo_cells):
        mp = motor_parameters
        return cls(a3=float(mp.get('a3', 0.0)), a2=float(mp.get('a2', 0.0)), a1=float(mp.get('a1', 0.0)),
                   v_ref=lipo_cells * V_REF_PER_CELL,
                   u_idle=float(mp.get('u_idle', DEFAULT_U_IDLE)),
                   r_eq=float(mp.get('r_eq', 0.0)), k_i=float(mp.get('k_i', 0.0)),
                   v_min=lipo_cells * V_MIN_PER_CELL,
                   motor_idle=float(mp.get('motor_idle', DEFAULT_MOTOR_IDLE)))

    def __post_init__(self):
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

    def validate(self):
        """空串 = 合法; 否则为原因 (同 C++ validateModel)。"""
        if not all(math.isfinite(v) for v in (self.a3, self.a2, self.a1, self.v_ref, self.u_idle,
                                               self.r_eq, self.k_i, self.v_min, self.motor_idle)):
            return 'parameters must be finite'
        ok, slope = monotonic_check(self.a3, self.a2, self.a1)
        if not ok:
            return f'thrust curve must satisfy a1 > 0 and dF/du > 0 on [0,1] (min slope {slope:.4g})'
        if not 0.0 <= self.u_idle < 1.0 or not 0.0 <= self.motor_idle < 1.0:
            return 'u_idle and motor_idle must be in [0, 1)'
        if self.v_ref <= 0.0 or not 0.0 <= self.v_min < self.v_ref:
            return 'require v_ref > 0 and 0 <= v_min < v_ref'
        if self.r_eq < 0.0 or self.k_i < 0.0:
            return 'r_eq and k_i must be non-negative'
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
        self.inverse = (float(f_max), tuple(float(v) for v in c))
        self.inverse_max_error = float(np.max(np.abs(x @ c - u)))

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
        """§4 步骤 3: 母线电压与电流负载补偿; 无电压遥测时不补偿。"""
        if not (math.isfinite(voltage) and voltage > 0.0):
            return u_std
        v = max(voltage, self.v_min)
        if self.r_eq > 0.0 and self.k_i > 0.0 and math.isfinite(current) and current >= 0.0:
            i_std = self.k_i * f_std
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
        """throttle() 换算到 Betaflight 集体油门 (custom link 0x20)。"""
        return duty_to_collective(self.throttle(f_req, env_factor, voltage, current), self.motor_idle)
