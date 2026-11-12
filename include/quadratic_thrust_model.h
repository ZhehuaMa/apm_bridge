#ifndef QUADRATIC_THRUST_MODEL_H
#define QUADRATIC_THRUST_MODEL_H

// fpv_test.md 推力-油门模型的在线实时反解 (§4), 与 scripts/propulsion_model.py 同口径。
//
// u 为整机归一化油门开度 (DShot 占空比, F(0)=0)。标准状态 (V_ref = cells·3.7 V,
// T0 = 273.15 K, P0 = 101325 Pa) 下:  F_std(u) = a3·u³ + a2·u² + a1·u  (总推力, N)。
// 怠速下限 u_idle 即飞控 motor_idle (CLI "get motor_idle", 0.01 % 单位): Betaflight
// 集体油门 c 输出占空比 u = u_idle + (1 − u_idle)·c, 故 §4 步骤 4 的下限 u_idle 恰对应
// c = 0 (换算见 dutyToCollective)。全部为 inline 定义, 可被多个翻译单元包含。

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace quadratic_thrust_model
{
    constexpr double kP0 = 101325.0;          // Pa
    constexpr double kT0 = 273.15;            // K
    constexpr double kForceEpsilon = 0.001;   // §5 零推力保护阈值 (N)
    constexpr int kNewtonMaxIter = 4;         // §4 步骤 2: 3~4 次
    constexpr double kNewtonTolerance = 1e-5; // §4 步骤 2: 1e-5 精度
    constexpr double kProjectMax = 1.2;       // §5 迭代投影区间 [0, 1.2]

    struct PropulsionModel
    {
        // 标定模型 F_std(u) = a3·u³ + a2·u² + a1·u (thrust_test_node 按 §3 拟合)
        double a3 = 0.0;
        double a2 = 0.0;
        double a1 = 0.0;
        double v_ref = 22.2;  // 基准电压 V_ref (cells·3.7)
        double u_idle = 0.05; // 怠速死区下限 (§4 步骤 4) = 飞控 motor_idle 占空比, 运行时由 CLI 读取
        double r_eq = 0.0;    // 全系统等效集中电阻 (Ω); 0 = 一阶电压补偿
        // 电流模型 I_std(F_std) = ((k_i3·F + k_i2)·F + k_i)·F (A)。§1.4 原线性假设
        // "I ∝ RPM² ∝ F" 被 2026-11 台架数据否定 (I/F 呈 U 形: 怠速 ~2.7 → 谷底 ~1.2
        // → 满油门 ~1.9 A/N), k_i2/k_i3 为二次/三次修正项; 三项全 0 = 线性/一阶补偿。
        double k_i = 0.0;
        double k_i2 = 0.0;
        double k_i3 = 0.0;
        double v_min = 12.0;  // 欠压保护下限 (cells·2.0, §5)
        // 动力系统最大集体油门保护上限 [0, 1] (0x10 集体油门域, u_idle 对应 0):
        // 节点在 pushControl 输出端对力通道与 raw 通道统一限幅; 1.0 = 不限制。
        // 与推力曲线无关 (非标定量), 仅是输出保护界。
        double throttle_limit = 1.0;

        // §4 步骤 2 初值: 有效油门区间 [u_idle, 1] 上的反函数三次拟合 (initInverse 生成)
        //   u ≈ c0 + c1·w + c2·w² + c3·w³,  w = sqrt(F / F(1))
        bool inverse_valid = false;
        double inverse_f_max = 0.0;
        double inverse_coef[4] = {};
    };

    inline bool configured(const PropulsionModel &m)
    {
        return m.a3 != 0.0 || m.a2 != 0.0 || m.a1 != 0.0;
    }

    inline double standardThrust(const PropulsionModel &m, double u)
    {
        return ((m.a3 * u + m.a2) * u + m.a1) * u;
    }

    inline double standardThrustSlope(const PropulsionModel &m, double u)
    {
        return (3.0 * m.a3 * u + 2.0 * m.a2) * u + m.a1;
    }

    /// 电流模型 I_std(F_std) (A), 与 propulsion_model.current_at_thrust 逐位一致。
    inline double currentAtThrust(const PropulsionModel &m, double f_std)
    {
        return ((m.k_i3 * f_std + m.k_i2) * f_std + m.k_i) * f_std;
    }

    inline bool currentModelActive(const PropulsionModel &m)
    {
        return m.k_i != 0.0 || m.k_i2 != 0.0 || m.k_i3 != 0.0;
    }

    /// min_{u∈[u_idle,1]} dF/du (端点与驻点; u_idle 先钳到 [0,1], 范围校验见 validateModel)。
    inline double minSlopeOnUnitInterval(const PropulsionModel &m)
    {
        const double lo = std::clamp(m.u_idle, 0.0, 1.0);
        double slope = std::min(standardThrustSlope(m, lo), standardThrustSlope(m, 1.0));
        if (m.a3 != 0.0)
        {
            const double vertex = -m.a2 / (3.0 * m.a3);
            if (vertex > lo && vertex < 1.0)
                slope = std::min(slope, standardThrustSlope(m, vertex));
        }
        return slope;
    }

    /// 与曲线无关的参数范围校验 (有限性、u_idle/v_ref/v_min/r_eq 范围); 空串 = 合法。
    /// 启动时 u_idle 仅为占位值 (FC motor_idle 未读), 曲线判据须等真实 u_idle
    /// (validateModel) 再执行, 故拆出此函数。电流模型只校验有限性: 三项拟合的
    /// 线性项 k_i 可为负 (合成电流非负即可), 非负性属曲线相关判据 (validateModel)。
    inline std::string validateParameters(const PropulsionModel &m)
    {
        for (double v : {m.a3, m.a2, m.a1, m.v_ref, m.u_idle, m.r_eq, m.k_i, m.k_i2, m.k_i3, m.v_min})
            if (!std::isfinite(v))
                return "parameters must be finite";
        if (m.u_idle < 0.0 || m.u_idle >= 1.0)
            return "u_idle must be in [0, 1)";
        if (!(m.v_ref > 0.0) || m.v_min < 0.0 || m.v_min >= m.v_ref)
            return "require v_ref > 0 and 0 <= v_min < v_ref";
        if (m.r_eq < 0.0)
            return "r_eq must be non-negative";
        return {};
    }

    /// §3.2 物理合法性与单调性检验 + 运行参数范围; 空串 = 合法。
    /// 单调窗口为可操作域 [u_idle, 1] (2026-11-10 修订, 与 propulsion_model.monotonic_check
    /// 同口径): u < u_idle 被 §4 步骤 4 的饱和限幅截止、运行时不可达, 真实桨凸数据的
    /// 无截距三次拟合常带完全落在其下的微小负斜率区。正推力条件取 F_std(u_idle) > 0:
    /// 斜率>0 使 F 在窗口上单调递增, min F = F(u_idle), 故它蕴含窗口内恒正 (含
    /// F_std(1) > 0) 且反解初值 w = sqrt(F/F(1)) 良定义; 单调 + F(1) > 0 并不蕴含
    /// 窗口恒正 (曲线可在窗口左端为负、中途穿零)。u_idle=0 时退化为原版 a1 > 0。
    inline std::string validateModel(const PropulsionModel &m)
    {
        const std::string param_error = validateParameters(m);
        if (!param_error.empty())
            return param_error;
        if (!(minSlopeOnUnitInterval(m) > 0.0) ||
            (m.u_idle > 0.0 && !(standardThrust(m, m.u_idle) > 0.0)))
            return "thrust curve must satisfy dF/du > 0 on [u_idle, 1] and F_std(u_idle) > 0";
        const double f_max = standardThrust(m, 1.0);
        if (f_max > 0.0)
            for (double s : {0.2, 0.4, 0.6, 0.8, 1.0})
                if (currentAtThrust(m, s * f_max) < 0.0)
                    return "current model I_std(F) must stay non-negative on the calibrated thrust range";
        return {};
    }

    /// §4 步骤 1 环境折算系数 (P0/P)·(T/T0) (R_spec 对消); 输入非法返回 NaN。
    inline double environmentFactor(double pressure_pa, double temperature_c)
    {
        const double t_kelvin = temperature_c + kT0;
        if (!std::isfinite(pressure_pa) || pressure_pa <= 0.0 || !std::isfinite(t_kelvin) || t_kelvin <= 0.0)
            return std::numeric_limits<double>::quiet_NaN();
        return (kP0 / pressure_pa) * (t_kelvin / kT0);
    }

    /// 在有效油门区间 [u_idle, 1] 上把 F_std 的反函数拟合为 w = sqrt(F/F(1)) 的三次多项式
    /// (纯二次/线性推力时精确), 作为牛顿迭代初值。返回区间内最大油门误差, 失败返回 -1。
    inline double initInverse(PropulsionModel &m, int samples = 256)
    {
        m.inverse_valid = false;
        if (!validateModel(m).empty() || samples < 8)
            return -1.0;
        const double lo = std::max(m.u_idle, 1e-3);
        const double f_max = standardThrust(m, 1.0);
        double s[7] = {};  // Σ w^k
        double t[4] = {};  // Σ w^k·u
        for (int i = 0; i < samples; ++i)
        {
            const double u = lo + (1.0 - lo) * i / (samples - 1);
            const double w = std::sqrt(standardThrust(m, u) / f_max);
            double wp = 1.0;
            for (int k = 0; k <= 6; ++k)
            {
                s[k] += wp;
                if (k < 4)
                    t[k] += wp * u;
                wp *= w;
            }
        }
        // 正规方程 4×4, 部分主元高斯消元
        double a[4][5];
        for (int r = 0; r < 4; ++r)
        {
            for (int c = 0; c < 4; ++c)
                a[r][c] = s[r + c];
            a[r][4] = t[r];
        }
        for (int col = 0; col < 4; ++col)
        {
            int piv = col;
            for (int r = col + 1; r < 4; ++r)
                if (std::abs(a[r][col]) > std::abs(a[piv][col]))
                    piv = r;
            if (std::abs(a[piv][col]) < 1e-14)
                return -1.0;
            if (piv != col)
                for (int c = 0; c < 5; ++c)
                    std::swap(a[piv][c], a[col][c]);
            for (int r = col + 1; r < 4; ++r)
            {
                const double ratio = a[r][col] / a[col][col];
                for (int c = col; c < 5; ++c)
                    a[r][c] -= ratio * a[col][c];
            }
        }
        double coef[4];
        for (int r = 3; r >= 0; --r)
        {
            double acc = a[r][4];
            for (int c = r + 1; c < 4; ++c)
                acc -= a[r][c] * coef[c];
            coef[r] = acc / a[r][r];
        }
        double max_error = 0.0;
        for (int i = 0; i <= 1000; ++i)
        {
            const double u = lo + (1.0 - lo) * i / 1000.0;
            const double w = std::sqrt(standardThrust(m, u) / f_max);
            const double fit = ((coef[3] * w + coef[2]) * w + coef[1]) * w + coef[0];
            const double err = std::abs(fit - u);
            if (!std::isfinite(err))
                return -1.0; // 防御: NaN 不得经 std::max(0, NaN) 吞掉后安装全 NaN 反函数
            max_error = std::max(max_error, err);
        }
        if (!std::isfinite(max_error))
            return -1.0;
        std::copy(coef, coef + 4, m.inverse_coef);
        m.inverse_f_max = f_max;
        m.inverse_valid = true;
        return max_error;
    }

    inline double initialGuess(const PropulsionModel &m, double f_std)
    {
        double u;
        if (m.inverse_valid)
        {
            const double w = std::sqrt(std::max(f_std, 0.0) / m.inverse_f_max);
            const double *c = m.inverse_coef;
            u = ((c[3] * w + c[2]) * w + c[1]) * w + c[0];
        }
        else
            u = std::sqrt(std::max(f_std, 0.0) / (m.a3 + m.a2 + m.a1 + 1e-4)); // §5 原始初值
        return std::clamp(u, 0.01, 1.0);
    }

    /// §4 步骤 2: 牛顿-拉夫逊解 a3·u³ + a2·u² + a1·u − F_std = 0, 迭代投影到 [0, 1.2]。
    inline double solveStandardThrottle(const PropulsionModel &m, double f_std)
    {
        double u = initialGuess(m, f_std);
        for (int i = 0; i < kNewtonMaxIter; ++i)
        {
            const double slope = standardThrustSlope(m, u);
            if (std::abs(slope) < 1e-6)
                break;
            const double delta = (standardThrust(m, u) - f_std) / slope;
            u = std::clamp(u - delta, 0.0, kProjectMax);
            if (std::abs(delta) < kNewtonTolerance)
                break;
        }
        return u;
    }

    /// §4 步骤 3: 母线电压与电流负载补偿。voltage 为低通后的母线电压, current 为低通后的
    /// 总电流; 电压无效 (<=0/NaN) 时不补偿。r_eq、电流模型 (k_i/k_i2/k_i3 任一非零)
    /// 与有效电流齐备时用完整式
    ///   u_cmd = (u_std·V_ref + (I − I_std)·R_eq) / V,  I_std = currentAtThrust(F_std)
    /// 否则用一阶工程式 u_cmd = u_std·V_ref / V。V 先钳到欠压下限 v_min。
    inline double compensate(const PropulsionModel &m, double u_std, double f_std,
                             double voltage, double current = std::numeric_limits<double>::quiet_NaN())
    {
        if (!std::isfinite(voltage) || voltage <= 0.0)
            return u_std;
        const double v = std::max(voltage, m.v_min);
        if (m.r_eq > 0.0 && currentModelActive(m) && std::isfinite(current) && current >= 0.0)
        {
            const double i_std = currentAtThrust(m, f_std);
            return (u_std * m.v_ref + (current - i_std) * m.r_eq) / v;
        }
        return u_std * (m.v_ref / v);
    }

    /// §4 步骤 1~4: 期望总推力 f_req (N) -> 输出占空比 u ∈ [u_idle, 1]。
    /// env_factor = environmentFactor(P, t) (未知时传 1); voltage/current 传 NaN 表示不补偿。
    inline double calculateThrottle(const PropulsionModel &m, double f_req, double env_factor = 1.0,
                                    double voltage = std::numeric_limits<double>::quiet_NaN(),
                                    double current = std::numeric_limits<double>::quiet_NaN())
    {
        if (!(f_req > kForceEpsilon) || !configured(m))
            return m.u_idle;
        if (!std::isfinite(env_factor) || env_factor <= 0.0)
            env_factor = 1.0;
        const double f_std = f_req * env_factor;
        const double u_std = solveStandardThrottle(m, f_std);
        const double u_cmd = compensate(m, u_std, f_std, voltage, current);
        return std::clamp(u_cmd, m.u_idle, 1.0);
    }

    /// 一阶低通 y += α(x − y), α = 1 − exp(−2π·f_c·dt): 截止频率与采样间隔无关
    /// (§4 要求母线电压经 2~5 Hz 低通; §5 的 α=0.02@1 kHz 即约 3.2 Hz)。
    struct LowPassFilter
    {
        double cutoff_hz = 3.0;
        double value = std::numeric_limits<double>::quiet_NaN();

        double update(double sample, double dt)
        {
            if (!std::isfinite(sample))
                return value;
            if (!std::isfinite(value) || !(dt > 0.0))
                value = sample;
            else
                value += (1.0 - std::exp(-2.0 * M_PI * cutoff_hz * dt)) * (sample - value);
            return value;
        }
    };

    /// Betaflight DShot 输出: 占空比 u = u_idle + (1 − u_idle)·c (dshot.c outputLow,
    /// u_idle = motor_idle/10000)。
    inline double collectiveToDuty(double collective, double u_idle)
    {
        return u_idle + (1.0 - u_idle) * std::clamp(collective, 0.0, 1.0);
    }

    inline double dutyToCollective(double duty, double u_idle)
    {
        return std::clamp((duty - u_idle) / (1.0 - u_idle), 0.0, 1.0);
    }

    /// 解析 custom link CLI "get motor_idle" 的回读 "motor_idle = 550" (单位 0.01 %,
    /// Betaflight 取值 0..2000) 为占空比 0.055; 名称不符/非整数/越界返回 NaN。
    inline double idleDutyFromCli(const std::string &text)
    {
        constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
        const auto eq = text.find('=');
        if (eq == std::string::npos)
            return kNan;
        auto trim = [](std::string s) {
            const auto first = s.find_first_not_of(" \t\r\n");
            const auto last = s.find_last_not_of(" \t\r\n");
            return first == std::string::npos ? std::string{} : s.substr(first, last - first + 1);
        };
        const std::string value = trim(text.substr(eq + 1));
        if (trim(text.substr(0, eq)) != "motor_idle" || value.empty() ||
            value.find_first_not_of("0123456789") != std::string::npos || value.size() > 5)
            return kNan;
        const int raw = std::stoi(value);
        return raw <= 2000 ? raw / 10000.0 : kNan;
    }

} // namespace quadratic_thrust_model

#endif // QUADRATIC_THRUST_MODEL_H
