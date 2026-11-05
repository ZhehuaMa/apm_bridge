#ifndef QUADRATIC_THRUST_MODEL_H
#define QUADRATIC_THRUST_MODEL_H

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

// 头文件内全部函数为 inline: 可被多个翻译单元包含而不违反 ODR / 触发未用函数告警。
namespace quadratic_thrust_model
{

    struct MotorParams
    {
        double A;
        double B;
        double C;
        double D;
        int n_motors;
        double thrust_limit;
        double spin_k;
        double volt_max;   // 满电电压 (cells*4.2): 线性补偿口径 (vbat 未启用时)
        double vbat_a;     // vbat 电压模型 y = a*V + b (thrust_test_node --vbat 台架拟合)。
        double vbat_b;     // a=0 (默认) 时未启用, 退回线性 volt_max/V 补偿。
        double volt_ref;   // 标准电压 (cells*3.7 标称): A..D 曲线与 vbat 补偿的锚点
        // 补偿电压下限 (fpv_test.md §4 欠压钳位): 低于此值的电压读数按此值补偿, 防止
        // 读数异常跌落时油门被成倍放大。<=0 不钳位。
        double volt_min = 0.0;

        // ---- 单调包络 (validateMotorParams 生成) ----
        // P(x) = A·x³+B·x²+C·x+D 在 x>0 的首个极大值点; 0 = 全域单调 (无截断)。
        // 多项式自变量截断到 [0, x_peak], 超过极大值的大推力请求饱和而不是回落。
        double x_peak = 0.0;

        // ---- 反演曲线预拟合 (initInverseFit 生成) ----
        // thrustToForce 的牛顿迭代在高频控制率下开销可观; 读取参数后在油门域
        // [0, thrust_limit] 上采样反演 x(throttle), 分段拟合三次曲线 (反演曲线
        // 全域上端急剧变陡, 单一三次多项式残差大), 运行时域内以多项式求值替代迭代。
        static constexpr int kInvFitMaxSegments = 8;
        bool inv_fit_valid = false;
        double inv_t_max = 0.0;        // 拟合油门域上界
        int inv_segments = 0;          // 实际段数
        double inv_seg_scale = 0.0;    // 段数/t_max: t -> 段索引
        double inv_coef[kInvFitMaxSegments][4] = {}; // 每段 [d, c, b, a]: x(t)=((a·t+b)·t+c)·t+d
    };

    constexpr double EPSILON = 1e-6; // 精度阈值
    constexpr int MAX_ITER = 100;   // 最大迭代次数，防止死循环

    // 计算函数值 f(x) = aX^3 + bX^2 + cX + d - Y
    inline double f(const MotorParams &params, double x, double targetY) {
        auto x2 = x * x;
        auto x3 = x2 * x;
        return (params.A * x3) + (params.B * x2) + (params.C * x) + params.D - targetY;
    }

    // 计算导数值 f'(x) = 3aX^2 + 2bX + c
    // 导数用于确定切线斜率，指引下一次迭代的方向
    inline double df(const MotorParams &params, double x) {
        return (3.0 * params.A * x * x) + (2.0 * params.B * x) + params.C;
    }

    // kp = (baro_pressure_pa / 101325.0) * (273.15 / (273.15 + temperature_c));

    // vbat 电压因子 phi(V) = (a*V+b)/(a*V_ref+b):
    // 当前电压下的力 <-> 标准电压 (V_ref) 口径力 的换算。
    // 标定端 (thrust_test_node --map-a/--map-b) 与本运行时对称使用。
    inline double vbatScale(const MotorParams &params, double battery_voltage)
    {
        return (params.vbat_a * battery_voltage + params.vbat_b) /
               (params.vbat_a * params.volt_ref + params.vbat_b);
    }

    // 补偿用电压: 非有限/<=0 视为无遥测 (返回 -1, 不补偿); 低于 volt_min 钳到 volt_min。
    // forceToThrust 与 thrustToForce 共用, 保证正反变换在钳位区也互逆。
    inline double compensationVoltage(const MotorParams &params, double battery_voltage)
    {
        if (!std::isfinite(battery_voltage) || battery_voltage <= 0.0)
            return -1.0;
        return params.volt_min > 0.0 ? std::max(battery_voltage, params.volt_min) : battery_voltage;
    }

    // spin 映射 (1-k)·t + k·t² 的逆: 已知映射后油门求映射前油门。
    inline double inverseSpin(double spin_k, double throttle)
    {
        if (std::abs(spin_k) <= EPSILON)
            return throttle;
        const double k_1 = spin_k - 1.0;
        return (k_1 + std::sqrt(k_1 * k_1 + 4.0 * spin_k * throttle)) / (2.0 * spin_k);
    }

    // 牛顿迭代内核: 求解 A·x³ + B·x² + C·x + D = throttle (原 thrustToForce 内联实现,
    // 抽出供 initInverseFit 采样与 thrustToForce 域外回退共用, 数值行为不变)。
    inline double newtonInvert(const MotorParams &params, double throttle)
    {
        // 使用牛顿法求解方程 motor_params.A * x^3 + motor_params.B * x^2 + motor_params.C * x - thrust = 0
        // 目标是找到 x，使得上述方程成立，然后计算对应的力

        // 1. 初始猜测值
        double x = 0.0;
        // 避免初始导数为 0 的情况（导数为 0 会导致除以 0 错误）
        if (std::abs(df(params, x)) < 1e-6) {
            x = 1.0;
        }

        // 2. 开始迭代
        for (int i = 0; i < MAX_ITER; ++i)
        {
            double y_curr = f(params, x, throttle);
            // 检查是否已经足够接近目标（收敛）
            if (std::abs(y_curr) < EPSILON) {
                break;
            }

            double slope = df(params, x);
            // 检查导数是否过小（防止除以零或发散）
            if (std::abs(slope) < 1e-6) {
                // 如果导数几乎为0，稍微扰动一下 x 继续尝试
                x += 0.01;
                continue;
            }

            // 牛顿法公式: x_new = x_old - f(x) / f'(x)
            double next_x = x - (y_curr / slope);

            // 检查两次迭代之间变化是否极小（另一种收敛判断）
            if (std::abs(next_x - x) < EPSILON) {
                x = next_x;
                break;
            }
            x = next_x;
        }
        return x;
    }

    /// 分段三次最小二乘拟合: 在 [t_lo, t_hi] 上对 newtonInvert 采样 n 点,
    /// 求解正规方程得 x(t) = ((a·t+b)·t+c)·t+d 的系数 [d, c, b, a]。
    /// 奇异/非有限值时返回 false。
    inline bool fitCubicSegment(const MotorParams &params, double t_lo, double t_hi,
                                int n, double out[4])
    {
        double s[7] = {0, 0, 0, 0, 0, 0, 0}; // S_k = Σ t^k, k=0..6
        double tv[4] = {0, 0, 0, 0};         // T_k = Σ t^k·x, k=0..3
        for (int i = 0; i < n; i++)
        {
            const double t = t_lo + (t_hi - t_lo) * i / (n - 1);
            const double x = newtonInvert(params, t);
            if (!std::isfinite(x))
                return false;
            double tp = 1.0;
            for (int k = 0; k <= 6; k++)
            {
                s[k] += tp;
                if (k < 4)
                    tv[k] += tp * x;
                tp *= t;
            }
        }
        // 正规方程 [S0..S3; ...]·[d c b a]ᵀ = [T0..T3]ᵀ, 高斯消元 (部分主元)
        double m[4][5] = {
            {s[0], s[1], s[2], s[3], tv[0]},
            {s[1], s[2], s[3], s[4], tv[1]},
            {s[2], s[3], s[4], s[5], tv[2]},
            {s[3], s[4], s[5], s[6], tv[3]},
        };
        for (int col = 0; col < 4; col++)
        {
            int piv = col;
            for (int r = col + 1; r < 4; r++)
                if (std::abs(m[r][col]) > std::abs(m[piv][col]))
                    piv = r;
            if (std::abs(m[piv][col]) < 1e-12)
                return false; // 样本退化
            if (piv != col)
                for (int c = 0; c < 5; c++)
                    std::swap(m[piv][c], m[col][c]);
            for (int r = col + 1; r < 4; r++)
            {
                const double ratio = m[r][col] / m[col][col];
                for (int c = col; c < 5; c++)
                    m[r][c] -= ratio * m[col][c];
            }
        }
        for (int r = 3; r >= 0; r--)
        {
            double acc = m[r][4];
            for (int c = r + 1; c < 4; c++)
                acc -= m[r][c] * out[c];
            out[r] = acc / m[r][r];
        }
        return std::isfinite(out[0] + out[1] + out[2] + out[3]);
    }

    /// 采样拟合反演曲线: 在油门域 [0, thrust_limit] (kp=1, spin_k=0 对应推力指令域;
    /// thrust_limit<=0 时取 1.0) 上分 segments 段 (<=kInvFitMaxSegments), 每段对
    /// newtonInvert 采样做三次最小二乘拟合, 写回 MotorParams。读取完参数后调用
    /// 一次, 之后 thrustToForce 域内以多项式求值替代牛顿迭代。
    /// 返回最大拟合残差 (N, 整机口径 = n_motors·单机残差, 稠密网格校验);
    /// 拟合失败返回 -1 (inv_fit_valid 保持 false, thrustToForce 退回纯牛顿迭代)。
    inline double initInverseFit(MotorParams &params, int segments = 8,
                                 int samples_per_segment = 32)
    {
        params.inv_fit_valid = false;
        if (segments < 1 || segments > MotorParams::kInvFitMaxSegments ||
            samples_per_segment < 8 || !std::isfinite(params.A + params.B + params.C + params.D))
            return -1.0;
        const double t_max = params.thrust_limit > 0.0 ? params.thrust_limit : 1.0;

        for (int k = 0; k < segments; k++)
        {
            const double t_lo = t_max * k / segments;
            const double t_hi = t_max * (k + 1) / segments;
            if (!fitCubicSegment(params, t_lo, t_hi, samples_per_segment, params.inv_coef[k]))
                return -1.0;
        }

        // 稠密网格残差校验 (每段 101 点, 非仅采样点)
        double max_res = 0.0;
        for (int k = 0; k < segments; k++)
        {
            const double t_lo = t_max * k / segments;
            const double t_hi = t_max * (k + 1) / segments;
            const double *c = params.inv_coef[k];
            for (int i = 0; i <= 100; i++)
            {
                const double t = t_lo + (t_hi - t_lo) * i / 100;
                const double x = newtonInvert(params, t);
                const double fit = ((c[3] * t + c[2]) * t + c[1]) * t + c[0];
                max_res = std::max(max_res, std::abs(x - fit));
            }
        }

        params.inv_t_max = t_max;
        params.inv_segments = segments;
        params.inv_seg_scale = static_cast<double>(segments) / t_max;
        params.inv_fit_valid = true;
        return max_res * params.n_motors;
    }

    inline double thrustToForce(const MotorParams &motor_params, double thrust, double kp, double battery_voltage = -1.0)
    {
        if (!std::isfinite(kp) || kp <= 0.0)
            throw std::invalid_argument("Air density ratio must be finite and positive");
        auto throttle = inverseSpin(motor_params.spin_k, thrust);

        // vbat 补偿: 系数为标准电压口径, 多项式反解后换算回当前电压的实际力;
        // 未启用时退回线性补偿 (油门侧)。
        battery_voltage = compensationVoltage(motor_params, battery_voltage);
        const bool vbat = battery_voltage > 0 && motor_params.vbat_a != 0.0;
        if (battery_voltage > 0 && !vbat)
            throttle *= battery_voltage / motor_params.volt_max;    // legacy linear voltage compensation


        // 反演: 域内用预拟合分段三次曲线 (常数时间), 域外 (负推力 / 高原 kp 缩放
        // 越界 / 未初始化拟合) 回退牛顿迭代, 数值与旧实现一致
        double x = 0.0;
        if (motor_params.inv_fit_valid && throttle >= 0.0 && throttle <= motor_params.inv_t_max)
        {
            int k = static_cast<int>(throttle * motor_params.inv_seg_scale);
            if (k >= motor_params.inv_segments)
                k = motor_params.inv_segments - 1;
            const double *c = motor_params.inv_coef[k]; // [d, c, b, a]
            x = ((c[3] * throttle + c[2]) * throttle + c[1]) * throttle + c[0];
        }
        else
            x = newtonInvert(motor_params, throttle);
        if (motor_params.x_peak > 0.0 && x > motor_params.x_peak)
            x = motor_params.x_peak;    // 与 forceToThrust 的单调包络对称

        if (vbat)
            x /= vbatScale(motor_params, battery_voltage);    // 标准口径力 -> 当前电压实际力
        double force = motor_params.n_motors * x * kp;
        return force < 0.0 ? 0.0 : force;
    }

    inline double forceToThrust(const MotorParams &motor_params, double force, double kp, double battery_voltage = -1.0)
    {
        if (!std::isfinite(kp) || kp <= 0.0)
            throw std::invalid_argument("Air density ratio must be finite and positive");
        // Density scales physical force, not throttle: convert before the nonlinear curve.
        force /= motor_params.n_motors * kp;
        // 拉力不可为负: 负/非有限请求按零推力处理, 不在标定域外 (x<0) 求三次多项式。
        if (!(force > 0.0))
            force = 0.0;

        // vbat 补偿: 请求力换算到标准电压口径后进多项式 (入口缩放, 与标定端对称);
        // 未启用时退回线性补偿 (油门出口侧)。
        battery_voltage = compensationVoltage(motor_params, battery_voltage);
        const bool vbat = battery_voltage > 0 && motor_params.vbat_a != 0.0;
        if (vbat)
            force *= vbatScale(motor_params, battery_voltage);
        // 单调包络: 越过多项式极大值的请求饱和在极大值, 不让油门随推力需求回落。
        if (motor_params.x_peak > 0.0 && force > motor_params.x_peak)
            force = motor_params.x_peak;
        auto force_2 = force * force;
        auto force_3 = force_2 * force;

        auto throttle = motor_params.A * force_3 + motor_params.B * force_2 + motor_params.C * force + motor_params.D;
        if (battery_voltage > 0 && !vbat)
            throttle *= motor_params.volt_max / battery_voltage;    // legacy linear voltage compensation
        auto thrust = (1.0 - motor_params.spin_k) * throttle + motor_params.spin_k * throttle * throttle; // linear compensation
        if (motor_params.thrust_limit > 0.0 && thrust > motor_params.thrust_limit)
            thrust = motor_params.thrust_limit;
        return thrust < 0.0 ? 0.0 : thrust;
    }

    /// 启动时参数校验 (fpv_test.md §3.2 物理合法性与单调性检验), 通过返回空串并写回
    /// x_peak; 失败返回原因, 调用方应拒绝以该模型飞行。
    /// 要求: 系数有限、C = P'(0) > 0 (零推力处油门随推力上升), 且 P 在达到油门域上界
    /// inverseSpin(thrust_limit) 之前单调递增; 越过极大值的部分由 x_peak 截断。
    /// voltage_compensation: 运行时是否传入电池电压 (此时校验 volt_max 与 vbat 曲线)。
    inline std::string validateMotorParams(MotorParams &params, bool voltage_compensation)
    {
        params.x_peak = 0.0;
        if (!std::isfinite(params.A) || !std::isfinite(params.B) ||
            !std::isfinite(params.C) || !std::isfinite(params.D))
            return "A..D must be finite";
        if (params.n_motors < 1)
            return "n must be >= 1";
        if (!std::isfinite(params.thrust_limit) || params.thrust_limit <= 0.0 || params.thrust_limit > 1.0)
            return "thrust_limit must be in (0, 1]";
        if (!std::isfinite(params.spin_k) || params.spin_k < 0.0 || params.spin_k > 1.0)
            return "spin_k must be in [0, 1]";
        if (!(params.C > 0.0))
            return "C must be positive (throttle must rise with force at zero thrust)";

        // P'(x) = 3A·x² + 2B·x + C 的最小正根即 P 在 x>0 的首个极大值 (P'(0) = C > 0)。
        // 判别式 <= 0 时 P' 不变号, P 全域单调。
        double peak = 0.0;
        if (params.A != 0.0)
        {
            const double disc = 4.0 * params.B * params.B - 12.0 * params.A * params.C;
            if (disc > 0.0)
            {
                const double s = std::sqrt(disc);
                const double r1 = (-2.0 * params.B - s) / (6.0 * params.A);
                const double r2 = (-2.0 * params.B + s) / (6.0 * params.A);
                for (double r : {r1, r2})
                    if (r > 0.0 && (peak == 0.0 || r < peak))
                        peak = r;
            }
        }
        else if (params.B < 0.0)
            peak = -params.C / (2.0 * params.B);
        if (peak > 0.0)
        {
            const double t_top = inverseSpin(params.spin_k, params.thrust_limit);
            const double p_peak = ((params.A * peak + params.B) * peak + params.C) * peak + params.D;
            if (p_peak < t_top)
            {
                char buf[160];
                std::snprintf(buf, sizeof(buf),
                              "throttle curve peaks at %.4f (force %.4g) below the required %.4f: "
                              "not monotonic over the throttle domain", p_peak, peak, t_top);
                return buf;
            }
            params.x_peak = peak;
        }

        if (voltage_compensation)
        {
            if (!std::isfinite(params.volt_max) || params.volt_max <= 0.0)
                return "volt_max must be positive for voltage compensation (check lipo_cells)";
            if (params.volt_min > 0.0 && params.volt_min >= params.volt_max)
                return "volt_min must be below volt_max";
            if (params.vbat_a != 0.0)
            {
                if (!std::isfinite(params.vbat_a) || !std::isfinite(params.vbat_b) ||
                    !std::isfinite(params.volt_ref) || params.volt_ref <= 0.0)
                    return "vbat_a/vbat_b/volt_ref must be finite with volt_ref > 0";
                const double v_lo = params.volt_min > 0.0 ? params.volt_min : 0.0;
                for (double v : {v_lo, params.volt_ref, params.volt_max})
                    if (!(params.vbat_a * v + params.vbat_b > 0.0))
                        return "vbat_a*V + vbat_b must stay positive over [volt_min, volt_max]";
            }
        }
        return {};
    }

} // namespace quadratic_thrust_model

#endif // QUADRATIC_THRUST_MODEL_H
