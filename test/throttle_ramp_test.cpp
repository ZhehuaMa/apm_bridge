// ThrottleShaper (EMA + linear ramp) tests for throttle_ramp.h.
#include "throttle_ramp.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
    using apm_bridge::ThrottleShaper;

    constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

    void require(bool condition, const char *message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }
    void require_near(double actual, double expected, double tol, const char *message)
    {
        if (!(std::abs(actual - expected) <= tol))
            throw std::runtime_error(message);
    }

    // 全部关闭: 直通, 目标阶跃即刻到达 (兼容旧行为的退化配置)。
    void disabled_passthrough()
    {
        ThrottleShaper s;
        s.configure(0.0, 0.0, 0);
        require_near(s.apply(0.7, 0.01), 0.7, 1e-12, "disabled: first sample must pass through");
        require_near(s.apply(0.2, 0.01), 0.2, 1e-12, "disabled: step down must pass through");
        require_near(s.apply(1.0, 0.01), 1.0, 1e-12, "disabled: step up must pass through");
    }

    // 加速斜坡: 满量程 1 s @10 ms 步进 = 每步 +1% 油门, 恰好 100 步到顶。
    void ramp_up_limits()
    {
        ThrottleShaper s;
        s.configure(1.0, 0.0, 0);
        s.reset(0.0);
        double out = 0.0;
        for (int i = 1; i <= 150; ++i)
        {
            out = s.apply(1.0, 0.01);
            require_near(out, std::min(1.0, 0.01 * i), 1e-9, "ramp up: per-step increment must be dt/T");
        }
        require(out == 1.0, "ramp up: must converge to the target without overshoot");
    }

    // 减速斜坡: 满量程 0.5 s @10 ms = 每步 -2% 油门 (比加速快一倍的缺省配置)。
    void ramp_down_limits()
    {
        ThrottleShaper s;
        s.configure(0.0, 0.5, 0);
        s.reset(1.0);
        double out = 1.0;
        for (int i = 1; i <= 60; ++i)
        {
            out = s.apply(0.0, 0.01);
            require_near(out, std::max(0.0, 1.0 - 0.02 * i), 1e-9, "ramp down: per-step decrement must be dt/T");
        }
        require(out == 0.0, "ramp down: must reach zero without undershoot");
    }

    // 双向斜坡: 加/减速独立限速, 幅度小于限速步长时精确跟踪。
    void bidirectional()
    {
        ThrottleShaper s;
        s.configure(1.0, 0.25, 0); // @10 ms: up +0.01/步, down -0.04/步
        s.reset(0.0);
        require_near(s.apply(0.005, 0.01), 0.005, 1e-12, "bidirectional: small rise tracked exactly");
        require_near(s.apply(0.8, 0.01), 0.015, 1e-12, "bidirectional: rise capped at dt/T_up");
        require_near(s.apply(0.0, 0.01), 0.0, 1e-12, "bidirectional: small drop tracked exactly");
        for (int i = 0; i < 9; ++i)
            s.apply(0.8, 0.01);
        require_near(s.output(), 0.09, 1e-9, "bidirectional: climbed under the up limit");
        require_near(s.apply(0.0, 0.01), 0.05, 1e-12, "bidirectional: drop capped at dt/T_down");
        require_near(s.apply(0.0, 0.01), 0.01, 1e-12, "bidirectional: continues from held output");
        require_near(s.apply(0.0, 0.01), 0.0, 1e-12, "bidirectional: reaches target without undershoot");
    }

    // EMA: α = 2/(N+1), 首样本直通; 之后按几何递推收敛, 与 EMAFilter 同参数化。
    void ema_smoothing()
    {
        ThrottleShaper s;
        s.configure(0.0, 0.0, 10); // α = 2/11
        const double alpha = 2.0 / 11.0;
        require(s.apply(1.0, 0.01) == 1.0, "ema: uninitialized filter passes the first sample through");
        double expect = 1.0;
        for (int i = 1; i <= 60; ++i)
        {
            const double out = s.apply(0.0, 0.01);
            expect = (1.0 - alpha) * expect;
            require_near(out, expect, 1e-12, "ema: decay must follow the (1-alpha) recursion");
        }
        require(expect < 1e-4, "ema: geometric decay toward the target");

        ThrottleShaper rise;
        rise.configure(0.0, 0.0, 4); // α = 0.4
        require(rise.apply(0.0, 0.01) == 0.0, "ema: zero first sample initializes to zero");
        require_near(rise.apply(0.5, 0.01), 0.2, 1e-12, "ema: first filtered sample = alpha*target");
        require_near(rise.apply(0.5, 0.01), 0.32, 1e-12, "ema: recursion toward a constant target");
    }

    // EMA 先行、斜坡殿后: 用快斜坡 (步长 0.2) 让 EMA (α≈0.18) 成为实际瓶颈,
    // 证明两级都参与; 冷初始化 EMA (否则首样本直通会让常值目标的 EMA 段惰性)。
    void ema_then_ramp_guarantee()
    {
        ThrottleShaper s;
        s.configure(0.05, 0.025, 10); // @10 ms: up 0.2/步, down 0.4/步
        s.reset(0.0);
        require(s.apply(0.0, 0.01) == 0.0, "ema+ramp: cold-init EMA at zero");
        double prev = 0.0;
        bool ema_lagged = false; // 首步输出 = α < 斜坡满步长 -> 步长被 EMA 压低
        for (int i = 0; i < 400; ++i)
        {
            const double out = s.apply(1.0, 0.01);
            if (i == 0)
                ema_lagged = out < 0.2 - 1e-9;
            require(out - prev <= 0.2 + 1e-12, "ema+ramp: output slope must never exceed the up-ramp limit");
            require(out >= prev - 1e-12, "ema+ramp: monotone non-decreasing toward a rising target");
            prev = out;
        }
        require(ema_lagged, "ema+ramp: EMA stage must actually engage (first step below the slew bound)");
        require_near(prev, 1.0, 1e-9, "ema+ramp: must converge to the target");
    }

    // reset: EMA 重新初始化 (首样本直通), 斜坡基点直接落到 reset 值
    // (旁路交界衔接: 输出连续, 不从旧基点回抽)。
    void reset_continuity()
    {
        ThrottleShaper s;
        s.configure(0.0, 0.0, 10);
        s.apply(1.0, 0.01); // ema = 1.0 (若 reset 未清 EMA, 下一采样将被拖向 1.0)
        s.reset(0.8);
        require_near(s.apply(0.9, 0.01), 0.9, 1e-12, "reset: EMA reinitialized (first sample passes through)");
        require_near(s.apply(0.9, 0.01), 0.9, 1e-12, "reset: converged output unchanged");

        ThrottleShaper r;
        r.configure(10.0, 10.0, 10); // 极慢斜坡
        r.reset(0.0);
        r.apply(1.0, 0.01);
        r.reset(0.8);
        require_near(r.apply(0.8, 0.01), 0.8, 1e-12, "reset: ramp base snaps for bypass handoff continuity");
    }

    // 钳制与防御: 目标/基点钳到 [0,1], NaN 目标保持上一输出, dt=0 冻结。
    void clamps_and_guards()
    {
        ThrottleShaper s;
        s.configure(1.0, 0.5, 10);
        s.reset(0.5);
        require(s.output() == 0.5, "clamp: reset base clamped to [0,1]");
        s.reset(3.0);
        require(s.output() == 1.0, "clamp: base above 1 clamps to 1");
        s.reset(-3.0);
        require(s.output() == 0.0, "clamp: base below 0 clamps to 0");
        require(s.apply(kNan, 0.01) == 0.0, "guard: NaN target keeps the previous output");
        require(s.apply(1.0, 0.0) == 0.0, "guard: dt=0 freezes the ramped output");
        ThrottleShaper off;
        off.configure(0.0, 0.0, 0);
        off.reset(0.0);
        off.apply(0.9, 0.01);
        require(off.apply(0.9, 0.0) == 0.9, "guard: dt=0 is a no-op even with ramps off");
    }
} // namespace

int main()
{
    disabled_passthrough();
    ramp_up_limits();
    ramp_down_limits();
    bidirectional();
    ema_smoothing();
    ema_then_ramp_guarantee();
    reset_continuity();
    clamps_and_guards();
    return 0;
}
