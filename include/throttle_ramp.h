#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace apm_bridge
{

/// 集体油门成形器: EMA 平滑 + 线性斜坡(加/减速)限速。
///
/// 目标油门先经 EMA 去抖(α = 2/(N+1), 与 EMAFilter 相同的窗口参数化,
/// 首样本直通), 再按时间相关的最大步长逼近 —— 加/减速方向分别以
/// ramp_up_s / ramp_down_s(0..1 满量程单向斜坡时间)限速, 保证相邻两次
/// 输出之差不超过 dt/ramp_*_s(如 1.0 s 满量程 @100 Hz 即每 10 ms 最多
/// 递增 1% 油门)。阶跃油门在低转速下定子电流缺乏反电动势抑制, 限速给
/// 电机转子/螺旋桨留出建立转速与反电动势的时间, 抑制大电流冲击与母线
/// 电压跌落; 减速方向限速同时抑制占空比骤降时的再生回馈电流尖峰。
///
/// dt 由调用方按实际帧间隔给出(须上限钳制, 避免停顿后单帧跳变)。
/// 非线程安全: 仅在单一控制推送上下文(pushControl 定时器)使用。
class ThrottleShaper
{
public:
    /// ramp_up_s/ramp_down_s: 0..1 满量程斜坡时间(s), 0 = 该方向不限速;
    /// ema_window: EMA 窗口样本数(< 2 = 不平滑, 直通)。
    void configure(double ramp_up_s, double ramp_down_s, int ema_window)
    {
        ramp_up_s_ = std::max(0.0, ramp_up_s);
        ramp_down_s_ = std::max(0.0, ramp_down_s);
        // 先转 double 再 +1: int 相加在 window = INT_MAX 时有符号溢出 (UB)
        ema_alpha_ = ema_window >= 2 ? 2.0 / (static_cast<double>(ema_window) + 1.0) : 1.0;
    }

    /// 复位: 斜坡基点 = base, EMA 回到未初始化(下一 apply 的目标直通为
    /// EMA 初值)。用于已知电机停转(上锁)或成形旁路(力通道/motor_direct)
    /// 交界处的连续性衔接。
    void reset(double base)
    {
        out_ = std::clamp(base, 0.0, 1.0);
        ema_ = 0.0;
        ema_init_ = false;
    }

    /// 目标油门 -> 成形输出(0..1)。dt 为距上次 apply 的实际间隔(s, >= 0;
    /// dt = 0 冻结输出, 配置为 0 的方向不限速; 非有限目标直接保持上一输出)。
    double apply(double target, double dt)
    {
        if (!std::isfinite(target))
            return out_;
        target = std::clamp(target, 0.0, 1.0);
        if (!ema_init_)
        {
            ema_ = target;
            ema_init_ = true;
        }
        else
            ema_ = ema_alpha_ * target + (1.0 - ema_alpha_) * ema_;
        const double up = ramp_up_s_ > 0.0
            ? std::max(0.0, dt) / ramp_up_s_ : std::numeric_limits<double>::infinity();
        const double down = ramp_down_s_ > 0.0
            ? std::max(0.0, dt) / ramp_down_s_ : std::numeric_limits<double>::infinity();
        out_ += std::clamp(ema_ - out_, -down, up);
        out_ = std::clamp(out_, 0.0, 1.0);
        return out_;
    }

    double output() const { return out_; }

private:
    double ramp_up_s_ = 0.0;   // 0 = 加速不限速
    double ramp_down_s_ = 0.0; // 0 = 减速不限速
    double ema_alpha_ = 1.0;   // 1 = EMA 直通
    double ema_ = 0.0;
    bool ema_init_ = false;
    double out_ = 0.0;
};

} // namespace apm_bridge
