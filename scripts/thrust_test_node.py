#!/usr/bin/python3
"""整机拉力测试 (fpv_test.md §2) 与推力-油门建模 (§3) 台架工具 (custom_link_bridge)。

整机通过夹具固定在台秤上 (DT30X 经 scale_reader_node 发布 /current_weight =
[weight_kg, stable_weight_kg(NaN=未稳定)])。工具经 /fpv/control_command_raw 直通油门,
按 §2.2 阶梯测试动作采集稳态数据点 (u_i, V_i, I_i, RPM_1..4,i, F_i, P_i, t_meas,i):

  * 油门 (DShot 占空比 u) 从 0.05 起以 0.05 步进至 1.00 (可调), 共 20 阶;
  * 每阶 2.5 s: 前 0.5 s 稳定等待丢弃, 后 2.0 s 内对所有数据流做同步均值;
  * u >= 0.70 的每阶结束后回落怠速运行 5.0 s 降温/去极化。

再按 §3 归一化与拟合 (scripts/propulsion_model.py):
  1. 环境推力归一化 F0 = F·(101325/P)·((t+273.15)/273.15)
  2. 四电机均方根转速 RPM_rms
  3. 多元线性最小二乘 u·V = (1/K_v*)·RPM_rms + R_eq·I 辨识 R_eq, K_v*
  4. 折算到 V_ref: F_std = F0·((u·V_ref − I_std·R_eq)/(u·V − I·R_eq))², I_std ≈ I·(V_ref/V)²
  5. 无截距三次多项式 F_std(u) = a3u³ + a2u² + a1u (正规方程) + 单调性检验
输出 CSV、带 motor_parameters 段的 *_result.yaml (直接粘贴进 custom_link_parameters.yaml)
与 PNG。

油门域: Betaflight 集体油门 c 与占空比 u 的关系为 u = u_idle + (1 − u_idle)·c, u_idle 即
飞控 motor_idle (默认 550 = 5.5%), 启动时经 /fpv/cli "get motor_idle" 读取, 不作参数;
低于 u_idle 的阶梯实际以 u_idle (c = 0) 运行并按实际值记录, 降温段同样回到 u_idle。

AirSim / SITL (无气压、无电压下降、无转速反馈): 传 --no-env-comp --no-voltage-comp
跳过步骤 1、3、4 (与节点 baro_compensation/voltage_compensation=false 对应);
转速缺失时 RPM_rms 记 NaN, R_eq 辨识跳过。--offboard 由主机请求 OFFBOARD。

安全: 台秤读数低于 --critical-weight (整机抬离) 或扫描中 FC 掉锁立即中止;
Ctrl+C 安全关停 (油门回最低、上锁), 已采数据照常输出。

用法 (需系统 python，conda 环境请先 deactivate):
  ros2 run apm_bridge thrust_test_node --lipo-cells 6                      # 真机/台架 (默认主机接管 OFFBOARD)
  ros2 run apm_bridge thrust_test_node --lipo-cells 6 --no-offboard        # 由飞手持 BOXOFFBOARD
  ros2 run apm_bridge thrust_test_node --no-env-comp --no-voltage-comp --yes   # AirSim (无气压/压降)

控制模式: 默认 MOTOR_DIRECT (0x20 mode_req=1, 纯集体油门, FC PID 差动旁路 —— 台架
大油门振动不经速率环正反馈; 启动时经 /fpv/cli 开 custom_link_motor_direct=ON, 退出
还原)。--rates 回退经典 BODY_RATES; --attitude 用 ATTITUDE 姿态回显。
"""

from __future__ import annotations

import argparse
import csv
import datetime
import math
import os
import select
import signal
import sys
import threading
import time

import numpy as np

import propulsion_model as pm

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions

    from quadrotor_msgs.msg import ControlCommand
    from quadrotor_msgs.srv import CliCommand
    from mavros_msgs.msg import State
    from sensor_msgs.msg import BatteryState, FluidPressure, Imu, Temperature
    from std_msgs.msg import Bool, Float32MultiArray

    _ROS_IMPORT_ERROR = None
except ImportError as exc:  # pragma: no cover - pure-function use (tests) needs no ROS
    _ROS_IMPORT_ERROR = exc
    Node = object

# 上锁后控制流继续保持, 让 arm=0 边沿随 0x20 帧送达 FC (SITL 验证语义)
DISARM_STREAM_TIME = 2.0
MIN_SCALE_RATE_HZ = 10.0  # §2.1 拉力采样率 >= 10 Hz


class bcolors:
    ERROR = '\033[91m'
    WARNING = '\033[93m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    ENDC = '\033[0m'


class FatalError(Exception):
    """不可恢复的错误 (安全关停后退出)。"""


# ======================================================================
# §2.2 阶梯测试动作 (纯函数, 离线可测)
# ======================================================================

def sop_levels(u_start, u_step, u_end):
    """u_start 起以 u_step 递增至 u_end (含), 默认 0.05..1.00 共 20 阶。"""
    if not (0.0 < u_start <= u_end <= 1.0) or not u_step > 0.0:
        raise ValueError('require 0 < u_start <= u_end <= 1 and u_step > 0')
    count = int(math.floor((u_end - u_start) / u_step + 1e-9)) + 1
    levels = [round(u_start + i * u_step, 6) for i in range(count)]
    if levels[-1] < u_end - 1e-9:
        levels.append(u_end)
    return levels


def sop_timeline(levels, step_time, settle_time, cooldown_threshold, cooldown_time, u_idle):
    """展开为 (动作, 占空比, 时长) 序列: hold_settle → hold_sample [→ cooldown]。"""
    if not 0.0 <= settle_time < step_time:
        raise ValueError('require 0 <= settle_time < step_time')
    timeline = []
    for u in levels:
        timeline.append(('settle', u, settle_time))
        timeline.append(('sample', u, step_time - settle_time))
        if u >= cooldown_threshold - 1e-9 and cooldown_time > 0.0:
            timeline.append(('cooldown', u_idle, cooldown_time))
    return timeline


def average_window(window, tare_kg, gravity):
    """一个稳态采样段的同步均值 -> 数据点 dict (缺失量为 NaN)。"""
    def mean(values):
        vals = [v for v in values if math.isfinite(v)]
        return sum(vals) / len(vals) if vals else float('nan')

    rpm = [mean(series) for series in window['rpm']] if window['rpm'] else []
    weight = mean(window['weight'])
    return {'u': window['u'], 'collective': window['collective'],
            'V': mean(window['V']), 'I': mean(window['I']), 'rpm': rpm,
            'F': (tare_kg - weight) * gravity if math.isfinite(weight) else float('nan'),
            'P': mean(window['P']), 't': mean(window['t']),
            'weight_kg': weight, 'n_weight': len(window['weight'])}


def verification_weights(w_kg, gravity, f_max, count=3):
    """§4 在线反解 + 台秤闭环验证的期望总推力 (N)。

    各取静重推力的 25%/50%/75%, 都相对整机/缆线的静重可分辨 (台秤误差
    ~10 g·g ≈ 0.1 N), 且不超出标定域 [0, f_max] (调用方传实测扫描上界的
    F_std, 见 verify_with_model)。静重可分辨性不满足时返回空列表 (验证段安全跳过)。
    """
    w = w_kg * gravity
    levels = [0.25 * i * w for i in range(1, count + 1)]
    min_resolvable = 0.010 * gravity  # 台秤分辨率 ~10 g
    # 小于分辨率的档舍去; 超过标定域上界 f_max 的档舍去 (f_max 不够覆盖 75% 静重时
    # 至少保留一档, 除非连最低档都越界)。
    return [f for f in levels if f >= min_resolvable and f <= f_max]


def verification_verify_pass(verification, max_rel_error=0.02):
    """标定验证判定: 每档 |相对误差| <= max_rel_error (2 %, 与 §3 拟合残差同量级)。"""
    return bool(verification) and all(abs(v['rel']) <= max_rel_error for v in verification)


def new_window(u, collective, n_motors=0):
    return {'u': u, 'collective': collective, 'weight': [], 'V': [], 'I': [],
            'rpm': [[] for _ in range(n_motors)], 'P': [], 't': []}


def cli_command(node, cmd, args, timeout=10.0):
    """经 /fpv/cli 执行一条 CLI 命令 (CMD_GET/CMD_SET); 实现见 pm.cli_command。"""
    return pm.cli_command(node, cmd, args, timeout=timeout)


def render_param_yaml(result, args):
    """生成可直接并入 custom_link_parameters.yaml 的片段。"""
    lines = ['custom_link_bridge:',
             '  ros__parameters:',
             f'    voltage_compensation: {str(not args.no_voltage_comp).lower()}',
             f'    baro_compensation: {str(not args.no_env_comp).lower()}',
             '    motor_parameters:',
             f"      a3: {result['a3']:.9g}",
             f"      a2: {result['a2']:.9g}",
             f"      a1: {result['a1']:.9g}",
             f"      r_eq: {result['r_eq']:.9g}",
             f"      k_i: {result['k_i']:.9g}"]
    return '\n'.join(lines)


# ======================================================================
# 节点
# ======================================================================

class ThrustTestNode(Node):

    def __init__(self, args):
        super().__init__('thrust_test_node')
        self.args = args
        self.lock = threading.RLock()
        self.abort_event = threading.Event()
        self.state_event = threading.Event()
        self.weight_event = threading.Event()

        # ---- 共享状态 (lock 保护) ----
        self.collective = 0.0
        self.testing = False
        self.disarming = False
        self.armed = False
        self.mode = ''
        self.orientation = None
        self.last_weight = float('nan')
        self.last_stable = float('nan')
        self.last_voltage = float('nan')
        self.last_current = float('nan')
        self.last_pressure = float('nan')
        self.last_temperature = float('nan')
        self.tare_samples = None          # 去皮期间收集的稳定重量
        self.window = None                # 当前稳态采样段 (None = 不采集)
        self.quiet = False
        self._arm_intent = False
        self._offboard_intent = False

        self.points = []
        self.tare_kg = float('nan')
        self.u_idle = float('nan')         # 飞控 motor_idle 占空比 (main 中经 /fpv/cli 读取)
        self.verification = None           # 标定验证结果列表 (见 verify_with_model)

        qos_be5 = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_be10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_pub = self.create_publisher(ControlCommand, '/fpv/control_command_raw', qos_be5)
        self.arm_pub = self.create_publisher(Bool, '/fpv/arm', qos_be10)
        # OFFBOARD 经通讯协议直接接管: 默认发布 /fpv/offboard; --no-offboard
        # 退回飞手侧 BOXOFFBOARD 开关授权。
        self.offboard_pub = (self.create_publisher(Bool, '/fpv/offboard', qos_be10)
                             if not args.no_offboard else None)

        self.create_subscription(State, '/mavros/state', self.on_state, 10)
        self.create_subscription(BatteryState, '/mavros/battery', self.on_battery, qos_be10)
        self.create_subscription(Float32MultiArray, '/current_weight', self.on_weight, qos_be10)
        self.create_subscription(Imu, '/mavros/imu/data', self.on_imu, qos_be10)
        self.create_subscription(FluidPressure, '/fpv/static_pressure', self.on_pressure, qos_be10)
        self.create_subscription(Temperature, '/fpv/temperature_baro', self.on_temperature, qos_be10)
        self.create_subscription(Float32MultiArray, '/fpv/motor_rpm', self.on_motor_rpm, qos_be10)

        self.create_timer(1.0 / args.control_rate, self.control_timer_cb)
        self.create_timer(0.5, self.arm_keepalive_cb)
        self.create_timer(1.0, self.status_timer_cb)

    # ---------------------------------------------------------- 订阅回调

    def _append(self, key, value):
        with self.lock:
            if self.window is not None:
                self.window[key].append(value)

    def on_state(self, msg):
        with self.lock:
            was_testing, was_armed, disarming = self.testing, self.armed, self.disarming
            self.armed = msg.armed
            self.mode = msg.mode
        self.state_event.set()
        if was_testing and was_armed and not msg.armed and not disarming:
            print(f'\n{bcolors.ERROR}*** FC DISARMED during test! Aborting. ***{bcolors.ENDC}')
            self.abort_event.set()

    def on_battery(self, msg):
        voltage = float('nan')
        if len(msg.cell_voltage) > 0 and msg.cell_voltage[0] > 0.0:
            voltage = float(msg.cell_voltage[0])  # 本项目约定: cell_voltage[0] = 整包
        elif math.isfinite(msg.voltage) and msg.voltage > 0.0:
            voltage = float(msg.voltage)
        # ROS 约定放电电流为负; §2.1 的 I 取幅值。NaN (无电流计) 保持 NaN。
        current = abs(float(msg.current)) if math.isfinite(msg.current) else float('nan')
        with self.lock:
            self.last_voltage = voltage
            self.last_current = current
            if self.window is not None:
                self.window['V'].append(voltage)
                self.window['I'].append(current)

    def on_weight(self, msg):
        if len(msg.data) < 2:
            return
        weight, stable = float(msg.data[0]), float(msg.data[1])
        with self.lock:
            self.last_weight = weight
            self.last_stable = stable
            if self.tare_samples is not None and math.isfinite(stable):
                self.tare_samples.append(stable)
            if self.window is not None:
                self.window['weight'].append(weight)
            testing = self.testing
        self.weight_event.set()
        if testing and weight < self.args.critical_weight:
            print(f'\n{bcolors.ERROR}*** scale weight {weight:.3f} kg below --critical-weight: '
                  f'airframe lifting off the scale, aborting ***{bcolors.ENDC}')
            self.abort_event.set()

    def on_imu(self, msg):
        with self.lock:
            self.orientation = msg.orientation

    def on_pressure(self, msg):
        if msg.fluid_pressure > 0.0:
            with self.lock:
                self.last_pressure = float(msg.fluid_pressure)
            self._append('P', float(msg.fluid_pressure))

    def on_temperature(self, msg):
        with self.lock:
            self.last_temperature = float(msg.temperature)
        self._append('t', float(msg.temperature))

    def on_motor_rpm(self, msg):
        with self.lock:
            if self.window is None:
                return
            series = self.window['rpm']
            while len(series) < len(msg.data):
                series.append([])
            for k, value in enumerate(msg.data):
                series[k].append(float(value))  # NaN = 双向 DShot 无效位, 均值时剔除

    # ---------------------------------------------------------- 定时器

    def control_timer_cb(self):
        """默认 MOTOR_DIRECT (mode_req=1): 纯集体油门, FC 侧 PID 差动旁路 —— 台架
        大油门振动不再经速率环正反馈失控。

        --attitude 改用 ATTITUDE + 姿态回显; --rates 回退经典 BODY_RATES 零角速度
        (旧固件无 custom_link_motor_direct 时的降级路径)。"""
        cmd = ControlCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        if getattr(self.args, 'motor_direct', False):
            cmd.control_mode = ControlCommand.MOTOR_DIRECT
        elif self.args.attitude:
            cmd.control_mode = ControlCommand.ATTITUDE
        else:
            cmd.control_mode = ControlCommand.BODY_RATES
        with self.lock:
            cmd.armed = self.testing
            cmd.collective_thrust = min(max(self.collective, 0.0), 1.0)
            orientation = self.orientation
        if orientation is not None:
            cmd.orientation = orientation
        else:
            cmd.orientation.w = 1.0
        self.cmd_pub.publish(cmd)

    def arm_keepalive_cb(self):
        with self.lock:
            armed_intent = self.testing or self._arm_intent
            offboard_intent = self._offboard_intent
        self.arm_pub.publish(Bool(data=armed_intent))
        if self.offboard_pub is not None:
            self.offboard_pub.publish(Bool(data=offboard_intent))

    def status_timer_cb(self):
        if self.quiet:
            return
        with self.lock:
            collective, weight, voltage = self.collective, self.last_weight, self.last_voltage
            mode, armed = self.mode, self.armed
        duty = pm.collective_to_duty(collective, self.u_idle)
        print(f'\r[u={duty:5.3f} c={collective:5.3f}] [w={weight:.3f} kg] [V={voltage:.2f}] '
              f'[mode={mode or "--"}{" armed" if armed else ""}] [pts={len(self.points)}]   ',
              end='', flush=True)

    # ---------------------------------------------------------- 动作

    def set_duty(self, duty):
        with self.lock:
            self.collective = pm.duty_to_collective(duty, self.u_idle)
            return self.collective

    def sleep_abortable(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self.abort_event.is_set():
                return False
            time.sleep(0.02)
        return not self.abort_event.is_set()

    def measure_tare(self, seconds):
        """起桨前稳定重量均值 (§2.1: 拉力 = 相对起桨前减小的重量)。"""
        with self.lock:
            self.tare_samples = []
        self.sleep_abortable(seconds)
        with self.lock:
            samples, self.tare_samples = self.tare_samples, None
            fallback = self.last_weight
        if samples:
            self.tare_kg = sum(samples) / len(samples)
        elif math.isfinite(fallback):
            self.tare_kg = fallback
        else:
            raise FatalError('no /current_weight data -- is scale_reader_node running?')
        print(f'{bcolors.OKBLUE}tare = {self.tare_kg:.4f} kg ({len(samples)} stable samples)'
              f'{bcolors.ENDC}')

    def run_sop(self, timeline):
        for action, duty, duration in timeline:
            if self.abort_event.is_set():
                return
            collective = self.set_duty(duty)
            if action == 'sample':
                with self.lock:
                    self.window = new_window(pm.collective_to_duty(collective, self.u_idle),
                                             collective)
                ok = self.sleep_abortable(duration)
                with self.lock:
                    window, self.window = self.window, None
                if not ok:
                    return
                self.record(window, duration)
            else:
                if action == 'cooldown':
                    print(f'cooldown at idle for {duration:.1f} s')
                if not self.sleep_abortable(duration):
                    return

    def live_inputs(self):
        """验证段用的最新温度/气压/电压/电流 (NaN = 无/已用 --no-* 关闭)。"""
        with self.lock:
            env = 1.0
            if not self.args.no_env_comp and math.isfinite(self.last_pressure) and \
                    math.isfinite(self.last_temperature):
                env = pm.environment_factor(self.last_pressure, self.last_temperature)
            voltage = self.last_voltage if not self.args.no_voltage_comp else float('nan')
            current = self.last_current if not self.args.no_voltage_comp else float('nan')
        return env, voltage, current

    def verify_with_model(self, model, duration):
        """§4 在线反解 + 台秤闭环验证: 当前温度/气压/电压(和电流)下请求几档期望推力,
        量实际拉力并与期望对比。返回结果列表, 中途被中止返回 None。

        为使"实测拉力 F_meas = 台秤减静重"恰与期望总推力同口径, 机体须保持静平衡
        (飞控只受零角速度, 不机动): 使用与标定段一致的控制通道。电缆拉力台秤误差
        的量级即静重推力分辨率的约束 (见 verification_weights)。"""
        # §4 反解在标定域内; 静重台秤下的期望推力须满足 |F_meas| >= 10 g·g (分辨)
        # 且不超过实测扫描上界的模型推力 (截短扫描时不能拿外推的 F_std(1) 验证)。
        u_max = max((p['u'] for p in self.points), default=1.0)
        f_max = model.thrust(min(u_max, 1.0))
        weights = verification_weights(self.tare_kg, self.args.gravity, f_max)
        if not weights:
            print(f'{bcolors.WARNING}verification skipped: static weight {self.tare_kg:.3f} kg '
                  f'does not resolve 10 g scale steps{bcolors.ENDC}')
            return []
        results = []
        for f_req in weights:
            # 与标定段同一 SOP 时序: 前 --settle-time 丢弃 (气流/压降瞬态)。在此等待段
            # 内重采样低通后的电压/电流并迭代油门 (§4 步骤 3 的反馈), 让电机动态
            # 与母线压降在进入采样段前收敛, 而不是用采前瞬间的读数。
            env, voltage, current = self.live_inputs()
            u = model.throttle(f_req, env, voltage, current)
            collective = self.set_duty(u)
            settle_end = time.monotonic() + self.args.settle_time
            while not self.abort_event.is_set() and time.monotonic() < settle_end:
                env, voltage, current = self.live_inputs()
                u = model.throttle(f_req, env, voltage, current)
                collective = self.set_duty(u)
                time.sleep(0.02)
            if self.abort_event.is_set():
                return None  # aborted
            with self.lock:
                self.window = new_window(u, collective)
            ok = self.sleep_abortable(duration)
            with self.lock:
                window, self.window = self.window, None
            if not ok:
                return None  # aborted mid-step
            f_meas = average_window(window, self.tare_kg, self.args.gravity)['F']
            if not math.isfinite(f_meas):
                print(f'{bcolors.WARNING}F_req={f_req:.3f} N: no scale data, skipped{bcolors.ENDC}')
                continue
            err = f_meas - f_req
            rel = err / f_req
            results.append({'f_req': f_req, 'f_meas': f_meas, 'u': u, 'env': env,
                            'voltage': voltage, 'current': current, 'error': err, 'rel': rel})
            print(f'F_req={f_req:.3f} N -> u={u:.4f} (c={collective:.4f}): '
                  f'F={f_meas:.3f} N, error {err:+.3f} N ({rel:+.1%})')
        with self.lock:
            self.collective = 0.0
        return results

    def record(self, window, duration):
        point = average_window(window, self.tare_kg, self.args.gravity)
        rate = point['n_weight'] / duration
        if not math.isfinite(point['F']):
            print(f'\n{bcolors.WARNING}u={point["u"]:.3f}: no scale data, point skipped{bcolors.ENDC}')
            return
        if rate < MIN_SCALE_RATE_HZ:
            print(f'\n{bcolors.WARNING}u={point["u"]:.3f}: scale rate {rate:.1f} Hz < '
                  f'{MIN_SCALE_RATE_HZ:.0f} Hz required by fpv_test.md{bcolors.ENDC}')
        self.points.append(point)
        rpm_rms = pm.rpm_rms(point['rpm'])
        print(f'\nu={point["u"]:.3f} (c={point["collective"]:.3f}): F={point["F"]:.3f} N, '
              f'V={point["V"]:.2f} V, I={point["I"]:.2f} A, RPMrms={rpm_rms:.0f}, '
              f'P={point["P"]:.0f} Pa, t={point["t"]:.1f} C')

    def start_testing(self):
        with self.lock:
            self.testing = True
            self._arm_intent = True
            self._offboard_intent = self.offboard_pub is not None
            self.collective = 0.0
        try:
            if self.offboard_pub is not None:
                self.offboard_pub.publish(Bool(data=True))
            self.arm_pub.publish(Bool(data=True))
            if not self._wait_armed(True, 15.0):
                hint = ('check that the pilot holds BOXOFFBOARD and the FC arming checks pass'
                        if self.args.no_offboard
                        else 'check the link (host OFFBOARD/arm request not accepted); '
                             'alternatively pass --no-offboard and hold BOXOFFBOARD')
                raise FatalError(f'FC did not ARM ({hint})')
        except Exception:
            with self.lock:
                self.testing = False
                self._offboard_intent = False
            raise
        print(f'{bcolors.OKGREEN}START TESTING...{bcolors.ENDC}')

    def shutdown_sequence(self):
        """油门回最低 + 上锁并等待确认 (executor 仍在 spin)。"""
        self.quiet = True
        with self.lock:
            self.collective = 0.0
            self.window = None
            self.disarming = True
            self._arm_intent = False
        try:
            self.arm_pub.publish(Bool(data=False))
            self._wait_armed(False, max(DISARM_STREAM_TIME, 5.0))
        finally:
            with self.lock:
                self.testing = False
                self.disarming = False
                self._offboard_intent = False
            if self.offboard_pub is not None:
                self.offboard_pub.publish(Bool(data=False))
            self.quiet = False

    def _wait_armed(self, target, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.abort_event.is_set() and target:
                return False
            with self.lock:
                if self.armed == target:
                    return True
            time.sleep(0.1)
        print(f'{bcolors.ERROR}timeout waiting for armed={target}{bcolors.ENDC}')
        return False


# ======================================================================
# 结果输出
# ======================================================================

def write_outputs(node, args, aborted):
    """CSV + _result.yaml + PNG。node 提供 points/tare/verification。"""
    points = node.points
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    os.makedirs(args.output_dir, exist_ok=True)
    base = os.path.join(args.output_dir, f'thrust_curve_{stamp}')
    n = 1
    while any(os.path.exists(base + suffix) for suffix in ('.csv', '_result.yaml', '.png')):
        n += 1
        base = os.path.join(args.output_dir, f'thrust_curve_{stamp}_{n}')
    v_ref = args.lipo_cells * pm.V_REF_PER_CELL

    result, error = None, None
    try:
        result = pm.calibrate(points, v_ref, voltage_normalization=not args.no_voltage_comp,
                              environment_normalization=not args.no_env_comp, u_idle=args.u_idle)
    except (ValueError, np.linalg.LinAlgError) as e:
        error = str(e)
    rows = result['rows'] if result else [{**p, 'F0': float('nan'), 'F_std': float('nan'),
                                            'I_std': float('nan'), 'rpm_rms': pm.rpm_rms(p['rpm'])}
                                           for p in points]
    n_rpm = max((len(p['rpm']) for p in points), default=0)

    csv_path = base + '.csv'
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['u', 'collective', 'V', 'I'] + [f'rpm{k + 1}' for k in range(n_rpm)] +
                        ['rpm_rms', 'F_N', 'P_Pa', 't_C', 'F0_N', 'F_std_N', 'I_std_A', 'n_weight'])
        for r in rows:
            rpm = list(r['rpm']) + [float('nan')] * (n_rpm - len(r['rpm']))
            writer.writerow([f"{r['u']:.6f}", f"{r['collective']:.6f}", f"{r['V']:.4f}", f"{r['I']:.4f}"] +
                            [f'{v:.2f}' for v in rpm] +
                            [f"{r['rpm_rms']:.2f}", f"{r['F']:.4f}", f"{r['P']:.1f}", f"{r['t']:.2f}",
                             f"{r['F0']:.4f}", f"{r['F_std']:.4f}", f"{r['I_std']:.4f}", r['n_weight']])
    print(f'CSV written to {csv_path}')

    yaml_path = base + '_result.yaml'
    with open(yaml_path, 'w') as f:
        f.write('# fpv_test.md thrust test result\n')
        f.write(f'date: {datetime.datetime.now().isoformat(timespec="seconds")}\n')
        f.write(f'aborted: {str(aborted).lower()}\n')
        f.write(f'points: {len(points)}\n')
        f.write(f'v_ref: {v_ref:.2f}\n')
        f.write(f'u_idle_fc_motor_idle: {args.u_idle:.4f}  # read from the FC at test time, not a parameter\n')
        f.write(f'environment_normalization: {str(not args.no_env_comp).lower()}\n')
        f.write(f'voltage_normalization: {str(not args.no_voltage_comp).lower()}\n')
        if result:
            st = result['stats']
            f.write(f"fit_rms_N: {st['rms']:.6g}\n")
            f.write(f"fit_max_error_N: {st['max_error']:.6g}\n")
            f.write(f"fit_r_squared: {st['r_squared']:.8f}\n")
            f.write(f"monotonic: {str(result['monotonic']).lower()}\n")
            f.write(f"min_slope_N: {result['min_slope']:.6g}\n")
            f.write(f"kv_rpm_per_volt: {result['kv']:.6g}\n")
            f.write(f"ke_volt_per_rpm: {result['ke']:.6g}\n")
            f.write(f"r_eq_ohm: {result['r_eq']:.6g}\n")
            f.write(f"k_i_A_per_N: {result['k_i']:.6g}\n")
            for w in result['warnings']:
                f.write(f'# warning: {w}\n')
        verification = node.verification
        if verification:
            f.write('verification:  # fpv_test.md §4 runtime check on the bench scale\n')
            for v in verification:
                f.write(f"  - {{f_req_N: {v['f_req']:.4f}, f_meas_N: {v['f_meas']:.4f}, "
                        f"error_N: {v['error']:+.4f}, error_pct: {100 * v['rel']:+.2f}, "
                        f"u: {v['u']:.6f}, env: {v['env']:.5f}, voltage_V: {v['voltage']:.3f}, "
                        f"current_A: {v['current']:.3f}}}\n")
            worst = max(abs(v['rel']) for v in verification)
            f.write(f'verification_worst_rel_error: {worst:.4f}\n')
            passed = verification_verify_pass(verification)
            f.write(f'verification_pass: {str(passed).lower()}  # |rel| <= 2% at every level\n')
            print(f"{bcolors.OKGREEN if passed else bcolors.WARNING}verification on the scale: "
                  f"{'PASS' if passed else 'FAIL'} (worst |rel error| {worst:.1%}){bcolors.ENDC}")
        if result and result['monotonic']:
            f.write('\n# ---- paste into parameters/custom_link_parameters.yaml ----\n')
            f.write(render_param_yaml(result, args) + '\n')
        elif result:
            f.write(f'# NOT MONOTONIC on [u_idle, 1]: model rejected (spec 3.2, 2026-11-10 '
                    f'revision), repeat the test\n')
        else:
            f.write(f'# fit failed: {error}\n')
    print(f'result YAML written to {yaml_path}')

    if result:
        print(f"{bcolors.OKGREEN}F_std(u) = {result['a3']:.6g} u^3 + {result['a2']:.6g} u^2 + "
              f"{result['a1']:.6g} u   (RMS {result['stats']['rms']:.4g} N, "
              f"R^2 {result['stats']['r_squared']:.6f}){bcolors.ENDC}")
        print(f"R_eq = {result['r_eq']:.5g} ohm, K_v* = {result['kv']:.5g} rpm/V, k_i = {result['k_i']:.5g} A/N")
        for w in result['warnings']:
            print(f'{bcolors.WARNING}warning: {w}{bcolors.ENDC}')
        if not result['monotonic']:
            print(f"{bcolors.ERROR}fitted curve rejected: {result.get('monotonic_reason') or 'gate failed'}"
                  f"{bcolors.ENDC}")
            print(f'{bcolors.WARNING}note: either the negative-slope region reaches into the '
                  f'commandable domain [u_idle, 1], or F_std at the idle duty is non-positive '
                  f'(typically a thrust-onset dead band or bad low-throttle data); '
                  f'the PNG is stamped REJECTED accordingly{bcolors.ENDC}')
    else:
        print(f'{bcolors.ERROR}fit failed: {error}{bcolors.ENDC}')

    if not args.no_plot and result:
        plot_results(result, base + '.png')
    return csv_path, yaml_path, result


def plot_results(result, png_path):
    """标定结果图: 实测 F (蓝 x) / 归一化 F_std (绿 o) + 拟合曲线。

    拟合曲线在实测标定域 [0, u_max] 上实线, u_max..1 段虚线 (纯外推) 并在 u_max
    处画竖线, 避免把外推段误读为标定结果; 非单调 (规格 §3.2 拒绝) 时标题盖章
    REJECTED 与最小斜率。返回 matplotlib figure (None = 无 matplotlib), 便于测试。
    """
    try:
        import matplotlib
        if not os.environ.get('DISPLAY'):
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f'{bcolors.WARNING}matplotlib unavailable ({e}), skipping plot{bcolors.ENDC}')
        return None
    try:
        rows = result['rows']
        # 与 calibrate 的 used 过滤同口径: 拟合实际约束的数据域上界
        used_u = [r['u'] for r in rows if r['u'] > 0.0 and math.isfinite(r['F_std'])]
        u_max = max(used_u) if used_u else 1.0

        def curve(u):
            return result['a3'] * u ** 3 + result['a2'] * u ** 2 + result['a1'] * u

        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot([r['u'] for r in rows], [r['F'] for r in rows], 'bx', label='measured F')
        ax.plot([r['u'] for r in rows], [r['F_std'] for r in rows], 'go', label='normalized F_std')
        u_fit = np.linspace(0.0, u_max, 200)
        ax.plot(u_fit, curve(u_fit), '-r', label='F_std(u) fit')
        title = f'zero-intercept cubic fit, calibrated u <= {u_max:.2f}'
        if u_max < 1.0 - 1e-9:
            u_ext = np.linspace(u_max, 1.0, 100)
            ax.plot(u_ext, curve(u_ext), '--r', alpha=0.55, label='extrapolated fit')
            ax.axvline(u_max, color='0.5', linestyle=':', linewidth=1.0)
            title += f'\nbeyond {u_max:.2f} dashed: extrapolated F(1)={curve(1.0):.1f} N'
        if not result['monotonic']:
            title += f'\nREJECTED (spec 3.2): {result.get("monotonic_reason") or "gate failed"}'
            ax.set_title(title, color='tab:red', fontsize=9.5)
        else:
            ax.set_title(title, fontsize=9.5)
        ax.set_xlabel('throttle duty u')
        ax.set_ylabel('total thrust [N]')
        ax.legend()
        ax.grid(True)
        fig.savefig(png_path)
        print(f'plot written to {png_path}')
        return fig
    except Exception as e:
        print(f'{bcolors.WARNING}plotting failed ({e}){bcolors.ENDC}')
        return None


# ======================================================================
# main
# ======================================================================

def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='fpv_test.md bench thrust test + propulsion model fit (custom_link_bridge)')
    parser.add_argument('--no-offboard', action='store_true',
                        help='do not publish /fpv/offboard; OFFBOARD stays with the pilot '
                             "holding BOXOFFBOARD (the default takes OFFBOARD via the link)")
    parser.add_argument('--u-start', type=float, default=0.05, help='first throttle duty (§2.2)')
    parser.add_argument('--u-step', type=float, default=0.05, help='throttle duty step (§2.2)')
    parser.add_argument('--u-end', type=float, default=1.00, help='last throttle duty (§2.2, adjustable); '
                         'a value below 1.0 leaves the model extrapolated above it -- the fit and every '
                         'downstream consumer still run to u=1')
    parser.add_argument('--step-time', type=float, default=2.5, help='duration of each step [s]')
    parser.add_argument('--settle-time', type=float, default=0.5,
                        help='discarded settling part of each step [s]; the rest is averaged')
    parser.add_argument('--cooldown-threshold', type=float, default=0.70,
                        help='steps at or above this duty are followed by an idle cooldown')
    parser.add_argument('--cooldown-time', type=float, default=5.0, help='idle cooldown [s]')
    parser.add_argument('--lipo-cells', type=int, default=6, help='V_ref = cells*3.7 V')
    parser.add_argument('--gravity', type=float, default=9.80665)
    parser.add_argument('--tare-time', type=float, default=2.0,
                        help='scale averaging before arming for the zero-thrust reference [s]')
    parser.add_argument('--critical-weight', type=float, default=1.0,
                        help='abort if the scale reads below this [kg] (lift-off protection)')
    parser.add_argument('--no-env-comp', action='store_true',
                        help='skip §3.1 step 1 (no baro, e.g. AirSim; pairs with baro_compensation=false)')
    parser.add_argument('--no-voltage-comp', action='store_true',
                        help='skip §3.1 steps 3-4 (no voltage sag, e.g. AirSim; pairs with '
                             'voltage_compensation=false)')
    parser.add_argument('--attitude', action='store_true',
                        help='use ATTITUDE + orientation echo instead of the default motor-direct '
                             'mode; verification holds thrust constant while tilted')
    parser.add_argument('--rates', action='store_true',
                        help='use the classic BODY_RATES (zero rates) control instead of the '
                             'default motor-direct mode (PID active - prone to vibration '
                             'positive feedback on the bench at high throttle)')
    parser.add_argument('--no-verify', action='store_true',
                        help='skip the post-fit §4 verification on the scale')
    parser.add_argument('--verify-sample-time', type=float, default=3.0,
                        help='averaging window per verification thrust level [s]; '
                             'the --settle-time wait runs before it, on top')
    parser.add_argument('--control-rate', type=float, default=100.0, help='command stream rate [Hz]')
    parser.add_argument('--yes', action='store_true', help='skip the Enter confirmation')
    parser.add_argument('--output-dir', type=str, default='.')
    parser.add_argument('--no-plot', action='store_true')
    args = parser.parse_args(argv)
    try:
        args.levels = sop_levels(args.u_start, args.u_step, args.u_end)
        # 校验时序参数; 实际时序在读到飞控 u_idle 后生成 (降温段回到 u_idle)
        sop_timeline(args.levels, args.step_time, args.settle_time,
                     args.cooldown_threshold, args.cooldown_time, 0.0)
    except ValueError as e:
        parser.error(str(e))
    return args


def confirm_start(args, abort_event):
    if args.yes or not sys.stdin.isatty():
        return True
    print(f'{bcolors.OKGREEN}Airframe fixed on the scale? Press Enter to START '
          f'({len(args.levels)} steps, u {args.levels[0]:.2f}..{args.levels[-1]:.2f}; '
          f'Ctrl+C to abort)...{bcolors.ENDC}', flush=True)
    while not abort_event.is_set():
        r, _, _ = select.select([sys.stdin], [], [], 0.2)
        if r:
            sys.stdin.readline()
            return True
    return False


def install_signal_handlers(abort_event):
    state = {'first': True}

    def handler(signum, frame):
        if state['first']:
            state['first'] = False
            print(f'\n{bcolors.WARNING}abort requested (signal {signum}), shutting down safely... '
                  f'(again = force exit){bcolors.ENDC}')
            abort_event.set()
        else:
            os._exit(1)

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)


def main():
    args = parse_args()
    if _ROS_IMPORT_ERROR is not None:
        sys.exit(f'ROS 2 not importable ({_ROS_IMPORT_ERROR}); source the workspace and use system python3')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ThrustTestNode(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin_guarded():
        try:
            executor.spin()
        except rclpy.executors.ExternalShutdownException:
            pass
        except Exception:
            import traceback
            traceback.print_exc()
            node.abort_event.set()

    spin_thread = threading.Thread(target=spin_guarded, daemon=True)
    spin_thread.start()
    install_signal_handlers(node.abort_event)

    exit_code, aborted, started = 0, False, False
    try:
        if not node.state_event.wait(timeout=15.0):
            raise FatalError('no /mavros/state within 15 s -- is custom_link_bridge_node running?')
        # u_idle = 飞控 motor_idle: 集体油门 0 的占空比, 也是降温段的怠速 (§2.2/§4 步骤 4)
        try:
            node.u_idle = pm.fetch_u_idle(node, timeout=10.0)
        except RuntimeError as e:
            raise FatalError(f'cannot read FC motor_idle via /fpv/cli: {e}')
        args.u_idle = node.u_idle
        # 默认走纯油门直通 (mode_req=1): FC 端 custom_link_motor_direct 闸门经
        # /fpv/cli 开启 (RAM-only, 断电即恢复 OFF, 退出时亦还原)。旧固件不认该
        # 设置 -> 告警并降级 BODY_RATES (PID 仍会响应振动)。
        args.motor_direct = not (args.rates or args.attitude)
        if args.motor_direct:
            try:
                cli_command(node, CliCommand.Request.CMD_SET, 'custom_link_motor_direct = ON')
                print(f'{bcolors.OKGREEN}motor-direct mode: FC gate enabled (custom_link_motor_direct=ON, '
                      f'reverted on exit/power-cycle){bcolors.ENDC}')
            except RuntimeError as e:
                args.motor_direct = False
                print(f'{bcolors.WARNING}motor-direct unavailable ({e}); falling back to BODY_RATES - '
                      f'the rate PID will react to bench vibration at high throttle{bcolors.ENDC}')
        # 低于 FC u_idle 的阶实际以 c=0 (怠速) 运行并被记录在 u_idle -- 塌缩成
        # 单个怠速数据点, 避免同 u 重复行 (motor_idle 调大时整段阶都会塌缩)。
        collapsed = sorted({round(max(u, node.u_idle), 6) for u in args.levels})
        if len(collapsed) < len(args.levels):
            print(f'{bcolors.WARNING}{len(args.levels) - len(collapsed)} SOP level(s) below FC '
                  f'u_idle {node.u_idle:.4f} collapse to a single idle step{bcolors.ENDC}')
            args.levels = collapsed
        args.timeline = sop_timeline(args.levels, args.step_time, args.settle_time,
                                     args.cooldown_threshold, args.cooldown_time, node.u_idle)
        total = sum(d for _, _, d in args.timeline)
        print(f'{bcolors.OKBLUE}FC motor_idle -> u_idle = {node.u_idle:.4f}; SOP: {len(args.levels)} steps '
              f'u={args.levels[0]:.2f}..{args.levels[-1]:.2f}, {args.step_time:.1f} s/step '
              f'({args.settle_time:.1f} s settle + {args.step_time - args.settle_time:.1f} s sample), '
              f'cooldown {args.cooldown_time:.1f} s after u>={args.cooldown_threshold:.2f}; '
              f'total {total:.0f} s{bcolors.ENDC}')
        if not node.weight_event.wait(timeout=5.0):
            raise FatalError('no /current_weight data -- is scale_reader_node running?')
        if not confirm_start(args, node.abort_event):
            aborted = True
        else:
            node.measure_tare(args.tare_time)
            started = True
            node.start_testing()
            node.run_sop(args.timeline)
            if node.points and not args.no_verify and not node.abort_event.is_set():
                v_ref = args.lipo_cells * pm.V_REF_PER_CELL
                result = pm.calibrate(node.points, v_ref,
                                      voltage_normalization=not args.no_voltage_comp,
                                      environment_normalization=not args.no_env_comp,
                                      u_idle=args.u_idle)
                if result['monotonic']:
                    model = pm.PropulsionModel(result['a3'], result['a2'], result['a1'],
                                               v_ref=v_ref,
                                               v_min=args.lipo_cells * pm.V_MIN_PER_CELL,
                                               u_idle=args.u_idle, r_eq=result['r_eq'],
                                               k_i=result['k_i'])
                    print(f'{bcolors.OKBLUE}verifying the calibration on the scale...{bcolors.ENDC}')
                    node.verification = node.verify_with_model(model, args.verify_sample_time)
                    if node.verification is None:
                        aborted = True
                else:
                    print(f'{bcolors.WARNING}verification skipped: fit is not monotonic{bcolors.ENDC}')
    except FatalError as e:
        print(f'{bcolors.ERROR}FATAL: {e}{bcolors.ENDC}')
        exit_code = 1
    finally:
        aborted = aborted or node.abort_event.is_set()
        if started:
            node.shutdown_sequence()
        if getattr(args, 'motor_direct', False):
            try:  # 还原 FC 闸门 (best-effort; 断电/重启亦自动恢复 OFF)
                cli_command(node, CliCommand.Request.CMD_SET, 'custom_link_motor_direct = OFF')
            except RuntimeError as e:
                print(f'{bcolors.WARNING}failed to revert custom_link_motor_direct=OFF '
                      f'({e}); power-cycling the FC also restores the default{bcolors.ENDC}')
        print('\nSTOP TESTING')
        executor.remove_node(node)
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)

    if node.points:
        _, _, result = write_outputs(node, args, aborted)
        if result is None or not result['monotonic']:
            exit_code = exit_code or 2
    else:
        print(f'{bcolors.WARNING}no data points collected{bcolors.ENDC}')
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
