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
  ros2 run apm_bridge thrust_test_node --lipo-cells 6                    # 真机, 飞手持 BOXOFFBOARD
  ros2 run apm_bridge thrust_test_node --offboard --no-env-comp --no-voltage-comp --yes   # AirSim
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


def new_window(u, collective, n_motors=0):
    return {'u': u, 'collective': collective, 'weight': [], 'V': [], 'I': [],
            'rpm': [[] for _ in range(n_motors)], 'P': [], 't': []}


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
        self.tare_samples = None          # 去皮期间收集的稳定重量
        self.window = None                # 当前稳态采样段 (None = 不采集)
        self.quiet = False
        self._arm_intent = False
        self._offboard_intent = False

        self.points = []
        self.tare_kg = float('nan')
        self.u_idle = float('nan')         # 飞控 motor_idle 占空比 (main 中经 /fpv/cli 读取)

        qos_be5 = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_be10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_pub = self.create_publisher(ControlCommand, '/fpv/control_command_raw', qos_be5)
        self.arm_pub = self.create_publisher(Bool, '/fpv/arm', qos_be10)
        self.offboard_pub = (self.create_publisher(Bool, '/fpv/offboard', qos_be10)
                             if args.offboard else None)

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
            self._append('P', float(msg.fluid_pressure))

    def on_temperature(self, msg):
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
        """ATTITUDE 模式 + 姿态回显 + 零角速度 + 直通集体油门。"""
        cmd = ControlCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.control_mode = ControlCommand.ATTITUDE
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
                raise FatalError('FC did not ARM (is BOXOFFBOARD held, or pass --offboard?)')
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

def write_outputs(points, args, aborted):
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    os.makedirs(args.output_dir, exist_ok=True)
    base = os.path.join(args.output_dir, f'thrust_curve_{stamp}')
    v_ref = args.lipo_cells * pm.V_REF_PER_CELL

    result, error = None, None
    try:
        result = pm.calibrate(points, v_ref, voltage_normalization=not args.no_voltage_comp,
                              environment_normalization=not args.no_env_comp)
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
            if result['monotonic']:
                f.write('\n# ---- paste into parameters/custom_link_parameters.yaml ----\n')
                f.write(render_param_yaml(result, args) + '\n')
            else:
                f.write('# NOT MONOTONIC on [0,1]: model rejected (fpv_test.md 3.2), repeat the test\n')
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
            print(f'{bcolors.ERROR}fitted curve violates dF/du > 0 on [0,1] (min slope '
                  f"{result['min_slope']:.4g}): rejected{bcolors.ENDC}")
    else:
        print(f'{bcolors.ERROR}fit failed: {error}{bcolors.ENDC}')

    if not args.no_plot and result:
        plot_results(result, base + '.png')
    return csv_path, yaml_path, result


def plot_results(result, png_path):
    try:
        import matplotlib
        if not os.environ.get('DISPLAY'):
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f'{bcolors.WARNING}matplotlib unavailable ({e}), skipping plot{bcolors.ENDC}')
        return
    try:
        rows = result['rows']
        plt.figure()
        plt.plot([r['u'] for r in rows], [r['F'] for r in rows], 'bx', label='measured F')
        plt.plot([r['u'] for r in rows], [r['F_std'] for r in rows], 'go', label='normalized F_std')
        u = np.linspace(0.0, 1.0, 200)
        plt.plot(u, result['a3'] * u ** 3 + result['a2'] * u ** 2 + result['a1'] * u, '-r',
                 label='F_std(u) fit')
        plt.xlabel('throttle duty u')
        plt.ylabel('total thrust [N]')
        plt.legend()
        plt.grid(True)
        plt.savefig(png_path)
        print(f'plot written to {png_path}')
    except Exception as e:
        print(f'{bcolors.WARNING}plotting failed ({e}){bcolors.ENDC}')


# ======================================================================
# main
# ======================================================================

def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='fpv_test.md bench thrust test + propulsion model fit (custom_link_bridge)')
    parser.add_argument('--offboard', action='store_true',
                        help='publish /fpv/offboard to request OFFBOARD from the host (AirSim / '
                             'pilotless bench); omit when the pilot holds BOXOFFBOARD')
    parser.add_argument('--u-start', type=float, default=0.05, help='first throttle duty (§2.2)')
    parser.add_argument('--u-step', type=float, default=0.05, help='throttle duty step (§2.2)')
    parser.add_argument('--u-end', type=float, default=1.00, help='last throttle duty (§2.2, adjustable)')
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
    parser.add_argument('--critical-weight', type=float, default=0.3,
                        help='abort if the scale reads below this [kg] (lift-off protection)')
    parser.add_argument('--no-env-comp', action='store_true',
                        help='skip §3.1 step 1 (no baro, e.g. AirSim; pairs with baro_compensation=false)')
    parser.add_argument('--no-voltage-comp', action='store_true',
                        help='skip §3.1 steps 3-4 (no voltage sag, e.g. AirSim; pairs with '
                             'voltage_compensation=false)')
    parser.add_argument('--control-rate', type=float, default=50.0, help='command stream rate [Hz]')
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
    except FatalError as e:
        print(f'{bcolors.ERROR}FATAL: {e}{bcolors.ENDC}')
        exit_code = 1
    finally:
        aborted = aborted or node.abort_event.is_set()
        if started:
            node.shutdown_sequence()
        print('\nSTOP TESTING')
        executor.remove_node(node)
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)

    if node.points:
        _, _, result = write_outputs(node.points, args, aborted)
        if result is None or not result['monotonic']:
            exit_code = exit_code or 2
    else:
        print(f'{bcolors.WARNING}no data points collected{bcolors.ENDC}')
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
