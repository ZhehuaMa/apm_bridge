#!/usr/bin/python3
"""Offline end-to-end test of thrust_test_node against a fake bridge + physical plant.

The FakeBridge emulates custom_link_bridge + FC + bench scale. It answers /fpv/cli
"get motor_idle" with a non-default 600 (u_idle = 0.06) so the tool must read it from
the FC; every commanded Betaflight collective is mapped to u = u_idle + (1 − u_idle)·c and applied to
the synthetic plant of propulsion_plant.py (F ∝ ρ·RPM², back-EMF, I ∝ RPM², pack sag).
It publishes /current_weight, /mavros/battery (V, I), /fpv/motor_rpm,
/fpv/static_pressure and /fpv/temperature_baro at a non-standard atmosphere.
The real tool then runs the fpv_test.md section 2 SOP (shortened) and section 3 fit.

Checks:
  hardware mode -- step order + idle cooldowns after u >= 0.70, CSV columns, R_eq/K_v
                   identification, fitted F_std(u) equal to the direct pipeline on the
                   ideal samples, and closed-loop thrust error < 1 % with the result
  AirSim mode   -- --no-env-comp --no-voltage-comp, no baro/sag/RPM/current: exact
                   linear model recovered

Usage (workspace with apm_bridge built and sourced, system python):
  python3 test/thrust_test_offline_test.py
"""

import csv
import math
import os
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from mavros_msgs.msg import State
from quadrotor_msgs.msg import ControlCommand
from quadrotor_msgs.srv import CliCommand
from sensor_msgs.msg import BatteryState, FluidPressure, Imu, Temperature
from std_msgs.msg import Bool, Float32MultiArray

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, '..', 'scripts', 'thrust_test_node.py')
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))
sys.path.insert(0, HERE)
import propulsion_model as pm  # noqa: E402
import propulsion_plant as plant  # noqa: E402

TARE = 6.0            # kg on the scale with motors stopped
GRAVITY = 9.80665
PRESSURE = 85000.0    # ~1500 m
TEMPERATURE = 30.0
FC_MOTOR_IDLE = 600             # Betaflight motor_idle (0.01 %), deliberately not the default
U_IDLE = FC_MOTOR_IDLE / 10000.0

# shortened SOP: 10 steps 0.1..1.0, 0.6 s/step (0.2 s settle), 0.3 s idle cooldown
SOP_ARGS = ['--yes', '--no-plot', '--u-start', '0.1', '--u-step', '0.1', '--u-end', '1.0',
            '--step-time', '0.6', '--settle-time', '0.2', '--cooldown-time', '0.3',
            '--tare-time', '0.5', '--control-rate', '50', '--gravity', str(GRAVITY)]


class FakeBridge(Node):

    def __init__(self, airsim, reject_motor_direct_set=False):
        super().__init__('fake_bridge')
        self.airsim = airsim
        self.reject_motor_direct_set = reject_motor_direct_set  # 仿真旧固件 (无该设置)
        self.motor_direct_on = False
        self.numeric_settings = {'failsafe_delay': 15}  # 数值型设置 (固件默认 1.5s)
        self.rearm_after_disarm = 0   # 解锁后到达的 arm=True (解锁/重锁竞争检测)
        self._saw_disarm = False
        self._saw_arm_level = False
        self.lock = threading.Lock()
        self.armed = False
        self.offboard = False   # /fpv/offboard 电平 (主机经链路直接接管, 无需飞手开关)
        self.collective = 0.0
        self.last_cmd_time = 0.0
        self.commands = []        # (time, collective, armed_field)
        self.arm_calls = []
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state_pub = self.create_publisher(State, '/mavros/state', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/mavros/battery', qos)
        self.imu_pub = self.create_publisher(Imu, '/mavros/imu/data', qos)
        self.weight_pub = self.create_publisher(Float32MultiArray, '/current_weight', qos)
        self.rpm_pub = self.create_publisher(Float32MultiArray, '/fpv/motor_rpm', qos)
        self.pressure_pub = self.create_publisher(FluidPressure, '/fpv/static_pressure', qos)
        self.temp_pub = self.create_publisher(Temperature, '/fpv/temperature_baro', qos)
        self.create_subscription(ControlCommand, '/fpv/control_command_raw', self.on_command, qos)
        self.create_subscription(Bool, '/fpv/arm', self.on_arm, qos)
        self.create_subscription(Bool, '/fpv/offboard', self.on_offboard, qos)
        self.cli_gets = []
        self.create_service(CliCommand, '/fpv/cli', self.on_cli)
        self.create_timer(0.1, self.publish_state)
        self.create_timer(0.02, self.publish_telemetry)

    def on_command(self, msg):
        with self.lock:
            self.last_cmd_time = time.monotonic()
            self.collective = msg.collective_thrust if msg.armed else 0.0
            self.commands.append((time.monotonic(), msg.collective_thrust, bool(msg.armed), msg))

    def on_cli(self, request, response):
        # 复刻固件 CLI 通道语义: motor_idle GET + custom_link_motor_direct GET/SET
        # (MODE_LOOKUP 值只认 ON/OFF 字符串; SET 回读新值) + 数值型设置。
        response.ok = True
        args = (request.args or '').strip()
        name = args.split('=', 1)[0].strip()
        if request.cmd == CliCommand.Request.CMD_GET and args == 'motor_idle':
            self.cli_gets.append(args)
            response.status, response.text = 0, f'motor_idle = {FC_MOTOR_IDLE}'
        elif name in self.numeric_settings and '=' not in args and \
                request.cmd == CliCommand.Request.CMD_GET:
            self.cli_gets.append(args)
            response.status = 0
            response.text = f'{name} = {self.numeric_settings[name]}'
        elif name in self.numeric_settings and request.cmd == CliCommand.Request.CMD_SET:
            try:
                value = int(args.split('=', 1)[1].strip())
            except (IndexError, ValueError):
                response.status, response.text = 1, 'SET FAILED'
                return response
            self.numeric_settings[name] = value
            response.status, response.text = 0, f'{name} = {value}'
        elif args.startswith('custom_link_motor_direct'):
            if self.reject_motor_direct_set:
                response.status, response.text = 1, 'SET FAILED'
            elif request.cmd == CliCommand.Request.CMD_SET:
                value = args.split('=', 1)[1].strip() if '=' in args else ''
                if value.upper() not in ('ON', 'OFF'):
                    response.status, response.text = 1, 'SET FAILED'
                else:
                    self.motor_direct_on = value.upper() == 'ON'
                    response.status, response.text = 0, f'custom_link_motor_direct = {value.upper()}'
            elif request.cmd == CliCommand.Request.CMD_GET and args == 'custom_link_motor_direct':
                response.status = 0
                response.text = f'custom_link_motor_direct = {"ON" if self.motor_direct_on else "OFF"}'
            else:
                response.status, response.text = 1, 'NOT FOUND'
        else:
            response.status, response.text = 1, 'NOT FOUND'
        return response

    def on_arm(self, msg):
        with self.lock:
            self.armed = msg.data
            self.arm_calls.append(msg.data)
            # 真机语义: host arm 是电平 (≡ BOXARM), 解锁后 True 会重新解锁。
            # 工具的上锁路径不得在"运行开始后的 False"之后重发 True。
            # 节点启动期 (start_testing 之前) 保活发布的初始 False 不算解锁。
            if msg.data:
                if self._saw_disarm:
                    self.rearm_after_disarm += 1
                self._saw_arm_level = True
            elif getattr(self, '_saw_arm_level', False):
                self._saw_disarm = True

    def on_offboard(self, msg):
        with self.lock:
            self.offboard = msg.data

    def publish_state(self):
        msg = State()
        msg.connected = True
        with self.lock:
            # 真实固件要求控制流新鲜 + OFFBOARD 请求才解锁/接管; 这里同样的语义,
            # 防止测试把"只是发了 arm 电平"误认为已接管。
            msg.armed = self.armed and self.offboard
            msg.mode = ('OFFBOARD' if msg.armed and
                        time.monotonic() - self.last_cmd_time < 0.5 else 'ACRO')
        self.state_pub.publish(msg)

    def plant_state(self):
        with self.lock:
            armed, collective = self.armed, self.collective
        duty = pm.collective_to_duty(collective, U_IDLE) if armed else 0.0
        if self.airsim:
            return {'F': plant.AIRSIM_A1 * duty, 'V': 16.8, 'I': math.nan, 'rpm': math.nan}
        return plant.state(duty, plant.density_ratio(PRESSURE, TEMPERATURE))

    def publish_telemetry(self):
        s = self.plant_state()
        weight = TARE - s['F'] / GRAVITY
        self.weight_pub.publish(Float32MultiArray(data=[weight, weight]))
        bat = BatteryState()
        bat.voltage = float(s['V'])
        bat.cell_voltage = [float(s['V'])]
        bat.current = -float(s['I'])
        self.battery_pub.publish(bat)
        imu = Imu()
        imu.orientation.w = 1.0
        self.imu_pub.publish(imu)
        rpms = [math.nan] * 4 if self.airsim else plant.motor_rpms(s['rpm'])
        self.rpm_pub.publish(Float32MultiArray(data=[float(r) for r in rpms]))
        if not self.airsim:
            self.pressure_pub.publish(FluidPressure(fluid_pressure=PRESSURE))
            self.temp_pub.publish(Temperature(temperature=TEMPERATURE))


def run_tool(airsim, out_dir, extra, reject_motor_direct_set=False):
    rclpy.init()
    fake = FakeBridge(airsim, reject_motor_direct_set)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(fake)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    time.sleep(0.5)
    try:
        proc = subprocess.run([sys.executable, SCRIPT, '--output-dir', out_dir] + SOP_ARGS + extra,
                              capture_output=True, text=True, timeout=120)
        with fake.lock:
            commands, arm_calls = list(fake.commands), list(fake.arm_calls)
            rearm = fake.rearm_after_disarm
        cli_gets = list(fake.cli_gets)
    finally:
        executor.shutdown()
        rclpy.shutdown()
        spin.join(timeout=2.0)
    return proc, commands, arm_calls, cli_gets, rearm


def read_outputs(out_dir):
    import yaml
    csv_file = sorted(f for f in os.listdir(out_dir) if f.endswith('.csv'))[-1]
    yaml_file = sorted(f for f in os.listdir(out_dir) if f.endswith('_result.yaml'))[-1]
    with open(os.path.join(out_dir, csv_file)) as f:
        rows = list(csv.DictReader(f))
    with open(os.path.join(out_dir, yaml_file)) as f:
        result = yaml.safe_load(f)
    return rows, result


def check(results, name, ok, detail=''):
    results[name] = bool(ok)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name} {detail}")


def hardware_mode(out_dir, results, reject_gate=False, prefix='hw'):
    label = 'old-firmware fallback' if reject_gate else 'motor-direct'
    print(f'== hardware mode ({label}: baro + sag + RPM + current) ==')
    proc, commands, arm_calls, cli_gets, rearm = run_tool(False, out_dir, ['--lipo-cells', '6'],
                                                                   reject_motor_direct_set=reject_gate)
    if proc.returncode != 0:
        print(proc.stdout[-3000:], proc.stderr[-2000:])
    check(results, f'{prefix}_exit', proc.returncode == 0, f'rc={proc.returncode}')
    check(results, f'{prefix}_arm_disarm', True in arm_calls and False in arm_calls)
    check(results, f'{prefix}_no_rearm_after_disarm', rearm == 0, f'{rearm} rearms')
    check(results, f'{prefix}_u_idle_from_fc', cli_gets[:1] == ['motor_idle'] and 'failsafe_delay' in cli_gets,
          f'/fpv/cli gets={cli_gets}')
    test_cmds = [msg for _, _, armed, msg in ((t, c, a, m) for t, c, a, m in commands) if armed]
    expected_mode = ControlCommand.BODY_RATES if reject_gate else ControlCommand.MOTOR_DIRECT
    mode_ok = test_cmds and all(m.control_mode == expected_mode for m in test_cmds) and \
        all(abs(m.bodyrates.x) < 1e-12 and abs(m.bodyrates.y) < 1e-12 and abs(m.bodyrates.z) < 1e-12
            for m in test_cmds)
    check(results, f'{prefix}_control_stream', mode_ok,
          f'{len(test_cmds)} armed cmds, mode={expected_mode}')

    # step order with an idle cooldown (collective 0) after each u >= 0.70 step

    # step order with an idle cooldown (collective 0) after each u >= 0.70 step
    sequence = []
    for _, c, armed, _ in commands:
        c = round(c, 6)
        if armed and (not sequence or sequence[-1] != c):
            sequence.append(c)
    expected = []
    for u in [round(0.1 * i, 1) for i in range(1, 11)]:
        expected.append(round(pm.duty_to_collective(u, U_IDLE), 6))
        if u >= 0.7:
            expected.append(0.0)
    expected = [0.0] + expected
    # 后续追加的是标定验证段的油门 (§4 在线反解), 不属于阶梯: 只比对前缀
    check(results, 'hw_sop_sequence', sequence[:len(expected)] == expected,
          f'\n      got {sequence}\n      exp prefix {expected}')

    rows, result = read_outputs(out_dir)
    check(results, 'hw_u_idle_recorded', result['u_idle_fc_motor_idle'] == U_IDLE and
          abs(float(rows[0]['u']) - (U_IDLE + (1 - U_IDLE) * float(rows[0]['collective']))) < 1e-6,
          f"u_idle={result['u_idle_fc_motor_idle']}")
    check(results, 'hw_csv_points', len(rows) == 10 and
          {'u', 'V', 'I', 'rpm1', 'rpm4', 'rpm_rms', 'F_N', 'P_Pa', 't_C', 'F0_N', 'F_std_N'} <= set(rows[0]),
          f'{len(rows)} rows')
    check(results, 'hw_identification', abs(result['r_eq_ohm'] - plant.R_EQ) < 1e-3 and
          abs(result['ke_volt_per_rpm'] - plant.KE) / plant.KE < 1e-3,
          f"R_eq={result['r_eq_ohm']:.5g} K_e={result['ke_volt_per_rpm']:.5g}")
    mp = result['custom_link_bridge']['ros__parameters']['motor_parameters']
    ideal = pm.calibrate([plant.bench_point(round(0.1 * i, 1), PRESSURE, TEMPERATURE) for i in range(1, 11)],
                         plant.V_REF)
    coef_ok = all(abs(mp[k] - ideal[k]) <= 1e-3 * max(1.0, abs(ideal[k])) for k in ('a3', 'a2', 'a1'))
    check(results, 'hw_fit_matches_pipeline', coef_ok and result['monotonic'],
          f"a3..a1=({mp['a3']:.5g}, {mp['a2']:.5g}, {mp['a1']:.5g})")
    check(results, 'hw_no_idle_parameter',
          set(mp) == {'a3', 'a2', 'a1', 'r_eq', 'k_i', 'k_i2', 'k_i3'}, f'{sorted(mp)}')
    model = pm.PropulsionModel.from_parameters(mp, 6, U_IDLE)
    verification = result.get('verification') or []
    ver_ok = verification and all(abs(v['error_pct']) < 2.0 for v in verification)
    check(results, 'hw_verification', bool(ver_ok),
          ', '.join(f"{v['f_req_N']:.2f}N:{v['error_pct']:+.1f}%" for v in verification))
    worst = 0.0
    for p, t, v_oc in ((101325.0, 0.0, 22.2), (PRESSURE, TEMPERATURE, 24.0), (95000.0, -10.0, 21.0)):
        for f_req in (10.0, 25.0, 45.0):
            realized, _ = plant.closed_loop_thrust(model, f_req, p, t, v_oc)
            worst = max(worst, abs(realized / f_req - 1.0))
    check(results, 'hw_closed_loop_thrust', worst < 0.01, f'worst error {100 * worst:.2f}%')


def airsim_mode(out_dir, results):
    print('== AirSim mode (no baro, no sag, no RPM, no current) ==')
    proc, _, _, _, _ = run_tool(True, out_dir, ['--no-env-comp', '--no-voltage-comp'])
    if proc.returncode != 0:
        print(proc.stdout[-3000:], proc.stderr[-2000:])
    check(results, 'airsim_exit', proc.returncode == 0, f'rc={proc.returncode}')
    rows, result = read_outputs(out_dir)
    mp = result['custom_link_bridge']['ros__parameters']['motor_parameters']
    check(results, 'airsim_linear_model', abs(mp['a1'] - plant.AIRSIM_A1) < 1e-3 and
          abs(mp['a2']) < 1e-3 and abs(mp['a3']) < 1e-3 and mp['r_eq'] == 0.0,
          f"a3..a1=({mp['a3']:.3g}, {mp['a2']:.3g}, {mp['a1']:.6g})")
    check(results, 'airsim_compensation_flags',
          result['custom_link_bridge']['ros__parameters']['voltage_compensation'] is False and
          result['custom_link_bridge']['ros__parameters']['baro_compensation'] is False)
    check(results, 'airsim_rpm_nan', all(math.isnan(float(r['rpm_rms'])) for r in rows))


def hardware_fallback_mode(out_dir, results):
    """旧固件降级路径: CLI SET custom_link_motor_direct 被拒 -> 告警 + BODY_RATES。"""
    print('== hardware mode (old firmware: gate SET rejected) ==')
    proc, commands, arm_calls, cli_gets, rearm_fb = run_tool(False, out_dir, ['--lipo-cells', '6'],
                                                                       reject_motor_direct_set=True)
    if proc.returncode != 0:
        print(proc.stdout[-3000:], proc.stderr[-2000:])
    check(results, 'fb_exit', proc.returncode == 0, f'rc={proc.returncode}')
    test_cmds = [m for _, _, armed, m in commands if armed]
    check(results, 'fb_body_rates_fallback', bool(test_cmds) and
          all(m.control_mode == ControlCommand.BODY_RATES for m in test_cmds))
    check(results, 'fb_warning_logged', 'falling back to BODY_RATES' in proc.stdout)


def main():
    # 本机默认 domain 可能有实机台架图 (scale_reader/mavros 等), 会把真实秤数据混进
    # FakeBridge 的 /current_weight (2026-11-12 实测: tare 混入 19.16 kg 真秤读数)。
    # 隔离到独立 domain; 显式设置时尊重操作者覆盖 (同 bench_angle_link_test 约定)。
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 40))
    results = {}
    with tempfile.TemporaryDirectory(prefix='thrust_test_offline_') as out_dir:
        os.makedirs(os.path.join(out_dir, 'hw'))
        os.makedirs(os.path.join(out_dir, 'hw_fallback'))
        os.makedirs(os.path.join(out_dir, 'airsim'))
        hardware_mode(os.path.join(out_dir, 'hw'), results)
        hardware_fallback_mode(os.path.join(out_dir, 'hw_fallback'), results)
        airsim_mode(os.path.join(out_dir, 'airsim'), results)
    print('\n===== summary =====')
    for k, v in results.items():
        print(f"  {'PASS' if v else 'FAIL'}  {k}")
    ok = all(results.values())
    print('RESULT:', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
