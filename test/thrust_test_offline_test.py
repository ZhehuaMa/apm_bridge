#!/usr/bin/python3
"""Offline end-to-end test of thrust_test_node against a fake bridge + physical plant.

The FakeBridge emulates custom_link_bridge + FC + bench scale: every commanded
Betaflight collective is mapped to the DShot duty u = 0.055 + 0.945·c and applied to
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
MOTOR_IDLE = 0.055

# shortened SOP: 10 steps 0.1..1.0, 0.6 s/step (0.2 s settle), 0.3 s idle cooldown
SOP_ARGS = ['--yes', '--no-plot', '--u-start', '0.1', '--u-step', '0.1', '--u-end', '1.0',
            '--step-time', '0.6', '--settle-time', '0.2', '--cooldown-time', '0.3',
            '--tare-time', '0.5', '--control-rate', '50', '--gravity', str(GRAVITY)]


class FakeBridge(Node):

    def __init__(self, airsim):
        super().__init__('fake_bridge')
        self.airsim = airsim
        self.lock = threading.Lock()
        self.armed = False
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
        self.create_timer(0.1, self.publish_state)
        self.create_timer(0.02, self.publish_telemetry)

    def on_command(self, msg):
        with self.lock:
            self.last_cmd_time = time.monotonic()
            self.collective = msg.collective_thrust if msg.armed else 0.0
            self.commands.append((time.monotonic(), msg.collective_thrust, bool(msg.armed)))

    def on_arm(self, msg):
        with self.lock:
            self.armed = msg.data
            self.arm_calls.append(msg.data)

    def publish_state(self):
        msg = State()
        msg.connected = True
        with self.lock:
            msg.armed = self.armed
            msg.mode = 'OFFBOARD' if self.armed and time.monotonic() - self.last_cmd_time < 0.5 else 'ACRO'
        self.state_pub.publish(msg)

    def plant_state(self):
        with self.lock:
            armed, collective = self.armed, self.collective
        duty = pm.collective_to_duty(collective, MOTOR_IDLE) if armed else 0.0
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


def run_tool(airsim, out_dir, extra):
    rclpy.init()
    fake = FakeBridge(airsim)
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
    finally:
        executor.shutdown()
        rclpy.shutdown()
        spin.join(timeout=2.0)
    return proc, commands, arm_calls


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


def hardware_mode(out_dir, results):
    print('== hardware mode (baro + sag + RPM + current) ==')
    proc, commands, arm_calls = run_tool(False, out_dir, ['--lipo-cells', '6'])
    if proc.returncode != 0:
        print(proc.stdout[-3000:], proc.stderr[-2000:])
    check(results, 'hw_exit', proc.returncode == 0, f'rc={proc.returncode}')
    check(results, 'hw_arm_disarm', True in arm_calls and False in arm_calls)

    # step order with an idle cooldown (collective 0) after each u >= 0.70 step
    sequence = []
    for _, c, armed in commands:
        c = round(c, 6)
        if armed and (not sequence or sequence[-1] != c):
            sequence.append(c)
    expected = []
    for u in [round(0.1 * i, 1) for i in range(1, 11)]:
        expected.append(round(pm.duty_to_collective(u, MOTOR_IDLE), 6))
        if u >= 0.7:
            expected.append(0.0)
    expected = [0.0] + expected
    check(results, 'hw_sop_sequence', sequence == expected, f'\n      got {sequence}\n      exp {expected}')

    rows, result = read_outputs(out_dir)
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
    model = pm.PropulsionModel.from_parameters(mp, 6)
    worst = 0.0
    for p, t, v_oc in ((101325.0, 0.0, 22.2), (PRESSURE, TEMPERATURE, 24.0), (95000.0, -10.0, 21.0)):
        for f_req in (10.0, 25.0, 45.0):
            realized, _ = plant.closed_loop_thrust(model, f_req, p, t, v_oc)
            worst = max(worst, abs(realized / f_req - 1.0))
    check(results, 'hw_closed_loop_thrust', worst < 0.01, f'worst error {100 * worst:.2f}%')


def airsim_mode(out_dir, results):
    print('== AirSim mode (no baro, no sag, no RPM, no current) ==')
    proc, _, _ = run_tool(True, out_dir, ['--no-env-comp', '--no-voltage-comp', '--offboard'])
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


def main():
    results = {}
    with tempfile.TemporaryDirectory(prefix='thrust_test_offline_') as out_dir:
        os.makedirs(os.path.join(out_dir, 'hw'))
        os.makedirs(os.path.join(out_dir, 'airsim'))
        hardware_mode(os.path.join(out_dir, 'hw'), results)
        airsim_mode(os.path.join(out_dir, 'airsim'), results)
    print('\n===== summary =====')
    for k, v in results.items():
        print(f"  {'PASS' if v else 'FAIL'}  {k}")
    ok = all(results.values())
    print('RESULT:', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
