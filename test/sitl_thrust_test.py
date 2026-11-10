#!/usr/bin/env python3
"""SITL end-to-end test for thrust_test_node (fpv_test.md bench SOP, AirSim-style options).

Brings up the Betaflight SITL firmware with the custom link on tcp:5763
(same provisioning as sitl_custom_link_test.py: ARM on AUX1, OFFBOARD on
AUX2, BOXOFFBOARD switch held on), the custom_link_bridge_node, and a
synthetic bench scale driven by the SITL motor outputs, then runs the
real thrust_test_node as a subprocess and checks:

  1. host arm + OFFBOARD takeover **through the link** (no pilot BOXOFFBOARD switch:
     the tool's /fpv/offboard -> mode_req drives the takeover) + motors follow the SOP
  2. SOP steps appear in the CSV with increasing force and a monotonic fpv_test.md fit
  3. clean shutdown: FC disarmed, OFFBOARD dropped, exit code 0

The synthetic scale is linear in the SITL motor outputs and SITL has no battery sag or
RPM feedback, so the tool runs with --no-env-comp --no-voltage-comp (AirSim settings).

Usage (workspace with apm_bridge built and sourced, system python):
  python3 test/sitl_thrust_test.py --binary <betaflight repo>/obj/betaflight_2026.6.1_SITL
"""

import argparse
import csv
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from mavros_msgs.msg import State
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float32MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sitl_custom_link_test import Feed, wait_for  # noqa: E402

LINK_TCP_PORT = 5763

BOX_ARM = 0        # permanent id
BOX_OFFBOARD = 58  # permanent id

# 合成台秤: 重量 = TARE - K_FORCE * sum(motors) / g
TARE = 5.0        # kg
K_FORCE = 6.0     # N per unit of summed motor output

# 工具参数: 5 阶 SOP (u = 0.2..0.6), 每阶 1.5 s (0.5 s 稳定 + 1.0 s 采样), 无降温段
TOOL_ARGS = ['--yes', '--no-plot', '--no-env-comp', '--no-voltage-comp',
             '--u-start', '0.2', '--u-step', '0.1', '--u-end', '0.6',
             '--step-time', '1.5', '--settle-time', '0.5', '--cooldown-time', '0',
             '--tare-time', '1.0', '--lipo-cells', '6']


class StateObserver(Node):
    def __init__(self):
        super().__init__('thrust_test_state_observer')
        self.state = State()
        self.state_events = []
        self.pressure_count = 0
        self.create_subscription(State, '/mavros/state', self._state, 10)
        self.create_subscription(FluidPressure, '/fpv/static_pressure',
                                 self._pressure,
                                 QoSProfile(depth=10,
                                            reliability=ReliabilityPolicy.BEST_EFFORT))

    def _state(self, msg):
        self.state = msg
        self.state_events.append((time.monotonic(), msg.armed, msg.mode))

    def _pressure(self, msg):
        self.pressure_count += 1


class WeightSim:
    """把 SITL 电机输出映射为台秤重量并发布 /current_weight。"""

    def __init__(self, feed, gravity=9.801):
        self.feed = feed
        self.gravity = gravity
        self.pub_node = Node('weight_sim')
        self.pub = self.pub_node.create_publisher(Float32MultiArray, '/current_weight',
                                                  QoSProfile(depth=10,
                                                             reliability=ReliabilityPolicy.BEST_EFFORT))
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.thread.start()

    def _loop(self):
        rate = 0.1
        while self.running:
            force = K_FORCE * sum(self.feed.motors)
            weight = TARE - force / self.gravity
            self.pub.publish(Float32MultiArray(data=[weight, weight]))
            time.sleep(rate)


def provision_and_start_sitl(binary, workdir):
    # Never kill other users' processes on a shared host: refuse to run instead.
    # Patterns are anchored to the executable at the start of the command line so
    # shells/editors that merely mention the name do not match.
    for pattern in (f"^{binary}", r"^[^ ]*custom_link_bridge_node( |$)"):
        running = subprocess.run(["pgrep", "-af", pattern], capture_output=True, text=True)
        if running.returncode == 0:
            print(f"refusing to start: '{pattern}' already running:\n{running.stdout.strip()}")
            return None, None

    os.makedirs(workdir, exist_ok=True)
    eeprom = os.path.join(workdir, "eeprom.bin")
    if os.path.exists(eeprom):
        os.remove(eeprom)

    cfg = os.path.join(workdir, "config.txt")
    with open(cfg, "w") as f:
        f.write("\n".join([
            "set small_angle = 180",
            f"aux 0 {BOX_ARM} 0 1700 2100 0 0",
            f"aux 1 {BOX_OFFBOARD} 1 1700 2100 0 0",
        ]) + "\n")
    res = subprocess.run([binary, "--config", cfg], cwd=workdir,
                         capture_output=True, text=True, timeout=60)
    if res.returncode != 0 or not os.path.exists(eeprom):
        print("provisioning failed:", res.stdout[-500:], res.stderr[-500:])
        return None, None

    sitl_log = open(os.path.join(workdir, "sitl.log"), "w")
    return subprocess.Popen(["stdbuf", "-oL", "-eL", binary], cwd=workdir,
                            stdout=sitl_log, stderr=sitl_log,
                            preexec_fn=os.setsid), sitl_log


def wait_link_ready(timeout=30.0):
    import socket
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            probe = socket.create_connection(("127.0.0.1", LINK_TCP_PORT), timeout=0.3)
            probe.settimeout(2.0)
            try:
                data = probe.recv(64)
            except socket.timeout:
                data = b""
            probe.close()
            if data[:1] == b"\xeb" or b"\xeb\x90" in data:
                return True
        except OSError:
            pass
        time.sleep(0.3)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary", required=True, help="path to betaflight_SITL executable")
    ap.add_argument("--workdir", default="/tmp/sitl_thrust_test")
    args = ap.parse_args()
    binary = os.path.abspath(args.binary)

    sitl, sitl_log = provision_and_start_sitl(binary, args.workdir)
    if sitl is None:
        return 1

    results = {}
    node_proc = None
    try:
        if not wait_link_ready():
            print("SITL did not stream custom link telemetry")
            return 1
        time.sleep(0.5)

        feed = Feed()
        # 不置 BOXOFFBOARD 开关 (AUX2 低): OFFBOARD 由工具经定制链路 mode_req 直接接管,
        # 验证新版"通讯协议直接进入 OFFBOARD"路径 (固件接受 mode_req 新边沿即接管)。
        feed.set_rc(5, 1000)  # AUX2 low: no pilot OFFBOARD switch

        rclpy.init()
        executor = rclpy.executors.SingleThreadedExecutor()
        observer = StateObserver()
        executor.add_node(observer)

        def spin_guarded():
            try:
                executor.spin()
            except rclpy.executors.ExternalShutdownException:
                pass

        spin = threading.Thread(target=spin_guarded, daemon=True)
        spin.start()

        weight_sim_node_holder = WeightSim(feed)
        executor.add_node(weight_sim_node_holder.pub_node)
        weight_sim_node_holder.start()

        node_proc = subprocess.Popen(
            ["ros2", "run", "apm_bridge", "custom_link_bridge_node", "--ros-args",
             "-p", "link.transport:=tcp",
             "-p", f"link.tcp_port:={LINK_TCP_PORT}",
             "-p", "mass:=2.0",
             "-r", "__node:=custom_link_bridge_under_test",
             "-r", "~/control_command_raw:=/fpv/control_command_raw",
             "-r", "~/arm:=/fpv/arm"],
            preexec_fn=os.setsid,
            stdout=open(os.path.join(args.workdir, "bridge.log"), "w"),
            stderr=subprocess.STDOUT)

        if not wait_for("bridge connected + state received",
                        lambda: observer.state.connected, timeout=30):
            return 1

        out_dir = os.path.join(args.workdir, "out")
        os.makedirs(out_dir, exist_ok=True)
        tool = subprocess.Popen(
            [sys.executable,
             os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', 'scripts', 'thrust_test_node.py'),
             '--output-dir', out_dir] + TOOL_ARGS,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

        motor_max = 0.0
        # 电机峰值监控 (工具运行期间; 前 3 s 是链路/解锁建立期, 不计入)
        t0 = time.monotonic()
        while tool.poll() is None:
            peak = max(feed.motors)
            if peak > motor_max and time.monotonic() - t0 > 3.0:
                motor_max = peak
            time.sleep(0.2)

        try:
            stdout, _ = tool.communicate(timeout=150.0)
        except subprocess.TimeoutExpired:
            tool.kill()
            stdout, _ = tool.communicate()
            print(stdout[-3000:])
            results['tool_exit'] = False

        results.setdefault('tool_exit', tool.returncode == 0)
        if tool.returncode != 0:
            print(stdout[-3000:])

        results['armed_during'] = any(a for _, a, _ in observer.state_events)
        results['offboard_during'] = any(m == 'OFFBOARD' for _, _, m in
                                         observer.state_events)
        results['motors_spun'] = motor_max > 0.25
        print(f'motor peak: {motor_max:.2f}')

        # 桥发布的环境气压话题 (fpv_test.md §3.1 环境归一化输入)
        results['pressure_topic'] = observer.pressure_count > 10
        print(f'pressure messages: {observer.pressure_count}')

        results['disarmed_after'] = not observer.state.armed
        results['offboard_dropped'] = observer.state.mode != 'OFFBOARD'

        csv_files = sorted(f for f in os.listdir(out_dir) if f.endswith('.csv'))
        ok = bool(csv_files)
        forces = []
        if ok:
            with open(os.path.join(out_dir, csv_files[-1])) as f:
                rows = list(csv.DictReader(f))
            ok = len(rows) == 5
            forces = [float(r['F_N']) for r in rows]
            ok = ok and all(forces[i] < forces[i + 1] for i in range(len(forces) - 1))
            ok = ok and forces[-1] > 1.0
        results['csv_points'] = ok
        print(f'csv forces: {forces}')

        # 结果 YAML 必须给出单调通过 + 可粘贴参数片段 (此前从不检查, 工具
        # 输出坏结果时端到端测试照绿)
        yaml_files = sorted(f for f in os.listdir(out_dir) if f.endswith('_result.yaml'))
        ok = bool(yaml_files)
        if ok:
            import yaml
            with open(os.path.join(out_dir, yaml_files[-1])) as f:
                result = yaml.safe_load(f)
            ok = result.get('monotonic') is True and \
                'custom_link_bridge' in result  # paste block present
        results['result_yaml'] = ok
        print(f'result yaml: monotonic={ok}')

        weight_sim_node_holder.running = False
        executor.remove_node(observer)
        executor.remove_node(weight_sim_node_holder.pub_node)
        rclpy.shutdown()
        weight_sim_node_holder.pub_node.destroy_node()
        observer.destroy_node()
    finally:
        if node_proc is not None:
            try:
                os.killpg(os.getpgid(node_proc.pid), signal.SIGTERM)
            except OSError:
                pass
        try:
            os.killpg(os.getpgid(sitl.pid), signal.SIGTERM)
        except OSError:
            pass
        for p in (node_proc, sitl):
            if p is None:
                continue
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        if 'sitl_log' in dir():
            sitl_log.close()

    print('\n===== summary =====')
    all_ok = True
    for k, v in results.items():
        print(f"  {'PASS' if v else 'FAIL'}  {k}")
        all_ok &= bool(v)
    print('RESULT:', 'PASS' if all_ok else 'FAIL')
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
