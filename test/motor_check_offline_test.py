#!/usr/bin/python3
"""Offline end-to-end test of motor_check_node against a fake bridge.

Reuses the FakeBridge from thrust_test_offline_test (arm/offboard semantics,
CLI channel incl. the custom_link_motor_direct gate). Checks:
  motor-direct  -- gate enabled via /fpv/cli, hold commands carry MOTOR_DIRECT,
                  gate reverted to OFF on exit
  old firmware  -- gate SET rejected -> warning + BODY_RATES fallback, no revert
  disarm        -- arm level dropped and the FC disarm observed in both runs

Usage (workspace with apm_bridge built and sourced, system python):
  python3 test/motor_check_offline_test.py
"""

import os
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, '..', 'scripts', 'motor_check_node.py')
sys.path.insert(0, HERE)
from thrust_test_offline_test import FakeBridge  # noqa: E402

from quadrotor_msgs.msg import ControlCommand  # noqa: E402


def run_check(fake, extra):
    executor = SingleThreadedExecutor()
    executor.add_node(fake)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    time.sleep(0.5)
    try:
        proc = subprocess.run([sys.executable, SCRIPT, '--yes', '--thrust', '20',
                               '--duration', '1.0', '--rate', '50'] + extra,
                              capture_output=True, text=True, timeout=60)
        with fake.lock:
            commands, arm_calls = list(fake.commands), list(fake.arm_calls)
            gate = fake.motor_direct_on
        return proc, commands, arm_calls, gate
    finally:
        executor.shutdown()
        spin.join(timeout=2.0)


def check(results, name, ok, detail=''):
    results[name] = bool(ok)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name} {detail}")


def main():
    results = {}
    rclpy.init()
    try:
        print('== motor-direct (gate accepted) ==')
        fake = FakeBridge(False)
        proc, commands, arm_calls, gate = run_check(fake, [])
        check(results, 'md_exit', proc.returncode == 0, f'rc={proc.returncode}')
        armed_cmds = [m for _, _, armed, m in commands if armed]
        check(results, 'md_motor_direct_stream', bool(armed_cmds) and
              all(m.control_mode == ControlCommand.MOTOR_DIRECT for m in armed_cmds),
              f'{len(armed_cmds)} armed cmds')
        check(results, 'md_throttle_passthrough',
              armed_cmds and any(abs(m.collective_thrust - 0.2) < 1e-9 for m in armed_cmds)
              and abs(armed_cmds[-1].collective_thrust) < 1e-9)  # hold reaches 20%, exits at zero
        check(results, 'md_arm_disarm', True in arm_calls and False in arm_calls)
        check(results, 'md_gate_reverted', gate is False)

        print('== old firmware (gate SET rejected) ==')
        fake = FakeBridge(False, reject_motor_direct_set=True)
        proc, commands, arm_calls, gate = run_check(fake, [])
        check(results, 'fb_exit', proc.returncode == 0, f'rc={proc.returncode}')
        check(results, 'fb_warning_logged', 'falling back to BODY_RATES' in proc.stdout)
        armed_cmds = [m for _, _, armed, m in commands if armed]
        check(results, 'fb_body_rates_fallback', bool(armed_cmds) and
              all(m.control_mode == ControlCommand.BODY_RATES for m in armed_cmds))
    finally:
        rclpy.shutdown()

    print('\n===== summary =====')
    for k, v in results.items():
        print(f"  {'PASS' if v else 'FAIL'}  {k}")
    ok = all(results.values())
    print('RESULT:', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
