"""Native ANGLE raw command transport through the real bridge, with a TCP fake FC only."""
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import State
from quadrotor_msgs.msg import ControlCommand
from std_msgs.msg import Bool

from cli_service_test import FakeFc
from sitl_odometry_test import encode, shutdown


def test_native_angle_transport_and_rate_compatibility(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(100 + os.getpid() % 40))
    binary = Path(__file__).resolve().parents[3] / "build/apm_bridge/custom_link_bridge_node"
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen()
    server.settimeout(5)
    context = Context()
    rclpy.init(context=context)
    node = Node("bench_angle_transport_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    states = []
    node.create_subscription(State, "/mavros/state", states.append, 10)
    raw = node.create_publisher(ControlCommand, "/custom_link_bridge/control_command_raw", qos_profile_sensor_data)
    offboard = node.create_publisher(Bool, "/custom_link_bridge/offboard", 10)
    process = connection = None
    try:
        with (tmp_path / "bridge.log").open("w") as log:
            process = subprocess.Popen([str(binary), "--ros-args", "-p", "link.transport:=tcp",
                "-p", f"link.tcp_port:={server.getsockname()[1]}", "-p", "gimbal_port:=''"],
                stdout=log, stderr=subprocess.STDOUT)
            connection = server.accept()[0]
            connection.setblocking(False)
            fc = FakeFc(connection)
            def pump(duration):
                deadline = time.monotonic() + duration
                while time.monotonic() < deadline:
                    assert process.poll() is None
                    slow = bytearray(36)
                    struct.pack_into("<I", slow, 0, round(time.monotonic()*1e6) & 0xffffffff)
                    struct.pack_into("<IH", slow, 30, (1 << 13) | 1, 1)
                    connection.sendall(encode(0x12, slow))
                    fc.pump()
                    executor.spin_once(timeout_sec=.005)
            deadline = time.monotonic() + 5
            while raw.get_subscription_count() == 0 or not states:
                assert time.monotonic() < deadline
                pump(.05)
            assert states[-1].mode == "OFFBOARD|ANGLE"
            command = ControlCommand(armed=True, control_mode=ControlCommand.ATTITUDE, collective_thrust=.12)
            roll, pitch = math.radians(2), math.radians(-3)
            cr, sr, cp, sp = math.cos(roll/2), math.sin(roll/2), math.cos(pitch/2), math.sin(pitch/2)
            command.orientation.x, command.orientation.y = sr*cp, cr*sp
            command.orientation.z, command.orientation.w = -sr*sp, cr*cp
            command.bodyrates.z = math.radians(4)

            # 默认油门成形开启 (up 1.0 s / down 0.5 s / EMA 10 @100 Hz):
            # 0x20 油门从 1000 单调爬向 1120 (1000 + 0.12*1000), 不允许一步到位。
            def frames(mode, since=0):
                return [f for f in fc.controls[since:] if f[3] == mode]

            def pump_until_converged(publish, mode, since=0, target=1120, timeout=5.):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    publish()
                    pump(.05)
                    mode_frames = frames(mode, since)
                    if mode_frames and mode_frames[-1][4] == target:
                        return mode_frames
                return frames(mode, since)

            angle = pump_until_converged(lambda: raw.publish(command), 3)
            assert angle, "no ANGLE frames received"
            assert all(f[2] == 0 for f in angle)
            assert angle[0][4] <= 1050, "throttle ramp must be engaged (first frame near idle)"
            assert angle[-1][4] == 1120, "throttle must converge to the commanded value"
            throttles = [f[4] for f in angle]
            assert throttles == sorted(throttles), "throttle must ramp monotonically"
            assert all(f[4] <= 1120 for f in angle), "throttle must not overshoot the command"
            assert all(abs(f[5]-20) <= 1 and abs(f[6]+30) <= 1 and abs(f[7]-40) <= 1 for f in angle)

            rates_cmd = ControlCommand(armed=True, control_mode=ControlCommand.BODY_RATES,
                                       collective_thrust=.12)
            raw.publish(rates_cmd)
            pump(.1)
            assert fc.controls[-1][3] == 2

            # 指令超时 (rc.control_timeout=0.5 s) -> 控制量失效: 无 arm/模式请求时
            # 停发 0x20; 失效边沿把成形基线归零, 恢复后从怠速重新爬坡而不是一步
            # 跳回 1120 (阶跃电流冲击正是成形要抑制的对象)。前 0.5 s 仍在发帧,
            # 先放掉超时窗口再断言停发。
            pump(.8)
            stale_mark = len(fc.controls)
            pump(.2)
            assert len(fc.controls) == stale_mark, "stale stream must stop transmitting"
            resumed = pump_until_converged(lambda: raw.publish(rates_cmd), 2, since=stale_mark)
            assert resumed, "no frames after recovery"
            assert resumed[0][4] <= 1050, "recovery must re-ramp from idle, not jump to the pre-gap value"
            assert resumed[-1][4] == 1120, "recovered throttle must converge again"

            offboard.publish(Bool(data=False))
            pump(.1)
            assert fc.controls[-1][3] == 0 and all(f[2] == 0 for f in fc.controls)
    finally:
        if connection is not None:
            connection.close()
        server.close()
        if process is not None:
            shutdown(process)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
