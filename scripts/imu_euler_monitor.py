#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""订阅 /mavros/imu/data,把姿态四元数解成 ZYX 欧拉角实时显示。

用途:custom link 姿态符号台架验证(0x10 attitude 直通链路)。

约定(与 custom_link_vehicle_node::handleFast / tf2 setRPY / 0x10 协议一致):
  机体系 FLU(x 前、y 左、z 上),ZYX 代数欧拉角:
    roll  右滚正
    pitch 低头正(= Betaflight 0x10 attitude[1] 同约定,抬头时为负)
    yaw   逆时针正(ENU 数学角,0=东;接收端已做 +90° 参考角旋转)
  gyro 为 FLU 角速度(rad/s -> deg/s),gyro_y 与 d(pitch)/dt 同号(链路健康时)。

台架判据(手持慢动作即可):
    机头抬起  -> pitch 变负,gyro_y 为负,dP/dt 为负(三者同号 = 链路健康)
    机头下压  -> 三者同时变正
    右滚      -> roll 正
    俯视逆时针转 -> yaw 增大

用法:
    source install/setup.bash
    python3 src/apm_bridge/scripts/imu_euler_monitor.py [--topic /mavros/imu/data] [--rate 10]
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


def quat_to_zyx_euler(w: float, x: float, y: float, z: float):
    """四元数 -> ZYX 欧拉角(rad),与 tf2 getRPY / Betaflight imu.c 同代数。

    pitch = asin(2(wy-xz))  (低头正,FLU 右手系)
    """
    s = math.sqrt(w * w + x * x + y * y + z * z)
    if s < 1e-9:
        return None
    w, x, y, z = w / s, x / s, y / s, z / s

    sinp = 2.0 * (w * y - x * z)
    sinp = max(-1.0, min(1.0, sinp))
    roll = math.atan2(2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class ImuEulerMonitor(Node):
    def __init__(self, topic: str, rate: float):
        super().__init__("imu_euler_monitor")
        self._period = 1.0 / rate
        self._last_print = 0.0

        # 数值微分状态:d(pitch)/dt 与 gyro_y 交叉验证
        self._last_pitch = None
        self._last_t = None
        self._dpitch_dt = float("nan")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 发布端是 best_effort,必须匹配
        )
        self.create_subscription(Imu, topic, self.on_imu, qos)
        print(f"[imu_euler_monitor] 订阅 {topic} (best_effort)")
        print("约定: roll 右滚+ | pitch 低头+ (抬头为负) | yaw 逆时针+ (ENU, 0=东)")
        print("判据: 机头抬起时 pitch / gyro_y / dP·dt⁻¹ 三者应同时为负\n")

    def on_imu(self, msg: Imu):
        now = self.get_clock().now().nanoseconds * 1e-9

        rpy = quat_to_zyx_euler(
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        )
        if rpy is None:
            return
        roll, pitch, yaw = rpy

        # d(pitch)/dt:用消息时间戳(接收端为 hostUs,单调);跨 ±90° 奇异跳变不采信
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:
            t = now
        if self._last_pitch is not None and t > self._last_t:
            dp = pitch - self._last_pitch
            dt = t - self._last_t
            if abs(dp) < math.radians(30.0):  # 滤掉过顶/奇异处的欧拉折返
                self._dpitch_dt = math.degrees(dp) / dt
        self._last_pitch, self._last_t = pitch, t

        if now - self._last_print < self._period:
            return
        self._last_print = now

        gx = math.degrees(msg.angular_velocity.x)
        gy = math.degrees(msg.angular_velocity.y)
        gz = math.degrees(msg.angular_velocity.z)
        dp = self._dpitch_dt

        line = (
            f"roll={math.degrees(roll):+08.2f}°  "
            f"pitch={math.degrees(pitch):+08.2f}° (低头+)  "
            f"yaw={math.degrees(yaw):+08.2f}°  |  "
            f"gyro=[{gx:+7.2f},{gy:+7.2f},{gz:+7.2f}]°/s  "
            f"dP/dt={dp:+7.2f}°/s"
        )
        sys.stdout.write("\r\033[K" + line)
        sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/mavros/imu/data")
    parser.add_argument("--rate", type=float, default=10.0, help="显示刷新率 Hz")
    args = parser.parse_args()

    rclpy.init()
    node = ImuEulerMonitor(args.topic, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write("\n")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
