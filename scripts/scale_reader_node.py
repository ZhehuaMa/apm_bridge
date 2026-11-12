#!/usr/bin/python3
"""DT30X 台秤串口读取节点 (kestrel_utils/scale_reader.py 的 ROS2 移植)。

协议: 9600 8N1, 行协议 "<weight> <status> <unit> [?...]"；
'?' 出现表示秤在运动中 (稳定值置 NaN)。连接后写 '0S' 启动、'CP' 连续输出，
退出时写 '0P' 停止。发布 /current_weight (std_msgs/Float32MultiArray):
data = [weight_kg, stable_weight_kg]  # stable 为 NaN 表示未稳定

串口打开失败 (含 USB 适配器拔出/挂死时的 EIO) 周期重试，流中断后自动重连；
适配器重插后无需重启节点。

用法 (需系统 python, conda 环境请先 deactivate):
  ros2 run apm_bridge scale_reader_node --ros-args -p port:=/dev/ttyUSB0
"""

import signal
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import serial
import serial.threaded
from std_msgs.msg import Float32MultiArray

# 连接后 '0S' -> 'CP' 的间隔 (同原版)
STARTUP_DELAY = 0.5
# 串口打开失败重试间隔
RETRY_INTERVAL = 5.0


class ScaleProtocol(serial.threaded.LineReader):
    """DT30X 行协议解析，直接在 reader 线程里发布。"""

    def __init__(self, publisher):
        super().__init__()
        self.publisher = publisher

    def connection_made(self, transport):
        super().connection_made(transport)

    def handle_line(self, line):
        line = line.strip()
        if len(line) == 0:
            return
        if line == 'OK!' or line == 'ES':
            return
        try:
            parts = line.split(' ')
            weight = float(parts[0])
            if parts[2] == 'g':
                weight *= 0.001
            if '?' in parts[2:]:
                stable_weight = float('nan')
            else:
                stable_weight = weight
            self.publisher.publish(Float32MultiArray(data=[weight, stable_weight]))
        except (IndexError, ValueError):
            return

    def connection_lost(self, exc):
        if exc:
            print(f'scale port error: {exc}')
        print('scale port closed')


class ScaleReaderNode(Node):

    def __init__(self):
        super().__init__('scale_reader')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)

        self.weight_pub = self.create_publisher(Float32MultiArray, '/current_weight',
                                                QoSProfile(depth=10))
        self.stop_event = threading.Event()
        self.comm = None
        self.thread = None

    def open_scale(self):
        """打开串口并启动连续输出；失败则周期重试直到 stop_event。

        注意 SerialException 是 OSError 的子类，而 USB 适配器拔出/挂死时
        open 会在 DTR/RTS ioctl 处抛原生 OSError(EIO) (pyserial 不包装)，
        故这里统一捕获 OSError。
        """
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        while not self.stop_event.is_set():
            try:
                self.comm = serial.Serial(port, baudrate, timeout=1,
                                          parity=serial.PARITY_NONE)
                self.thread = serial.threaded.ReaderThread(
                    self.comm, lambda: ScaleProtocol(self.weight_pub))
                self.thread.start()
                self.thread._connection_made.wait()
                if not self.thread.alive:
                    raise serial.SerialException('connection lost right after open')
                protocol = self.thread.protocol
                protocol.write_line('0S')
                self.stop_event.wait(STARTUP_DELAY)
                protocol.write_line('CP')
            except (OSError, RuntimeError, AttributeError) as e:
                # OSError: 串口打开/读写失败 (含 USB 挂死的原生 EIO);
                # RuntimeError: ReaderThread.start() 失败 (线程耗尽);
                # AttributeError: reader 线程刚退出, protocol 已置 None
                self.get_logger().error(f'cannot open {port}: {e!r}, '
                                        f'retry in {RETRY_INTERVAL:.0f}s')
                self.close_scale()
                self.stop_event.wait(RETRY_INTERVAL)
                continue
            self.get_logger().info(f'scale streaming on {port} @ {baudrate}')
            return True
        return False

    def close_scale(self):
        """停止输出并关闭串口 (幂等)。"""
        if self.thread is not None:
            try:
                self.thread.protocol.write_line('0P')
            except Exception:
                pass
            try:
                self.thread.close()  # 线程从未 start 成功时 join() 会抛 RuntimeError
            except Exception:
                pass
            self.thread = None
        if self.comm is not None:
            try:
                self.comm.close()
            except Exception:
                pass
            self.comm = None


def main():
    rclpy.init()
    node = ScaleReaderNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    def _spin():
        try:
            executor.spin()
        except rclpy.executors.ExternalShutdownException:
            pass  # rclpy.shutdown() 打断 spin 属正常关停路径
        except KeyboardInterrupt:
            pass

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()
    # SIGTERM/SIGINT 都走干净关停，保证向秤写 '0P'
    signal.signal(signal.SIGTERM, lambda *_: node.stop_event.set())

    opened = False
    try:
        while not node.stop_event.is_set():
            if not node.open_scale():
                break
            opened = True
            # 监视流是否断开 (ReaderThread 死亡)，断开则重连
            while node.thread is not None and node.thread.alive \
                    and not node.stop_event.is_set():
                node.stop_event.wait(0.5)
            if node.stop_event.is_set():
                break
            node.get_logger().error('scale stream lost, '
                                    f'reconnect in {RETRY_INTERVAL:.0f}s')
            node.close_scale()
            node.stop_event.wait(RETRY_INTERVAL)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_scale()
        node.get_logger().info('scale reader exiting')
        executor.remove_node(node)
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
    return 0 if opened else 1


if __name__ == '__main__':
    import sys
    sys.exit(main())
