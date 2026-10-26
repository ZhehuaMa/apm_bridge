# Production deployment and acceptance

Use ROS 2 Humble on Ubuntu 22.04 ARM64 boards. Build and test instructions
for the native x86_64 image and ARM64 output are in
[build-and-test.md](build-and-test.md). A host build does not establish physical
serial, gimbal or flight-controller acceptance.

The source workspace is `/home/mzh/nickluo/apm_bridge` on `mzh`.
`firefly-production` is an SSH alias from the original workstation. This host
has no configured board access. The board address and paths below describe
the recorded deployment; verify them on the intended device before use.

## Device and configuration requirements

| Item | Recorded board setup |
| --- | --- |
| SSH | `firefly-production`, originally `root@192.168.77.218` |
| Flight controller (FC) | `/dev/ttyS8`, 921600 baud |
| Gimbal | `/dev/ttyS10`, 115200 baud, C20S |
| Source deployment link | `/opt/apm_bridge/current` |
| Historical message workspace | `/home/hzhy/workspace/fpv_ws` |

The FC needs matching Betaflight firmware built with `USE_CUSTOM_LINK`.
Assign its wired UART to `CUSTOM_LINK` and match the baud rate. Build the real
adjacent `quadrotor_msgs` source for the same architecture, including its
`CliCommand` service. A runtime bundle is not a development dependency.

Review the current `parameters/custom_link_parameters.yaml` against the device:
mass is 1.067 kg, gravity is 9.8065 m/s², the battery is 6S/1800 mAh, and the
thrust limit is 0.9. The mission trigger is CH10/AUX6 (zero-based index 9).
Gimbal AIM requests +75 degrees (`90 + pitch_aim`, with `pitch_aim=-15`);
`pitch_max=85`. These are shipped settings, not measurements of a new device.
The YAML also sets `cli.bind_rx_on_connect=true`, which starts receiver
pairing on every link connection. Use false for normal operation after pairing.

`custom_link.launch.py` overrides the YAML ports with its launch defaults:
`/dev/ttyTHS1` for the FC and an empty gimbal port. Empty selects a fixed camera.
Always pass the real ports when using that launch file. The current source
launch starts only the bridge. `bench_pid.launch.py` separately starts the
bench tuner and defaults both `enabled` and `rig_confirmed` to false.

## 1. Prepare a new source release

Use a new release directory and keep the previous one for rollback. The
following example stages both real packages. Run it on the development host
after confirming board access. Source files are only read during packaging.

```bash
cd /home/mzh/nickluo/apm_bridge
tar --exclude='__pycache__' --exclude='.pytest_cache' \
  -czf /tmp/apm-bridge-source.tar.gz \
  CMakeLists.txt package.xml include src launch parameters scripts test docs deploy
tar -C /home/mzh/nickluo/quadrotor_msgs \
  -czf /tmp/apm-bridge-messages.tar.gz CMakeLists.txt package.xml msg srv
ssh firefly-production \
  'mkdir -p /opt/apm_bridge/releases/20261023-source/src/apm_bridge /opt/apm_bridge/releases/20261023-source/src/quadrotor_msgs'
ssh firefly-production \
  'tar -xzf - -C /opt/apm_bridge/releases/20261023-source/src/apm_bridge' \
  < /tmp/apm-bridge-source.tar.gz
ssh firefly-production \
  'tar -xzf - -C /opt/apm_bridge/releases/20261023-source/src/quadrotor_msgs' \
  < /tmp/apm-bridge-messages.tar.gz
```

Replace `20261023-source` throughout with the new release ID if it already
exists. Including `scripts/` is required: CMake installs the calibration tool,
bench tuner and their Python helper modules.

## 2. Build on the board

Run on the board with its system ROS/Python environment. Build and install
outputs stay outside the package source directories. One compiler job limits
memory use. This example builds both packages rather than depending on a
possibly older installed message service.

```bash
source /opt/ros/humble/setup.bash
cd /opt/apm_bridge/releases/20261023-source
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build \
  --packages-up-to apm_bridge --executor sequential \
  --build-base build --install-base install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  '-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG' \
  -DBUILD_TESTING=OFF -DAPM_BRIDGE_AIRSIM=OFF
source install/setup.bash
```

Expect both `quadrotor_msgs` and `apm_bridge` to finish. This builds
`custom_link_bridge_node`, `gimbal_selftest`, and the installed Python tools.
Tests are disabled explicitly for this production build; it is not a test
pass. Run the separate development checks in [build-and-test.md](build-and-test.md).

## 3. Check the physical protocol

Power both devices, cross TX/RX and connect grounds. Only one process may own
a serial port. Stop any existing bridge service and other serial users first.
The test receives telemetry and sends time-sync requests; it sends no arming
or flight-control frames.

```bash
cd /opt/apm_bridge/releases/20261023-source
systemctl stop apm-bridge
ls -l /dev/ttyS8 /dev/ttyS10
fuser /dev/ttyS8 /dev/ttyS10
python3 src/apm_bridge/test/hw_custom_link_test.py \
  --port /dev/ttyS8 --baud 921600 --duration 20
```

The current script supports `--port`, `--baud` and `--duration`. It has no
`--inverted` option. Its attitude check requires roll/pitch magnitudes below
90 degrees, so the inverted-board procedure from the historical acceptance
cannot be reproduced with this script unchanged. Use a suitable upright test
setup or a separately reviewed orientation-aware checker; do not disregard
a failed attitude check.

Expect three telemetry streams near 200/100/10 Hz, no sequence gaps, median
time-sync round-trip latency below 20 ms, and absolute timestamp lag below
50 ms. The script checks stationary sensors, plausible pressure/temperature
and voltage, disarmed status and no failsafe. It prints `RESULT: PASS` or
`RESULT: FAIL` and returns nonzero on failure.

For a longer drift investigation, keep the bridge stopped and run:

```bash
python3 src/apm_bridge/test/timesync_drift_test.py \
  --port /dev/ttyS8 --baud 921600 --duration 180 \
  --csv /tmp/apm-timesync.csv
```

This captures real exchanges and replays old/improved algorithms in Python,
including injected clock steps and FC reboots. Its `A current` labels denote
the old algorithm. Current C++ uses a monotonic clock, a latency-filtered sample
window, a fitted clock offset/drift rate, and reset/relock logic. The script
reports metrics and returns success after sufficient capture; it does not
apply a numerical pass/fail threshold or directly execute the C++ model.
`test/client_timesync_selftest.cpp` is absent from this checkout.

## 4. Start the bridge and verify ROS data

For a foreground check, load the new release and launch with explicit ports:

```bash
source /opt/ros/humble/setup.bash
source /opt/apm_bridge/releases/20261023-source/install/setup.bash
ros2 launch apm_bridge custom_link.launch.py \
  serial_port:=/dev/ttyS8 baudrate:=921600 \
  gimbal_port:=/dev/ttyS10 gimbal_model:=C20S
```

Review the pairing setting before launching. This launch maps private topics
`~/control_command`, `~/control_command_raw`, `~/arm`, `~/offboard`, and
`~/low_level_feedback` to `/fpv/...`. A direct executable invocation must
supply these mappings itself.

In another sourced shell, inspect data using ROS 2 Humble CLI commands:

```bash
ros2 topic echo /mavros/state mavros_msgs/msg/State --once --no-daemon
ros2 topic echo /mavros/imu/data sensor_msgs/msg/Imu \
  --once --no-daemon --qos-reliability best_effort
ros2 topic echo /fpv/gimbal sensor_msgs/msg/Imu --once --no-daemon
ros2 topic hz /mavros/imu/data
ros2 topic delay /mavros/imu/data
```

Stop each continuous `hz` or `delay` command with Ctrl+C. Repeat for the topics
below, observing each for at least 60 seconds. The CLI's received rate can be
lower than the publisher rate under load. Subscribers must accept best-effort
sensor data; a reliable-only subscriber cannot match a best-effort publisher.

| Topic | Nominal rate with the recorded hardware |
| --- | ---: |
| `/mavros/imu/data` | 200 Hz |
| `/mavros/rc/in` | 100 Hz |
| `/fpv/gimbal` with a responding serial gimbal | About 100 Hz |
| `/mavros/state` | 10 Hz |
| `/mavros/battery` | 10 Hz |
| `/fpv/static_pressure` | 100 Hz |

Verify `connected=true`, `armed=false`, continuous fresh samples, and absolute
message timestamp differences below 50 ms. `topic delay` reports summary
statistics, not an automated assertion that every sample met that bound.
There is no current `test/ros_telemetry_test.py`; the historical six-topic
`PASS` report below came from an earlier acceptance tool. Record current CLI
observations separately rather than claiming that automated test ran.

`Custom link up` proves only that the transport opened. Confirm increasing
receive counts and actual gimbal status replies. A fixed camera publishes at
the vehicle IMU rate; this is not evidence of serial gimbal communication.
`/fpv/odom` requires an armed FC with valid GPS/origin/fusion flags, so no
odometry while disarmed is expected. See [README](../README.md) for the wire
contract and coordinate frames.

## 5. Existing services and rollback

The existing `deploy/` files describe two legacy bridge-only layouts:

- `deploy/apm-bridge.service` uses `/opt/apm_bridge/current` and
  `deploy/run-production.sh`. The script sources the historical message
  workspace and then the release install.
- `deploy/binary/` uses `/opt/apm_bridge-binary/current` and configuration
  under `/etc/apm_bridge`.

Both launch only the bridge and omit the `~/offboard` to `/fpv/offboard`
remapping present in the source launch. They do not implement the newer
two-node package supervisor. Review the selected service and its actual
configuration before reuse. Do not assume source launch behavior applies to
those scripts. Service templates are outside this documentation update.

Once a reviewed service points to the accepted release, stop foreground
processes before starting it. The existing templates set CPU affinity to 0–3,
`ROS_LOG_DIR=/var/log/apm_bridge`, `LogsDirectory=apm_bridge`, and
`Restart=on-failure`. Inspect the service and logs:

```bash
systemctl cat apm-bridge
systemctl status apm-bridge --no-pager
journalctl -u apm-bridge -n 50 --no-pager
systemctl show apm-bridge -p CPUAffinity -p NRestarts
```

Enable startup only after device and ROS acceptance. Record actual reboot
verification separately from configuration state:

```bash
systemctl enable apm-bridge
systemctl is-enabled apm-bridge
systemctl is-active apm-bridge
```

For rollback, stop the service and restore its previously accepted release,
configuration and matching message environment. The dated backup
`/opt/apm_bridge/backups/pre-timesync-fix.tar.gz` belongs to the 2026-09-23
incident; it is not a general rollback target for current releases.

## Historical deployment context (2026-09-23)

The acceptance below used release `/opt/apm_bridge/releases/76fd1ba-20260923`
with source baseline `76fd1ba` plus then-uncommitted time-sync changes and
acceptance tools. The FC was inverted and powered by an approximately 12 V
adapter without a battery; the configured 6S battery settings were retained.
Initial 0 Hz telemetry was traced to an unpowered FC. The original time-sync
regression reported approximately 418 ms error before its fix and passed a
15 ms limit afterward. These facts describe that deployment and its tools,
not the current checker or a new test run. The original board evidence follows
unchanged.

## 验收记录（2026-09-23）

- 原生 aarch64 构建成功，云台 C200T、C40D、C20S 协议仿真全部通过。
- 实物 C20S 持续回包，状态 `MANUAL`、温控就绪；姿态话题约 100 Hz。
- 飞控倒放、适配器供电的 20 秒协议测试全部通过。
- 实测三类遥测为 200.7、100.4、10.0 Hz，无序号缺口。
- 对时往返延迟中位数 18.29 ms；协议测试消息时间偏差最大 15.21 ms。
- 飞控未解锁、无失效保护；重力加速度约 9.77 m/s²，供电约 11.83 V。
- 对时回归在本机和开发板通过。静态检查仍报告两处未修改代码的历史警告：油门取整及解析器跨字段访问。仓库没有配置 clangd 检查命令。

本记录验证部署、设备通信及 ROS 数据链路。没有执行电机解锁或实际飞行测试；质量、推力模型、电池容量和云台行程仍应与飞行硬件匹配。

### 修复版最终验收

60 秒 ROS 验收六个话题全部 `PASS`：

| 话题 | 实测频率 | 消息时间偏差范围 |
| --- | ---: | ---: |
| `/mavros/imu/data` | 200.59 Hz | −6.37～27.63 ms |
| `/mavros/rc/in` | 100.43 Hz | −5.10～10.57 ms |
| `/fpv/gimbal` | 100.01 Hz | 1.32～11.53 ms |
| `/mavros/state` | 10.06 Hz | 1.12～12.93 ms |
| `/mavros/battery` | 10.06 Hz | −4.29～11.69 ms |
| `/fpv/static_pressure` | 100.41 Hz | −4.55～13.45 ms |

状态为 `connected=true`、`armed=false`、`mode=ANGLE`。负时间偏差表示消息时间戳略超前于接收端当前时间；本次所有样本均在绝对偏差 50 ms 的验收范围内。

原始结果保存于开发板 `/opt/apm_bridge/acceptance/ros-telemetry-20260923.json.log`。服务已通过完整数据链路验收，可启用开机启动；开机启动的配置状态与是否实际重启验证应分别记录。

最终服务状态：`enabled`、`active/running`，`NRestarts=0`。已启用开机启动，但本次未重启开发板；运行验收针对当前已启动的修复版进程。
