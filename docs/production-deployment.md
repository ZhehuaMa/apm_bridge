# 生产开发板部署与验收

> 2026-10-15 开发环境迁移说明：源码位于 `mzh:/home/mzh/nickluo/apm_bridge`，见 `AGENTS.local.md`。本文的生产板路径与既往部署记录保留原义。`firefly-production` 是原工作站的 SSH 别名，远程开发机尚未配置；部署前须先验证访问配置。

## 环境与发布内容

- SSH：`ssh firefly-production`，当前地址为 `root@192.168.77.218`。
- 系统：Ubuntu 22.04，aarch64，ROS 2 Humble。
- 发布目录：`/opt/apm_bridge/releases/76fd1ba-20260923`。
- 当前版本链接：`/opt/apm_bridge/current`。
- 源码基线：`76fd1ba`，另包含本次对时修复和验收工具。修复尚未提交 Git。
- 消息依赖工作区：`/home/hzhy/workspace/fpv_ws`，提供 `quadrotor_msgs` 自定义 ROS 消息。
- 飞控：`/dev/ttyS8`，921600 波特率。
- 云台：`/dev/ttyS10`，115200 波特率，C20S。

飞控需要运行启用 `USE_CUSTOM_LINK` 的定制 Betaflight 固件。飞控实际接线 UART 必须配置为 `CUSTOM_LINK` 功能，并与开发板使用相同波特率。

## Build architecture and historical board procedure

An ARM64 container is not required to compile this package. The current default x86_64 image can build native host tests and cross-compile both ARM64 executables with the real ARM64 `quadrotor_msgs` dependency. See [build and test instructions](build-and-test.md). The board build below remains a supported alternative and preserves the dated production procedure. Cross-build/link checks do not replace the physical protocol and ROS telemetry acceptance below.

## 1. 上传源码

在本地仓库根目录执行。后续新版本应改用新的发布目录，保留旧版以便回退。

```bash
tar -czf /tmp/apm-bridge-source.tar.gz \
  CMakeLists.txt package.xml include src launch parameters test deploy docs
ssh firefly-production \
  'mkdir -p /opt/apm_bridge/releases/76fd1ba-20260923/src/apm_bridge && tar -xzf - -C /opt/apm_bridge/releases/76fd1ba-20260923/src/apm_bridge' \
  < /tmp/apm-bridge-source.tar.gz
```

## 2. 在开发板构建

后续命令均在开发板执行。使用系统 ROS 对应的工具链和 Python 环境；板上未配置 conda。单任务构建用于降低内存峰值。

```bash
source /opt/ros/humble/setup.bash
source /home/hzhy/workspace/fpv_ws/install/setup.bash
cd /opt/apm_bridge/releases/76fd1ba-20260923
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build \
  --packages-select apm_bridge --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG'
source install/local_setup.bash
```

成功标志为 `1 package finished`。必须加载上述消息依赖工作区，仅加载基础 ROS 环境会找不到 `quadrotor_msgs`。

## 3. 检查连接并验收飞控协议

确认飞控与云台均已供电，串口 TX/RX 交叉连接且共地。同一串口只能由一个测试或桥接进程使用。

```bash
ls -l /dev/ttyS8 /dev/ttyS10
# 已安装服务时先停止，以释放串口。
systemctl stop apm-bridge
fuser /dev/ttyS8 /dev/ttyS10
python3 src/apm_bridge/test/hw_custom_link_test.py \
  --port /dev/ttyS8 --baud 921600 --duration 20 --inverted
```

`--inverted` 仅用于已确认倒放的飞控，仍会检查姿态范围。正放时去掉该参数。本次用户确认飞控倒放，使用约 12 V 适配器供电，没有接电池，因此没有更改生产配置中的 6S 电池参数。

测试只接收遥测并发送对时请求，不发送解锁和飞行控制帧。预期结果：

- 三类遥测约 200、100、10 Hz，序号连续。
- 对时往返延迟中位数小于 20 ms，消息时间偏差小于 50 ms。
- 静止传感器数值与放置姿态一致，飞控未解锁、无失效保护。
- 最终输出 `RESULT: PASS`。失败时先处理具体失败项，再恢复服务并复测。

## 4. 安装并启动服务

首次部署创建当前版本链接；链接已存在时检查其目标，避免无意切换版本。

```bash
readlink -f /opt/apm_bridge/current
# 仅首次部署执行：
# ln -s /opt/apm_bridge/releases/76fd1ba-20260923 /opt/apm_bridge/current
cp /opt/apm_bridge/current/src/apm_bridge/deploy/apm-bridge.service \
  /etc/systemd/system/apm-bridge.service
systemctl daemon-reload
systemctl start apm-bridge
systemctl status apm-bridge --no-pager
```

启动脚本明确指定生产串口，并保持原 launch 的话题映射。不要直接使用原 launch 的默认参数：默认飞控端口 `/dev/ttyTHS1` 和空云台端口会覆盖 YAML 中的正确设置。

服务通过 `LogsDirectory=apm_bridge` 创建日志目录，并设置 `ROS_LOG_DIR=/var/log/apm_bridge`。这是必要配置：systemd 环境缺少用户主目录信息时，ROS 曾因日志目录无法确定而退出。

脚本直接执行节点，使 systemd 能检测节点退出并按 `Restart=on-failure` 重启。

## 5. 验证真实 ROS 数据

```bash
source /home/hzhy/workspace/fpv_ws/install/setup.bash
source /opt/apm_bridge/current/install/local_setup.bash
python3 /opt/apm_bridge/current/src/apm_bridge/test/ros_telemetry_test.py --duration 60
```

此测试只订阅数据，不发布控制指令。它验证六个话题持续更新、发布频率正确、消息时间戳与开发板当前时间相差小于 50 ms，以及飞控已连接且未解锁。预期最终输出 `RESULT: PASS`。

看到节点名称或 `Custom link up` 日志不等于验收通过；串口打开和有效遥测到达是不同条件。云台数据也必须结合实际串口回包和状态日志确认。

## 6. 日常运行与开机启动

完整验收通过后启用开机启动：

```bash
systemctl enable apm-bridge
systemctl is-enabled apm-bridge
systemctl is-active apm-bridge
journalctl -u apm-bridge -n 50 --no-pager
```

常用管理命令：`systemctl restart apm-bridge` 和 `systemctl stop apm-bridge`。

需要前台排查时，先停服务，避免两个进程竞争串口：

```bash
systemctl stop apm-bridge
APM_BRIDGE_RELEASE=/opt/apm_bridge/current \
  bash /opt/apm_bridge/current/src/apm_bridge/deploy/run-production.sh
# 前台退出后恢复服务：
# systemctl start apm-bridge
```

## 排障与本次修复

### 完全没有飞控遥测

执行 `cat /proc/tty/driver/serial`。如果串口 8 的 `rx` 不增长，检查供电、接线、UART 选择及飞控固件配置。本次首次验收的 0 Hz 原因为飞控未供电，上电后通信恢复。

### 遥测正常但时间戳越来越超前

本次实测原版本消息时间戳超前约 0.3 秒，并持续增长。原对时逻辑将偏置变化视为异常，导致后续有效样本持续被拒绝。现改为使用通过往返延迟检查的最新偏置，避免持续漂移造成永久失准。

回归测试模拟持续漂移与时钟跳变：旧代码偏差约 418 ms，修复后通过 15 ms 门限。运行方法：

```bash
cd /opt/apm_bridge/current/src/apm_bridge
g++ -std=c++17 -pthread -I include \
  test/client_timesync_selftest.cpp src/custom_link/client.cpp \
  -o /tmp/client_timesync_selftest
/tmp/client_timesync_selftest
```

### ROS CLI 后台缓存异常

若 CLI 报 `xmlrpc.client.Fault` 和 `!rclpy.ok()`，可绕过后台缓存：

```bash
ros2 topic echo /fpv/gimbal sensor_msgs/msg/Imu --once --no-daemon
```

### 回退此次对时修复

修复前源码和节点二进制已保存于 `/opt/apm_bridge/backups/pre-timesync-fix.tar.gz`。如需回退，停止服务后恢复，再启动：

```bash
systemctl stop apm-bridge
tar -xzf /opt/apm_bridge/backups/pre-timesync-fix.tar.gz -C /opt/apm_bridge/current
systemctl start apm-bridge
```

该回退会恢复原有时间戳缺陷，只作为紧急恢复手段。

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
