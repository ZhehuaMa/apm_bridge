# apm_bridge 项目约定

## 项目与依赖

- 本仓库是 ROS 2 Humble `ament_cmake` 包，使用 C++17。
- `quadrotor_msgs` 是必需的源码依赖，不包含在本仓库或基础镜像中。本机依赖仓库位于
  `${HOME}/projects/nickluo/quadrotor_msgs`。
- 标准构建环境使用 ARM64 镜像
  `drone-tracker-rk3576:humble-arm64-rknn2.3.2`。镜像已包含其余 ROS 依赖，包括
  `rclcpp`、`mavros_msgs`、`tf2` 和标准消息包。
- `dist/` 中的发布包只包含运行时文件，不能替代 `quadrotor_msgs` 的源码或开发安装前缀。

## Docker 构建验证

从本仓库根目录执行以下命令。将两个源码仓库只读挂载，并在容器临时目录中生成
`build`、`install` 和 `log`，不要把容器生成物写入源码目录。

```bash
docker run --rm \
  -v "$PWD:/workspace/apm_bridge:ro" \
  -v "$HOME/projects/nickluo/quadrotor_msgs:/workspace/quadrotor_msgs:ro" \
  drone-tracker-rk3576:humble-arm64-rknn2.3.2 \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    mkdir -p /tmp/ws/src
    ln -s /workspace/apm_bridge /tmp/ws/src/apm_bridge
    ln -s /workspace/quadrotor_msgs /tmp/ws/src/quadrotor_msgs
    cd /tmp/ws
    CMAKE_BUILD_PARALLEL_LEVEL=1 colcon --log-base /tmp/ws/log build \
      --packages-up-to apm_bridge \
      --executor sequential \
      --build-base /tmp/ws/build \
      --install-base /tmp/ws/install \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        "-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG"
    test -x /tmp/ws/install/apm_bridge/lib/apm_bridge/custom_link_bridge_node
    test -x /tmp/ws/install/apm_bridge/lib/apm_bridge/gimbal_selftest
  '
```

成功标志是 `Summary: 2 packages finished`，且末尾两个产物检查均通过。macOS 宿主机与
ARM64 容器之间偶尔会出现数秒级 `Clock skew detected` 警告；如果构建成功且产物检查
通过，该警告本身不是失败。出现实际编译错误时，不要忽略错误或用该警告解释失败。

## 修改与验证

- 修改 C++、CMake 或依赖声明后，至少运行上述完整 Docker 构建。
- `quadrotor_msgs` 的消息定义由相邻仓库维护；不要在本仓库复制生成的消息头文件或共享库。
- 生产板构建和部署细节见 `docs/production-deployment.md`。
