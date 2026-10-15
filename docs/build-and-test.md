# Build and test architectures

The default development image is `drone-tracker-rk3576:humble-amd64-rknn2.3.2` (`linux/amd64`). `apm_bridge` does not require an ARM64 container. Image architecture identifies the tools running in Docker; output architecture identifies the machine that can run the compiled program.

| Mode | Output | Checks and boundaries |
| --- | --- | --- |
| Native amd64 image | x86_64 `custom_link_bridge_node`, `gimbal_selftest` and message libraries | Real ROS dependencies; C200T/C40D/C20S virtual-serial protocol simulation verified. |
| Cross compiler in amd64 image | ARM64 versions of both executables and messages | Clean build, target dependency resolution and all three virtual-serial protocol simulations verified on 2026-10-15. Running needs ARM64 Linux or QEMU. |
| Explicit ARM64 image | ARM64 versions of both executables and messages | Existing build and virtual-serial simulation verified under QEMU; hardware remains separate. |

Always build the actual adjacent `quadrotor_msgs` source for the same target. Runtime release files in `dist/` cannot replace this development dependency. No registered CTest/colcon test cases exist in these two packages; zero discovered tests is not a passing regression suite.

## Native build

```bash
# Run on the development host. The source mounts are read-only.
mkdir -p /tmp/apm_bridge-native
docker run --rm --network none --platform linux/amd64 \
  --user "$(id -u):$(id -g)" \
  -v /home/mzh/nickluo/apm_bridge:/src/apm_bridge:ro \
  -v /home/mzh/nickluo/quadrotor_msgs:/src/quadrotor_msgs:ro \
  -v /tmp/apm_bridge-native:/out \
  drone-tracker-rk3576:humble-amd64-rknn2.3.2 bash -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    mkdir -p /out/ws/src
    ln -sfn /src/apm_bridge /out/ws/src/apm_bridge
    ln -sfn /src/quadrotor_msgs /out/ws/src/quadrotor_msgs
    cd /out/ws
    CMAKE_BUILD_PARALLEL_LEVEL=4 colcon --log-base /out/ws/log build \
      --packages-up-to apm_bridge --executor sequential \
      --build-base /out/ws/build --install-base /out/ws/install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
    source /out/ws/install/setup.bash
    file /out/ws/install/apm_bridge/lib/apm_bridge/*
    python3 /src/apm_bridge/test/gimbal_pysim_test.py \
      /out/ws/install/apm_bridge/lib/apm_bridge/gimbal_selftest
  '
```

The last command runs the existing protocol simulation in the same native container. This uses a pseudo-terminal, a virtual serial connection. It does not test physical wiring or firmware. `client_timesync_selftest.cpp` is a separate directly compiled regression; production serial and ROS telemetry checks are described in [production deployment](production-deployment.md).

## ARM64 output from the default image

Cross-compilation runs x86_64 compilers that generate ARM64 binaries. The toolchain selects real target headers and libraries under `/opt/arm64-sysroot`; setting `CMAKE_SYSTEM_PROCESSOR` alone would not achieve this.

```bash
mkdir -p /tmp/apm_bridge-cross
docker run --rm --network none --platform linux/amd64 \
  --user "$(id -u):$(id -g)" \
  -v /home/mzh/nickluo/apm_bridge:/src/apm_bridge:ro \
  -v /home/mzh/nickluo/quadrotor_msgs:/src/quadrotor_msgs:ro \
  -v /tmp/apm_bridge-cross:/out \
  drone-tracker-rk3576:humble-amd64-rknn2.3.2 bash -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cmake -S /src/quadrotor_msgs -B /out/build/quadrotor_msgs \
      -DCMAKE_TOOLCHAIN_FILE=/opt/toolchains/aarch64-humble.cmake \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/out/install/quadrotor_msgs
    cmake --build /out/build/quadrotor_msgs -j4
    cmake --install /out/build/quadrotor_msgs
    cmake -S /src/apm_bridge -B /out/build/apm_bridge \
      -DCMAKE_TOOLCHAIN_FILE=/opt/toolchains/aarch64-humble.cmake \
      -DCMAKE_BUILD_TYPE=Release \
      -Dquadrotor_msgs_DIR=/out/install/quadrotor_msgs/share/quadrotor_msgs/cmake \
      -DCMAKE_INSTALL_PREFIX=/out/install/apm_bridge
    cmake --build /out/build/apm_bridge -j4
    cmake --install /out/build/apm_bridge
    file /out/install/apm_bridge/lib/apm_bridge/*
  '
```

Do not source the ARM install into native Python. To inspect target dependencies inside the amd64 container:

```bash
/opt/qemu-arm64/bin/qemu-aarch64 -L /opt/arm64-sysroot \
  /opt/arm64-sysroot/lib/ld-linux-aarch64.so.1 \
  --library-path /out/install/quadrotor_msgs/lib:/opt/arm64-sysroot/opt/ros/humble/lib:/opt/arm64-sysroot/usr/lib/aarch64-linux-gnu \
  --list /out/install/apm_bridge/lib/apm_bridge/custom_link_bridge_node
```

The loader list must resolve the installed ARM message libraries and all target dependencies. Compilation and loader inspection do not prove Betaflight/custom-link, gimbal or flight-controller behavior.

## Fallback and evidence

For an explicit ARM64-image fallback, take the native recipe, change `--platform linux/amd64` to `--platform linux/arm64`, and select `drone-tracker-rk3576:humble-arm64-rknn2.3.2`. Use a separate host output directory such as `/tmp/apm_bridge-arm64-image`. That compiler produces ARM64 output without the cross toolchain. On this x86_64 host its compiler and test programs run through QEMU, an ARM instruction emulator. A failed build or test is not a reason to switch architecture silently.

Fresh cross-build commands, logs, ARM64 ELF machine checks and complete loader lists are retained under `/tmp/nickluo-submodule-image-audit-20261015` (`cross-ros.sh`, `logs/`). The unchanged Python protocol simulator also ran against the genuine cross-built ARM harness under the explicitly selected QEMU and passed all three models. The wrapper only launches the ARM executable; it changes no simulator assertions. ELF is Linux's executable-file format; its machine field identifies ARM64 or x86_64 output. Earlier native and ARM-image evidence is linked in the parent workspace's `docs/amd64-build-validation-20261015.md`. No new device access or deployment was performed.
