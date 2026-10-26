# Build and test architectures

The default development image is `drone-tracker-rk3576:humble-amd64-rknn2.3.2` (`linux/amd64`). `apm_bridge` does not require an ARM64 container. Image architecture identifies the tools running in Docker; output architecture identifies the machine that can run the compiled program.

| Mode | Output | Checks and boundaries |
| --- | --- | --- |
| Native amd64 image | x86_64 `custom_link_bridge_node`, `gimbal_selftest` and message libraries | Real ROS dependencies; current test results are recorded below. |
| Cross compiler in amd64 image | ARM64 versions of both executables and messages | Clean build, target dependency resolution and all three virtual-serial protocol simulations verified on 2026-10-26. Running needs ARM64 Linux or QEMU. |
| Explicit ARM64 image | ARM64 versions of both executables and messages | Historical build and virtual-serial simulation verified under QEMU; hardware remains separate. |

Always build the actual adjacent `quadrotor_msgs` source for the same target,
including its `CliCommand` service. Runtime release files in `dist/` cannot
replace this development dependency. Use ROS 2 Humble and C++17.

With `BUILD_TESTING=ON`, `apm_bridge` registers eight CTest groups. CTest is
CMake's test runner; colcon invokes it and collects the results. The message
package has no registered tests. Zero discovered tests is not a passing
regression suite.

## Native build and tests

```bash
# Run on the development host. The source mounts are read-only.
mkdir -p /tmp/apm_bridge-native
docker run --rm --network none --platform linux/amd64 \
  --user "$(id -u):$(id -g)" \
  -e PYTHONDONTWRITEBYTECODE=1 -e ROS_LOG_DIR=/out/ros-log \
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
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DAPM_BRIDGE_AIRSIM=OFF
    source /out/ws/install/setup.bash
    file /out/ws/install/apm_bridge/lib/apm_bridge/*
    ctest --test-dir /out/ws/build/apm_bridge -N
    test_status=0
    colcon --log-base /out/ws/log test --packages-select apm_bridge \
      --build-base /out/ws/build --install-base /out/ws/install \
      --event-handlers console_direct+ --return-code-on-test-failure || test_status=$?
    colcon --log-base /out/ws/log test-result \
      --test-result-base /out/ws/build --verbose || test_status=$?
    APM_BRIDGE_TEST_BINARY=/out/ws/install/apm_bridge/lib/apm_bridge/custom_link_bridge_node \
      PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider \
      /src/apm_bridge/test/cli_service_test.py -q || test_status=$?
    exit "$test_status"
  '
```

`ctest -N` should list the following groups. Each Python group can contain
multiple test cases; a group count is not the number of assertions or cases.

| CTest group | Coverage |
| --- | --- |
| `gimbal_tracking_test` | Line-of-sight (LOS) tracking math and rate limits |
| `sitl_bridge_test` | Launch parameters, motor fitting and vertical estimator |
| `bench_pid_test` | Tuning logic and ROS interactions with simulated telemetry and services |
| `bench_angle_link_test` | Attitude commands through the real bridge with a fake FC |
| `gimbal_node_test` | Mission/RC behavior with a fake FC and virtual serial gimbal |
| `gimbal_protocol_test` | C200T/C40D/C20S virtual-serial protocol simulation |
| `sitl_relay_test` | TCP frame forwarding between simulator, bridge and FC |
| `gimbal_interface_test` | Hardware/strapdown lifecycle and orientation; camera RPC when enabled |

The final pytest command runs the separate CLI service regression, which is
not registered as its own CTest group. `APM_BRIDGE_TEST_BINARY` selects its
bridge executable. CMake supplies the equivalent path for `gimbal_node_test`
and `GIMBAL_SELFTEST` for the registered protocol simulation.

The protocol simulation can also be run directly in the sourced container:

```bash
python3 /src/apm_bridge/test/gimbal_pysim_test.py \
  /out/ws/install/apm_bridge/lib/apm_bridge/gimbal_selftest
```

It uses a pseudo-terminal, a virtual serial connection. It does not test
physical wiring or firmware. Known source/test mismatches are listed below;
do not treat the build recipe as a claim that all current tests pass.

## Manual integration and time-sync checks

Run these from the package directory in a sourced ROS 2 Humble environment.
Firmware tests need a matching external Betaflight SITL executable; AirSim
flight tests additionally need the live simulator and camera RPC build.

```bash
python3 test/sitl_custom_link_test.py --binary /path/to/betaflight_SITL.elf
python3 test/sitl_odometry_test.py --binary /path/to/betaflight_SITL.elf
```

These tests start firmware processes and use TCP/UDP ports. Stop other SITL
sessions first; see each script for its prerequisites. For live camera flight
validation, follow the dated procedure and prerequisites in [README](../README.md).

`test/timesync_drift_test.py` needs a real FC serial link. It captures clock
exchanges and compares the old and improved algorithms through Python replay,
including wall-clock steps and FC reboot injection. It reports drift metrics;
successful exit after capture does not establish a numerical acceptance pass.
Its `A current` output labels refer to the older algorithm; production
`src/custom_link/client.cpp` now uses the improved model. There is no
`test/client_timesync_selftest.cpp` in the current checkout.
See [production deployment](production-deployment.md) for serial commands and
ROS telemetry checks.

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
      -DBUILD_TESTING=OFF \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/out/install/quadrotor_msgs
    cmake --build /out/build/quadrotor_msgs -j4
    cmake --install /out/build/quadrotor_msgs
    cmake -S /src/apm_bridge -B /out/build/apm_bridge \
      -DCMAKE_TOOLCHAIN_FILE=/opt/toolchains/aarch64-humble.cmake \
      -DBUILD_TESTING=OFF -DAPM_BRIDGE_AIRSIM=OFF \
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

The loader list must resolve the installed ARM message libraries and all target
dependencies. This cross-build disables tests explicitly and builds the bridge
and protocol harness. Native tests do not establish ARM runtime behavior;
run target tests on ARM64 or through an explicitly configured emulator.
Compilation and loader inspection do not prove physical device behavior.

## Package an ARM64 runtime

Package the installed application and messages from the same ARM64 build.
The collector reads these files; it does not compile them. It verifies that
the collected Linux executables and Python extensions target ARM64.
It can run on the x86_64 host or in the default image. This example collects
the cross-build above with read-only input mounts:

```bash
mkdir -p /tmp/apm_bridge-packages
docker run --rm --network none --platform linux/amd64 \
  --user "$(id -u):$(id -g)" \
  -e PYTHONDONTWRITEBYTECODE=1 \
  -e APM_BRIDGE_SOURCE_COMMIT="$(git -C /home/mzh/nickluo/apm_bridge rev-parse HEAD)" \
  -v /home/mzh/nickluo/apm_bridge:/src/apm_bridge:ro \
  -v /tmp/apm_bridge-cross/install:/inputs:ro \
  -v /tmp/apm_bridge-packages:/packages \
  drone-tracker-rk3576:humble-amd64-rknn2.3.2 bash -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    python3 /src/apm_bridge/deploy/binary/collect_runtime.py \
      --templates /src/apm_bridge/deploy/binary \
      --app-prefix /inputs/apm_bridge \
      --messages-prefix /inputs/quadrotor_msgs \
      --output /packages \
      --source-commit "$APM_BRIDGE_SOURCE_COMMIT"
  '
```

The package includes `custom_link_bridge_node`, the installed
`bench_pid_tuner` and its Python helpers, both parameter files, and the
custom message C/C++ libraries and Python bindings. The Python bindings are
generated code and compiled extensions that let the tuner use custom ROS
messages and services. ROS 2 Humble, system Python 3.10, `rclpy`, NumPy and
PyYAML remain target runtime prerequisites. On the target, `install.sh --check`
validates dynamic-library dependencies and Python imports against those
installed runtimes before deployment.
The launcher supervises both processes: if either exits unexpectedly, it
stops the companion and returns failure so systemd can restart them together.
All five bridge topic remappings from the source launch are retained.

The tuner is packaged and started with both `enabled=false` and
`rig_confirmed=false` by default. Append `--enable-bench-pid` to the collector
command to set both packaged flags to true. This does not start a tuning
trial; the tuner's existing telemetry checks and RC switch sequence still
apply. To override a foreground launch, use `run.sh --enable-bench-pid` or
`run.sh --disable-bench-pid`. For the installed service, set the single
`BENCH_PID_ENABLED=true|false` switch in `/etc/apm_bridge/runtime.env` and
restart `apm-bridge`. The launcher applies that value to both ROS parameters,
including when it overrides a custom tuner YAML.

See [production deployment](production-deployment.md#binary-release-deployment)
for installation, configuration backups and target acceptance. Packaging
checks do not replace the separate native tests or physical acceptance.

## Fallback and evidence

For an explicit ARM64-image fallback, take the native recipe, change `--platform linux/amd64` to `--platform linux/arm64`, and select `drone-tracker-rk3576:humble-arm64-rknn2.3.2`. Use a separate host output directory such as `/tmp/apm_bridge-arm64-image`. That compiler produces ARM64 output without the cross toolchain. On this x86_64 host its compiler and test programs run through QEMU, an ARM instruction emulator. A failed build or test is not a reason to switch architecture silently.

The 2026-10-15 audit recorded cross-build commands, ARM64 ELF machine checks
and loader lists under `/tmp/nickluo-submodule-image-audit-20261015`
(`cross-ros.sh`, `logs/`). That temporary directory is no longer present on this
host. The audit also reported all three protocol models passing against the
cross-built ARM harness through QEMU. ELF is Linux's executable-file format;
its machine field identifies ARM64 or x86_64 output. The parent workspace's
`docs/submodule-image-audit-20261015.md` and
`docs/amd64-build-validation-20261015.md` retain the historical summaries.
These dated results do not certify the current checkout or physical devices.

## Historical native verification (2026-10-23)

A clean native build of the adjacent messages and `apm_bridge` succeeded on
image `025d0aa88983b0646a1b7a4d25cbc13dc96a6206e0148475bbfd5fad15057421`.
It used read-only source mounts, external outputs, `BUILD_TESTING=ON`, and
`APM_BRIDGE_AIRSIM=OFF`. CTest discovered eight groups: five passed and three
failed. The failures are:

- `sitl_bridge_test`: the motor test expects a thrust limit of 0.5, but the
  current SITL YAML sets 1.0. Another case needs the absent
  `parameters/sitl_motor_calibration.csv` (17 cases passed, two failed).
- `bench_angle_link_test`: the script derives a fixed executable path from
  the source directory. With this layout it tries
  `/build/apm_bridge/custom_link_bridge_node`, instead of the executable in
  `/out/ws/build/apm_bridge`. It does not read `APM_BRIDGE_TEST_BINARY`.
- `gimbal_protocol_test`: its C200T golden comparison expected `op_value=-960`
  but observed `-300`. The test failed; all three models were not certified
  by this run.

`bench_pid_test` passed all 111 cases, and `gimbal_node_test` passed all four.
The three C++ groups and the two separate CLI service cases also passed.
Build/test logs are under
`/tmp/apm-bridge-cleanup-20261023-colvlfre/native/`; this temporary directory
may be removed by the operating system. These results do not include an
AirSim RPC build, new ARM64 validation, or physical device acceptance.

## Native and ARM64 release verification (2026-10-26)

Source branch `rk3576` at `ed0be957a1d393b06dda4776c7930a205bd9159c`
was rebuilt from clean output directories with adjacent `quadrotor_msgs` at
`bc8bf6cbf30f30593bf978d878ad448af5f9cc20`. The native build used the same
`025d0aa8...` image ID recorded above with `BUILD_TESTING=ON`. The ARM64
cross-build used its target toolchain with `BUILD_TESTING=OFF`. Both built
`custom_link_bridge_node` and `gimbal_selftest`, with camera RPC disabled.
Source mounts were read-only, and build, install and log outputs were outside
source. The installed tuner and message files came from these fresh builds.

The initial native CTest run discovered eight groups and passed six:

| Group | Result |
| --- | --- |
| `gimbal_tracking_test` | Passed |
| `sitl_bridge_test` | 17 cases passed; two failed for the thrust-limit expectation and missing CSV described above |
| `bench_pid_test` | All 111 cases passed |
| `bench_angle_link_test` | Failed to locate its hard-coded executable; its one case passed in a separate rerun with a matching read-only `/build` mount |
| `gimbal_node_test` | All four cases passed |
| `gimbal_protocol_test` | Passed |
| `sitl_relay_test` | Passed |
| `gimbal_interface_test` | Passed |

The separate CLI service regression passed both cases. The full native suite
is not reported as passing. The C200T protocol failure from 2026-10-23 was not
observed in this run. All three protocol models also passed separately with
the fresh ARM64 `gimbal_selftest` through QEMU 8.2.2.

ARM64 runtime checks verified executable/library architecture, dependency
resolution, `ControlCommand` and `LowLevelFeedback` serialization, and
`CliCommand` request/response serialization and ROS CLI discovery. The
release supervisor started both real ARM64 nodes, preserved all five bridge
topic remappings, and stopped both on normal shutdown or a child failure.
Startup used a virtual serial port with the absent gimbal and receiver
binding disabled. That particular release explicitly enabled the tuner with
both `enabled=true` and `rig_confirmed=true`; this is a release override of
the disabled source defaults. Its installer and supervisor were copied from
the previous deployment package, rather than generated by the repository's
then-current deployment tools.

The release report is
`dist/apm_bridge-ubuntu22.04-arm64-20261026-ed0be957a1d3-42896589-cpu0-3-performance-two-nodes-enabled.BUILD-RESULT.txt`.
Detailed logs remain under `/tmp/apm-bridge-release-20261026-Nt22FE/` and in
the release archive's `validation/` directory. Temporary host files can be
removed by the operating system. No board installation, systemd activation,
physical FC/gimbal check, serial time-sync drift check, firmware SITL test or
live AirSim camera/flight test ran. Emulated startup and protocol checks do
not establish physical device or flight acceptance.

## Deployment tooling verification (2026-10-26)

The maintained repository deployment tools were verified separately from
the earlier release above. Eight focused tests passed with:

```bash
python3 -B -m unittest discover -s deploy/binary/tests -v
```

All five deployment shell scripts passed Bash syntax checks. Default and
enabled bundles were collected from the same fresh ARM64 application and
message installs. With real ARM64 processes under QEMU, both bundles passed
library dependency checks, custom interface serialization and discovery,
two-node startup on CPUs 0–3, all five bridge remappings, clean shutdown,
and companion cleanup after a child failure. The default tuner's actual
parameters were `enabled=false` and `rig_confirmed=false`, with state
`DISABLED`. The enabled variant loaded both as true and reached `IDLE`.
The source deployment wrapper also started both native nodes with each
choice, using the release's local install without the historical message
workspace. `APM_MESSAGES_SETUP` is now an optional explicit override.

The actual installer passed `--check`, default installation and enabled
installation in a disposable ARM64 container. An injected service-start
failure restored all three configuration files, the service unit, release
link, service state and CPU/NPU performance settings. These transaction
checks used simulated UART devices and sysfs files (the kernel's hardware
settings interface), plus a `systemctl` stub that recorded service operations.
They establish installer behavior in
that controlled environment; no real board or systemd service was tested.

Evidence is under `/tmp/apm-bridge-deployment-fix-20261026-YyCXH6/`, including
`final/default-runtime.log`, `final/enabled-runtime.log`,
`source-wrapper-checks.log` and `install-checks.log`. The native CTest suite
was not rerun for this tooling change because C++, CMake and dependencies
were unchanged. Its earlier failures remain recorded above. Physical
devices, firmware, live AirSim and flight acceptance remain outside these
deployment checks.
