# Offline bridge and bench tuner deployment

This package runs `custom_link_bridge_node` (the bridge, sometimes called
`custom_link_node`) and `bench_pid_tuner` together under `apm-bridge.service`.
Both processes start even when tuning is disabled. By default the tuner loads
`enabled=false` and `rig_confirmed=false` and publishes its disabled status.

## Target requirements

Use a matching ARM64 board with Ubuntu 22.04, glibc 2.35, systemd, and CPU cores
0–3 available. The board must expose CPU and NPU frequency controls supporting
`performance`. Enable the flight-controller UART `/dev/ttyS8` and, when used,
gimbal UART `/dev/ttyS10` in the board device configuration first.

Install a matching ROS 2 Humble runtime under `/opt/ros/humble`, including
`rclcpp`, `rclpy`, Fast DDS, `mavros_msgs` and standard message libraries.
`/usr/bin/python3` must be Python 3.10 with NumPy and PyYAML available. The
package includes custom `quadrotor_msgs` libraries, generated Python bindings,
message/service definitions, and ROS resource indexes. It includes the tuner,
`bench_pid_core.py` and `sitl_support.py`. It does not bundle system or standard
ROS runtime libraries. The target needs no compiler, source workspace, or
internet access during installation.

## Create a package

First build and install the real adjacent `quadrotor_msgs` source and this
package for ARM64. Use separate target output directories as described in the
repository's `docs/build-and-test.md`. Run the collector against those installed
prefixes; `--app-prefix` and `--messages-prefix` are intentionally required.

From the repository root, for example:

```bash
python3 -B deploy/binary/collect_runtime.py \
  --app-prefix /tmp/apm-bridge-cross/install/apm_bridge \
  --messages-prefix /tmp/apm-bridge-cross/install/quadrotor_msgs \
  --output /tmp/apm-bridge-packages \
  --source-commit "$(git rev-parse HEAD)"
```

The default package disables both tuner flags. To explicitly enable them for
an approved captive rig, add `--enable-bench-pid` to that command. This sets
both `enabled=true` and `rig_confirmed=true` in the packaged tuner YAML and
sets `BENCH_PID_ENABLED=true` in its runtime environment. The source YAML
remains unchanged. All other RC, telemetry, ownership and tuning checks remain
active; enabling does not arm the flight controller or automatically start a
trial. Follow `config/bench_pid_README.md` before using the tuner.

The collector rejects non-ARM64 executable and message libraries, including
Python extensions. On an ARM64 host it also runs `ldd` to check dynamic library
resolution. On an x86_64 cross-build host it checks the ELF architecture only;
`manifest.json` records this scope. Always complete the target dependency
checks below. Do not interpret cross-packaging as target runtime acceptance.

Release names include a content identifier and `two-nodes-disabled` or
`two-nodes-enabled`. Changes to either node, configuration, custom interfaces,
or deployment scripts change that identifier. Existing releases are never
overwritten. The collector writes the release directory, its `.tar` archive,
a `.tar.sha256` sidecar, and an internal `SHA256SUMS` inventory.

## Install and start at boot

Copy and extract the entire archive on the board, then use its actual directory:

```bash
tar -xf apm_bridge-ubuntu22.04-arm64-<version>.tar
cd apm_bridge-ubuntu22.04-arm64-<version>
bash install.sh --check
sudo bash install.sh
```

`--check` verifies the OS, CPU availability, CPU/NPU policy support, checksums,
all application and message shared libraries, Python imports, and custom ROS
message/service serialization. It makes no system configuration changes and
does not test UART devices or physical communications. Normal installation
also checks that the configured serial devices exist.

The installer copies the release into `/opt/apm_bridge-binary/releases/`,
updates the `current` symlink, installs the systemd service, starts it and enables
boot startup. It checks that the service stays active without restarting.
It sets CPU and NPU frequency policies to `performance` and reapplies them
before every service start. Kernel thermal limits still apply.

Each installation replaces these files with the package defaults:

- `/etc/apm_bridge/parameters.yaml`: bridge configuration.
- `/etc/apm_bridge/bench_pid.yaml`: tuner configuration.
- `/etc/apm_bridge/runtime.env`: ports, communication domain and tuner switch.

The installer backs up all three files and the prior service under the printed
`/var/tmp/apm-bridge-backup.*` path. A failed installation restores the previous
configuration, service, release link, service enable/running state, and CPU/NPU
policies. Reinstalling an identical release validates its files and restarts
the service. A release name with different contents is rejected.

## Configuration and tuner switch

| Setting in `/etc/apm_bridge/runtime.env` | Default |
| --- | --- |
| `FC_PORT` | `/dev/ttyS8` |
| `LINK_BAUD` | `921600` |
| `GIMBAL_PORT` | `/dev/ttyS10` |
| `GIMBAL_MODEL` | `C20S` |
| `ROS_DOMAIN_ID` | `0` |
| `BENCH_PID_ENABLED` | `false`; enabled packages set `true` |

Set `GIMBAL_PORT=""` when no gimbal is connected. Use the same `ROS_DOMAIN_ID`
for communicating ROS nodes. This identifies their discovery group.
`APM_CONFIG` and `APM_BENCH_CONFIG` can select alternative absolute YAML paths.

Set `BENCH_PID_ENABLED=true` to enable both tuner interlocks together, or
`BENCH_PID_ENABLED=false` to disable both. The launcher supplies both ROS
parameters after the YAML file, so this single switch overrides the YAML's
`enabled` and `rig_confirmed` settings. Restart the service after editing:

```bash
sudo systemctl restart apm-bridge
```

Only trusted administrators should edit `runtime.env`: the launcher reads it
as Bash assignments. Upgrading replaces local edits with the new package's
configuration. Keep your changes separately or restore the installation backup.
Do not edit files in the extracted release; its checksum verification would fail.

## Supervision and diagnostics

The shared supervisor starts both nodes on CPU cores 0–3. It preserves all five
bridge mappings: `~/control_command`, `~/control_command_raw`, `~/arm`,
`~/offboard` and `~/low_level_feedback` map to the corresponding `/fpv/` topics.
Arguments after an optional tuner switch go to the bridge only.

If either child exits, the supervisor stops its companion and exits with a
failure so systemd can restart the pair. A normal stop or Ctrl+C terminates
and waits for both nodes. Unresponsive children are killed after five seconds.
Tuner reports go under `/var/log/apm_bridge/bench_pid`.

```bash
systemctl is-enabled apm-bridge
systemctl is-active apm-bridge
systemctl status apm-bridge --no-pager
journalctl -u apm-bridge -n 60 --no-pager
systemctl show apm-bridge -p CPUAffinity
cat /sys/devices/system/cpu/cpufreq/policy*/scaling_governor
cat /sys/class/devfreq/*npu*/governor
```

Expect `enabled`, `active`, affinity `0-3`, and `performance` governor values.
Confirm both ROS node names `/custom_link_bridge` and `/bench_pid_tuner` using
a compatible ROS CLI on the board or another computer in the same domain.
A default package must report both tuner parameters false and status `DISABLED`.
An enabled package must report both parameters true; its idle status depends
on the live interlocks.

For foreground diagnosis, stop the background service first:

```bash
sudo systemctl stop apm-bridge
sudo /opt/apm_bridge-binary/current/run.sh --disable-bench-pid
# Ctrl+C stops both processes; restore the background service afterwards.
sudo systemctl start apm-bridge
```

The first argument may instead be `--enable-bench-pid`; either switch overrides
the installed environment for that foreground run. Without a switch the
launcher uses the installed `BENCH_PID_ENABLED` value.

## Hardware acceptance and release evidence

A running process or `Custom link up` log only confirms startup or an opened
port. Confirm increasing flight-controller receive counters and actual gimbal
state replies. Using a compatible ROS CLI, verify approximately 200 Hz on
`/mavros/imu/data`, 100 Hz on `/mavros/rc/in`, and 100 Hz on `/fpv/gimbal` when
the corresponding hardware is connected. See the repository's
`docs/production-deployment.md` for dated evidence and full acceptance steps.
Reboot the board only when appropriate and repeat the service and telemetry
checks to validate boot startup. The installer does not reboot it.

`manifest.json` records the source revision, node names, both tuner flags,
architecture and dependency-check scope. `SHA256SUMS` covers the packaged tuner
and helpers as well as the bridge and custom interfaces. License declarations
are retained under `runtime/share/<package>/package.xml`; see `licenses/`.

To run the focused host regressions from the repository root:

```bash
python3 -B -m unittest discover -s deploy/binary/tests -v
```
