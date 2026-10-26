#!/usr/bin/env bash
# Offline installation on matching Ubuntu 22.04 ARM64 boards.
set -euo pipefail
export LC_ALL=C
bundle_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
check_only=false
case "${1:-}" in
    --check) check_only=true ;;
    --help|-h)
        echo "Usage: sudo bash install.sh [--check]"
        echo "Install, start and enable the service by default; --check makes no system changes."
        exit 0 ;;
    "") ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
esac
[[ $# -le 1 ]] || { echo "Too many arguments" >&2; exit 2; }
fail() { echo "Error: $*" >&2; exit 1; }
for cmd in sha256sum ldd uname getconf find sort cp mv install ln readlink mktemp systemctl \
           cat cmp chown chmod mkdir rm sleep taskset; do
    command -v "$cmd" >/dev/null || fail "Missing system command: $cmd"
done
[[ "$(uname -m)" == aarch64 ]] || fail "Only aarch64 (ARM64) boards are supported."
for cpu in 0 1 2 3; do
    taskset -c "$cpu" /bin/true 2>/dev/null || fail "CPU $cpu is offline or unavailable in this environment."
done
[[ -r /etc/os-release ]] || fail "Cannot identify the operating system."
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]] || fail "Ubuntu 22.04 is required."
[[ "$(getconf GNU_LIBC_VERSION)" == 'glibc 2.35' ]] || fail "Ubuntu 22.04 glibc 2.35 is required."
[[ -f /opt/ros/humble/lib/librclcpp.so && -f /opt/ros/humble/lib/librmw_fastrtps_cpp.so ]] \
    || fail "Install the matching ROS 2 Humble runtime, including Fast DDS, first."
[[ -r "$bundle_dir/SHA256SUMS" ]] || fail "The bundle is missing SHA256SUMS."
(cd "$bundle_dir" && sha256sum --check --strict --quiet SHA256SUMS) || fail "Bundle checksum verification failed."
source "$bundle_dir/performance.sh"
check_performance_governors || fail "CPU/NPU performance policy checks failed."
version="$(cat "$bundle_dir/VERSION")"
[[ "$version" =~ ^[a-zA-Z0-9][a-zA-Z0-9._-]+$ ]] || fail "Invalid release version."
[[ -x "$bundle_dir/bin/custom_link_bridge_node" ]] || fail "Missing bridge executable."
[[ -e /lib/ld-linux-aarch64.so.1 ]] || fail "Missing ARM64 system dynamic loader."
# Check runtime plugins as well as directly linked application libraries.
while IFS= read -r binary; do
    if ! output="$(LD_LIBRARY_PATH="$bundle_dir/runtime/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu" ldd "$binary" 2>&1)"; then
        fail "Dependency check failed: $binary：$output"
    fi
    if [[ "$output" == *'not found'* ]]; then
        fail "Missing shared library: $binary：$output"
    fi
done < <(find "$bundle_dir/bin" "$bundle_dir/runtime" -type f \( -name '*.so*' -o -name custom_link_bridge_node \) | sort)
# Check Python dependencies for the second ROS node before changing the system.
[[ -x /usr/bin/python3 ]] || fail "/usr/bin/python3 is required for bench_pid_tuner."
(
    # ROS Humble setup scripts read optional unset variables.
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    export LD_LIBRARY_PATH="$bundle_dir/runtime/lib:${LD_LIBRARY_PATH:-}"
    export PYTHONPATH="$bundle_dir/runtime/libexec/apm_bridge:$bundle_dir/runtime/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
    /usr/bin/python3 -B - "$bundle_dir" <<'PY'
import runpy
import sys
from pathlib import Path
assert sys.version_info[:2] == (3, 10), "ROS Humble requires Python 3.10"
import rclpy
import numpy
import yaml
from rclpy.serialization import serialize_message, deserialize_message
from rclpy.type_support import check_for_type_support
from quadrotor_msgs.msg import ControlCommand, LowLevelFeedback
from quadrotor_msgs.srv import CliCommand
root = Path(sys.argv[1])
runpy.run_path(str(root / "runtime/libexec/apm_bridge/bench_pid_tuner"), run_name="package_preflight")
for message in [ControlCommand, LowLevelFeedback, CliCommand.Request, CliCommand.Response]:
    check_for_type_support(message)
    deserialize_message(serialize_message(message()), message)
parameters = yaml.safe_load((root / "config/bench_pid.yaml").read_text())["bench_pid_tuner"]["ros__parameters"]
assert type(parameters["enabled"]) is bool
assert parameters["enabled"] == parameters["rig_confirmed"]
PY
) || fail "bench_pid_tuner Python dependencies could not be loaded."
echo "Passed system, checksum and runtime dependency checks ($version)."
if $check_only; then
    exit 0
fi
[[ "$EUID" -eq 0 ]] || fail "Run sudo bash install.sh."
[[ -d /run/systemd/system ]] || fail "The target system must run systemd."
base=/opt/apm_bridge-binary
config_dir=/etc/apm_bridge
unit=/etc/systemd/system/apm-bridge.service
[[ ! -e "$base/current" || -L "$base/current" ]] || fail "$base/current exists and is not a symlink."
# Each deployment installs bundled configuration; failure restores the previous configuration.
mkdir -p "$config_dir" "$base/releases"
config_changed=false
backup="$(mktemp -d /var/tmp/apm-bridge-backup.XXXXXX)"
old_link="$(readlink "$base/current" 2>/dev/null || true)"
old_active=false
old_enabled=false
systemctl is-active --quiet apm-bridge && old_active=true
systemctl is-enabled --quiet apm-bridge 2>/dev/null && old_enabled=true
if [[ -e "$unit" || -L "$unit" ]]; then
    cp -a "$unit" "$backup/apm-bridge.service"
fi
changed_service=false
performance_changed=false
original_governors=()
stage=""
rollback() {
    local status=$?
    trap - EXIT
    if [[ "$status" -ne 0 ]]; then
        echo "Deployment failed. Restoring previous configuration and service. Backup: $backup" >&2
        if $config_changed; then
            for name in parameters.yaml runtime.env bench_pid.yaml; do
                rm -f "$config_dir/$name"
                if [[ -e "$backup/$name" || -L "$backup/$name" ]]; then
                    cp -a "$backup/$name" "$config_dir/$name"
                fi
            done
        fi
        if $changed_service; then
            systemctl stop apm-bridge || true
            systemctl disable apm-bridge >/dev/null 2>&1 || true
            rm -f "$unit"
            if [[ -e "$backup/apm-bridge.service" || -L "$backup/apm-bridge.service" ]]; then
                cp -a "$backup/apm-bridge.service" "$unit"
            fi
            rm -f "$base/current"
            if [[ -n "$old_link" ]]; then
                ln -s "$old_link" "$base/current"
            fi
            systemctl daemon-reload || true
            if $old_enabled; then systemctl enable apm-bridge || true; fi
            if $old_active; then systemctl start apm-bridge || true; fi
        fi
        if $performance_changed; then
            for index in "${!performance_governors[@]}"; do
                printf '%s\n' "${original_governors[$index]}" > "${performance_governors[$index]}" \
                    || echo "Cannot restore governor: ${performance_governors[$index]}" >&2
            done
        fi
    fi
    if [[ -n "$stage" && -d "$stage" ]]; then rm -rf -- "$stage"; fi
    exit "$status"
}
trap rollback EXIT
for name in parameters.yaml runtime.env bench_pid.yaml; do
    if [[ -e "$config_dir/$name" || -L "$config_dir/$name" ]]; then
        cp -a "$config_dir/$name" "$backup/$name"
    fi
done
config_changed=true
rm -f "$config_dir/parameters.yaml" "$config_dir/runtime.env" "$config_dir/bench_pid.yaml"
install -m 0644 "$bundle_dir/config/parameters.yaml" "$config_dir/parameters.yaml"
install -m 0644 "$bundle_dir/config/runtime.env" "$config_dir/runtime.env"
install -m 0644 "$bundle_dir/config/bench_pid.yaml" "$config_dir/bench_pid.yaml"
# Read the launcher port settings and check that board UART devices exist.
source "$config_dir/runtime.env"
[[ -c "${FC_PORT:-/dev/ttyS8}" ]] || fail "FC serial port missing: ${FC_PORT:-/dev/ttyS8}; enable the board UART first."
if [[ -n "${GIMBAL_PORT-/dev/ttyS10}" ]]; then
    [[ -c "${GIMBAL_PORT-/dev/ttyS10}" ]] || fail "Gimbal serial port missing: ${GIMBAL_PORT-/dev/ttyS10}."
fi
release="$base/releases/$version"
if [[ -e "$release" ]]; then
    cmp -s "$bundle_dir/SHA256SUMS" "$release/SHA256SUMS" || fail "Release name already exists with different contents: $release"
    (cd "$release" && sha256sum --check --strict --quiet SHA256SUMS) || fail "Installed release checksum verification failed."
else
    stage="$(mktemp -d "$base/releases/.stage.XXXXXX")"
    cp -a "$bundle_dir/." "$stage/"
    chown -R root:root "$stage"
    chmod -R a+rX "$stage"
    (cd "$stage" && sha256sum --check --strict --quiet SHA256SUMS)
    mv "$stage" "$release"
    stage=""
fi
for governor in "${performance_governors[@]}"; do
    original_governors+=("$(cat "$governor")")
    printf '%s\t%s\n' "$governor" "${original_governors[-1]}" >> "$backup/governors.tsv"
done
performance_changed=true
apply_performance_governors || fail "Cannot apply CPU/NPU performance policies."
changed_service=true
# Stop the old service before switching releases to avoid serial port contention.
systemctl stop apm-bridge 2>/dev/null || true
ln -s "releases/$version" "$base/.current.$$"
mv -Tf "$base/.current.$$" "$base/current"
install -m 0644 "$release/apm-bridge.service" "$unit"
systemctl daemon-reload
systemctl reset-failed apm-bridge 2>/dev/null || true
systemctl enable apm-bridge
systemctl start apm-bridge
sleep 5
systemctl is-active --quiet apm-bridge || fail "Service did not remain active; inspect journalctl -u apm-bridge."
[[ "$(systemctl show apm-bridge -p NRestarts --value)" == 0 ]] || fail "Service restarted after startup; inspect the logs."
systemctl is-enabled --quiet apm-bridge || fail "Service is not enabled at boot."
echo "Deployment succeeded: $release"
echo "Service is active and enabled at boot; status: systemctl status apm-bridge"
echo "CPU affinity: 0-3 for both nodes and their threads."
echo "CPU and NPU governors: performance; reapplied before every service start."
echo "Configuration: $config_dir/{parameters.yaml,bench_pid.yaml,runtime.env}"
echo "Previous configuration and service backup: $backup"
echo "Follow README.md for real FC and gimbal acceptance; process health alone is insufficient."
