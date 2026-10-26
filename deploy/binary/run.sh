#!/usr/bin/env bash
# Use only the target's ROS runtime and the packaged project interfaces.
set -eo pipefail
release_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
env_file="${APM_ENV_FILE:-/etc/apm_bridge/runtime.env}"
if [[ -f "$env_file" ]]; then
    set -a
    source "$env_file"
    set +a
fi
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH="$release_dir/runtime/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
export AMENT_PREFIX_PATH="$release_dir/runtime:/opt/ros/humble"
export PYTHONPATH="$release_dir/runtime/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/var/log/apm_bridge}"
export APM_CONFIG="${APM_CONFIG:-/etc/apm_bridge/parameters.yaml}"
export APM_BENCH_CONFIG="${APM_BENCH_CONFIG:-/etc/apm_bridge/bench_pid.yaml}"
export APM_BRIDGE_EXECUTABLE="$release_dir/bin/custom_link_bridge_node"
export APM_BENCH_EXECUTABLE="$release_dir/runtime/libexec/apm_bridge/bench_pid_tuner"
exec /bin/bash "$release_dir/supervise.sh" "$@"
