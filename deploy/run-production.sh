#!/usr/bin/env bash
# Run both nodes from a source-workspace deployment.
set -eo pipefail
release_dir="${APM_BRIDGE_RELEASE:-/opt/apm_bridge/current}"
source /opt/ros/humble/setup.bash
if [[ -n "${APM_MESSAGES_SETUP:-}" ]]; then source "$APM_MESSAGES_SETUP"; fi
source "${release_dir}/install/local_setup.bash"
app="$release_dir/install/apm_bridge"
export APM_CONFIG="${APM_CONFIG:-$app/share/apm_bridge/parameters/custom_link_parameters.yaml}"
export APM_BENCH_CONFIG="${APM_BENCH_CONFIG:-$app/share/apm_bridge/parameters/bench_pid.yaml}"
export APM_BRIDGE_EXECUTABLE="$app/lib/apm_bridge/custom_link_bridge_node"
export APM_BENCH_EXECUTABLE="$app/lib/apm_bridge/bench_pid_tuner"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/var/log/apm_bridge}"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
exec /bin/bash "$script_dir/supervise.sh" "$@"
