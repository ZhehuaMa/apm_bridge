#!/usr/bin/env bash
# Run the bridge and tuner as one service; either child exiting fails the service.
set -euo pipefail
enabled="${BENCH_PID_ENABLED:-false}"
case "${1:-}" in
    --enable-bench-pid) enabled=true; shift ;;
    --disable-bench-pid) enabled=false; shift ;;
esac
[[ "$enabled" == true || "$enabled" == false ]] || {
    echo "BENCH_PID_ENABLED must be true or false." >&2; exit 2;
}
for config in "$APM_CONFIG" "$APM_BENCH_CONFIG"; do
    [[ -r "$config" ]] || { echo "Cannot read parameters: $config" >&2; exit 1; }
done
children=()
cleanup() {
    trap - INT TERM EXIT
    if [[ ${#children[@]} -gt 0 ]]; then
        kill -TERM "${children[@]}" 2>/dev/null || true
        # Bound shutdown even if a child fails to respond to SIGTERM.
        /usr/bin/python3 -c '
import os, signal, sys, time
time.sleep(5)
for child in sys.argv[1:]:
    try:
        os.kill(int(child), signal.SIGKILL)
    except ProcessLookupError:
        pass
' "${children[@]}" &
        reaper=$!
        for child in "${children[@]}"; do wait "$child" 2>/dev/null || true; done
        kill "$reaper" 2>/dev/null || true
        wait "$reaper" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 0' INT TERM
taskset -c 0-3 "$APM_BRIDGE_EXECUTABLE" \
    --ros-args -r __node:=custom_link_bridge --params-file "$APM_CONFIG" \
    -p link.transport:=serial -p "link.serial_port:=${FC_PORT:-/dev/ttyS8}" \
    -p "link.baudrate:=${LINK_BAUD:-921600}" \
    -p "gimbal_port:=\"${GIMBAL_PORT-/dev/ttyS10}\"" -p "gimbal.model:=${GIMBAL_MODEL:-C20S}" \
    -r '~/low_level_feedback:=/fpv/low_level_feedback' \
    -r '~/control_command:=/fpv/control_command' \
    -r '~/control_command_raw:=/fpv/control_command_raw' \
    -r '~/arm:=/fpv/arm' -r '~/offboard:=/fpv/offboard' "$@" &
children+=("$!")
taskset -c 0-3 /usr/bin/python3 -B "$APM_BENCH_EXECUTABLE" \
    --ros-args -r __node:=bench_pid_tuner --params-file "$APM_BENCH_CONFIG" \
    -p "enabled:=$enabled" -p "rig_confirmed:=$enabled" \
    -p "output_dir:=${ROS_LOG_DIR:-/var/log/apm_bridge}/bench_pid" &
children+=("$!")
status=0
wait -n "${children[@]}" || status=$?
echo "A ROS node exited (status $status); stopping both nodes." >&2
if [[ "$status" -eq 0 ]]; then status=1; fi
exit "$status"
