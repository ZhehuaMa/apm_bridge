#!/usr/bin/env bash
# Set all CPU and NPU frequency governors; leave GPU and other devices unchanged.
set -euo pipefail

check_performance_governors() {
    performance_governors=()
    local policy device name available cpu_count=0 npu_count=0
    for policy in /sys/devices/system/cpu/cpufreq/policy*; do
        [[ -d "$policy" ]] || continue
        performance_governors+=("$policy/scaling_governor")
        available="$(cat "$policy/scaling_available_governors")"
        [[ " $available " == *' performance '* ]] || {
            echo "CPU policy does not support performance: $policy" >&2; return 1;
        }
        cpu_count=$((cpu_count + 1))
    done
    for device in /sys/class/devfreq/*; do
        [[ -d "$device" ]] || continue
        name="${device##*/}"
        if [[ -r "$device/name" ]]; then name+=" $(cat "$device/name")"; fi
        [[ "${name,,}" == *npu* ]] || continue
        performance_governors+=("$device/governor")
        available="$(cat "$device/available_governors")"
        [[ " $available " == *' performance '* ]] || {
            echo "NPU does not support performance: $device" >&2; return 1;
        }
        npu_count=$((npu_count + 1))
    done
    [[ "$cpu_count" -gt 0 && "$npu_count" -gt 0 ]] || {
        echo "CPU or NPU governor interfaces are missing; check the drivers." >&2; return 1;
    }
    local governor
    for governor in "${performance_governors[@]}"; do
        [[ -r "$governor" ]] || { echo "Cannot read governor: $governor" >&2; return 1; }
    done
}

apply_performance_governors() {
    local governor
    for governor in "${performance_governors[@]}"; do
        printf 'performance\n' > "$governor" || return 1
        [[ "$(cat "$governor")" == performance ]] || {
            echo "performance governor was not applied: $governor" >&2; return 1;
        }
        echo "performance: $governor"
    done
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    [[ $# -eq 0 || ( $# -eq 1 && "$1" == --check ) ]] || {
        echo "Usage: bash performance.sh [--check]" >&2; exit 2;
    }
    check_performance_governors
    if [[ "${1:-}" != --check ]]; then apply_performance_governors; fi
fi
