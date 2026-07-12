#!/usr/bin/env bash
set -euo pipefail

interval="${1:-1}"

read_value() {
  local path="$1"
  if [ -e "$path" ]; then
    cat "$path" 2>/dev/null || true
  else
    printf "-"
  fi
}

while true; do
  ts="$(date '+%H:%M:%S')"
  cpu0="$(read_value /sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq)"
  cpu4="$(read_value /sys/devices/system/cpu/cpufreq/policy4/scaling_cur_freq)"
  npu_freq="$(read_value /sys/class/devfreq/27700000.npu/cur_freq)"
  npu_load="$(read_value /sys/class/devfreq/27700000.npu/load)"
  ddr_freq="$(read_value /sys/class/devfreq/dmc/cur_freq)"
  mem="$(awk '/MemTotal|MemAvailable/ {printf "%s=%sMB ", $1, int($2/1024)}' /proc/meminfo)"
  printf "%s cpu0=%s cpu4=%s npu=%s load=%s ddr=%s %s\n" "$ts" "$cpu0" "$cpu4" "$npu_freq" "$npu_load" "$ddr_freq" "$mem"
  sleep "$interval"
done
