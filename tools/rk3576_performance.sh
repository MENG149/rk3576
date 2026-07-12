#!/usr/bin/env bash
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "Run with sudo: sudo bash tools/rk3576_performance.sh" >&2
  exit 1
fi

write_optional() {
  local path="$1"
  local value="$2"
  if [ -e "$path" ]; then
    echo "$value" > "$path" 2>/dev/null || true
  fi
}

write_required() {
  local label="$1"
  local path="$2"
  local value="$3"
  if [ ! -e "$path" ]; then
    echo "Missing required ${label} node: ${path}" >&2
    return 1
  fi
  if ! echo "$value" > "$path"; then
    echo "Failed to set ${label}=${value}: ${path}" >&2
    return 1
  fi
}

write_first_existing() {
  local label="$1"
  local value="$2"
  shift 2
  local path
  for path in "$@"; do
    if [ -e "$path" ]; then
      write_required "$label" "$path" "$value"
      return
    fi
  done
  echo "No supported node found for ${label}" >&2
  return 1
}

read_if_exists() {
  local label="$1"
  local path="$2"
  if [ -e "$path" ]; then
    printf "%s=%s\n" "$label" "$(cat "$path" 2>/dev/null || true)"
  fi
}

for cpu in 0 1 2 3 4 5 6 7; do
  write_optional "/sys/devices/system/cpu/cpu${cpu}/cpuidle/state1/disable" 1
done

# CPU 使用 performance governor，不写 userspace/scaling_setspeed 锁频。
write_required cpu_policy0_governor \
  /sys/devices/system/cpu/cpufreq/policy0/scaling_governor performance
write_required cpu_policy4_governor \
  /sys/devices/system/cpu/cpufreq/policy4/scaling_governor performance

# NPU 和 DDR 使用板端最高性能 governor。
write_first_existing npu_governor performance \
  /sys/class/devfreq/27700000.npu/governor \
  /sys/devices/platform/27700000.npu/devfreq/27700000.npu/governor
write_required ddr_governor /sys/class/devfreq/dmc/governor performance

echo "Performance request applied. Current values:"
read_if_exists cpu_policy0_governor /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
read_if_exists cpu_policy0 /sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq
read_if_exists cpu_policy4_governor /sys/devices/system/cpu/cpufreq/policy4/scaling_governor
read_if_exists cpu_policy4 /sys/devices/system/cpu/cpufreq/policy4/scaling_cur_freq
read_if_exists npu_freq /sys/class/devfreq/27700000.npu/cur_freq
read_if_exists npu_governor /sys/class/devfreq/27700000.npu/governor
read_if_exists ddr_freq /sys/class/devfreq/dmc/cur_freq
read_if_exists ddr_governor /sys/class/devfreq/dmc/governor
