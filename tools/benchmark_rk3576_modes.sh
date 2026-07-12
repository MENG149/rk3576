#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

duration="${DURATION:-20s}"
camera="${CAMERA:-/dev/video73}"
serial_port="${SERIAL_PORT:-/dev/ttyUSB0}"
baudrate="${BAUDRATE:-115200}"

base_args=(
  --model models/best_rk3576_int8.rknn
  --camera "$camera"
  --width 640 --height 480 --fps 60
  --fourcc MJPG
  --fast
  --output-on-result
  --serial-port "$serial_port"
  --baudrate "$baudrate"
  --serial-interval 0.02
  --stats-interval 1
)

run_case() {
  local name="$1"
  shift
  echo
  echo "===== $name ====="
  timeout "$duration" python -u run_usb_rknn.py "${base_args[@]}" "$@" || true
}

run_case "1 worker, core 0" --workers 1 --core-mask 0
run_case "1 worker, core 0_1" --workers 1 --core-mask 01
run_case "2 workers, cores 0 and 1" --workers 2 --worker-core-masks 0,1
run_case "4 workers, oversubscribe 0 and 1" --workers 4 --worker-core-masks 0,1,0,1
