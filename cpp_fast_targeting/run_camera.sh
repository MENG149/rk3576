#!/usr/bin/env bash
# Run rkfast_target on the live /dev/video73 camera feed with the fixed
# FP16 + OpenCV-refine pipeline (2026-07-12). Build has RGA hardware
# preprocessing enabled (see CMakeLists RKFAST_USE_RGA). Note: measured on
# 2026-07-12, /dev/video73 itself caps out at ~27.5fps raw capture regardless
# of format (MJPG/YUYV) or resolution up to 1280x720, so with this camera
# RGA will not raise frame_fps (CPU preprocessing ~12ms was never the
# bottleneck). RGA is kept on anyway so total_ms has slack and it's ready
# once the camera is upgraded to something faster than ~30fps.
#
# Also measured 2026-07-12: without pinned thread affinity, frame_fps was
# bimodal, randomly landing on ~13.7 or ~27.5 between runs (busy-spin main
# loop + camera-capture thread + detector thread competing for the same
# cores with no fixed assignment). Pinning them to separate big cores
# (5/6/7) made frame_fps consistently hit the ~27.5fps camera ceiling every
# run. Do not remove --camera-affinity/--detector-affinity/--main-affinity
# below without re-testing several runs in a row.
#
# Usage on the board:
#   ./run_camera.sh                     # headless, prints $AIM lines to stdout
#   ./run_camera.sh --serial /dev/ttyS4  # also send $AIM lines over serial
#   ./run_camera.sh --no-perf            # skip system performance governors
#   ./run_camera.sh --input-mode legacy  # force the compatibility input path
#
set -euo pipefail
cd "$(dirname "$0")"

MODEL="../models/best_n_stage1_512_fp16.rknn"
CAMERA="/dev/video73"
SERIAL_PORT=""
# Dual NPU core measured ~2x faster than default single-core on this board
# (2026-07-12: frame_fps 13.7->27.5, rknn 19.7ms->16ms).
CORE_MASK="01"
INPUT_MODE="auto"
APPLY_PERF=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --serial)
      SERIAL_PORT="$2"
      shift 2
      ;;
    --model)
      MODEL="$2"
      shift 2
      ;;
    --camera)
      CAMERA="$2"
      shift 2
      ;;
    --core-mask)
      CORE_MASK="$2"
      shift 2
      ;;
    --perf)
      APPLY_PERF=1
      shift
      ;;
    --no-perf)
      APPLY_PERF=0
      shift
      ;;
    --input-mode)
      INPUT_MODE="$2"
      shift 2
      ;;
    *)
      echo "unknown arg: $1" >&2
      exit 1
      ;;
  esac
done

# Apply CPU/NPU/DDR performance governors by default. CPU fixed-frequency
# writes are intentionally not used; pass --no-perf only for diagnostics.
if [[ "$APPLY_PERF" -eq 1 ]]; then
  if [[ -f ../tools/rk3576_performance.sh ]]; then
    sudo bash ../tools/rk3576_performance.sh
  else
    echo "tools/rk3576_performance.sh not found" >&2
    exit 1
  fi
fi

if [[ ! -f build/rkfast_target ]]; then
  echo "build/rkfast_target not found. Build it first:" >&2
  echo "  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DRKFAST_USE_RGA=OFF" >&2
  echo "  cmake --build build --parallel \$(nproc)" >&2
  exit 1
fi

ARGS=(
  --model "$MODEL"
  --camera "$CAMERA"
  --width 1280 --height 720 --fps 30 --fourcc MJPG
  --input-size 512 --class-count 1 --target-class 0
  --conf 0.25 --iou 0.50 --max-det 1
  --detect-interval-locked 0
  --refine-min-score 0.45
  --core-mask "$CORE_MASK"
  --rknn-input-mode "$INPUT_MODE"
  --camera-affinity 5
  --detector-affinity 6
  --main-affinity 7
  --no-window
  --stats-interval 1
)

if [[ -n "$SERIAL_PORT" ]]; then
  ARGS+=(--serial-port "$SERIAL_PORT" --baudrate 115200 --serial-interval 0.02)
fi

echo "model=$MODEL camera=$CAMERA core_mask=$CORE_MASK input_mode=$INPUT_MODE serial=${SERIAL_PORT:-none(stdout)}"
exec ./build/rkfast_target "${ARGS[@]}"
