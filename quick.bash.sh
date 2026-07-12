#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
PROJECT_ROOT="$(pwd)"
BUILD_DIR="$PROJECT_ROOT/cpp_fast_targeting/build"

SKIP_BUILD=0
BUILD_TYPE="${BUILD_TYPE:-Debug}"
USE_RGA="${USE_RGA:-ON}"
SHOW_WINDOW=1

MODEL="${MODEL:-$PROJECT_ROOT/models/best_yolo26n_512_rk3576_int8.rknn}"
CAMERA="${CAMERA:-/dev/video73}"
INPUT_SIZE="${INPUT_SIZE:-512}"
CLASS_COUNT="${CLASS_COUNT:-1}"
TARGET_CLASS="${TARGET_CLASS:-0}"
CORE_MASK="${CORE_MASK:-01}"
CONF="${CONF:-0.45}"
IOU="${IOU:-0.50}"
STATS_INTERVAL="${STATS_INTERVAL:-1.0}"

EXTRA_ARGS=()

usage() {
  cat <<'EOF'
Usage:
  bash quick.bash.sh [options] [extra rkfast_target args...]
  bash quick_start.sh [options] [extra rkfast_target args...]

Default behavior:
  - builds cpp_fast_targeting as Debug
  - enables RGA if available
  - starts rkfast_target with an OpenCV debug window

Options:
  --skip-build       Run existing build/rkfast_target
  --release          Build Release instead of Debug
  --debug            Build Debug
  --window           Show debug window
  --no-window        Disable debug window
  --model PATH       RKNN model path
  --camera PATH      Camera device path
  -h, --help         Show this help

Environment overrides:
  MODEL CAMERA INPUT_SIZE CLASS_COUNT TARGET_CLASS CORE_MASK CONF IOU
  STATS_INTERVAL BUILD_TYPE USE_RGA
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --release)
      BUILD_TYPE=Release
      shift
      ;;
    --debug)
      BUILD_TYPE=Debug
      shift
      ;;
    --window)
      SHOW_WINDOW=1
      shift
      ;;
    --no-window)
      SHOW_WINDOW=0
      shift
      ;;
    --model)
      MODEL="${2:?missing value for --model}"
      shift 2
      ;;
    --camera)
      CAMERA="${2:?missing value for --camera}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ "$MODEL" != /* ]]; then
  MODEL="$PROJECT_ROOT/$MODEL"
fi

echo "======================================"
echo "  RK3576 RKNN window debug startup"
echo "======================================"
echo "Project: $PROJECT_ROOT"
echo "Model:   $MODEL"
echo "Camera:  $CAMERA"
echo "Build:   $BUILD_TYPE, RGA=$USE_RGA"
echo ""

if [[ ! -f "$MODEL" ]]; then
  echo "ERROR: model file not found: $MODEL"
  echo "Available RKNN models:"
  ls -1 "$PROJECT_ROOT"/models/*.rknn 2>/dev/null || true
  exit 1
fi

if [[ ! -e "$CAMERA" ]]; then
  echo "WARNING: camera device does not exist yet: $CAMERA"
  echo "Check it with: ls -l /dev/video*"
  echo ""
fi

echo "[1/3] Enabling performance mode..."
if [[ -f "$PROJECT_ROOT/tools/rk3576_performance.sh" ]]; then
  sudo bash "$PROJECT_ROOT/tools/rk3576_performance.sh" || true
else
  echo "WARNING: tools/rk3576_performance.sh not found, skipping."
fi
echo ""

echo "[2/3] Building C++ target..."
if [[ -d "$BUILD_DIR" && ! -w "$BUILD_DIR" ]]; then
  echo "Build directory is not writable. Fixing owner with sudo..."
  sudo chown -R "$(id -u):$(id -g)" "$BUILD_DIR"
fi

if [[ -f "$BUILD_DIR/CMakeCache.txt" && ! -w "$BUILD_DIR/CMakeCache.txt" ]]; then
  echo "CMake cache is not writable. Fixing owner with sudo..."
  sudo chown "$(id -u):$(id -g)" "$BUILD_DIR/CMakeCache.txt"
fi

if [[ "$SKIP_BUILD" -eq 1 && -x "$BUILD_DIR/rkfast_target" ]]; then
  echo "Skip build: using existing cpp_fast_targeting/build/rkfast_target"
else
  cmake -S "$PROJECT_ROOT/cpp_fast_targeting" \
        -B "$BUILD_DIR" \
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
        -DRKFAST_USE_RGA="$USE_RGA" \
        -DCMAKE_CXX_FLAGS_DEBUG="-g -O0"
  cmake --build "$BUILD_DIR" -- -j"$(nproc)"
fi
echo ""

WINDOW_ARG="--window"
if [[ "$SHOW_WINDOW" -eq 0 ]]; then
  WINDOW_ARG="--no-window"
elif [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "WARNING: DISPLAY/WAYLAND_DISPLAY is empty."
  echo "The debug window needs a desktop session, HDMI display, or SSH X11 forwarding."
  echo ""
fi

echo "[3/3] Running rkfast_target..."
echo "Press q or Esc in the debug window to exit."
echo "======================================"
echo ""

exec "$BUILD_DIR/rkfast_target" \
  --model "$MODEL" \
  --camera "$CAMERA" \
  --width 640 --height 480 --fps 60 \
  --fourcc MJPG \
  --input-size "$INPUT_SIZE" \
  --class-count "$CLASS_COUNT" \
  --target-class "$TARGET_CLASS" \
  --core-mask "$CORE_MASK" \
  --conf "$CONF" \
  --iou "$IOU" \
  --max-det 1 \
  --detect-interval-locked 2 \
  --lead-time 0.05 \
  --predict-decay 0.9 \
  --corner-alpha 0.5 \
  --corner-reset-dist 40 \
  --prior-iou-weight 0.20 \
  --motion-roi-gain 1.5 \
  --motion-speed-ref 300 \
  "$WINDOW_ARG" \
  --stats-interval "$STATS_INTERVAL" \
  "${EXTRA_ARGS[@]}"
