#!/usr/bin/env bash
# Run rkfast_target on the live /dev/video73 camera feed with the fixed
# FP16 + OpenCV-refine pipeline (2026-07-12). Build has RGA hardware
# preprocessing enabled (see CMakeLists RKFAST_USE_RGA).
#
# Camera updated 2026-07-15 to HF868-2 (OV2710, 2.7mm/135deg HFOV), replacing
# the old sensor that capped out at ~27.5fps raw capture regardless of format
# or resolution up to 1280x720. Per the HF868-2 datasheet, 640x480 MJPG is
# the only mode that reaches 120fps (YUY2 caps at 30fps at this resolution;
# 720p/1080p MJPG modes cap at 30-60fps). Higher fps means shorter per-frame
# exposure, which should reduce motion blur and sharpen ROI corner
# measurements feeding the feedforward velocity estimate -- but verify actual
# frame_fps on this board first (the previous camera's 27.5fps ceiling was a
# board/driver limit, not just a sensor spec, so do not assume 120fps holds
# until measured).
#
# Also measured 2026-07-12: without pinned thread affinity, frame_fps was
# bimodal, randomly landing on ~13.7 or ~27.5 between runs (busy-spin main
# loop + camera-capture thread + detector thread competing for the same
# cores with no fixed assignment). Pinning them to separate big cores
# (5/6/7) made frame_fps consistently hit the camera's capture ceiling every
# run. Do not remove --camera-affinity/--detector-affinity/--main-affinity
# below without re-testing several runs in a row.
#
# Usage on the board:
#   ./run_camera.sh                     # headless, prints $AIM lines to stdout
#   ./run_camera.sh --serial /dev/ttyS4  # also send $AIM lines over serial
#   ./run_camera.sh --no-perf            # skip system performance governors
#   ./run_camera.sh --input-mode legacy  # force the compatibility input path
#   ./run_camera.sh --fps 60              # fall back if 120fps doesn't hold up
#   ./run_camera.sh --window              # enable the local debug window
#   ./run_camera.sh --mjpeg               # stream to http://<board-ip>:8080/
#   ./run_camera.sh --mjpeg 9000          # stream on a custom port
#   ./run_camera.sh                       # auto exposure (default, robust to light)
#   ./run_camera.sh --exposure 70         # force fixed manual exposure (stable fps)
#   ./run_camera.sh -- --kalman-gate 10   # pass advanced options to the binary
#
set -euo pipefail
cd "$(dirname "$0")"

MODEL="../models/best_n_stage1_512_fp16.rknn"
CAMERA="/dev/video73"
# HF868-2 only reaches 120fps in MJPG at 640x480 (see header note above).
WIDTH=640
HEIGHT=480
FPS=120
FOURCC="MJPG"
SERIAL_PORT=""
SERIAL_INTERVAL="0.02"
STDOUT_INTERVAL="0.20"
# Receiver (STM32 erweiyuntai) frame size the vision packet is scaled into.
PACKET_W=800
PACKET_H=480
SHOW_WINDOW=0
MJPEG_PORT=0
MJPEG_QUALITY=70
# Camera exposure strategy for the HF868-2. Default is AUTO exposure
# (Aperture Priority), chosen 2026-07-18 for robustness across lighting
# rather than a fixed frame rate: the sensor tracks scene brightness on its
# own, so the feed stays usable when the room light changes. Verified on the
# board that Aperture Priority genuinely engages -- exposure_time_absolute
# goes flags=inactive (driver-controlled) and the frame is well-exposed
# (earlier claims that only Manual worked were wrong, never actually tested).
#
# With auto exposure we also allow exposure_dynamic_framerate=1 so the sensor
# can lengthen exposure past the 8.33ms/120fps frame period in dark scenes;
# frame_fps then drops in low light (acceptable -- detection robustness is the
# goal here, not a fixed fps). This sensor has no auto-gain control, so we pin
# a moderate manual gain floor (AUTO_GAIN) to avoid underexposure without
# amplifying noise in bright scenes.
#
# Manual override: pass --exposure N (and optionally --gain N) to switch back
# to Manual Mode with a fixed exposure. In Manual mode keep exposure <=~75
# (7.5ms) at 120fps or the sensor stretches the frame time and tanks frame_fps
# (measured: exposure=300 (30ms) -> frame_fps 25.9; exposure<=75 -> full
# ~99fps). Fixed manual (exposure=70/gain=90) is the verified stable-fps value.
AUTO_EXPOSURE=1        # 1 = Aperture Priority (auto); forced to 0 if --exposure given
AUTO_GAIN=60           # manual gain floor used in auto-exposure mode (2026-07-18: 60 brighter, no wash-out; gain costs no frame time)
EXPOSURE_TIME=70       # used only in manual mode (--exposure)
GAIN=90                # used only in manual mode
EXTRA_ARGS=()
# Dual NPU core measured ~2x faster than default single-core on this board
# (2026-07-12: frame_fps 13.7->27.5, rknn 19.7ms->16ms).
CORE_MASK="01"
INPUT_MODE="auto"
APPLY_PERF=1
APPLY_CAMERA_CTRL=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --serial)
      SERIAL_PORT="$2"
      shift 2
      ;;
    --serial-interval)
      SERIAL_INTERVAL="$2"
      shift 2
      ;;
    --stdout-interval)
      STDOUT_INTERVAL="$2"
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
    --width)
      WIDTH="$2"
      shift 2
      ;;
    --height)
      HEIGHT="$2"
      shift 2
      ;;
    --fps)
      FPS="$2"
      shift 2
      ;;
    --fourcc)
      FOURCC="$2"
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
    --no-camera-ctrl)
      APPLY_CAMERA_CTRL=0
      shift
      ;;
    --exposure)
      # Any explicit exposure switches to Manual Mode.
      AUTO_EXPOSURE=0
      EXPOSURE_TIME="$2"
      shift 2
      ;;
    --gain)
      GAIN="$2"
      AUTO_GAIN="$2"
      shift 2
      ;;
    --auto-exposure)
      AUTO_EXPOSURE=1
      shift
      ;;
    --input-mode)
      INPUT_MODE="$2"
      shift 2
      ;;
    --window)
      SHOW_WINDOW=1
      shift
      ;;
    --no-window)
      SHOW_WINDOW=0
      shift
      ;;
    --mjpeg)
      # Optional numeric arg; default port 8080 when omitted or non-numeric.
      if [[ "${2:-}" =~ ^[0-9]+$ ]]; then
        MJPEG_PORT="$2"
        shift 2
      else
        MJPEG_PORT=8080
        shift
      fi
      ;;
    --mjpeg-quality)
      MJPEG_QUALITY="$2"
      shift 2
      ;;
    --)
      shift
      EXTRA_ARGS=("$@")
      break
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

# Program the camera every run. The sensor keeps whatever a previous session
# last wrote across power cycles (verified 2026-07-18: a leftover fixed
# exposure from an fps benchmark blacked out the feed), so never rely on
# driver defaults -- set the mode explicitly here.
#
# AUTO_EXPOSURE=1 (default): Aperture Priority. The driver tracks scene
# brightness on its own (confirmed: exposure_time_absolute goes
# flags=inactive and the frame is well-exposed), so the feed stays usable as
# room light changes. exposure_dynamic_framerate=1 lets it lengthen exposure
# past the 8.33ms/120fps frame period in dark scenes -- frame_fps drops in low
# light, which is the accepted trade for robustness. No auto-gain control
# exists, so a moderate manual gain floor (AUTO_GAIN) prevents underexposure
# without over-amplifying noise in bright scenes. Auto white balance on.
#
# AUTO_EXPOSURE=0 (--exposure given): Manual Mode with fixed exposure/gain.
# Keep exposure <=~75 at 120fps or frame_fps tanks (see header note).
#
# Best-effort: control names/existence vary by UVC firmware, so failures are
# non-fatal. Skip all of this with --no-camera-ctrl.
if [[ "${APPLY_CAMERA_CTRL:-1}" -eq 1 ]] && command -v v4l2-ctl &>/dev/null; then
  v4l2-ctl -d "$CAMERA" -c white_balance_automatic=1 2>/dev/null || true
  if [[ "$AUTO_EXPOSURE" -eq 1 ]]; then
    # Order matters: enable auto mode, then the dynamic-framerate permission,
    # then set the manual gain floor (gain stays user-controlled in this mode).
    v4l2-ctl -d "$CAMERA" -c auto_exposure=3 2>/dev/null || true
    v4l2-ctl -d "$CAMERA" -c exposure_dynamic_framerate=1 2>/dev/null || true
    v4l2-ctl -d "$CAMERA" -c gain="$AUTO_GAIN" 2>/dev/null || true
  else
    # Manual: hold the frame rate, pin exposure and gain explicitly.
    v4l2-ctl -d "$CAMERA" -c exposure_dynamic_framerate=0 2>/dev/null || true
    v4l2-ctl -d "$CAMERA" -c auto_exposure=1 2>/dev/null || true
    v4l2-ctl -d "$CAMERA" -c exposure_time_absolute="$EXPOSURE_TIME" 2>/dev/null || true
    v4l2-ctl -d "$CAMERA" -c gain="$GAIN" 2>/dev/null || true
  fi
fi

if [[ ! -f build/rkfast_target ]]; then
  echo "build/rkfast_target not found. Build it first:" >&2
  echo "  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DRKFAST_USE_RGA=ON" >&2
  echo "  cmake --build build --parallel \$(nproc)" >&2
  exit 1
fi

ARGS=(
  --model "$MODEL"
  --camera "$CAMERA"
  --width "$WIDTH" --height "$HEIGHT" --fps "$FPS" --fourcc "$FOURCC"
  --input-size 512 --class-count 1 --target-class 0
  # max-det bounds the emitted target(s); candidate-pool keeps more post-NMS
  # candidates so prior/track association can pick a nearer/higher-IoU box
  # under clutter instead of blindly taking the single highest-conf one.
  --conf 0.25 --iou 0.50 --max-det 1 --candidate-pool 10
  # RKNN optimized to ~73fps capacity (2026-07-15), still short of
  # 120fps/8.3ms frames. Once locked, skip 1 frame (60Hz anchor submission,
  # ~18% headroom under 73fps) instead of running every frame; ROI refine +
  # Kalman carry tracking and feedforward velocity in between.
  --detect-interval-locked 1
  --refine-min-score 0.45
  --core-mask "$CORE_MASK"
  --rknn-input-mode "$INPUT_MODE"
  --camera-affinity 5
  --detector-affinity 6
  --main-affinity 7
  --stdout-interval "$STDOUT_INTERVAL"
  --stats-interval 1
)

if [[ "$SHOW_WINDOW" -eq 1 ]]; then
  ARGS+=(--window)
else
  ARGS+=(--no-window)
fi

if [[ "$MJPEG_PORT" -gt 0 ]]; then
  ARGS+=(--mjpeg-port "$MJPEG_PORT" --mjpeg-quality "$MJPEG_QUALITY")
fi

if [[ -n "$SERIAL_PORT" ]]; then
  # erweiyuntai STM32 gimbal expects the 12-byte binary packet, receiver frame
  # 800x480 (K230 output). Coords/velocity are scaled from capture into that.
  ARGS+=(--serial-port "$SERIAL_PORT" --baudrate 115200
        --serial-interval "$SERIAL_INTERVAL"
        --serial-packet
        --packet-frame-width "$PACKET_W" --packet-frame-height "$PACKET_H")
fi

ARGS+=("${EXTRA_ARGS[@]}")

echo "model=$MODEL camera=$CAMERA core_mask=$CORE_MASK input_mode=$INPUT_MODE serial=${SERIAL_PORT:-none(stdout)}"
echo "capture=${WIDTH}x${HEIGHT} fps=$FPS fourcc=$FOURCC"
if [[ "$MJPEG_PORT" -gt 0 ]]; then
  echo "mjpeg stream: open http://<board-ip>:$MJPEG_PORT/ in a browser"
fi
exec ./build/rkfast_target "${ARGS[@]}"
