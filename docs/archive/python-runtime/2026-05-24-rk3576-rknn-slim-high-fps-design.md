# RK3576 RKNN Slim High-FPS Design

## Goal

Trim the project into a high-FPS RK3576 board runtime while keeping RKNN model inference as the core path. The first slim version keeps USB camera capture, RKNNLite model loading and inference, YOLO postprocess, target tracking, `$AIM` serial output, performance tuning, and an optional local OpenCV window for seeing the camera image.

## Keep

- `run_usb_rknn.py` as the single runtime entry.
- `models/best_rk3576_int8.rknn` as the default RK3576 INT8 model.
- `rknn_target/rknn_runtime.py` for RKNNLite import, RKNN model loading, NPU core mask selection, runtime init, inference, and release.
- `rknn_target/postprocess.py` for letterbox preprocessing, YOLO output decode, box mapping, confidence filtering, and NMS.
- `rknn_target/worker.py` for background RKNN inference and multi-worker high-FPS experiments.
- `rknn_target/io.py` for USB camera latest-frame capture and `$AIM` serial/stdout output.
- `rknn_target/tracker.py`, `types.py`, and `config.py` for target smoothing, state, and shared data structures.
- `rknn_target/debug.py` for the local OpenCV camera/debug window.
- `tools/rk3576_performance.sh` and `tools/benchmark_rk3576_modes.sh` for RK3576 FPS tuning.
- `requirements-rk3576.txt` with only board runtime dependencies.

## Remove

- Browser preview support: `rknn_target/preview.py`, `--preview-*` CLI options, and `run_preview_rk3576.sh`.
- JSON and binary output protocols. Keep only `$AIM` text output.
- Geometry refinement support: `rknn_target/geometry.py`, `--no-geometry`, and `--geometry-candidates`.
- SSH monitor helper: `tools/rk3576_monitor.sh`.
- Test-only and cache files: `tests/`, `.pytest_cache/`, and `__pycache__/`.
- Long README tuning sections that describe removed features.
- Unused imports and arguments left by the removed features.

## Runtime Flow

The camera thread keeps only the newest USB frame. The RKNN worker pool submits a frame only when at least one worker is free, so stale frames are dropped. Each worker performs letterbox preprocessing, RKNNLite inference, YOLO decode, and candidate selection. Candidate selection uses confidence first, then tracker-prior distance/IoU when a prior exists. The main loop updates the tracker and emits one `$AIM` line over serial or stdout.

The OpenCV window is only enabled with `--window`. The default remains headless for FPS.

## CLI Shape

Keep the useful high-FPS arguments:

- `--model`
- `--camera`
- `--camera-api`
- `--width`
- `--height`
- `--fps`
- `--fourcc`
- `--input-size`
- `--target-class`
- `--class-count`
- `--conf`
- `--iou`
- `--max-det`
- `--min-box-size`
- `--core-mask`
- `--workers`
- `--worker-core-masks`
- `--serial-port`
- `--baudrate`
- `--serial-interval`
- `--output-interval`
- `--output-on-result`
- `--window`
- `--no-window`
- `--display-interval`
- `--stats-interval`
- `--warmup`
- `--opencv-threads`
- `--fast`
- `--disable-gc`
- `--memory-trim-interval`

Remove:

- `--protocol`
- `--preview-port`
- `--preview-host`
- `--preview-fps`
- `--preview-quality`
- `--no-geometry`
- `--geometry-candidates`

## Commands

Fast board run:

```bash
python3 run_usb_rknn.py \
  --model models/best_rk3576_int8.rknn \
  --camera /dev/video73 --width 640 --height 480 --fps 60 \
  --fourcc MJPG --fast --workers 2 --worker-core-masks 0,1 \
  --no-window --output-on-result \
  --serial-port /dev/ttyUSB0 --baudrate 115200 --serial-interval 0.02
```

Local camera/debug window:

```bash
python3 run_usb_rknn.py \
  --model models/best_rk3576_int8.rknn \
  --camera /dev/video73 --width 640 --height 480 --fps 60 \
  --fourcc MJPG --fast --workers 2 --worker-core-masks 0,1 \
  --window --display-interval 2 --output-on-result \
  --serial-port /dev/ttyUSB0 --baudrate 115200 --serial-interval 0.02
```

## Verification

Local syntax check:

```bash
python -m py_compile run_usb_rknn.py rknn_target/*.py
```

Board verification:

1. Run `sudo bash tools/rk3576_performance.sh`.
2. Run the fast board command and confirm RKNN model loads.
3. Confirm `stats` reports inference FPS and timing.
4. Confirm `$AIM` output appears on serial or stdout.
5. Run with `--window` and confirm the camera image and target overlay are visible.

## Acceptance Criteria

- RKNN model inference remains functional on RK3576.
- The default run is headless and high-FPS oriented.
- Local OpenCV window is available with `--window`.
- Removed preview/protocol/test files are no longer imported by runtime code.
- `$AIM` output keeps the same field order as before.
