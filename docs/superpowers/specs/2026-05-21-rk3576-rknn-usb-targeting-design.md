# RK3576 RKNN USB Targeting Design

## Goal

Run a static INT8 RKNN YOLO model on an RK3576 board with a USB camera, keeping actual end-to-end FPS high while stabilizing rectangle target output for gimbal control.

## Structure

Use a small multi-file Python project instead of one large script:

- `run_usb_rknn.py` owns CLI and the main loop.
- `rknn_target/rknn_runtime.py` owns RKNNLite model load and NPU runtime setup.
- `rknn_target/io.py` owns USB camera latest-frame capture and serial/stdout output.
- `rknn_target/postprocess.py` owns YOLO decode, letterbox mapping, and NMS.
- `rknn_target/geometry.py` owns rectangle refinement and stable candidate choice.
- `rknn_target/tracker.py` owns smoothing, short miss prediction, and output state.

This keeps the board deployment command simple while making FPS/stability tuning localized.

## Runtime Flow

The USB camera thread continuously stores only the latest frame. The RKNN worker thread takes a frame only when idle, so old frames are dropped instead of queued. The main loop consumes the newest completed inference, updates the tracker, and emits `$AIM` or binary output.

## Performance Choices

- Static `640` input, `batch=1`, INT8 RKNN.
- Default `core_mask=auto`.
- V4L2 USB camera with MJPEG and buffer size 1.
- Reused letterbox input buffer to reduce allocation.
- Optional geometry refinement limited to top candidates.
- Headless `--no-window` mode for competition.

## Stability Choices

- Confidence filtering, NMS, and target class filtering.
- Candidate score includes previous-track proximity and IoU.
- Alpha-beta center smoothing with short miss prediction.
- Geometry refinement source is marked as `rknn_geo` for debugging.

## Verification

Local tests cover letterbox coordinate mapping, YOLO output decode, NMS, and tracker miss behavior. Full RKNN FPS verification must be run on the RK3576 board.

