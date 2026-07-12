# RK3576 C++ Fast Targeting Design

Date: 2026-05-26

## Goal

Build a standalone C++ runtime inside `rk3576_rknn_targeting` for RK3576 camera targeting with:

- RKNN YOLO detection
- ROI dark-rectangle refinement
- short-horizon tracking and prediction
- `$AIM` serial/stdout output
- performance-first structure for 50 Hz+ targeting output

The target is not "full YOLO on every frame". The target is:

- run YOLO only when needed
- refine inside a small ROI on most frames
- keep control output above 50 Hz when the target is already locked

## Constraints

- Board: RK3576, Ubuntu 24
- Runtime: direct RKNN C API
- Camera: USB camera over V4L2/OpenCV
- Model: `best_rk3576_int8.rknn`
- Priority: throughput and low jitter over feature breadth

## Architecture

`capture thread -> latest frame -> detector worker -> ROI rect refine -> tracker -> $AIM output`

Key rules:

1. Keep only the latest frame. Never build a long queue.
2. Use one dedicated RKNN worker thread so camera capture and ROI refine keep moving.
3. In `SEARCHING`, submit whole-frame detection as often as the worker is free.
4. In `LOCKED`, skip some detection frames and use ROI refine plus prediction on the in-between frames.
5. When ROI refine succeeds, trust its center and box over the coarse YOLO box.

## Modules

- `camera_source`: latest-frame USB capture with buffer size 1
- `rknn_detector`: model load, preprocess, RKNN inference, YOLO decode, candidate selection
- `detector_worker`: async inference thread with latest result cache
- `rect_refiner`: ROI binary/edge contour search for dark rectangle corners and center
- `tracker`: alpha-beta style center/size tracking with short prediction window
- `aim_writer`: `$AIM` output over serial or stdout
- `main`: state machine, scheduling, stats, debug window

## Performance Rules

- Build in `Release` with `-O3`
- Use `cv::setNumThreads(1)` to avoid OpenCV oversubscription
- Default input size is `640`; expected fallback path is `512` if needed
- Keep `max_det` low, ideally `1`
- Do not allocate in the hot path except RKNN output buffers required by runtime
- Preallocate the RGB input tensor buffer
- Detection scheduling is explicit: locked tracking uses `detect_interval_locked`
- Debug drawing is off by default

## First-Version Scope

Included:

- single-camera runtime
- single RKNN worker
- generic YOLO matrix decode compatible with the current Python runtime
- ROI rectangle refine
- tracking and `$AIM` output

Not included in this first C++ drop:

- RGA/DMA-BUF zero-copy path
- multi-worker multi-runtime balancing
- board-specific CPU affinity management
- full corner transport to the gimbal protocol

Those are valid next optimizations, but the first code drop should remove Python overhead, keep the pipeline small, and preserve the scheduling strategy that actually creates 50 Hz+ output.
