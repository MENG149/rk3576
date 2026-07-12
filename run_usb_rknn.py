from __future__ import annotations

import argparse
import ctypes
import gc
import os
import time
from dataclasses import replace
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "2")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import cv2
import numpy as np

from rknn_target.config import CameraConfig, PROJECT_ROOT, RuntimeConfig, TrackerConfig, parse_camera
from rknn_target.debug import draw_debug
from rknn_target.io import AimOutputWriter, UsbCamera
from rknn_target.rknn_runtime import RknnYoloRuntime
from rknn_target.tracker import RectTracker
from rknn_target.worker import RknnInferPool, RknnInferWorker


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RK3576 RKNN YOLO USB-camera rectangle targeting.")
    parser.add_argument("--model", type=Path, default=PROJECT_ROOT / "models" / "best_rk3576_int8.rknn")
    parser.add_argument("--camera", type=parse_camera, default=0)
    parser.add_argument("--camera-api", choices=("v4l2", "any"), default="v4l2")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--input-size", type=int, default=640)
    parser.add_argument("--target-class", type=int, default=0, help="-1 means choose best class.")
    parser.add_argument("--class-count", type=int, default=2)
    parser.add_argument("--conf", type=float, default=0.45)
    parser.add_argument("--iou", type=float, default=0.50)
    parser.add_argument("--max-det", type=int, default=8)
    parser.add_argument("--min-box-size", type=float, default=6.0)
    parser.add_argument(
        "--core-mask",
        choices=("auto", "all", "0", "1", "2", "01", "02", "12", "default"),
        default="auto",
    )
    parser.add_argument("--workers", type=int, default=1, help="Parallel RKNN runtime instances. Try 2 on RK3576.")
    parser.add_argument(
        "--worker-core-masks",
        default="",
        help="Comma-separated masks for workers, for example 0,1,2. Empty means derive from --core-mask.",
    )
    parser.add_argument("--target-fps", type=float, default=0.0, help="0 means run main loop as fast as possible.")
    parser.add_argument("--track-alpha", type=float, default=0.55)
    parser.add_argument("--track-beta", type=float, default=0.20)
    parser.add_argument("--track-box-alpha", type=float, default=0.45)
    parser.add_argument("--max-predict-misses", type=int, default=3)
    parser.add_argument("--serial-port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--serial-interval", type=float, default=0.0, help="0 means write every output frame.")
    parser.add_argument("--output-interval", type=float, default=0.2, help="Stdout fallback interval in seconds.")
    parser.add_argument("--output-on-result", action="store_true", help="Send output only when a new inference finishes.")
    parser.add_argument("--no-window", dest="no_window", action="store_true", default=True)
    parser.add_argument("--window", dest="no_window", action="store_false", help="Show OpenCV debug window.")
    parser.add_argument("--display-interval", type=int, default=2)
    parser.add_argument("--stats-interval", type=float, default=1.0)
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--opencv-threads", type=int, default=1)
    parser.add_argument("--verbose-rknn", action="store_true")
    parser.add_argument("--fast", action="store_true", help="Prefer throughput: keep only one detection and send on result.")
    parser.add_argument("--disable-gc", action="store_true", help="Disable Python GC for minimum jitter.")
    parser.add_argument(
        "--memory-trim-interval",
        type=float,
        default=30.0,
        help="Seconds between gc.collect()+malloc_trim; 0 disables it.",
    )
    args = parser.parse_args()
    if args.fast:
        args.max_det = min(args.max_det, 1)
        if args.serial_interval <= 0.0:
            args.serial_interval = 0.02
        args.output_on_result = True
    args.workers = max(1, args.workers)
    return args


def warmup_runtime(runtime: RknnYoloRuntime, input_size: int, count: int) -> None:
    dummy = np.full((input_size, input_size, 3), 114, dtype=np.uint8)
    for _ in range(max(0, count)):
        runtime.inference(dummy)


def worker_core_masks(core_mask: str, worker_core_masks: str, worker_count: int) -> list[str]:
    explicit = [item.strip() for item in worker_core_masks.split(",") if item.strip()]
    if explicit:
        return [explicit[idx % len(explicit)] for idx in range(worker_count)]
    if worker_count > 1 and core_mask == "auto":
        cores = ["0", "1", "2"]
        return [cores[idx % len(cores)] for idx in range(worker_count)]
    return [core_mask for _ in range(worker_count)]


def trim_process_memory() -> None:
    if os.name != "posix":
        return
    try:
        ctypes.CDLL("libc.so.6").malloc_trim(0)
    except Exception:
        pass


def main() -> int:
    args = parse_args()
    cv2.setNumThreads(max(1, args.opencv_threads))
    cv2.setUseOptimized(True)

    runtime_config = RuntimeConfig(
        model=args.model,
        input_size=args.input_size,
        target_class=args.target_class,
        class_count=args.class_count,
        conf_threshold=args.conf,
        iou_threshold=args.iou,
        max_det=args.max_det,
        min_box_size=args.min_box_size,
        core_mask=args.core_mask,
    )
    tracker_config = TrackerConfig(
        target_fps=args.target_fps,
        track_alpha=args.track_alpha,
        track_beta=args.track_beta,
        track_box_alpha=args.track_box_alpha,
        max_predict_misses=args.max_predict_misses,
    )
    camera_config = CameraConfig(
        camera=args.camera,
        api=args.camera_api,
        width=args.width,
        height=args.height,
        fps=args.fps,
        fourcc=args.fourcc,
    )

    runtimes: list[RknnYoloRuntime] = []
    infer_workers: list[RknnInferWorker] = []
    masks = worker_core_masks(args.core_mask, args.worker_core_masks, args.workers)
    for idx, mask in enumerate(masks):
        worker_config = replace(runtime_config, core_mask=mask)
        runtime = RknnYoloRuntime(worker_config, verbose=args.verbose_rknn)
        runtime.open()
        warmup_runtime(runtime, runtime_config.input_size, args.warmup)
        runtimes.append(runtime)
        infer_workers.append(RknnInferWorker(runtime, worker_config))
        print(f"worker {idx} ready core_mask={mask}")

    camera = UsbCamera(camera_config)
    worker = RknnInferPool(infer_workers)
    tracker = RectTracker(tracker_config)
    writer = AimOutputWriter(args.serial_port, args.baudrate, args.output_interval, args.serial_interval)

    main_fps = 0.0
    last_loop = time.perf_counter()
    last_stats = last_loop
    last_result_seq = 0
    latest_infer_fps = 0.0
    latest_timings = None
    frame_count = 0
    last_frame_seq = 0
    last_memory_trim = time.perf_counter()
    frame_interval = 1.0 / args.target_fps if args.target_fps > 0 else 0.0

    if args.disable_gc:
        gc.disable()
    print("running; press q in the debug window to exit")
    try:
        while True:
            loop_start = time.perf_counter()
            frame, _, frame_seq = camera.read()
            if frame is None:
                time.sleep(0.005)
                continue
            if frame_seq == last_frame_seq:
                time.sleep(0.001)
                continue
            last_frame_seq = frame_seq

            frame_count += 1
            if not worker.busy:
                worker.submit(frame, tracker.prior())

            seq, detection, latest_infer_fps, result_ts, latest_timings = worker.get_result()
            new_result = seq != last_result_seq
            if new_result:
                last_result_seq = seq
                if detection is None:
                    tracker.miss(result_ts, frame.shape)
                else:
                    tracker.update(detection, result_ts, frame.shape)

            snapshot = tracker.snapshot()
            frame_h, frame_w = frame.shape[:2]
            if not args.output_on_result or new_result:
                writer.send(snapshot, frame_w, frame_h)

            now = time.perf_counter()
            main_fps = main_fps * 0.9 + (1.0 / max(now - last_loop, 1e-6)) * 0.1
            last_loop = now

            if args.stats_interval > 0 and now - last_stats >= args.stats_interval:
                timing_text = ""
                if latest_timings is not None:
                    timing_text = (
                        f" pre={latest_timings.preprocess_ms:.1f}ms"
                        f" rknn={latest_timings.rknn_ms:.1f}ms"
                        f" post={latest_timings.postprocess_ms:.1f}ms"
                        f" select={latest_timings.select_ms:.1f}ms"
                        f" total={latest_timings.total_ms:.1f}ms"
                    )
                print(
                    f"stats frame_fps={main_fps:.1f} infer_fps={latest_infer_fps:.1f}"
                    f"{timing_text} state={snapshot.state} conf={snapshot.conf:.2f}"
                )
                last_stats = now
            if args.memory_trim_interval > 0.0 and now - last_memory_trim >= args.memory_trim_interval:
                gc.collect()
                trim_process_memory()
                last_memory_trim = now

            if not args.no_window and frame_count % max(1, args.display_interval) == 0:
                cv2.imshow("RKNN YOLO Target", draw_debug(frame, snapshot, main_fps, latest_infer_fps))
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
            elif not args.no_window:
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break

            sleep_time = frame_interval - (time.perf_counter() - loop_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)
    finally:
        if args.disable_gc:
            gc.enable()
        writer.close()
        worker.stop()
        camera.release()
        for runtime in runtimes:
            runtime.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
