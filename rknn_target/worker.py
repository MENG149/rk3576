from __future__ import annotations

import threading
import time

import numpy as np

from .config import RuntimeConfig
from .postprocess import box_iou, decode_yolo_outputs, letterbox_to_square
from .rknn_runtime import RknnYoloRuntime
from .types import Detection, InferTimings


class RknnInferWorker:
    def __init__(self, runtime: RknnYoloRuntime, config: RuntimeConfig) -> None:
        self.runtime = runtime
        self.config = config
        self._input: np.ndarray | None = None
        self._prior: tuple[float, float, float, float] | None = None
        self._result: tuple[int, Detection | None, float, float, InferTimings] = (
            0,
            None,
            0.0,
            time.perf_counter(),
            InferTimings(),
        )
        self._seq = 0
        self._infer_fps = 0.0
        self._timings = InferTimings()
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._stop = threading.Event()
        self._busy = False
        self._pending = False
        self._input_rgb = np.empty((config.input_size, config.input_size, 3), dtype=np.uint8)
        self._thread = threading.Thread(target=self._run, name="rknn-infer", daemon=True)
        self._thread.start()

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._busy or self._pending

    def submit(self, frame: np.ndarray, prior: tuple[float, float, float, float] | None) -> bool:
        with self._lock:
            if self._busy or self._pending:
                return False
            self._input = frame
            self._prior = prior
            self._pending = True
            self._event.set()
            return True

    def get_result(self) -> tuple[int, Detection | None, float, float, InferTimings]:
        with self._lock:
            return self._result

    def stop(self) -> None:
        self._stop.set()
        self._event.set()
        self._thread.join(timeout=1.0)

    def _run(self) -> None:
        while not self._stop.is_set():
            if not self._event.wait(timeout=0.1):
                continue

            with self._lock:
                frame = self._input
                prior = self._prior
                self._input = None
                self._prior = None
                self._pending = False
                self._busy = True
            self._event.clear()

            if frame is None:
                with self._lock:
                    self._busy = False
                continue

            start = time.perf_counter()
            detection = None
            timings = InferTimings()
            try:
                detection, timings = self._infer_one(frame, prior)
            except Exception as exc:
                print(f"RKNN inference error: {exc}")

            now = time.perf_counter()
            timings.total_ms = (now - start) * 1000.0
            inst_fps = 1.0 / max(now - start, 1e-6)
            self._infer_fps = inst_fps if self._infer_fps <= 0.0 else self._infer_fps * 0.85 + inst_fps * 0.15
            self._timings = timings if self._timings.total_ms <= 0.0 else _smooth_timings(self._timings, timings)

            with self._lock:
                self._seq += 1
                self._result = (self._seq, detection, self._infer_fps, now, self._timings)
                self._busy = False

    def _infer_one(
        self, frame: np.ndarray, prior: tuple[float, float, float, float] | None
    ) -> tuple[Detection | None, InferTimings]:
        timings = InferTimings()
        frame_h, frame_w = frame.shape[:2]
        t0 = time.perf_counter()
        scale, pad_x, pad_y = letterbox_to_square(frame, self._input_rgb, self.config.input_size)
        t1 = time.perf_counter()
        outputs = self.runtime.inference(self._input_rgb)
        t2 = time.perf_counter()
        detections = decode_yolo_outputs(
            outputs,
            scale=scale,
            pad_x=pad_x,
            pad_y=pad_y,
            frame_w=frame_w,
            frame_h=frame_h,
            input_size=self.config.input_size,
            target_class=self.config.target_class,
            class_count=self.config.class_count,
            conf_threshold=self.config.conf_threshold,
            iou_threshold=self.config.iou_threshold,
            max_det=self.config.max_det,
            min_box_size=self.config.min_box_size,
        )
        t3 = time.perf_counter()
        detection = choose_detection(detections, prior, frame_w, frame_h)
        t4 = time.perf_counter()
        timings.preprocess_ms = (t1 - t0) * 1000.0
        timings.rknn_ms = (t2 - t1) * 1000.0
        timings.postprocess_ms = (t3 - t2) * 1000.0
        timings.select_ms = (t4 - t3) * 1000.0
        return detection, timings


def choose_detection(
    detections: list[Detection],
    prior: tuple[float, float, float, float] | None,
    frame_w: int,
    frame_h: int,
) -> Detection | None:
    if not detections:
        return None
    if prior is None:
        return max(detections, key=lambda item: item.conf)

    px, py, pw, ph = prior
    prior_box = (px - pw * 0.5, py - ph * 0.5, px + pw * 0.5, py + ph * 0.5)
    diag = float(np.hypot(frame_w, frame_h))
    best: Detection | None = None
    best_score = -1.0
    for detection in detections:
        dist = float(np.hypot(detection.center[0] - px, detection.center[1] - py)) / max(diag, 1.0)
        score = detection.conf
        score += 0.35 * max(0.0, 1.0 - dist * 3.0)
        score += 0.30 * box_iou(detection.box, prior_box)
        if score > best_score:
            best_score = score
            best = detection
    return best


def _smooth_timings(previous: InferTimings, current: InferTimings, alpha: float = 0.15) -> InferTimings:
    beta = 1.0 - alpha
    return InferTimings(
        preprocess_ms=previous.preprocess_ms * beta + current.preprocess_ms * alpha,
        rknn_ms=previous.rknn_ms * beta + current.rknn_ms * alpha,
        postprocess_ms=previous.postprocess_ms * beta + current.postprocess_ms * alpha,
        select_ms=previous.select_ms * beta + current.select_ms * alpha,
        total_ms=previous.total_ms * beta + current.total_ms * alpha,
    )


class RknnInferPool:
    def __init__(self, workers: list[RknnInferWorker]) -> None:
        if not workers:
            raise ValueError("RknnInferPool needs at least one worker")
        self.workers = workers
        self._next_worker = 0
        self._worker_seq = [0 for _ in workers]
        self._global_seq = 0
        self._last_result_ts = 0.0
        self._aggregate_fps = 0.0
        self._latest: tuple[int, Detection | None, float, float, InferTimings] = (
            0,
            None,
            0.0,
            time.perf_counter(),
            InferTimings(),
        )

    @property
    def busy(self) -> bool:
        return all(worker.busy for worker in self.workers)

    def submit(self, frame: np.ndarray, prior: tuple[float, float, float, float] | None) -> bool:
        count = len(self.workers)
        for offset in range(count):
            idx = (self._next_worker + offset) % count
            if self.workers[idx].submit(frame, prior):
                self._next_worker = (idx + 1) % count
                return True
        return False

    def get_result(self) -> tuple[int, Detection | None, float, float, InferTimings]:
        newest: tuple[int, Detection | None, float, float, InferTimings] | None = None
        new_count = 0
        for idx, worker in enumerate(self.workers):
            seq, detection, _, result_ts, timings = worker.get_result()
            if seq <= self._worker_seq[idx]:
                continue
            self._worker_seq[idx] = seq
            new_count += 1
            if newest is None or result_ts >= newest[3]:
                newest = (seq, detection, 0.0, result_ts, timings)

        if newest is None:
            return self._latest

        result_ts = newest[3]
        if self._last_result_ts > 0.0 and result_ts > self._last_result_ts:
            inst_fps = new_count / max(result_ts - self._last_result_ts, 1e-6)
            self._aggregate_fps = (
                inst_fps if self._aggregate_fps <= 0.0 else self._aggregate_fps * 0.85 + inst_fps * 0.15
            )
        self._last_result_ts = result_ts
        self._global_seq += new_count
        self._latest = (self._global_seq, newest[1], self._aggregate_fps, result_ts, newest[4])
        return self._latest

    def stop(self) -> None:
        for worker in self.workers:
            worker.stop()
