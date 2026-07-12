from __future__ import annotations

from .config import TrackerConfig
from .postprocess import box_size
from .types import Detection, TrackSnapshot


class RectTracker:
    def __init__(self, config: TrackerConfig) -> None:
        self.config = config
        self.initialized = False
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0
        self.h = 0.0
        self.last_ts: float | None = None
        self.misses = config.max_predict_misses + 1
        self.conf = 0.0
        self.count = 0
        self.state = "SEARCHING"
        self.source = "none"

    def _dt(self, timestamp: float) -> float:
        if self.last_ts is None:
            self.last_ts = timestamp
            fps = self.config.target_fps if self.config.target_fps > 0 else 60.0
            return 1.0 / max(fps, 1.0)
        dt = max(1e-3, min(0.25, timestamp - self.last_ts))
        self.last_ts = timestamp
        return dt

    def _clamp(self, frame_w: int, frame_h: int) -> None:
        self.x = max(0.0, min(float(frame_w - 1), self.x))
        self.y = max(0.0, min(float(frame_h - 1), self.y))
        self.w = max(1.0, min(float(frame_w), self.w))
        self.h = max(1.0, min(float(frame_h), self.h))

    def prior(self) -> tuple[float, float, float, float] | None:
        if not self.initialized or self.misses > self.config.max_predict_misses:
            return None
        return self.x, self.y, self.w, self.h

    def update(self, detection: Detection, timestamp: float, frame_shape: tuple[int, ...]) -> None:
        frame_h, frame_w = frame_shape[:2]
        dt = self._dt(timestamp)
        mx, my = detection.center
        dw, dh = box_size(detection.box)

        if not self.initialized or self.misses > self.config.max_predict_misses:
            self.x, self.y = mx, my
            self.vx = 0.0
            self.vy = 0.0
            self.w, self.h = dw, dh
            self.initialized = True
        else:
            px = self.x + self.vx * dt
            py = self.y + self.vy * dt
            rx = mx - px
            ry = my - py
            self.x = px + self.config.track_alpha * rx
            self.y = py + self.config.track_alpha * ry
            self.vx += self.config.track_beta * rx / dt
            self.vy += self.config.track_beta * ry / dt
            self.w = (1.0 - self.config.track_box_alpha) * self.w + self.config.track_box_alpha * dw
            self.h = (1.0 - self.config.track_box_alpha) * self.h + self.config.track_box_alpha * dh

        self._clamp(frame_w, frame_h)
        self.misses = 0
        self.conf = detection.conf
        self.count = max(1, int(detection.count))
        self.state = "LOCKED"
        self.source = "rknn"

    def miss(self, timestamp: float, frame_shape: tuple[int, ...]) -> None:
        if not self.initialized:
            self.state = "SEARCHING"
            return

        frame_h, frame_w = frame_shape[:2]
        dt = self._dt(timestamp)
        self.misses += 1
        if self.misses <= self.config.max_predict_misses:
            self.x += self.vx * dt
            self.y += self.vy * dt
            self._clamp(frame_w, frame_h)
            self.state = "LOST"
            self.source = "predict"
            self.count = 1
            self.conf = max(0.0, self.conf * 0.6)
            return

        self.state = "SEARCHING"
        self.source = "none"
        self.count = 0

    def snapshot(self) -> TrackSnapshot:
        active = self.initialized and self.misses <= self.config.max_predict_misses
        if not active:
            return TrackSnapshot(state="SEARCHING", misses=self.misses)

        box = (
            self.x - self.w * 0.5,
            self.y - self.h * 0.5,
            self.x + self.w * 0.5,
            self.y + self.h * 0.5,
        )
        return TrackSnapshot(
            active=True,
            box=box,
            center=(self.x, self.y),
            conf=self.conf,
            count=self.count,
            state=self.state,
            misses=self.misses,
            source=self.source,
        )
