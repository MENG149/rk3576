from __future__ import annotations

import queue
import threading
import time

import cv2
import numpy as np

from .config import CameraConfig
from .types import TrackSnapshot


class UsbCamera:
    def __init__(self, config: CameraConfig) -> None:
        api_flag = cv2.CAP_V4L2 if config.api == "v4l2" and hasattr(cv2, "CAP_V4L2") else 0
        self.cap = cv2.VideoCapture(config.camera, api_flag) if api_flag else cv2.VideoCapture(config.camera)
        if not self.cap.isOpened() and api_flag:
            self.cap = cv2.VideoCapture(config.camera)
        if not self.cap.isOpened():
            raise RuntimeError(f"could not open USB camera: {config.camera}")

        if config.fourcc:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config.fourcc[:4]))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
        self.cap.set(cv2.CAP_PROP_FPS, config.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.frame: np.ndarray | None = None
        self.timestamp = 0.0
        self.seq = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="usb-camera", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if ok:
                with self._lock:
                    self.frame = frame
                    self.timestamp = time.perf_counter()
                    self.seq += 1
            else:
                time.sleep(0.003)

    def read(self) -> tuple[np.ndarray | None, float, int]:
        with self._lock:
            return self.frame, self.timestamp, self.seq

    def release(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.cap.release()


class AimOutputWriter:
    def __init__(
        self,
        serial_port: str | None,
        baudrate: int,
        output_interval: float,
        serial_interval: float = 0.0,
    ) -> None:
        self.output_interval = output_interval
        self.serial_interval = serial_interval
        self._serial = None
        self._queue: queue.Queue[str] = queue.Queue(maxsize=2)
        self._stop = threading.Event()
        self._last_stdout = 0.0
        self._last_serial = 0.0
        if serial_port:
            try:
                import serial

                self._serial = serial.Serial(serial_port, baudrate=baudrate, timeout=0.01)
                print(f"serial connected: {serial_port} @ {baudrate}")
            except Exception as exc:
                print(f"serial unavailable ({exc}); falling back to stdout")
        self._thread = threading.Thread(target=self._run, name="output-writer", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                payload = self._queue.get(timeout=0.05)
            except queue.Empty:
                continue

            if self._serial is not None:
                now = time.perf_counter()
                if self.serial_interval > 0.0 and now - self._last_serial < self.serial_interval:
                    continue
                try:
                    self._serial.write((payload + "\n").encode("utf-8"))
                    self._last_serial = now
                except Exception:
                    pass
                continue

            now = time.perf_counter()
            if now - self._last_stdout >= self.output_interval:
                print(payload)
                self._last_stdout = now

    def send(self, snapshot: TrackSnapshot, frame_w: int, frame_h: int) -> None:
        payload = self._format_aim(snapshot, frame_w, frame_h)

        try:
            self._queue.put_nowait(payload)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(payload)
            except queue.Full:
                pass

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        if self._serial is not None:
            self._serial.close()

    @staticmethod
    def _offset(snapshot: TrackSnapshot, frame_w: int, frame_h: int) -> tuple[float, float]:
        if not snapshot.active:
            return 0.0, 0.0
        return snapshot.center[0] - frame_w * 0.5, snapshot.center[1] - frame_h * 0.5

    def _format_aim(self, snapshot: TrackSnapshot, frame_w: int, frame_h: int) -> str:
        found = 1 if snapshot.active else 0
        cx, cy = snapshot.center if snapshot.active else (0.0, 0.0)
        ox, oy = self._offset(snapshot, frame_w, frame_h)
        return (
            f"$AIM,{found},{snapshot.state},{cx:.1f},{cy:.1f},"
            f"{ox:.1f},{oy:.1f},{snapshot.conf:.3f},{snapshot.source}"
        )
