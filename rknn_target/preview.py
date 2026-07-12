from __future__ import annotations

import html
import queue
import threading
import time
from http import server
from socketserver import ThreadingMixIn

import cv2
import numpy as np

from .debug import draw_debug
from .types import TrackSnapshot


class PreviewServer:
    def __init__(
        self,
        host: str,
        port: int,
        fps: float,
        jpeg_quality: int,
        title: str = "RK3576 RKNN Preview",
    ) -> None:
        self.host = host
        self.port = port
        self.fps = max(1.0, float(fps))
        self.jpeg_quality = max(20, min(95, int(jpeg_quality)))
        self.title = title
        self._queue: queue.Queue[tuple[np.ndarray, TrackSnapshot, float, float]] = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._frame_cond = threading.Condition()
        self._latest_jpeg = self._encode_placeholder("waiting for frames")
        self._latest_seq = 0
        self._server: _PreviewHTTPServer | None = None
        self._http_thread: threading.Thread | None = None
        self._encode_thread = threading.Thread(target=self._encode_loop, name="preview-encode", daemon=True)

    def start(self) -> None:
        if self._server is not None:
            return
        self._server = _PreviewHTTPServer((self.host, self.port), _PreviewHandler, self)
        self._http_thread = threading.Thread(target=self._server.serve_forever, name="preview-http", daemon=True)
        self._http_thread.start()
        self._encode_thread.start()

    @property
    def url(self) -> str:
        if self._server is None:
            return f"http://{self.host}:{self.port}"
        host, port = self._server.server_address[:2]
        display_host = "127.0.0.1" if host in ("0.0.0.0", "") else host
        return f"http://{display_host}:{port}"

    def submit(self, frame: np.ndarray, snapshot: TrackSnapshot, main_fps: float, infer_fps: float) -> None:
        if self._server is None:
            return
        try:
            self._queue.put_nowait((frame.copy(), snapshot, main_fps, infer_fps))
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait((frame.copy(), snapshot, main_fps, infer_fps))
            except queue.Full:
                pass

    def get_frame(self, last_seq: int | None, timeout: float) -> tuple[bytes, int]:
        deadline = time.perf_counter() + max(0.0, timeout)
        with self._frame_cond:
            while last_seq is not None and self._latest_seq == last_seq and not self._stop.is_set():
                remaining = deadline - time.perf_counter()
                if remaining <= 0.0:
                    break
                self._frame_cond.wait(timeout=remaining)
            return self._latest_jpeg, self._latest_seq

    def stop(self) -> None:
        self._stop.set()
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
        if self._http_thread is not None:
            self._http_thread.join(timeout=1.0)
            self._http_thread = None
        self._encode_thread.join(timeout=1.0)
        with self._frame_cond:
            self._frame_cond.notify_all()

    def _encode_loop(self) -> None:
        frame_interval = 1.0 / self.fps
        last_emit = 0.0
        while not self._stop.is_set():
            try:
                frame, snapshot, main_fps, infer_fps = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue

            now = time.perf_counter()
            sleep_time = frame_interval - (now - last_emit)
            if sleep_time > 0.0:
                time.sleep(sleep_time)

            debug_frame = draw_debug(frame, snapshot, main_fps, infer_fps)
            ok, encoded = cv2.imencode(
                ".jpg",
                debug_frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
            if not ok:
                continue
            last_emit = time.perf_counter()
            with self._frame_cond:
                self._latest_jpeg = encoded.tobytes()
                self._latest_seq += 1
                self._frame_cond.notify_all()

    def _encode_placeholder(self, text: str) -> bytes:
        canvas = np.full((480, 640, 3), 32, dtype=np.uint8)
        cv2.putText(canvas, self.title, (24, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        cv2.putText(canvas, text, (24, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (160, 220, 255), 2)
        ok, encoded = cv2.imencode(".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        return encoded.tobytes() if ok else b""

    def html_page(self) -> bytes:
        title = html.escape(self.title)
        body = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{title}</title>
  <style>
    body {{
      margin: 0;
      background: #111;
      color: #eee;
      font: 14px/1.4 Arial, sans-serif;
    }}
    .wrap {{
      max-width: 1100px;
      margin: 0 auto;
      padding: 16px;
    }}
    img {{
      width: 100%;
      height: auto;
      display: block;
      background: #000;
      border: 1px solid #333;
    }}
    .meta {{
      margin-top: 10px;
      color: #bbb;
    }}
    code {{
      color: #fff;
    }}
  </style>
</head>
<body>
  <div class="wrap">
    <h2>{title}</h2>
    <img src="/stream.mjpg" alt="preview stream">
    <div class="meta">MJPEG preview. Reload this page if the connection drops.</div>
  </div>
</body>
</html>
"""
        return body.encode("utf-8")


class _PreviewHTTPServer(ThreadingMixIn, server.HTTPServer):
    daemon_threads = True

    def __init__(self, server_address: tuple[str, int], handler_class: type[server.BaseHTTPRequestHandler], preview: PreviewServer) -> None:
        self.preview = preview
        super().__init__(server_address, handler_class)


class _PreviewHandler(server.BaseHTTPRequestHandler):
    server_version = "RKPreview/1.0"

    def do_GET(self) -> None:
        if self.path in ("/", "/index.html"):
            content = self.server.preview.html_page()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)
            return
        if self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.send_header("Pragma", "no-cache")
            self.send_header("Connection", "close")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last_seq: int | None = None
            while True:
                frame, last_seq = self.server.preview.get_frame(last_seq, timeout=5.0)
                if not frame:
                    continue
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    break
            return
        if self.path == "/health":
            body = b"ok"
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)

    def log_message(self, format: str, *args: object) -> None:
        return
