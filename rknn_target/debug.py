from __future__ import annotations

import cv2
import numpy as np

from .types import TrackSnapshot


def draw_debug(frame: np.ndarray, snapshot: TrackSnapshot, main_fps: float, infer_fps: float) -> np.ndarray:
    overlay = frame.copy()
    frame_h, frame_w = overlay.shape[:2]
    cx0, cy0 = frame_w // 2, frame_h // 2
    cv2.line(overlay, (cx0 - 18, cy0), (cx0 + 18, cy0), (80, 80, 80), 1)
    cv2.line(overlay, (cx0, cy0 - 18), (cx0, cy0 + 18), (80, 80, 80), 1)

    if snapshot.active:
        x1, y1, x2, y2 = [int(round(v)) for v in snapshot.box]
        color = (0, 180, 255)
        cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
        cx, cy = int(round(snapshot.center[0])), int(round(snapshot.center[1]))
        cv2.drawMarker(overlay, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
        cv2.putText(
            overlay,
            f"{snapshot.state} conf={snapshot.conf:.2f} count={snapshot.count}",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
        )
    else:
        cv2.putText(overlay, "SEARCHING", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.putText(
        overlay,
        f"main={main_fps:.1f} infer={infer_fps:.1f}",
        (12, 56),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2,
    )
    return overlay
