from __future__ import annotations

from collections.abc import Sequence
import math
from typing import Any

import cv2
import numpy as np

from .postprocess import box_iou, clip_box
from .types import Detection


RECT_KERNEL = np.ones((3, 3), dtype=np.uint8)


def refine_rectangle(frame: np.ndarray, box: Sequence[float]) -> dict[str, Any] | None:
    frame_h, frame_w = frame.shape[:2]
    x1, y1, x2, y2 = [float(v) for v in box]
    bw, bh = max(1.0, x2 - x1), max(1.0, y2 - y1)
    margin = max(2.0, min(bw, bh) * 0.12)

    rx1 = int(max(0, math.floor(x1 - margin)))
    ry1 = int(max(0, math.floor(y1 - margin)))
    rx2 = int(min(frame_w, math.ceil(x2 + margin)))
    ry2 = int(min(frame_h, math.ceil(y2 + margin)))
    if rx2 - rx1 < 12 or ry2 - ry1 < 12:
        return None

    roi = frame[ry1:ry2, rx1:rx2]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    median = float(np.median(blurred))
    low = max(20, int(0.66 * median))
    high = min(220, max(low + 35, int(1.33 * median)))
    edges = cv2.Canny(blurred, low, high)
    edges = cv2.dilate(edges, RECT_KERNEL, iterations=1)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    roi_area = float((rx2 - rx1) * (ry2 - ry1))
    min_area = max(80.0, roi_area * 0.06)
    best: dict[str, Any] | None = None
    best_score = 0.0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue

        rect = cv2.minAreaRect(contour)
        (cx, cy), (rw, rh), _ = rect
        if rw < 6 or rh < 6:
            continue

        rect_area = max(float(rw * rh), 1.0)
        fill = min(1.0, float(area) / rect_area)
        area_ratio = min(1.0, float(area) / max(roi_area, 1.0))

        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
        quad_bonus = 1.0 if len(approx) == 4 and cv2.isContourConvex(approx) else 0.72

        aspect = max(rw, rh) / max(min(rw, rh), 1.0)
        aspect_score = 1.0 if aspect <= 6.0 else max(0.0, 1.0 - (aspect - 6.0) * 0.12)

        score = 0.45 * fill + 0.25 * area_ratio + 0.20 * quad_bonus + 0.10 * aspect_score
        if score > best_score:
            quad = cv2.boxPoints(rect)
            quad[:, 0] += rx1
            quad[:, 1] += ry1
            refined_box = clip_box(
                (
                    float(np.min(quad[:, 0])),
                    float(np.min(quad[:, 1])),
                    float(np.max(quad[:, 0])),
                    float(np.max(quad[:, 1])),
                ),
                frame_w,
                frame_h,
            )
            best = {
                "center": (float(cx + rx1), float(cy + ry1)),
                "box": refined_box,
                "quad": quad.astype(np.int32),
                "score": float(score),
            }
            best_score = score

    if best is None or best_score < 0.58:
        return None
    return best


def choose_stable_detection(
    frame: np.ndarray,
    detections: Sequence[Detection],
    prior: tuple[float, float, float, float] | None,
    geometry_candidates: int,
) -> Detection | None:
    if not detections:
        return None

    frame_h, frame_w = frame.shape[:2]
    diag = math.hypot(frame_w, frame_h)
    best: Detection | None = None
    best_score = -1.0
    prior_box = None
    if prior is not None:
        px, py, pw, ph = prior
        prior_box = (px - pw * 0.5, py - ph * 0.5, px + pw * 0.5, py + ph * 0.5)

    for rank, detection in enumerate(detections):
        candidate = Detection(
            box=detection.box,
            center=detection.center,
            conf=detection.conf,
            class_id=detection.class_id,
            count=detection.count,
        )
        geometry_score = 0.0
        if rank < geometry_candidates:
            refined = refine_rectangle(frame, candidate.box)
            if refined is not None:
                candidate.box = refined["box"]
                candidate.center = refined["center"]
                candidate.geometry_ok = True
                candidate.geometry_score = float(refined["score"])
                candidate.quad = refined["quad"]
                geometry_score = candidate.geometry_score

        score = float(candidate.conf)
        score += 0.20 * geometry_score if candidate.geometry_ok else -0.03

        if prior is not None and prior_box is not None:
            px, py, _, _ = prior
            dist = math.hypot(candidate.center[0] - px, candidate.center[1] - py) / max(diag, 1.0)
            score += 0.35 * max(0.0, 1.0 - dist * 3.0)
            score += 0.30 * box_iou(candidate.box, prior_box)

        if score > best_score:
            best_score = score
            best = candidate

    return best

