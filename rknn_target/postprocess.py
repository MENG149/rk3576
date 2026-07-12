from __future__ import annotations

from collections.abc import Sequence

import cv2
import numpy as np

from .types import Detection


def clip_box(
    box: Sequence[float], frame_w: int, frame_h: int
) -> tuple[float, float, float, float]:
    x1, y1, x2, y2 = [float(v) for v in box]
    x1 = max(0.0, min(float(frame_w - 1), x1))
    y1 = max(0.0, min(float(frame_h - 1), y1))
    x2 = max(0.0, min(float(frame_w - 1), x2))
    y2 = max(0.0, min(float(frame_h - 1), y2))
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1
    return x1, y1, x2, y2


def box_center(box: Sequence[float]) -> tuple[float, float]:
    x1, y1, x2, y2 = box
    return (float(x1) + float(x2)) * 0.5, (float(y1) + float(y2)) * 0.5


def box_size(box: Sequence[float]) -> tuple[float, float]:
    x1, y1, x2, y2 = box
    return max(1.0, float(x2) - float(x1)), max(1.0, float(y2) - float(y1))


def box_iou(a: Sequence[float], b: Sequence[float]) -> float:
    ax1, ay1, ax2, ay2 = [float(v) for v in a]
    bx1, by1, bx2, by2 = [float(v) for v in b]
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    intersection = iw * ih
    if intersection <= 0.0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    return intersection / max(area_a + area_b - intersection, 1e-6)


def nms_xyxy(
    boxes: np.ndarray, scores: np.ndarray, iou_threshold: float, limit: int
) -> list[int]:
    if boxes.size == 0:
        return []

    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]

    keep: list[int] = []
    while order.size > 0 and len(keep) < limit:
        idx = int(order[0])
        keep.append(idx)
        if order.size == 1:
            break

        rest = order[1:]
        xx1 = np.maximum(x1[idx], x1[rest])
        yy1 = np.maximum(y1[idx], y1[rest])
        xx2 = np.minimum(x2[idx], x2[rest])
        yy2 = np.minimum(y2[idx], y2[rest])
        inter_w = np.maximum(0.0, xx2 - xx1)
        inter_h = np.maximum(0.0, yy2 - yy1)
        inter = inter_w * inter_h
        union = areas[idx] + areas[rest] - inter
        iou = np.where(union > 0.0, inter / union, 0.0)
        order = rest[iou <= iou_threshold]

    return keep


def letterbox_to_square(
    src_bgr: np.ndarray, dst_rgb: np.ndarray, input_size: int
) -> tuple[float, int, int]:
    frame_h, frame_w = src_bgr.shape[:2]
    scale = min(input_size / frame_w, input_size / frame_h)
    new_w = max(1, int(round(frame_w * scale)))
    new_h = max(1, int(round(frame_h * scale)))
    pad_x = (input_size - new_w) // 2
    pad_y = (input_size - new_h) // 2

    _fill_letterbox_padding(dst_rgb, pad_x, pad_y, new_w, new_h)
    roi = dst_rgb[pad_y : pad_y + new_h, pad_x : pad_x + new_w]
    if new_w == frame_w and new_h == frame_h:
        cv2.cvtColor(src_bgr, cv2.COLOR_BGR2RGB, dst=roi)
    else:
        interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
        cv2.resize(src_bgr, (new_w, new_h), dst=roi, interpolation=interpolation)
        cv2.cvtColor(roi, cv2.COLOR_BGR2RGB, dst=roi)
    return scale, pad_x, pad_y


def _fill_letterbox_padding(dst_rgb: np.ndarray, pad_x: int, pad_y: int, new_w: int, new_h: int) -> None:
    bottom = pad_y + new_h
    right = pad_x + new_w
    if pad_y > 0:
        dst_rgb[:pad_y, :, :] = 114
    if bottom < dst_rgb.shape[0]:
        dst_rgb[bottom:, :, :] = 114
    if pad_x > 0:
        dst_rgb[pad_y:bottom, :pad_x, :] = 114
    if right < dst_rgb.shape[1]:
        dst_rgb[pad_y:bottom, right:, :] = 114


def _prediction_matrix(outputs: Sequence[np.ndarray]) -> np.ndarray:
    matrices: list[np.ndarray] = []
    for output in outputs:
        arr = np.asarray(output)
        arr = np.squeeze(arr)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        if arr.ndim != 2:
            continue
        if arr.shape[0] <= 256 and arr.shape[0] < arr.shape[1]:
            arr = arr.T
        matrices.append(arr.astype(np.float32, copy=False))

    if not matrices:
        return np.empty((0, 0), dtype=np.float32)
    if len(matrices) == 1:
        return matrices[0]

    width = matrices[0].shape[1]
    compatible = [item for item in matrices if item.shape[1] == width]
    if not compatible:
        return np.empty((0, 0), dtype=np.float32)
    return np.concatenate(compatible, axis=0)


def decode_yolo_outputs(
    outputs: Sequence[np.ndarray],
    *,
    scale: float,
    pad_x: int,
    pad_y: int,
    frame_w: int,
    frame_h: int,
    input_size: int,
    target_class: int,
    class_count: int,
    conf_threshold: float,
    iou_threshold: float,
    max_det: int,
    min_box_size: float,
) -> list[Detection]:
    pred = _prediction_matrix(outputs)
    if pred.shape[1] < 5:
        return []

    coords = pred[:, :4].astype(np.float32, copy=False)
    finite_mask = np.isfinite(coords).all(axis=1)
    if not np.any(finite_mask):
        return []

    coords = coords[finite_mask]
    rest = pred[finite_mask, 4:].astype(np.float32, copy=False)
    if rest.size == 0:
        return []

    if np.nanmax(coords) <= 2.0:
        coords = coords * float(input_size)

    if class_count > 0 and rest.shape[1] == class_count:
        class_scores = rest
    elif class_count > 0 and rest.shape[1] >= class_count + 1:
        objectness = rest[:, 0:1]
        class_scores = rest[:, 1 : 1 + class_count] * objectness
    else:
        class_scores = rest

    if target_class < 0:
        class_ids = np.argmax(class_scores, axis=1)
        scores = class_scores[np.arange(class_scores.shape[0]), class_ids]
    elif target_class < class_scores.shape[1]:
        class_ids = np.full(class_scores.shape[0], target_class, dtype=np.int32)
        scores = class_scores[:, target_class]
    else:
        return []

    mask = np.isfinite(scores) & (scores >= conf_threshold)
    if not np.any(mask):
        return []

    coords = coords[mask]
    scores = scores[mask]
    class_ids = class_ids[mask]

    cx = coords[:, 0]
    cy = coords[:, 1]
    bw = coords[:, 2]
    bh = coords[:, 3]
    boxes = np.stack((cx - bw * 0.5, cy - bh * 0.5, cx + bw * 0.5, cy + bh * 0.5), axis=1)

    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - float(pad_x)) / max(scale, 1e-6)
    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - float(pad_y)) / max(scale, 1e-6)
    boxes[:, 0] = np.clip(boxes[:, 0], 0.0, float(frame_w - 1))
    boxes[:, 1] = np.clip(boxes[:, 1], 0.0, float(frame_h - 1))
    boxes[:, 2] = np.clip(boxes[:, 2], 0.0, float(frame_w - 1))
    boxes[:, 3] = np.clip(boxes[:, 3], 0.0, float(frame_h - 1))

    sizes = boxes[:, 2:4] - boxes[:, 0:2]
    size_mask = (sizes[:, 0] >= min_box_size) & (sizes[:, 1] >= min_box_size)
    if not np.any(size_mask):
        return []

    boxes = boxes[size_mask]
    scores = scores[size_mask]
    class_ids = class_ids[size_mask]

    if max_det <= 1:
        keep = [int(np.argmax(scores))]
    else:
        keep = nms_xyxy(boxes, scores, iou_threshold, max_det)
    detections: list[Detection] = []
    candidate_count = int(len(scores))
    for idx in keep:
        box = tuple(float(v) for v in boxes[idx])
        detections.append(
            Detection(
                box=box,
                center=box_center(box),
                conf=float(scores[idx]),
                class_id=int(class_ids[idx]),
                count=candidate_count,
            )
        )
    return detections
