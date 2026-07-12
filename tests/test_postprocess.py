import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rknn_target.postprocess import decode_yolo_outputs, letterbox_to_square, nms_xyxy  # noqa: E402


def test_letterbox_maps_640x480_usb_frame_with_vertical_padding():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    dst = np.empty((640, 640, 3), dtype=np.uint8)

    scale, pad_x, pad_y = letterbox_to_square(frame, dst, 640)

    assert scale == 1.0
    assert pad_x == 0
    assert pad_y == 80


def test_decode_yolo_output_maps_letterbox_box_to_frame_pixels():
    output = np.zeros((1, 6, 8400), dtype=np.float32)
    output[0, 0, 0] = 320.0
    output[0, 1, 0] = 320.0
    output[0, 2, 0] = 100.0
    output[0, 3, 0] = 80.0
    output[0, 4, 0] = 0.9
    output[0, 5, 0] = 0.1

    detections = decode_yolo_outputs(
        [output],
        scale=1.0,
        pad_x=0,
        pad_y=80,
        frame_w=640,
        frame_h=480,
        input_size=640,
        target_class=0,
        class_count=2,
        conf_threshold=0.45,
        iou_threshold=0.5,
        max_det=4,
        min_box_size=6.0,
    )

    assert len(detections) == 1
    det = detections[0]
    assert det.class_id == 0
    assert math.isclose(det.conf, 0.9, rel_tol=1e-5)
    assert np.allclose(det.box, [270.0, 200.0, 370.0, 280.0])
    assert np.allclose(det.center, [320.0, 240.0])


def test_nms_keeps_best_and_removes_overlap():
    boxes = np.array(
        [
            [10.0, 10.0, 110.0, 110.0],
            [14.0, 14.0, 114.0, 114.0],
            [300.0, 300.0, 350.0, 350.0],
        ],
        dtype=np.float32,
    )
    scores = np.array([0.9, 0.8, 0.7], dtype=np.float32)

    keep = nms_xyxy(boxes, scores, iou_threshold=0.5, limit=8)

    assert keep == [0, 2]

