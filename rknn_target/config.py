from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


@dataclass
class RuntimeConfig:
    model: Path
    input_size: int = 640
    target_class: int = 0
    class_count: int = 2
    conf_threshold: float = 0.45
    iou_threshold: float = 0.50
    max_det: int = 8
    min_box_size: float = 6.0
    core_mask: str = "auto"


@dataclass
class TrackerConfig:
    target_fps: float = 0.0
    track_alpha: float = 0.55
    track_beta: float = 0.20
    track_box_alpha: float = 0.45
    max_predict_misses: int = 3


@dataclass
class CameraConfig:
    camera: int | str = 0
    api: str = "v4l2"
    width: int = 640
    height: int = 480
    fps: int = 60
    fourcc: str = "MJPG"


def parse_camera(value: str) -> int | str:
    return int(value) if value.isdigit() else value
