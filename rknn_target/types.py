from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Detection:
    box: tuple[float, float, float, float]
    center: tuple[float, float]
    conf: float
    class_id: int
    count: int


@dataclass
class TrackSnapshot:
    active: bool = False
    box: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    center: tuple[float, float] = (0.0, 0.0)
    conf: float = 0.0
    count: int = 0
    state: str = "SEARCHING"
    misses: int = 0
    source: str = "none"


@dataclass
class InferTimings:
    preprocess_ms: float = 0.0
    rknn_ms: float = 0.0
    postprocess_ms: float = 0.0
    select_ms: float = 0.0
    total_ms: float = 0.0
