import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rknn_target.config import TrackerConfig  # noqa: E402
from rknn_target.tracker import RectTracker  # noqa: E402
from rknn_target.types import Detection  # noqa: E402


def test_tracker_holds_brief_miss_then_searches():
    tracker = RectTracker(TrackerConfig(max_predict_misses=1, target_fps=50.0))
    detection = Detection(
        box=(90.0, 95.0, 130.0, 135.0),
        center=(110.0, 115.0),
        conf=0.8,
        class_id=0,
        count=1,
    )

    tracker.update(detection, timestamp=1.0, frame_shape=(480, 640, 3))
    locked = tracker.snapshot()
    tracker.miss(timestamp=1.02, frame_shape=(480, 640, 3))
    lost = tracker.snapshot()
    tracker.miss(timestamp=1.04, frame_shape=(480, 640, 3))
    searching = tracker.snapshot()

    assert locked.active
    assert locked.state == "LOCKED"
    assert lost.active
    assert lost.state == "LOST"
    assert not searching.active
    assert searching.state == "SEARCHING"

