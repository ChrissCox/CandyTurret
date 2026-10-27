"""YOLO pose model: person boxes, 17 COCO keypoints and (optionally) tracker ids in one pass."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

# COCO keypoint indices
NOSE, L_EYE, R_EYE, L_EAR, R_EAR = 0, 1, 2, 3, 4
L_SHOULDER, R_SHOULDER, L_ELBOW, R_ELBOW, L_WRIST, R_WRIST = 5, 6, 7, 8, 9, 10
L_HIP, R_HIP, L_KNEE, R_KNEE, L_ANKLE, R_ANKLE = 11, 12, 13, 14, 15, 16
HEAD = (NOSE, L_EYE, R_EYE, L_EAR, R_EAR)
SKELETON = [(5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12), (11, 12),
            (11, 13), (13, 15), (12, 14), (14, 16), (0, 1), (0, 2), (1, 3), (2, 4)]

MODELS_DIR = Path(__file__).resolve().parents[1] / "models"


@dataclass
class Person:
    box: np.ndarray          # x1, y1, x2, y2 in pixels
    conf: float
    kpts: np.ndarray         # 17x2 pixels
    kconf: np.ndarray        # 17 keypoint confidences
    track_id: int | None = None


class PoseDetector:
    def __init__(self, model="yolo11n-pose.pt", imgsz=640, conf=0.4, device="auto", tracker="bytetrack.yaml"):
        from ultralytics import YOLO
        path = Path(model)
        if not path.is_absolute() and path.parent == Path("."):
            MODELS_DIR.mkdir(exist_ok=True)
            path = MODELS_DIR / path.name          # downloads land in host/models/
        self.model = YOLO(str(path))
        self.kw = dict(imgsz=imgsz, conf=conf, verbose=False)
        if device != "auto":
            self.kw["device"] = device
        self.tracker = tracker

    def __call__(self, bgr: np.ndarray, track: bool = False) -> list[Person]:
        if track:
            # ByteTrack wants the low-score boxes too (its second matching pass), so track at 0.1 and
            # let perception hold fire on anyone under the configured confidence.
            r = self.model.track(bgr, persist=True, tracker=self.tracker, **dict(self.kw, conf=0.1))[0]
        else:
            r = self.model.predict(bgr, **self.kw)[0]
        return people_from_result(r)

    def reset_tracker(self):
        """Drop all tracker ids (camera restarted, recording looped)."""
        predictor = getattr(self.model, "predictor", None)
        for tr in getattr(predictor, "trackers", None) or []:
            tr.reset()


def people_from_result(r) -> list[Person]:
    if r.boxes is None or len(r.boxes) == 0:
        return []
    keep = r.boxes.cls.cpu().numpy() == 0                  # person class only
    boxes = r.boxes.xyxy.cpu().numpy()
    confs = r.boxes.conf.cpu().numpy()
    ids = r.boxes.id.int().cpu().numpy() if r.boxes.id is not None else None
    if r.keypoints is not None and r.keypoints.conf is not None:
        kxy, kc = r.keypoints.xy.cpu().numpy(), r.keypoints.conf.cpu().numpy()
        kc = np.where((kxy == 0).all(-1), 0.0, kc)          # Ultralytics parks keypoints under 0.5 at (0, 0)
    else:
        kxy, kc = np.zeros((len(boxes), 17, 2)), np.zeros((len(boxes), 17))
    return [Person(boxes[i], float(confs[i]), kxy[i], kc[i], None if ids is None else int(ids[i]))
            for i in range(len(boxes)) if keep[i]]
