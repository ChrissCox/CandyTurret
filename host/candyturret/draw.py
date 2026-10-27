"""OpenCV drawing helpers shared by the step apps."""
from __future__ import annotations

import cv2
import numpy as np

from .detect import SKELETON

GREEN = (90, 220, 90)
RED = (70, 70, 235)
YELLOW = (0, 215, 255)
ORANGE = (0, 150, 255)
CYAN = (230, 210, 60)
WHITE = (240, 240, 240)
GRAY = (150, 150, 150)
DARK = (25, 25, 25)
FONT = cv2.FONT_HERSHEY_SIMPLEX


def ipt(p) -> tuple[int, int]:
    return int(round(float(p[0]))), int(round(float(p[1])))


def text(img, s: str, org, color=WHITE, scale=0.5, thick=1, bg=True):
    (w, h), base = cv2.getTextSize(s, FONT, scale, thick)
    x, y = ipt(org)
    if bg:
        cv2.rectangle(img, (x - 2, y - h - 3), (x + w + 2, y + base), DARK, -1)
    cv2.putText(img, s, (x, y), FONT, scale, color, thick, cv2.LINE_AA)


def box(img, b, color, thick=2):
    cv2.rectangle(img, ipt(b[:2]), ipt(b[2:]), color, thick)


def skeleton(img, kpts, kconf, min_conf, color=GRAY):
    for a, b in SKELETON:
        if kconf[a] >= min_conf and kconf[b] >= min_conf:
            cv2.line(img, ipt(kpts[a]), ipt(kpts[b]), color, 1, cv2.LINE_AA)
    for (x, y), c in zip(kpts, kconf):
        if c >= min_conf:
            cv2.circle(img, ipt((x, y)), 2, color, -1)


def poly(img, pts, color, thick=1):
    cv2.polylines(img, [np.round(pts).astype(np.int32)], True, color, thick, cv2.LINE_AA)


def crosshair(img, p, color, r=12):
    x, y = ipt(p)
    cv2.circle(img, (x, y), r, color, 2, cv2.LINE_AA)
    cv2.line(img, (x - 2 * r, y), (x - r // 2, y), color, 2)
    cv2.line(img, (x + r // 2, y), (x + 2 * r, y), color, 2)
    cv2.line(img, (x, y - 2 * r), (x, y - r // 2), color, 2)
    cv2.line(img, (x, y + r // 2), (x, y + 2 * r), color, 2)


def bar(img, org, frac, color, w=60, h=6):
    x, y = ipt(org)
    cv2.rectangle(img, (x, y), (x + w, y + h), DARK, -1)
    cv2.rectangle(img, (x, y), (x + int(w * max(0.0, min(1.0, frac))), y + h), color, -1)


def hud(img, lines, org=(8, 20), scale=0.5):
    x, y = org
    for s in lines:
        text(img, s, (x, y), WHITE, scale)
        y += int(22 * scale / 0.5)


def banner(img, s: str, color):
    (w, h), _ = cv2.getTextSize(s, FONT, 0.8, 2)
    x = (img.shape[1] - w) // 2
    y = img.shape[0] - 18
    cv2.rectangle(img, (x - 10, y - h - 10), (x + w + 10, y + 10), DARK, -1)
    cv2.putText(img, s, (x, y), FONT, 0.8, color, 2, cv2.LINE_AA)


def depth_view(depth_m: np.ndarray | None, shape, max_m: float = 6.0) -> np.ndarray:
    if depth_m is None:
        out = np.zeros((*shape[:2], 3), np.uint8)
        text(out, "no depth", (10, 30))
        return out
    d = np.clip(depth_m / max_m, 0, 1)
    img = cv2.applyColorMap((255 - d * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[depth_m <= 0] = 0
    return img


def waiting(shape=(480, 848)) -> np.ndarray:
    img = np.full((*shape, 3), 30, np.uint8)
    text(img, "waiting for camera frames...  (q quits)", (20, 40), YELLOW, 0.7, 2)
    return img


class FpsMeter:
    def __init__(self, alpha=0.1):
        self.alpha, self.dt, self.last = alpha, None, None

    def tick(self, now: float) -> float:
        if self.last is not None:
            dt = now - self.last
            self.dt = dt if self.dt is None else (1 - self.alpha) * self.dt + self.alpha * dt
        self.last = now
        return 1.0 / self.dt if self.dt else 0.0
