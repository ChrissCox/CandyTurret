"""Step 1: D435 streaming with person boxes and distances.

Distance = median depth inside the middle of the torso (shoulders to hips). If the torso isn't
visible it falls back to the middle of the box and shows "~" in front of the number.
"""
from __future__ import annotations

import time

import cv2
import numpy as np

from .. import draw as D
from ..camera import open_camera
from ..detect import PoseDetector
from ..perception import analyze, shrink
from . import save_capture, show

TITLE = "CandyTurret step 1: people + distance (q quits)"


def run(cfg, args):
    cam = open_camera(cfg, args.source, args.bag, args.record, args.fake_depth)
    d = cfg.detector
    det = PoseDetector(d.model, d.imgsz, d.conf, d.device, d.tracker)
    fps = D.FpsMeter()
    show_depth = False
    if not args.headless:
        cv2.namedWindow(TITLE, cv2.WINDOW_NORMAL)
    n = 0
    try:
        while True:
            frame = cam.read()
            if frame is None:
                if not args.headless and show(TITLE, D.waiting(), args, 0, "step1") == -2:
                    break
                continue
            t0 = time.perf_counter()
            people = det(frame.color)
            infer_ms = (time.perf_counter() - t0) * 1000
            img = frame.color.copy()
            for p in people:
                a = analyze(p, frame, cfg, people)
                ok = a.range_m is not None
                D.box(img, p.box, D.GREEN if ok else D.ORANGE)
                D.skeleton(img, p.kpts, p.kconf, cfg.aim.keypoint_min_conf)
                if a.quad is not None:
                    D.poly(img, shrink(a.quad, cfg.aim.torso_shrink), D.CYAN)
                if frame.depth_m is None:
                    label = "no depth"
                elif a.range_m is None:
                    label = "?" if not a.holds else a.holds[-1]
                else:
                    label = f"{'~' if a.approx_range else ''}{a.range_m:.2f} m"
                D.text(img, f"{label}  ({p.conf:.2f})", (p.box[0] + 2, p.box[1] - 6), D.GREEN if ok else D.ORANGE, 0.6, 2)
            D.hud(img, [f"{fps.tick(time.perf_counter()):4.1f} fps   detector {infer_ms:4.0f} ms",
                        f"{len(people)} people   [d] depth view  [s] screenshot  [q] quit"])
            if args.fake_depth is not None:
                D.text(img, f"FAKE DEPTH {args.fake_depth} m", (8, img.shape[0] - 10), D.RED)
            view = np.hstack([img, D.depth_view(frame.depth_m, img.shape, cfg.camera.max_depth_m)]) if show_depth else img
            n += 1
            key = show(TITLE, view, args, n, "step1")
            if key == -2:
                break
            if key == ord("d"):
                show_depth = not show_depth
            if key == ord("s"):
                print("saved", save_capture(view, "step1"))
    finally:
        cam.close()
        cv2.destroyAllWindows()
