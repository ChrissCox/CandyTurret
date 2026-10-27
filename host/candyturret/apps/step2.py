"""Step 2: torso aim point + tracking. Nothing moves or fires yet.

Raise both hands above your head for a second to opt in. The turret will only ever lock onto
people who have opted in. The banner shows whether it WOULD fire at the locked target right now.
"""
from __future__ import annotations

import time

import cv2
import numpy as np

from .. import draw as D
from ..camera import open_camera
from ..detect import PoseDetector
from ..perception import analyze, shrink
from ..tracking import PeopleTracker, TargetSelector
from . import save_capture, show

TITLE = "CandyTurret step 2: aim point + tracking (q quits)"


def project(intr, p):
    return np.array([intr.fx * p[0] / p[2] + intr.cx, intr.fy * p[1] / p[2] + intr.cy])


def run(cfg, args):
    cam = open_camera(cfg, args.source, args.bag, args.record, args.fake_depth)
    d = cfg.detector
    det = PoseDetector(d.model, d.imgsz, d.conf, d.device, d.tracker)
    tracker = PeopleTracker(cfg)
    selector = TargetSelector(cfg)
    fps = D.FpsMeter()
    show_depth = False
    mc = cfg.aim.keypoint_min_conf
    if not args.headless:
        cv2.namedWindow(TITLE, cv2.WINDOW_NORMAL)
    n = 0
    try:
        while True:
            frame = cam.read()
            if frame is None:
                if not args.headless and show(TITLE, D.waiting(), args, 0, "step2") == -2:
                    break
                continue
            if frame.reset:                                  # recording looped or camera restarted
                tracker.reset()
                selector.clear()
                det.reset_tracker()
            t0 = time.perf_counter()
            people = det(frame.color, track=True)
            infer_ms = (time.perf_counter() - t0) * 1000
            aims = [analyze(p, frame, cfg, people) for p in people]
            tracks = tracker.update(aims, frame.t)
            center = (frame.intr.cx, frame.intr.cy)
            target = selector.update(tracks, frame.t, center)
            img = frame.color.copy()

            for tr in tracks:
                if tr.last_seen != frame.t:
                    continue
                a, p = tr.aim, tr.aim.person
                is_target = target is not None and tr.id == target.id
                color = D.YELLOW if is_target else (D.GREEN if tr.opted_in else D.GRAY)
                D.box(img, p.box, color, 3 if is_target else 1)
                D.skeleton(img, p.kpts, p.kconf, mc, color if is_target else D.GRAY)
                rng = f"{a.range_m:.2f} m" if a.range_m is not None else "-- m"
                tag = "IN" if tr.opted_in else "hands up to join"
                D.text(img, f"#{tr.id} {rng} {tag}", (p.box[0] + 2, p.box[1] - 6), color, 0.55, 1)
                if not tr.opted_in and tr.hands_up_since is not None:
                    D.bar(img, (p.box[0] + 2, p.box[1] + 4), tr.opt_in_progress(frame.t, cfg.tracking.opt_in_hold_s), D.GREEN)
                if a.aim_px is not None and not is_target:
                    cv2.circle(img, D.ipt(a.aim_px), 4, color, -1)

            if target is not None:
                a = target.aim
                lost = frame.t - target.last_seen
                if a.quad is not None and lost == 0:
                    D.poly(img, shrink(a.quad, cfg.aim.torso_shrink), D.CYAN)
                if a.aim_px is not None and lost == 0:
                    D.crosshair(img, a.aim_px, D.GREEN if a.fire_ok else D.RED)
                    if a.range_m:
                        r = int(cfg.aim.head_clearance_m * frame.intr.fx / a.range_m)
                        cv2.circle(img, D.ipt(a.aim_px), r, D.RED, 1, cv2.LINE_AA)   # no-head zone
                if target.kf is not None:
                    v = target.kf.velocity
                    p0 = target.kf.position
                    p1 = p0 + v * 0.5                                         # where they'll be in 0.5 s
                    if p0[2] > 0.1 and p1[2] > 0.1:
                        cv2.arrowedLine(img, D.ipt(project(frame.intr, p0)), D.ipt(project(frame.intr, p1)),
                                        D.YELLOW, 2, tipLength=0.2)
                    D.text(img, f"speed {np.linalg.norm(v):.1f} m/s", (target.aim.person.box[0] + 2, target.aim.person.box[3] + 16), D.YELLOW)
                if lost != 0:
                    D.banner(img, f"HOLD: target #{target.id} lost {abs(lost):.1f} s", D.RED)
                elif a.fire_ok:
                    D.banner(img, f"WOULD FIRE at #{target.id}  ({a.range_m:.2f} m)", D.GREEN)
                else:
                    D.banner(img, "HOLD: " + ", ".join(a.holds), D.RED)
            elif any(tr.last_seen == frame.t for tr in tracks):
                D.banner(img, "Raise both hands for 1 s to join", D.WHITE)

            D.hud(img, [f"{fps.tick(time.perf_counter()):4.1f} fps   detector {infer_ms:4.0f} ms",
                        "[n] next target  [c] re-pick nearest  [d] depth  [s] screenshot  [q] quit"])
            if args.fake_depth is not None:
                D.text(img, f"FAKE DEPTH {args.fake_depth} m", (8, img.shape[0] - 50), D.RED)
            view = np.hstack([img, D.depth_view(frame.depth_m, img.shape, cfg.camera.max_depth_m)]) if show_depth else img
            n += 1
            key = show(TITLE, view, args, n, "step2")
            if key == -2:
                break
            if key == ord("n"):
                selector.cycle(tracks, frame.t)
            elif key == ord("c"):
                selector.clear()
            elif key == ord("d"):
                show_depth = not show_depth
            elif key == ord("s"):
                print("saved", save_capture(view, "step2"))
    finally:
        cam.close()
        cv2.destroyAllWindows()
