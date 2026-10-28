"""Per-person aim analysis: torso aim point, torso-only range, head clearance and hold reasons."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from .camera import Frame, Intrinsics
from .detect import HEAD, L_HIP, L_SHOULDER, R_HIP, R_SHOULDER, Person

TORSO = (L_SHOULDER, R_SHOULDER, R_HIP, L_HIP)        # polygon order


@dataclass
class Aim:
    person: Person
    quad: np.ndarray | None = None          # 4x2 torso corners, or None if not all four are visible
    aim_px: np.ndarray | None = None        # (u, v) on the chest/belly
    depth_z: float | None = None            # median depth inside the shrunken torso (camera z)
    range_m: float | None = None            # straight-line distance to the aim point
    valid_frac: float = 0.0                 # share of torso pixels with a depth reading
    approx_range: bool = False              # range came from the box middle, not the torso
    point_cam: np.ndarray | None = None     # aim point in camera frame, meters
    head_clear_m: float | None = None       # sideways distance from aim point to nearest own head keypoint
    other_head_m: float | None = None       # same for anyone else's head
    blocked_frac: float = 0.0               # share of the no-head zone with something in front of the torso
    holds: list[str] = field(default_factory=list)

    @property
    def fire_ok(self) -> bool:
        return not self.holds


def torso_quad(p: Person, min_conf: float) -> np.ndarray | None:
    idx = list(TORSO)
    if np.all(p.kconf[idx] >= min_conf):
        return p.kpts[idx].astype(np.float32)
    return None


def aim_point(quad: np.ndarray, frac: float) -> np.ndarray:
    shoulders = (quad[0] + quad[1]) / 2
    hips = (quad[2] + quad[3]) / 2
    return shoulders + frac * (hips - shoulders)


def shrink(poly: np.ndarray, s: float) -> np.ndarray:
    c = poly.mean(axis=0)
    return c + s * (poly - c)


def region_values(depth: np.ndarray, poly: np.ndarray) -> tuple[np.ndarray, float]:
    """Nonzero depth pixels inside a polygon, and their share of all pixels in it."""
    h, w = depth.shape
    x0, y0 = np.floor(poly.min(axis=0)).astype(int)
    x1, y1 = np.ceil(poly.max(axis=0)).astype(int)
    x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, w - 1), min(y1, h - 1)
    if x1 <= x0 or y1 <= y0:
        return np.empty(0, np.float32), 0.0
    roi = depth[y0:y1 + 1, x0:x1 + 1]
    mask = np.zeros(roi.shape, np.uint8)
    cv2.fillPoly(mask, [np.round(poly - [x0, y0]).astype(np.int32)], 1)
    vals = roi[mask.astype(bool)]
    if vals.size == 0:
        return np.empty(0, np.float32), 0.0
    good = vals[vals > 0]
    return good, good.size / vals.size


def region_depth(depth: np.ndarray, poly: np.ndarray) -> tuple[float | None, float]:
    """Median of the nonzero depth pixels inside a polygon -> (median or None, share of pixels with depth)."""
    good, frac = region_values(depth, poly)
    return (float(np.median(good)), frac) if good.size else (None, 0.0)


def disk(center, r: float, n: int = 24) -> np.ndarray:
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.stack([center[0] + r * np.cos(a), center[1] + r * np.sin(a)], axis=1).astype(np.float32)


def box_middle(box: np.ndarray) -> np.ndarray:
    """Fallback depth patch: middle 30% of the width, 25-55% down the box."""
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    return np.array([[x1 + 0.35 * w, y1 + 0.25 * h], [x2 - 0.35 * w, y1 + 0.25 * h],
                     [x2 - 0.35 * w, y1 + 0.55 * h], [x1 + 0.35 * w, y1 + 0.55 * h]], np.float32)


def point_depth(depth: np.ndarray, u: float, v: float, r: int = 3) -> float | None:
    h, w = depth.shape
    ui, vi = int(round(u)), int(round(v))
    patch = depth[max(vi - r, 0):min(vi + r + 1, h), max(ui - r, 0):min(ui + r + 1, w)]
    good = patch[patch > 0]
    return float(np.median(good)) if good.size else None


def head_points(p: Person, min_conf: float) -> list[np.ndarray]:
    return [p.kpts[i] for i in HEAD if p.kconf[i] >= min_conf]


def head_clearance(points, aim_px: np.ndarray, z_aim: float, depth: np.ndarray | None,
                   intr: Intrinsics) -> float | None:
    """Smallest sideways distance (m) between the aim line and any of these head/face keypoints.

    Measured at the nearer of the two depths, so a head leaning toward the camera in front of
    the chest counts as closer, not farther. None if there are no points."""
    best = None
    for u, v in points:
        z = z_aim
        if depth is not None:
            zh = point_depth(depth, u, v)
            if zh is not None:
                z = min(z, zh)
        d = math.hypot((u - aim_px[0]) / intr.fx, (v - aim_px[1]) / intr.fy) * z
        best = d if best is None else min(best, d)
    return best


def analyze(p: Person, frame: Frame, cfg, others=()) -> Aim:
    """Everything steps 1 and 2 draw for one person, plus the reasons not to fire at them.
    `others` are the other people in the frame: their heads count against the line of fire too."""
    a, cam = cfg.aim, cfg.camera
    out = Aim(p)
    out.quad = torso_quad(p, a.keypoint_min_conf)
    depth = frame.depth_m
    torso_vals = None

    if out.quad is not None:
        out.aim_px = aim_point(out.quad, a.aim_fraction)
        if depth is not None:
            torso_vals, out.valid_frac = region_values(depth, shrink(out.quad, a.torso_shrink))
            out.depth_z = float(np.median(torso_vals)) if torso_vals.size else None
    elif depth is not None:
        out.depth_z, out.valid_frac = region_depth(depth, box_middle(p.box))
        out.approx_range = out.depth_z is not None
    z = out.depth_z
    if z is not None and out.aim_px is not None and not out.approx_range:
        out.point_cam = frame.intr.deproject(out.aim_px[0], out.aim_px[1], z)
        out.range_m = float(np.linalg.norm(out.point_cam))        # off-axis people are farther than z
    elif z is not None:
        out.range_m = z

    h = out.holds
    if p.conf < cfg.detector.conf:
        h.append("low confidence")
    if out.quad is None:
        h.append("no torso")
    if depth is None:
        h.append("no depth")
    elif z is None or out.valid_frac < a.min_valid_depth_fraction:
        h.append("depth unreliable")
        out.point_cam = None
    elif not cam.min_depth_m <= z <= cam.max_depth_m:
        h.append(f"outside camera range ({z:.1f} m)")
        out.range_m = out.point_cam = None
    elif out.range_m < a.min_fire_range_m:
        h.append(f"too close (<{a.min_fire_range_m:g} m)")
    elif out.range_m > a.max_fire_range_m:
        h.append(f"too far (>{a.max_fire_range_m:g} m)")

    if out.point_cam is not None:
        mc, intr = a.keypoint_min_conf, frame.intr
        out.head_clear_m = head_clearance(head_points(p, mc), out.aim_px, z, depth, intr)
        if out.head_clear_m is None:
            if a.require_head_visible:
                h.append("head not visible")
        elif out.head_clear_m < a.head_clearance_m:
            h.append(f"head too close ({out.head_clear_m * 100:.0f} cm)")
        theirs = [pt for o in others if o is not p for pt in head_points(o, mc)]
        out.other_head_m = head_clearance(theirs, out.aim_px, z, depth, intr)
        if out.other_head_m is not None and out.other_head_m < a.head_clearance_m:
            h.append(f"someone else's head in the way ({out.other_head_m * 100:.0f} cm)")
        if torso_vals is not None and torso_vals.size:
            # anything well in front of the torso inside the no-head zone: a bystander, a raised hand, a door
            ref = float(np.percentile(torso_vals, 80))
            near, _ = region_values(depth, disk(out.aim_px, a.head_clearance_m * intr.fx / z))
            if near.size:
                out.blocked_frac = float(np.mean(near < ref - a.blocked_margin_m))
                if out.blocked_frac > a.blocked_fraction:
                    h.append(f"something in front ({out.blocked_frac:.0%})")
    return out
