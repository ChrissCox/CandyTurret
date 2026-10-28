"""Tracks across frames: 3D smoothing + velocity, the opt-in gesture, and a sticky target lock.

Track ids come from the YOLO tracker (ByteTrack). Ids can swap when people cross, so opt-in is
dropped whenever a track's 3D position jumps further than a person can move.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .detect import HEAD, L_HIP, L_SHOULDER, L_WRIST, R_HIP, R_SHOULDER, R_WRIST, Person
from .perception import Aim

HUMAN_SPEED = 2.5                     # m/s a person can plausibly move between 3D fixes


class KalmanCV3:
    """Constant-velocity Kalman filter on a 3D point, meters and seconds."""

    def __init__(self, p: np.ndarray, t: float, accel_noise: float):
        self.x = np.zeros(6)
        self.x[:3] = p
        self.P = np.diag([0.05 ** 2] * 3 + [1.5 ** 2] * 3)
        self.t = t
        self.q = accel_noise

    def predict(self, t: float) -> np.ndarray:
        dt = t - self.t
        if dt > 0:
            F = np.eye(6)
            F[:3, 3:] = np.eye(3) * dt
            Q = np.zeros((6, 6))
            for i in range(3):
                Q[i, i] = dt ** 4 / 4
                Q[i, i + 3] = Q[i + 3, i] = dt ** 3 / 2
                Q[i + 3, i + 3] = dt ** 2
            self.x = F @ self.x
            self.P = F @ self.P @ F.T + Q * self.q ** 2
            self.t = t
        return self.x[:3].copy()

    def update(self, z: np.ndarray, R: np.ndarray):
        H = np.hstack([np.eye(3), np.zeros((3, 3))])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - H @ self.x)
        self.P = (np.eye(6) - K @ H) @ self.P

    @property
    def position(self) -> np.ndarray:
        return self.x[:3].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.x[3:].copy()

    def at(self, t: float) -> np.ndarray:
        return self.x[:3] + self.x[3:] * (t - self.t)


def measurement_noise(p: np.ndarray) -> np.ndarray:
    """Camera-frame noise for a torso aim point at depth z. Depth term follows the D435's
    error-grows-with-z-squared model; the sideways term is keypoint jitter."""
    z = float(p[2])
    lateral = 0.03 + 0.005 * z
    depth = 0.02 + 0.0036 * z * z
    return np.diag([lateral ** 2, lateral ** 2, depth ** 2])


def hands_up(p: Person, min_conf: float) -> bool:
    """Opt-in gesture: both wrists clearly above the top of the head (image y grows downward).

    Face keypoints sit at eye/ear level, so the top of the head is estimated half the eye-to-shoulder
    distance above the highest one. Hands resting on the head stay below that line. With no face
    visible, the line is 0.7 torso lengths above the shoulders (torso length doesn't shrink when the
    person turns, unlike shoulder width)."""
    ok = lambda i: p.kconf[i] >= min_conf
    if not (ok(L_WRIST) and ok(R_WRIST) and ok(L_SHOULDER) and ok(R_SHOULDER)):
        return False
    shoulder_y = (p.kpts[L_SHOULDER][1] + p.kpts[R_SHOULDER][1]) / 2
    head_y = [p.kpts[i][1] for i in HEAD if ok(i)]
    if head_y:
        face_top = min(head_y)
        top = face_top - 0.5 * max(shoulder_y - face_top, 0.0)
    elif ok(L_HIP) and ok(R_HIP):
        torso = float(np.linalg.norm((p.kpts[L_SHOULDER] + p.kpts[R_SHOULDER]) / 2 - (p.kpts[L_HIP] + p.kpts[R_HIP]) / 2))
        top = shoulder_y - 0.7 * torso
    else:
        return False
    return p.kpts[L_WRIST][1] < top and p.kpts[R_WRIST][1] < top


@dataclass
class Track:
    id: int
    aim: Aim
    first_seen: float
    last_seen: float
    kf: KalmanCV3 | None = None
    last_point: np.ndarray | None = None
    last_point_t: float = 0.0
    opted_in: bool = False
    hands_up_since: float | None = None
    swaps: int = 0                    # times the jump guard fired

    def opt_in_progress(self, t: float, hold_s: float) -> float:
        if self.opted_in:
            return 1.0
        if self.hands_up_since is None:
            return 0.0
        return min(1.0, (t - self.hands_up_since) / hold_s)


class PeopleTracker:
    def __init__(self, cfg):
        self.tc = cfg.tracking
        self.min_conf = cfg.aim.keypoint_min_conf
        self.tracks: dict[int, Track] = {}

    def update(self, aims: list[Aim], t: float) -> list[Track]:
        tc = self.tc
        for a in aims:
            tid = a.person.track_id
            if tid is None:
                continue
            tr = self.tracks.get(tid)
            if tr is None:
                tr = self.tracks[tid] = Track(tid, a, t, t)
            gap = t - tr.last_seen
            tr.aim, tr.last_seen = a, t
            if gap > tc.opt_in_gap_s:                 # not watched continuously: may be someone else now
                tr.opted_in = False
                tr.hands_up_since = None

            if hands_up(a.person, self.min_conf):
                if tr.hands_up_since is None:
                    tr.hands_up_since = t
                if t - tr.hands_up_since >= tc.opt_in_hold_s:
                    tr.opted_in = True
            else:
                tr.hands_up_since = None

            p = a.point_cam
            if p is None:
                continue
            if tr.last_point is not None:
                allowed = tc.max_jump_m + HUMAN_SPEED * min(t - tr.last_point_t, tc.opt_in_gap_s)
                if np.linalg.norm(p - tr.last_point) > allowed:   # probably a different person now
                    tr.opted_in = False
                    tr.hands_up_since = None
                    tr.kf = None
                    tr.swaps += 1
            if tr.kf is None or gap > tc.lock_timeout_s:
                tr.kf = KalmanCV3(p, t, tc.accel_noise)
            else:
                tr.kf.predict(t)
                tr.kf.update(p, measurement_noise(p))
            tr.last_point, tr.last_point_t = tr.kf.position, t

        for tid in [k for k, tr in self.tracks.items() if t - tr.last_seen > tc.forget_s]:
            del self.tracks[tid]
        return list(self.tracks.values())

    def reset(self):
        """Forget everyone (camera restarted, recording looped)."""
        self.tracks.clear()


class TargetSelector:
    """Keeps one target locked until it is gone, so the turret doesn't flick between people."""

    def __init__(self, cfg):
        self.tc = cfg.tracking
        self.locked: int | None = None

    def eligible(self, tr: Track, t: float) -> bool:
        return (tr.opted_in or not self.tc.require_opt_in) and 0 <= t - tr.last_seen <= self.tc.lock_timeout_s

    def update(self, tracks: list[Track], t: float, center_px) -> Track | None:
        current = next((tr for tr in tracks if tr.id == self.locked), None)
        if current is not None and self.eligible(current, t):
            return current
        cands = [tr for tr in tracks if self.eligible(tr, t) and tr.last_seen == t and tr.aim.aim_px is not None]
        if not cands:
            self.locked = None
            return None
        best = min(cands, key=lambda tr: float(np.hypot(*(tr.aim.aim_px - np.asarray(center_px)))))
        self.locked = best.id
        return best

    def cycle(self, tracks: list[Track], t: float) -> Track | None:
        """Lock the next eligible person to the right (wraps around)."""
        cands = sorted((tr for tr in tracks
                        if self.eligible(tr, t) and tr.last_seen == t and tr.aim.aim_px is not None),
                       key=lambda tr: tr.aim.aim_px[0])
        if not cands:
            return None
        ids = [tr.id for tr in cands]
        i = (ids.index(self.locked) + 1) % len(ids) if self.locked in ids else 0
        self.locked = ids[i]
        return cands[i]

    def clear(self):
        self.locked = None
