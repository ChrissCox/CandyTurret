"""Color + depth-aligned-to-color frames from the D435, or a stand-in source for testing without it."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


@dataclass
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def deproject(self, u: float, v: float, z: float) -> np.ndarray:
        """Pixel (u, v) at depth z meters -> camera-frame point (x right, y down, z forward).
        Pinhole only: the D435 color stream reports zero distortion, so this matches
        rs2_deproject_pixel_to_point there."""
        return np.array([(u - self.cx) * z / self.fx, (v - self.cy) * z / self.fy, z])


@dataclass
class Frame:
    color: np.ndarray            # HxWx3 BGR uint8
    depth_m: np.ndarray | None   # HxW float32 meters, aligned to color; 0 = no reading
    intr: Intrinsics
    t: float                     # seconds, camera clock when there is one
    index: int
    reset: bool = False          # time jumped back (recording looped, camera restarted): forget tracks


class RealSenseCamera:
    def __init__(self, width=848, height=480, fps=30, bag: str | None = None, record: str | None = None):
        import pyrealsense2 as rs
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        if bag:
            cfg.enable_device_from_file(str(bag), repeat_playback=True)
        else:
            cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
            cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
            if record:
                cfg.enable_record_to_file(str(record))
        self.cfg = cfg
        profile = self.pipeline.start(cfg)
        dev = profile.get_device()
        self.depth_scale = dev.first_depth_sensor().get_depth_scale()
        self.usb = dev.get_info(rs.camera_info.usb_type_descriptor) if dev.supports(rs.camera_info.usb_type_descriptor) else "?"
        if not bag and not self.usb.startswith("3"):
            print(f"WARNING: D435 is on USB {self.usb}, not USB 3. Depth will be slow or fail; "
                  "reseat the USB-C plug or use a USB 3 port and cable.")
        self.align = rs.align(rs.stream.color)
        i = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.intr = Intrinsics(i.width, i.height, i.fx, i.fy, i.ppx, i.ppy)
        self.rgb = bag is not None and profile.get_stream(rs.stream.color).format() == rs.format.rgb8
        self._n = 0
        self._misses = 0
        self._last_t = None
        self._restarted = False

    def read(self) -> Frame | None:
        """None when no frame came within 200 ms. After ~5 s of nothing, restarts the pipeline."""
        try:
            ok, frames = self.pipeline.try_wait_for_frames(200)
        except RuntimeError:                 # pipeline not running: the last restart failed
            ok = False
            time.sleep(0.2)
        if not ok:
            self._misses += 1
            if self._misses >= 25:
                self._misses = 0
                self._restart()
            return None
        self._misses = 0
        frames = self.align.process(frames)
        c, d = frames.get_color_frame(), frames.get_depth_frame()
        if not c or not d:
            return None
        color = np.asanyarray(c.get_data()).copy()
        if self.rgb:
            color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
        depth = np.asanyarray(d.get_data()).astype(np.float32) * self.depth_scale
        self._n += 1
        t = c.get_timestamp() / 1000.0
        reset = self._restarted or (self._last_t is not None and t <= self._last_t)
        self._last_t, self._restarted = t, False
        return Frame(color, depth, self.intr, t, self._n, reset)

    def _restart(self):
        print("D435: no frames for 5 s, restarting the camera pipeline")
        try:
            self.pipeline.stop()
        except RuntimeError:
            pass
        try:
            self.pipeline.start(self.cfg)
            self._restarted = True
        except RuntimeError as e:
            print("D435 restart failed:", e)

    def close(self):
        try:
            self.pipeline.stop()
        except RuntimeError:                 # already stopped by a failed restart
            pass


class VideoCamera:
    """A video file, webcam index or still image, for working without the D435.
    Depth is either missing or a flat fake value, so distances are not real."""

    def __init__(self, source: str, fake_depth_m: float | None = None, hfov_deg: float = 69.0):
        self.still = None
        if Path(source).suffix.lower() in IMAGE_SUFFIXES:
            self.still = cv2.imread(source)
            if self.still is None:
                raise FileNotFoundError(source)
            h, w = self.still.shape[:2]
        else:
            self.cap = cv2.VideoCapture(int(source) if source.isdigit() else source)
            if not self.cap.isOpened():
                raise FileNotFoundError(source)
            w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self.live = source.isdigit()                          # webcam: wall clock; files: media time
        f = (w / 2) / math.tan(math.radians(hfov_deg / 2))   # D435 color is about 69 deg wide
        self.intr = Intrinsics(w, h, f, f, w / 2, h / 2)
        self.fake_depth = fake_depth_m
        self._n = 0
        self._t0 = time.monotonic()

    def read(self) -> Frame | None:
        reset = False
        if self.still is not None:
            img = self.still.copy()
            time.sleep(1 / 30)
            t = self._n / 30
        else:
            ok, img = self.cap.read()
            if not ok:                                   # loop the file
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, img = self.cap.read()
                if not ok:
                    return None
                reset = True
            t = time.monotonic() - self._t0 if self.live else self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000
        depth = None
        if self.fake_depth is not None:
            depth = np.full(img.shape[:2], self.fake_depth, np.float32)
        self._n += 1
        return Frame(img, depth, self.intr, t, self._n, reset)

    def close(self):
        if self.still is None:
            self.cap.release()


def open_camera(cfg, source: str = "realsense", bag: str | None = None, record: str | None = None,
                fake_depth_m: float | None = None):
    if bag or source == "realsense":
        c = cfg.camera
        return RealSenseCamera(c.width, c.height, c.fps, bag=bag, record=record)
    return VideoCamera(source, fake_depth_m)
