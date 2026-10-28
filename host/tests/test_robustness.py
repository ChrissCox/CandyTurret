"""Regressions from the 2026-10-28 code review: each test pins one bug that was fixed."""
import numpy as np
import pytest

from candyturret import config
from candyturret import protocol as P
from candyturret.apps import DOWN, LEFT, RIGHT, UP, keycode, save_capture
from candyturret.camera import RealSenseCamera
from candyturret.link import TurretLink
from candyturret.perception import analyze
from candyturret.sim_esp32 import SimPort
from candyturret.tracking import PeopleTracker, TargetSelector

from .test_perception import H, W, depth_scene, frame, person
from .test_tracking import aim_for


class DeadPipeline:
    """A RealSense pipeline after a failed restart: every call raises."""

    def try_wait_for_frames(self, timeout_ms):
        raise RuntimeError("try_wait_for_frames cannot be called before start()")

    def stop(self):
        raise RuntimeError("stop() cannot be called before start()")

    def start(self, cfg):
        raise RuntimeError("No device connected")


def test_camera_survives_a_failed_restart(capsys):
    cam = RealSenseCamera.__new__(RealSenseCamera)
    cam.pipeline, cam.cfg = DeadPipeline(), None
    cam._misses, cam._restarted, cam._last_t, cam._n = 0, False, None, 0
    for _ in range(30):                                # crosses the 25-miss restart threshold
        assert cam.read() is None
    assert "restart failed" in capsys.readouterr().out
    cam.close()                                        # must not raise either


@pytest.mark.parametrize("raw,code", [(2424832, 2424832), (65361, 65361), (0x100000 | 65361, 65361),
                                      (0x10000 | 65363, 65363), (0x100000 | 65364, 65364), (2490368, 2490368)])
def test_arrow_keys_with_modifier_bits(raw, code):
    got, ch = keycode(raw)
    assert got == code and ch is None
    assert got in LEFT | UP | RIGHT | DOWN


@pytest.mark.parametrize("raw,ch", [(ord("a"), "a"), (0x10000 | ord("A"), "A"), (0x100000 | ord("q"), "q"), (27, "\x1b")])
def test_letters_with_modifier_bits(raw, ch):
    assert keycode(raw)[1] == ord(ch)


def test_no_key():
    assert keycode(-1) == (-1, None)


def test_screenshots_in_the_same_second_dont_overwrite():
    img = np.zeros((4, 4, 3), np.uint8)
    a, b = save_capture(img, "pytest"), save_capture(img, "pytest")
    try:
        assert a != b and a.exists() and b.exists()
    finally:
        a.unlink(missing_ok=True)
        b.unlink(missing_ok=True)


def test_ack_for_a_frame_we_never_sent_is_not_latency():
    link = TurretLink(SimPort(), setpoint_hz=1)
    try:
        link._sent_at[200] = 0.0
        tm = dict(t_ms=1, ack_seq=200, state=1, fault=0, switches=0, pan_cdeg=0, tilt_cdeg=0,
                  rpm=0, rpm_target=0, shots=0, press_id=0, vbat_cv=0)
        link.latency_s = None
        link._last_ack = None
        link._handle(P.Packet(P.Msg.TELEMETRY, 0, tm))
        assert link.latency_s is None
    finally:
        link.close()


def test_unreliable_depth_does_not_feed_the_tracker():
    cfg = config.load()
    p = person()
    d = depth_scene(p)
    d[::2, :] = 0                                      # 50% holes...
    d[1::2, ::2] = 0                                   # ...then 75%: under the 30% valid minimum
    a = analyze(p, frame(d), cfg)
    assert "depth unreliable" in a.holds and a.point_cam is None


def test_next_target_skips_people_out_of_view():
    cfg = config.load()
    tr, sel = PeopleTracker(cfg), TargetSelector(cfg)
    a, b = person(cx=300), person(cx=600)
    a.track_id, b.track_id = 1, 2
    tracks = tr.update([aim_for(a, [-0.5, 0, 3], (300, 200)), aim_for(b, [0.5, 0, 3], (600, 200))], 0.0)
    for t in tracks:
        t.opted_in = True
    tracks = tr.update([aim_for(a, [-0.5, 0, 3], (300, 200))], 0.1)    # b not seen this frame
    sel.update(tracks, 0.1, (300, 240))
    assert sel.cycle(tracks, 0.1).id == 1
