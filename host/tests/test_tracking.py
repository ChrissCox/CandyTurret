import numpy as np
import pytest

from candyturret import config
from candyturret.detect import L_SHOULDER, L_WRIST, NOSE, R_SHOULDER, R_WRIST, Person
from candyturret.perception import Aim
from candyturret.tracking import KalmanCV3, PeopleTracker, TargetSelector, hands_up, measurement_noise

from .test_perception import person


@pytest.fixture
def cfg():
    return config.load()


def aim_for(p: Person, point, aim_px=(424, 200)):
    a = Aim(p)
    a.point_cam = None if point is None else np.asarray(point, float)
    a.aim_px = np.asarray(aim_px, float)
    return a


def raise_hands(p: Person, dy=-80):
    p.kpts[L_WRIST] = p.kpts[NOSE] + [30, dy]
    p.kpts[R_WRIST] = p.kpts[NOSE] + [-30, dy]
    return p


def test_kalman_finds_velocity():
    rng = np.random.default_rng(0)
    kf = KalmanCV3(np.array([0, 0, 3.0]), 0.0, 3.0)
    v = np.array([0.8, 0.0, -0.5])
    for i in range(1, 60):
        t = i / 30
        z = np.array([0, 0, 3.0]) + v * t + rng.normal(0, 0.02, 3)
        kf.predict(t)
        kf.update(z, measurement_noise(z))
    assert np.allclose(kf.velocity, v, atol=0.15)
    assert np.allclose(kf.at(2.0 + 0.3), np.array([0, 0, 3.0]) + v * 2.3, atol=0.1)


def test_hands_up_gesture():
    p = person()
    assert not hands_up(p, 0.5)
    assert hands_up(raise_hands(p), 0.5)
    p.kconf[R_WRIST] = 0.1
    assert not hands_up(p, 0.5)


def test_hands_on_head_is_not_opt_in():
    p = raise_hands(person(), dy=-20)              # wrists just above the eyes: resting on the head
    assert not hands_up(p, 0.5)


def test_turned_away_hands_at_neck_is_not_opt_in():
    p = person(head=False)
    p.kpts[L_SHOULDER] = (439, 160)                # turned 3/4: shoulders only 30 px apart
    p.kpts[R_SHOULDER] = (409, 160)
    p.kpts[L_WRIST] = (440, 150)                   # tying a ponytail
    p.kpts[R_WRIST] = (410, 150)
    assert not hands_up(p, 0.5)


def test_opt_in_needs_a_full_second(cfg):
    tr = PeopleTracker(cfg)
    p = raise_hands(person())
    for i in range(25):                              # 0.8 s at 30 fps
        [track] = tr.update([aim_for(p, [0, 0, 3])], i / 30)
    assert not track.opted_in and 0.7 < track.opt_in_progress(i / 30, 1.0) < 1.0
    for i in range(25, 40):
        [track] = tr.update([aim_for(p, [0, 0, 3])], i / 30)
    assert track.opted_in


def test_jump_revokes_opt_in(cfg):
    tr = PeopleTracker(cfg)
    p = raise_hands(person())
    for i in range(40):
        tr.update([aim_for(p, [0, 0, 3])], i / 30)
    p = person()                                     # hands down, then the id lands on someone 1.5 m away
    [track] = tr.update([aim_for(p, [1.5, 0, 3])], 40 / 30)
    assert not track.opted_in and track.swaps == 1


def test_walking_does_not_revoke(cfg):
    tr = PeopleTracker(cfg)
    p = raise_hands(person())
    for i in range(40):
        tr.update([aim_for(p, [0, 0, 3])], i / 30)
    p = person()
    for i in range(40, 100):
        [track] = tr.update([aim_for(p, [1.5 * (i - 40) / 30, 0, 3])], i / 30)   # 1.5 m/s
    assert track.opted_in


def test_opt_in_needs_continuous_sight(cfg):
    tr = PeopleTracker(cfg)
    p = raise_hands(person())
    tr.update([aim_for(p, [0, 0, 3])], 0.0)
    [track] = tr.update([aim_for(p, [0, 0, 3])], 1.0)   # seen twice, a second apart
    assert not track.opted_in


def test_id_rematched_after_gap_loses_opt_in(cfg):
    tr = PeopleTracker(cfg)
    p = raise_hands(person())
    for i in range(40):
        tr.update([aim_for(p, [0, 0, 3])], i / 30)
    assert tr.tracks[1].opted_in
    # A leaves; 0.6 s later the tracker hands id 1 to B standing 1.5 m away
    [track] = tr.update([aim_for(person(), [1.5, 0, 3])], 39 / 30 + 0.6)
    assert not track.opted_in


def test_tracks_expire(cfg):
    tr = PeopleTracker(cfg)
    tr.update([aim_for(person(), [0, 0, 3])], 0.0)
    assert tr.update([], cfg.tracking.forget_s + 0.1) == []


def two_people(cfg, t, near_id=1):
    a, b = person(cx=300), person(cx=600)
    a.track_id, b.track_id = 1, 2
    return [aim_for(a, [-0.5, 0, 3], (300, 200)), aim_for(b, [0.5, 0, 3], (600, 200))]


def test_selector_requires_opt_in(cfg):
    tr, sel = PeopleTracker(cfg), TargetSelector(cfg)
    tracks = tr.update(two_people(cfg, 0), 0.0)
    assert sel.update(tracks, 0.0, (424, 240)) is None


def test_selector_is_sticky(cfg):
    tr, sel = PeopleTracker(cfg), TargetSelector(cfg)
    for t in tr.update(two_people(cfg, 0), 0.0):
        t.opted_in = True
    tracks = tr.update(two_people(cfg, 0), 1 / 30)
    first = sel.update(tracks, 1 / 30, (310, 240))
    assert first.id == 1
    # center moves next to person 2, but the lock holds
    assert sel.update(tracks, 1 / 30, (600, 240)).id == 1
    # person 1 drops out; person 2 stays in view the whole time
    only_b = [a for a in two_people(cfg, 0) if a.person.track_id == 2]
    t = 1 / 30
    while t < 0.3:
        t += 1 / 30
        tracks = tr.update(only_b, t)
    assert sel.update(tracks, t, (600, 240)).id == 1                    # still locked within the timeout
    while t < 0.3 + cfg.tracking.lock_timeout_s + 0.1:
        t += 1 / 30
        tracks = tr.update(only_b, t)
    assert sel.update(tracks, t, (600, 240)).id == 2                    # gone past the timeout: switches


def test_time_going_backwards_is_not_eligible(cfg):
    tr, sel = PeopleTracker(cfg), TargetSelector(cfg)
    [track] = tr.update([aim_for(person(), [0, 0, 3])], 100.0)
    track.opted_in = True
    assert not sel.eligible(track, 90.0)          # recording looped: last_seen is "in the future"


def test_selector_cycle(cfg):
    tr, sel = PeopleTracker(cfg), TargetSelector(cfg)
    for t in tr.update(two_people(cfg, 0), 0.0):
        t.opted_in = True
    tracks = tr.update(two_people(cfg, 0), 0.1)
    sel.update(tracks, 0.1, (300, 240))
    assert sel.cycle(tracks, 0.1).id == 2
    assert sel.cycle(tracks, 0.1).id == 1
