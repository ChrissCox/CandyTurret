import numpy as np
import pytest

from candyturret import config
from candyturret.camera import Frame, Intrinsics
from candyturret.detect import (L_EAR, L_EYE, L_HIP, L_SHOULDER, L_WRIST, NOSE, R_EAR, R_EYE, R_HIP,
                                R_SHOULDER, R_WRIST, Person)
from candyturret.perception import analyze, region_depth, shrink

W, H = 848, 480
INTR = Intrinsics(W, H, 600.0, 600.0, W / 2, H / 2)


@pytest.fixture
def cfg():
    return config.load()


def person(cx=424, top=80, scale=1.0, head=True, hips=True):
    """A standing person facing the camera. Pixel layout scaled around (cx, top)."""
    k = np.zeros((17, 2))
    c = np.zeros(17)

    def put(i, x, y):
        k[i] = (cx + x * scale, top + y * scale)
        c[i] = 0.9

    if head:
        put(NOSE, 0, 30); put(L_EYE, 8, 22); put(R_EYE, -8, 22); put(L_EAR, 18, 26); put(R_EAR, -18, 26)
    put(L_SHOULDER, 45, 80); put(R_SHOULDER, -45, 80)
    if hips:
        put(L_HIP, 30, 220); put(R_HIP, -30, 220)
    put(L_WRIST, 60, 200); put(R_WRIST, -60, 200)
    box = np.array([cx - 80 * scale, top, cx + 80 * scale, top + 400 * scale])
    return Person(box, 0.9, k, c, 1)


def depth_scene(p: Person, z_person=3.0, z_back=6.5):
    """Background far away, person's whole box at z_person except a strip of background
    showing between the arms and the torso edge."""
    d = np.full((H, W), z_back, np.float32)
    x1, y1, x2, y2 = p.box.astype(int)
    d[y1:y2, x1:x2] = z_person
    d[y1:y2, x1:x1 + 25] = z_back                  # box edges hit the wall
    d[y1:y2, x2 - 25:x2] = z_back
    d[::7, ::5] = 0                                # D435-style holes
    return d


def frame(depth):
    return Frame(np.zeros((H, W, 3), np.uint8), depth, INTR, 0.0, 1)


def test_torso_median_ignores_background(cfg):
    p = person()
    a = analyze(p, frame(depth_scene(p)), cfg)
    assert a.quad is not None and not a.approx_range
    assert a.depth_z == pytest.approx(3.0)
    assert a.range_m == pytest.approx(3.0, rel=1e-3)          # aim point is almost on-axis
    assert a.fire_ok, a.holds
    # a slim person: more wall than person inside the box. The box median lands on the wall,
    # the torso median still finds the person.
    slim = depth_scene(p)
    slim[:, int(p.box[0]):int(p.box[0]) + 45] = 6.5
    slim[:, int(p.box[2]) - 45:int(p.box[2])] = 6.5
    corners = np.array([[p.box[0], p.box[1]], [p.box[2], p.box[1]], [p.box[2], p.box[3]], [p.box[0], p.box[3]]], np.float32)
    whole, _ = region_depth(slim, corners)
    assert whole == pytest.approx(6.5)
    assert analyze(p, frame(slim), cfg).depth_z == pytest.approx(3.0)


def test_aim_point_is_between_shoulders_and_hips(cfg):
    p = person()
    a = analyze(p, frame(depth_scene(p)), cfg)
    sh_y, hip_y = p.kpts[L_SHOULDER][1], p.kpts[L_HIP][1]
    assert sh_y < a.aim_px[1] < hip_y
    assert a.aim_px[1] == pytest.approx(sh_y + cfg.aim.aim_fraction * (hip_y - sh_y))
    assert a.point_cam[2] == pytest.approx(3.0)


def test_off_axis_range_is_straight_line_distance(cfg):
    p = person(cx=780)                             # far right of the frame
    a = analyze(p, frame(depth_scene(p, 3.0)), cfg)
    assert a.depth_z == pytest.approx(3.0)
    assert a.range_m > 3.15                        # farther than the camera-z depth
    assert a.range_m == pytest.approx(float(np.linalg.norm(a.point_cam)))


def test_head_close_to_aim_point_holds(cfg):
    p = person()
    p.kpts[NOSE] = p.kpts[L_SHOULDER] * 0.5 + p.kpts[R_SHOULDER] * 0.5 + [0, 60]   # bent over, face at chest height
    a = analyze(p, frame(depth_scene(p)), cfg)
    assert any("head too close" in h for h in a.holds)


def test_head_in_front_of_torso_counts_at_its_own_depth(cfg):
    p = person()
    d = depth_scene(p, z_person=4.0)
    # nose 0.5 m above the aim point at the torso's 4 m depth would be clear, but the head is at 1.6 m
    a0 = analyze(p, frame(d), cfg)
    u, v = p.kpts[NOSE].astype(int)
    d[v - 4:v + 5, u - 4:u + 5] = 1.6
    a1 = analyze(p, frame(d), cfg)
    assert a1.head_clear_m < a0.head_clear_m


def test_no_head_visible_holds_when_required(cfg):
    p = person(head=False)
    a = analyze(p, frame(depth_scene(p)), cfg)
    assert "head not visible" in a.holds


def test_range_gates(cfg):
    p = person()
    assert any("too close" in h for h in analyze(p, frame(depth_scene(p, 1.2)), cfg).holds)
    assert any("too far" in h for h in analyze(p, frame(depth_scene(p, 5.5)), cfg).holds)
    far = analyze(p, frame(depth_scene(p, 7.5, 9.0)), cfg)
    assert far.range_m is None and any("outside camera range" in h for h in far.holds)


def test_no_hips_uses_box_and_holds(cfg):
    p = person(hips=False)
    a = analyze(p, frame(depth_scene(p)), cfg)
    assert a.quad is None and a.approx_range and "no torso" in a.holds


def test_missing_depth(cfg):
    p = person()
    a = analyze(p, frame(None), cfg)
    assert "no depth" in a.holds and a.range_m is None
    a = analyze(p, frame(np.zeros((H, W), np.float32)), cfg)
    assert "depth unreliable" in a.holds


def test_shrink_keeps_center():
    q = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], np.float32)
    s = shrink(q, 0.5)
    assert np.allclose(s.mean(axis=0), q.mean(axis=0)) and np.allclose(s[0], [2.5, 2.5])


def bystander_head_at(target: Person, z_head: float, d: np.ndarray):
    """A second person whose face sits right on the target's aim point, nearer the camera."""
    other = person(cx=424, top=40)
    other.track_id = 2
    a = analyze(target, frame(d), config.load())
    u, v = a.aim_px
    for i, (dx, dy) in zip((NOSE, L_EYE, R_EYE, L_EAR, R_EAR), ((0, 0), (8, -8), (-8, -8), (18, -4), (-18, -4))):
        other.kpts[i] = (u + dx, v + dy)
    d[int(v) - 25:int(v) + 25, int(u) - 25:int(u) + 25] = z_head
    return other


@pytest.mark.parametrize("z_head", [2.0, 2.5, 3.0])
def test_bystander_head_in_front_of_target_holds(cfg, z_head):
    target = person()
    d = depth_scene(target, z_person=3.5)
    other = bystander_head_at(target, z_head, d)
    a = analyze(target, frame(d), cfg, [target, other])
    assert not a.fire_ok
    assert any("someone else's head" in h for h in a.holds)


def test_something_in_front_of_torso_holds(cfg):
    target = person()
    d = depth_scene(target, z_person=3.5)
    a0 = analyze(target, frame(d), cfg)
    assert a0.fire_ok, a0.holds
    u, v = a0.aim_px.astype(int)
    d[v - 20:v + 20, u - 20:u + 20] = 2.0          # an undetected object (or a hand) between turret and chest
    a = analyze(target, frame(d), cfg)
    assert any("something in front" in h for h in a.holds)


def test_low_confidence_person_holds(cfg):
    p = person()
    p.conf = 0.2
    assert "low confidence" in analyze(p, frame(depth_scene(p)), cfg).holds
