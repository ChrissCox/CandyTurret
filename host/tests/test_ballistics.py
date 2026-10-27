import math

import pytest

from candyturret import ballistics as B, config


def test_no_drag_matches_textbook():
    c = B.Candy(cd=0.0)
    y, t, v = B.flight(10.0, math.radians(30), 5.0, c)
    vx, vy = 10 * math.cos(math.radians(30)), 10 * math.sin(math.radians(30))
    t_exact = 5.0 / vx
    assert t == pytest.approx(t_exact, rel=1e-4)
    assert y == pytest.approx(vy * t_exact - 0.5 * B.G * t_exact ** 2, abs=1e-4)


def test_drag_slows_it_down():
    c = B.Candy()
    _, t_drag, v_drag = B.flight(20.0, 0.05, 5.0, c)
    _, t_free, v_free = B.flight(20.0, 0.05, 5.0, B.Candy(cd=0.0))
    assert t_drag > t_free and v_drag < v_free
    assert v_drag == pytest.approx(20.0 * math.exp(-c.k * 5.0), rel=0.02)   # flat shot: v = v0 e^(-kx)


@pytest.mark.parametrize("v0,x,y", [(13, 1.5, 0), (13, 5, 0), (20, 4, 0.3), (13, 3, -0.4)])
def test_solved_angle_hits(v0, x, y):
    c = B.Candy()
    a = B.solve_angle(v0, x, y, c)
    assert a is not None
    hy, _, _ = B.flight(v0, a, x, c)
    assert hy == pytest.approx(y, abs=0.003)


def test_out_of_reach():
    assert B.solve_angle(5.0, 20.0, 0.0, B.Candy()) is None


def test_rpm_speed_inverse_and_single_wheel_needs_double():
    for layout in ("single_wheel", "dual_wheel"):
        rpm = B.rpm_for_speed(13.0, layout, 0.05, 0.8)
        assert B.speed_for_rpm(rpm, layout, 0.05, 0.8) == pytest.approx(13.0)
    assert B.rpm_for_speed(13, "single_wheel", 0.05, 0.8) == pytest.approx(2 * B.rpm_for_speed(13, "dual_wheel", 0.05, 0.8))


def test_config_candy():
    c = B.Candy.from_cfg(config.load())
    assert 0.0005 < c.mass_kg < 0.002 and c.k > 0


def test_temp_list_covers_unknowns():
    keys = {k for k, _, _ in config.temp_values()}
    for must in ("camera.offset_forward_mm", "aim.head_clearance_m", "launcher.velocity_cap_mps",
                 "turret.pan_min_deg", "projectile.mass_g", "host.computer", "controls.fire_input"):
        assert must in keys
