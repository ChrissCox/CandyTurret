"""M&M flight model: a point mass with quadratic drag, no lift.

It is a starting guess. Step 7 fires at known distances and fits a range -> (tilt, RPM) table,
and the table replaces this model once it exists.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

G = 9.81
RHO = 1.2                                   # air density, kg/m^3, room temperature


@dataclass
class Candy:
    mass_kg: float = 0.00086
    diameter_m: float = 0.0134
    thickness_m: float = 0.0069
    cd: float = 0.7

    @classmethod
    def from_cfg(cls, cfg) -> "Candy":
        p = cfg.projectile
        return cls(p.mass_g / 1000, p.diameter_mm / 1000, p.thickness_mm / 1000, p.drag_coefficient)

    @property
    def area(self) -> float:
        """Edge-on frontal area: a spun M&M should fly flat like a tiny frisbee."""
        return math.pi / 4 * self.diameter_m * self.thickness_m

    @property
    def k(self) -> float:
        """Drag deceleration = k * speed^2, per meter."""
        return RHO * self.cd * self.area / (2 * self.mass_kg)


def flight(v0: float, angle_rad: float, x_target: float, candy: Candy, dt: float = 0.0005, t_max: float = 3.0):
    """Fly from (0, 0) until x = x_target. Returns (height, time, speed) there, or None if it never gets there."""
    k = candy.k

    def deriv(s):
        _, _, vx, vy = s
        v = math.hypot(vx, vy)
        return (vx, vy, -k * v * vx, -G - k * v * vy)

    s = (0.0, 0.0, v0 * math.cos(angle_rad), v0 * math.sin(angle_rad))
    t = 0.0
    while t < t_max:
        k1 = deriv(s)
        k2 = deriv(tuple(a + dt / 2 * b for a, b in zip(s, k1)))
        k3 = deriv(tuple(a + dt / 2 * b for a, b in zip(s, k2)))
        k4 = deriv(tuple(a + dt * b for a, b in zip(s, k3)))
        n = tuple(a + dt / 6 * (b + 2 * c + 2 * d + e) for a, b, c, d, e in zip(s, k1, k2, k3, k4))
        if n[0] >= x_target:
            f = (x_target - s[0]) / (n[0] - s[0])
            y = s[1] + f * (n[1] - s[1])
            speed = math.hypot(s[2] + f * (n[2] - s[2]), s[3] + f * (n[3] - s[3]))
            return y, t + f * dt, speed
        if n[2] <= 0 or n[1] < -10:
            return None
        s, t = n, t + dt
    return None


def solve_angle(v0: float, x: float, y: float, candy: Candy) -> float | None:
    """Lowest launch angle (radians) that passes through (x, y) relative to the muzzle; None if out of reach."""
    def miss(a):
        r = flight(v0, a, x, candy)
        return -1e9 if r is None else r[0] - y

    lo = math.radians(-20)
    if miss(lo) > 0:
        return None                               # target is too far below
    for deg in range(-19, 61):
        hi = math.radians(deg)
        if miss(hi) >= 0:
            break
        lo = hi
    else:
        return None
    for _ in range(40):
        mid = (lo + hi) / 2
        if miss(mid) >= 0:
            hi = mid
        else:
            lo = mid
    return (lo + hi) / 2


def contact_factor(layout: str) -> float:
    """Ideal exit speed / wheel rim speed. One wheel rolling the candy along a fixed backstop moves
    its center at half the rim speed; two matched counter-spinning wheels move it at the full rim speed."""
    return 0.5 if layout == "single_wheel" else 1.0


def rpm_for_speed(v: float, layout: str, wheel_diameter_m: float, slip: float) -> float:
    rim = v / (slip * contact_factor(layout))
    return rim / (math.pi * wheel_diameter_m) * 60


def speed_for_rpm(rpm: float, layout: str, wheel_diameter_m: float, slip: float) -> float:
    return rpm / 60 * math.pi * wheel_diameter_m * slip * contact_factor(layout)


def table(candy: Candy, speeds=(13.0, 20.0), ranges=(1.5, 2, 2.5, 3, 4, 5, 6)):
    """Rows of (speed, range, tilt_deg, flight_s, impact_speed, impact_J) for a target level with the muzzle."""
    rows = []
    for v in speeds:
        for x in ranges:
            a = solve_angle(v, x, 0.0, candy)
            if a is None:
                rows.append((v, x, None, None, None, None))
                continue
            _, t, vi = flight(v, a, x, candy)
            rows.append((v, x, math.degrees(a), t, vi, 0.5 * candy.mass_kg * vi * vi))
    return rows
