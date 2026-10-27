"""Step 3: serial link + manual pan/tilt from the host. The flywheel is held at 0 and the host
never asks to arm, so nothing can fire in this step.

Keys: a/d or left/right = pan 1 deg (A/D = 5 deg), w/s or up/down = tilt 1 deg (W/S = 5 deg),
c = center, x = freeze the host's sending (the ESP32 should hit LINK_TIMEOUT within 250 ms),
q = quit. With --port sim: m = flip the pretend arm switch, p = press the pretend trigger.
"""
from __future__ import annotations

import collections
import math
import time

import cv2
import numpy as np

from .. import draw as D
from .. import protocol as P
from ..link import TurretLink
from ..sim_esp32 import SimESP32, SimPort, limits_from_cfg
from . import DOWN, LEFT, RIGHT, UP, save_capture, window_closed

TITLE = "CandyTurret step 3: manual pan/tilt (q quits)"
SWITCH_NAMES = [(P.SW_ARM, "ARM"), (P.SW_POWER_OK, "POWER"), (P.SW_TRIGGER, "TRIGGER"),
                (P.SW_PAN_LIMIT_LO, "PAN-LO"), (P.SW_PAN_LIMIT_HI, "PAN-HI"),
                (P.SW_TILT_LIMIT_LO, "TILT-LO"), (P.SW_TILT_LIMIT_HI, "TILT-HI"), (P.SW_FEEDER_HOME, "FEED-HOME")]


def enum_name(enum, v):
    try:
        return enum(v).name
    except ValueError:
        return f"?{v}"


def dial(img, center, radius, angle_deg, lo, hi, cmd_deg, label, vertical=False):
    """Top-down pan dial (0 = forward = up on screen, + = left) or side tilt dial (+ = up)."""
    cx, cy = center
    cv2.circle(img, center, radius, D.GRAY, 1, cv2.LINE_AA)

    def tip(a, r):
        th = math.radians(a)
        if vertical:
            return int(cx + r * math.cos(th)), int(cy - r * math.sin(th))
        return int(cx - r * math.sin(th)), int(cy - r * math.cos(th))

    for a in (lo, hi):
        cv2.line(img, center, tip(a, radius), D.RED, 1, cv2.LINE_AA)
    cv2.line(img, center, tip(cmd_deg, radius), D.CYAN, 1, cv2.LINE_AA)
    cv2.line(img, center, tip(angle_deg, radius - 6), D.YELLOW, 3, cv2.LINE_AA)
    D.text(img, f"{label} {angle_deg:+6.1f} (cmd {cmd_deg:+6.1f})", (cx - radius, cy + radius + 22), D.WHITE, 0.5, 1, bg=False)


def run(cfg, args):
    port = args.port or cfg.link.port
    sim = None
    if port == "sim":
        sim = SimESP32(limits_from_cfg(cfg))
        port_obj = SimPort(sim)
    else:
        port_obj = port
    link = TurretLink(port_obj, cfg.link.setpoint_hz, cfg.link.timeout_ms)
    t = cfg.turret
    pan = tilt = 0.0
    log = collections.deque(maxlen=6)
    cv2.namedWindow(TITLE, cv2.WINDOW_AUTOSIZE)
    try:
        while True:
            link.touch()
            link.set_flywheel(0)
            link.set_host_arm(False)
            link.set_aim(pan, tilt)
            while not link.events.empty():
                ev = link.events.get_nowait()
                log.append(f"{time.strftime('%H:%M:%S')} {enum_name(P.Msg, ev.type)} {ev.data}")

            img = np.full((460, 760, 3), 30, np.uint8)
            tm, info = link.telemetry, link.info
            ok = link.link_ok and not link.paused
            status = "LINK OK" if ok else ("HOST FROZEN (x)" if link.paused else "LINK LOST")
            D.text(img, status, (16, 30), D.GREEN if ok else D.RED, 0.8, 2, bg=False)
            lat = f"{link.latency_s * 1000:.0f} ms" if link.latency_s is not None else "--"
            D.text(img, f"port {link.port_name if sim is None else 'sim'}   latency {lat}   bad frames {sum(link.dec.errors.values())}"
                        f"   reboots {link.reboots}", (190, 28), D.WHITE, 0.45, 1, bg=False)
            if link.port_error:
                D.text(img, f"port error: {link.port_error}", (16, 52), D.RED, 0.45, 1, bg=False)
            if info:
                proto = "OK" if link.proto_ok else f"MISMATCH (fw {info.proto}, host {P.PROTO_VERSION})"
                D.text(img, f"protocol {proto}   fw {info.fw_version}   rpm cap {info.rpm_cap}   "
                            f"min range {info.min_range_cm} cm   features {info.features:#04x}",
                       (16, 74), D.WHITE if link.proto_ok else D.RED, 0.45, 1, bg=False)
            if tm:
                fault = enum_name(P.Fault, tm.fault)
                D.text(img, f"state {enum_name(P.State, tm.state)}   fault {fault}   rpm {tm.rpm}/{tm.rpm_target}"
                            f"   battery {tm.vbat_cv / 100:.2f} V   shots {tm.shots}   press {tm.press_id}",
                       (16, 98), D.RED if tm.fault else D.WHITE, 0.45, 1, bg=False)
                on = [n for bit, n in SWITCH_NAMES if tm.switches & bit]
                D.text(img, "switches: " + (" ".join(on) or "none"), (16, 120), D.WHITE, 0.45, 1, bg=False)
                dial(img, (190, 250), 90, tm.pan_cdeg / 100, t.pan_min_deg, t.pan_max_deg, pan, "pan")
                dial(img, (520, 250), 90, tm.tilt_cdeg / 100, t.tilt_min_deg, t.tilt_max_deg, tilt, "tilt", vertical=True)
            for i, s in enumerate(log):
                D.text(img, s[:110], (16, 380 + 14 * i), D.GRAY, 0.38, 1, bg=False)
            keys = "a/d pan  w/s tilt  (caps = 5 deg)  c center  x freeze host  q quit"
            if sim:
                keys += "   sim: m arm switch, p trigger"
            D.text(img, keys, (16, 452), D.CYAN, 0.42, 1, bg=False)
            cv2.imshow(TITLE, img)

            key = cv2.waitKeyEx(15)
            ch = key & 0xFF if key != -1 and key < 0x10000 else None
            if key == -1:
                pass
            elif ch in (ord("q"), 27) or window_closed(TITLE):
                break
            elif key in LEFT or ch in (ord("a"), ord("A")):
                pan += 5 if ch == ord("A") else 1
            elif key in RIGHT or ch in (ord("d"), ord("D")):
                pan -= 5 if ch == ord("D") else 1
            elif key in UP or ch in (ord("w"), ord("W")):
                tilt += 5 if ch == ord("W") else 1
            elif key in DOWN or ch in (ord("s"), ord("S")):
                tilt -= 5 if ch == ord("S") else 1
            elif ch == ord("c"):
                pan = tilt = 0.0
            elif ch == ord("x"):
                link.paused = not link.paused
            elif ch == ord("m") and sim:
                sim.arm_switch = not sim.arm_switch
            elif ch == ord("p") and sim:
                sim.press_trigger()
            elif ch == ord("o"):
                print("saved", save_capture(img, "step3"))
            pan = min(max(pan, t.pan_min_deg), t.pan_max_deg)
            tilt = min(max(tilt, t.tilt_min_deg), t.tilt_max_deg)
            if window_closed(TITLE):
                break
    finally:
        link.close()
        cv2.destroyAllWindows()
