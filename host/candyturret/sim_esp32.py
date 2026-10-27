"""A pretend ESP32 for testing the host without hardware.

It speaks docs/PROTOCOL.md and follows the rules in docs/FIRMWARE_SAFETY.md, so it doubles as an
executable reading of that spec. It is a test double, not firmware: the real firmware is written
separately and only has to match the docs.
"""
from __future__ import annotations

import math
import random
import threading
import time
from dataclasses import dataclass

from . import protocol as P
from .protocol import Fault, Msg, ShotResult, State

RELEASE_MS = 60          # sim feeder: the candy leaves the indexer pocket 60 ms into a feed...
FEED_MS = 80             # ...and the indexer is back home (closed) at 80 ms


@dataclass
class FirmwareLimits:
    """The constants the real firmware hard-codes. Values here are TEMP until the hardware tests."""
    rpm_cap: int = 15000                 # TEMP: set from the chronograph test in step 5
    pan_min_cdeg: int = -4000
    pan_max_cdeg: int = 4000
    tilt_min_cdeg: int = -1000
    tilt_max_cdeg: int = 3000
    fire_pan_min_cdeg: int = -3500      # fire window: stricter than the motion limits, never toward the rider
    fire_pan_max_cdeg: int = 3500
    fire_tilt_min_cdeg: int = -500
    fire_tilt_max_cdeg: int = 2500
    min_range_cm: int = 150
    link_timeout_ms: int = 200
    press_window_ms: int = 400          # a FIRE must name a press younger than this
    min_shot_interval_ms: int = 400     # feed start to feed start
    spinup_timeout_ms: int = 1500
    rpm_settle_ms: int = 100
    max_tol_fraction: float = 0.05      # FIRE.rpm_tol may be at most 5% of FIRE.rpm
    aim_tol_cdeg: int = 50
    dip_fraction: float = 0.015         # TEMP: set from the RPM trace in step 6
    dip_window_ms: int = 300
    feed_timeout_ms: int = 300
    max_no_dip: int = 1                 # one NO_DIP latches FEED: the next feed could push two candies
    overspeed_fraction: float = 1.05
    overspeed_ms: int = 50
    sensor_fault_ms: int = 300
    telemetry_period_ms: int = 20


def limits_from_cfg(cfg) -> FirmwareLimits:
    """Sim limits that match config/turret.toml (the real firmware hard-codes its own).
    The RPM cap assumes zero slip, the fastest case, until the chronograph says otherwise."""
    from .ballistics import rpm_for_speed
    lc, t = cfg.launcher, cfg.turret
    cap = rpm_for_speed(lc.velocity_cap_mps, lc.layout, lc.wheel_diameter_mm / 1000, 1.0)
    margin = 500                                                   # fire window sits 5 deg inside the motion limits
    return FirmwareLimits(
        rpm_cap=min(int(cap), 65535),
        pan_min_cdeg=round(t.pan_min_deg * 100), pan_max_cdeg=round(t.pan_max_deg * 100),
        tilt_min_cdeg=round(t.tilt_min_deg * 100), tilt_max_cdeg=round(t.tilt_max_deg * 100),
        fire_pan_min_cdeg=round(t.pan_min_deg * 100) + margin, fire_pan_max_cdeg=round(t.pan_max_deg * 100) - margin,
        fire_tilt_min_cdeg=round(t.tilt_min_deg * 100) + margin, fire_tilt_max_cdeg=round(t.tilt_max_deg * 100) - margin,
        min_range_cm=round(cfg.aim.min_fire_range_m * 100), link_timeout_ms=cfg.link.timeout_ms)


class SimESP32:
    FW_VERSION = 1

    def __init__(self, limits: FirmwareLimits | None = None, seed: int = 0):
        self.lim = limits or FirmwareLimits()
        self.rng = random.Random(seed)
        self.boot_id = self.rng.randrange(1, 0xFFFF)
        self.dec = P.FrameDecoder()
        self.inbox = bytearray()
        self.out = bytearray()
        self.seq = 0
        self.now = 0
        self.started = False

        # sim-only physical inputs
        self.arm_switch = False
        self.power_ok = True
        self.hopper = 200
        self.jam_next = False
        self.rpm_sensor_broken = False
        self._pending_press = False

        # firmware state
        self.state = State.BOOT
        self.fault = Fault.NONE
        self.armed = False                   # latched by an arm-switch OFF->ON edge (R2)
        self.arm_prev = False
        self.had_link = False
        self.last_rx = None                  # last valid SETPOINT
        self.ack_seq = 0
        self.sp_pan = self.sp_tilt = 0
        self.sp_rpm = 0
        self.host_arm = False
        self.pan = self.tilt = 0.0           # cdeg, actual
        self.rpm = 0.0
        self.rpm_read = 0.0
        self.rpm_target = 0
        self.press_id = 0
        self.press_t = 0
        self.press_used = True               # boot with nothing to fire
        self.shots = 0
        self.no_dip_run = 0
        self.last_feed_start = None
        self.shot = None
        self.feeder_home_at = 0              # feeder is home (closed) from this time on
        self.over_since = None
        self.sensor_since = None
        self.next_telemetry = 0

    # ---- sim-only controls -------------------------------------------------
    def press_trigger(self):
        self._pending_press = True

    @property
    def feeder_home(self) -> bool:
        return (self.shot is None or self.shot["phase"] != "feed") and self.now >= self.feeder_home_at

    # ---- I/O ----------------------------------------------------------------
    def feed(self, data: bytes):
        self.inbox += data

    def _send(self, mtype, **fields):
        self.out += P.encode(mtype, self.seq, **fields)
        self.seq = (self.seq + 1) & 0xFF

    def _info(self):
        L = self.lim
        self._send(Msg.INFO, proto=P.PROTO_VERSION, fw_version=self.FW_VERSION, boot_id=self.boot_id,
                   rpm_cap=L.rpm_cap, pan_min_cdeg=L.pan_min_cdeg, pan_max_cdeg=L.pan_max_cdeg,
                   tilt_min_cdeg=L.tilt_min_cdeg, tilt_max_cdeg=L.tilt_max_cdeg,
                   min_range_cm=L.min_range_cm, link_timeout_ms=L.link_timeout_ms,
                   features=P.FEAT_RPM_SENSOR)

    def _shot_result(self, shot_id, result, rpm_before=0, rpm_min=0, dip_ms=0):
        self._send(Msg.SHOT, shot_id=shot_id, result=int(result), rpm_before=int(rpm_before),
                   rpm_min=int(rpm_min), dip_ms=int(dip_ms))

    def _in_fire_window(self, pan, tilt) -> bool:
        L = self.lim
        return L.fire_pan_min_cdeg <= pan <= L.fire_pan_max_cdeg and L.fire_tilt_min_cdeg <= tilt <= L.fire_tilt_max_cdeg

    # ---- main loop ------------------------------------------------------------
    def tick(self, now_ms: int) -> bytes:
        dt = max(0, now_ms - self.now) / 1000.0
        self.now = now_ms
        L = self.lim
        if not self.started:
            self.started = True
            self.state = State.DISARMED
            if self.arm_switch:
                self.fault = Fault.ARM_ON_AT_BOOT
            self.arm_prev = self.arm_switch
            self._info()

        for pkt in self.dec.feed(bytes(self.inbox)):
            self._handle(pkt)
        self.inbox.clear()
        link_ok = self.last_rx is not None and self.now - self.last_rx <= L.link_timeout_ms

        # trigger presses (R5): a press while not ARMED is born used; presses expire for good
        if self._pending_press:
            self._pending_press = False
            self.press_id = self.press_id % 0xFFFF + 1          # 1..65535, 0 means "none yet"
            self.press_t = self.now
            self.press_used = self.state != State.ARMED
            self._send(Msg.TRIGGER, press_id=self.press_id, t_ms=self.now & 0xFFFFFFFF)
        if not self.press_used and self.now - self.press_t >= L.press_window_ms:
            self.press_used = True

        # physics, using the targets chosen last tick
        feeding = self.shot is not None and self.shot["phase"] == "feed"
        if link_ok and not feeding:                              # aim is frozen during a feed (R7)
            step = 12000 * dt                                    # 120 deg/s slew
            self.pan += max(-step, min(step, self.sp_pan - self.pan))
            self.tilt += max(-step, min(step, self.sp_tilt - self.tilt))
        tau = 0.25 if self.rpm_target > self.rpm else 1.5         # spin up fast, coast down slow
        if self.power_ok:
            self.rpm += (self.rpm_target - self.rpm) * (1 - math.exp(-dt / tau))
        else:
            self.rpm *= math.exp(-dt / 1.5)
        self.rpm_read = 0.0 if self.rpm_sensor_broken else self.rpm * (1 + self.rng.uniform(-0.002, 0.002))

        # latch faults (R4)
        if self.fault == Fault.NONE:
            if self.had_link and not link_ok:
                self.fault = Fault.LINK_TIMEOUT
            elif self.armed and not self.power_ok:
                self.fault = Fault.KILL
        if self.rpm_read > L.rpm_cap * L.overspeed_fraction:
            self.over_since = self.over_since if self.over_since is not None else self.now
            if self.now - self.over_since >= L.overspeed_ms and self.fault == Fault.NONE:
                self.fault = Fault.OVERSPEED
        else:
            self.over_since = None
        if self.rpm_target > 1000 and self.rpm_read < 100:
            self.sensor_since = self.sensor_since if self.sensor_since is not None else self.now
            if self.now - self.sensor_since >= L.sensor_fault_ms and self.fault == Fault.NONE:
                self.fault = Fault.RPM_SENSOR
        else:
            self.sensor_since = None
        # clear faults: only with the arm switch OFF and that fault's cause gone
        if self.fault != Fault.NONE and not self.arm_switch:
            cause_gone = {Fault.LINK_TIMEOUT: link_ok,
                          Fault.KILL: self.power_ok,
                          Fault.OVERSPEED: self.rpm_read < L.rpm_cap,
                          Fault.FEED: self.feeder_home,
                          Fault.RPM_SENSOR: False}.get(self.fault, True)   # sensor fault needs a reboot
            if cause_gone:
                self.fault = Fault.NONE

        # arming (R2): edge-triggered on the switch, dropped the moment any condition fails
        ready = self.host_arm and link_ok and self.power_ok and self.fault == Fault.NONE
        if not (self.arm_switch and ready):
            self.armed = False
        elif self.arm_switch and not self.arm_prev:
            self.armed = True
        self.arm_prev = self.arm_switch
        if self.fault != Fault.NONE:
            self.state = State.FAULT
        else:
            self.state = (State.FIRING if self.shot else State.ARMED) if self.armed else State.DISARMED

        if self.shot is not None:
            if self.armed:
                self._advance_shot()
            else:
                self._abort_shot()

        if self.armed:
            want = self.shot["rpm"] if self.shot else self.sp_rpm
            self.rpm_target = min(want, L.rpm_cap)
        else:
            self.rpm_target = 0

        if self.now >= self.next_telemetry:
            self.next_telemetry = self.now + L.telemetry_period_ms
            self._send(Msg.TELEMETRY, t_ms=self.now & 0xFFFFFFFF, ack_seq=self.ack_seq,
                       state=int(self.state), fault=int(self.fault), switches=self._switches(),
                       pan_cdeg=round(self.pan), tilt_cdeg=round(self.tilt),
                       rpm=min(65535, round(self.rpm_read)), rpm_target=self.rpm_target,
                       shots=self.shots & 0xFFFF, press_id=self.press_id, vbat_cv=1480)
        out, self.out = bytes(self.out), bytearray()
        return out

    def _switches(self):
        s = 0
        if self.arm_switch:
            s |= P.SW_ARM
        if self.power_ok:
            s |= P.SW_POWER_OK
        if self.feeder_home:
            s |= P.SW_FEEDER_HOME
        return s

    def _handle(self, pkt: P.Packet):
        if pkt.type >= 0x80:                     # ESP32->host types are never valid input here
            return
        L = self.lim
        self.ack_seq = pkt.seq
        if pkt.type == Msg.HELLO:
            self._info()
        elif pkt.type == Msg.SETPOINT:           # the only frame that counts as a heartbeat
            self.last_rx = self.now
            self.had_link = True
            self.sp_pan = max(L.pan_min_cdeg, min(L.pan_max_cdeg, pkt.pan_cdeg))
            self.sp_tilt = max(L.tilt_min_cdeg, min(L.tilt_max_cdeg, pkt.tilt_cdeg))
            self.sp_rpm = min(pkt.fly_rpm, L.rpm_cap)
            self.host_arm = bool(pkt.flags & P.FLAG_HOST_ARM)
        elif pkt.type == Msg.FIRE:
            self._on_fire(pkt)

    def _on_fire(self, f: P.Packet):
        """Checks in the order FIRMWARE_SAFETY.md R5 lists them."""
        L = self.lim
        names_latest = f.press_id != 0 and f.press_id == self.press_id
        fresh = names_latest and not self.press_used and self.now - self.press_t < L.press_window_ms
        if names_latest:
            self.press_used = True                                # a press is spent by the first FIRE naming it
        if self.shot:
            return self._shot_result(f.shot_id, ShotResult.REJ_BUSY)
        if self.state != State.ARMED:
            return self._shot_result(f.shot_id, ShotResult.REJ_NOT_ARMED)
        if not fresh:
            return self._shot_result(f.shot_id, ShotResult.REJ_NO_PRESS)
        if f.range_cm < L.min_range_cm:
            return self._shot_result(f.shot_id, ShotResult.REJ_RANGE)
        if f.rpm == 0 or f.rpm > L.rpm_cap:
            return self._shot_result(f.shot_id, ShotResult.REJ_RPM_CAP)
        if f.rpm_tol > f.rpm * L.max_tol_fraction:
            return self._shot_result(f.shot_id, ShotResult.REJ_BAD_REQUEST)
        if self.last_feed_start is not None and self.now - self.last_feed_start < L.min_shot_interval_ms:
            return self._shot_result(f.shot_id, ShotResult.REJ_RATE)
        if not (self._in_fire_window(self.pan, self.tilt) and self._in_fire_window(self.sp_pan, self.sp_tilt)):
            return self._shot_result(f.shot_id, ShotResult.REJ_FIRE_ZONE)
        self.shot = dict(id=f.shot_id, rpm=f.rpm, tol=f.rpm_tol, phase="spin", t0=self.now, settled=None)

    def _advance_shot(self):
        L, s, rpm = self.lim, self.shot, self.rpm_read
        if s["phase"] == "spin":
            if not (self._in_fire_window(self.pan, self.tilt) and self._in_fire_window(self.sp_pan, self.sp_tilt)):
                self._shot_result(s["id"], ShotResult.REJ_FIRE_ZONE)          # swung out while spinning up
                self.shot = None
                return
            aim_ok = abs(self.pan - self.sp_pan) <= L.aim_tol_cdeg and abs(self.tilt - self.sp_tilt) <= L.aim_tol_cdeg
            rpm_ok = s["rpm"] - s["tol"] <= rpm <= min(s["rpm"] + s["tol"], L.rpm_cap)
            if rpm_ok and aim_ok:
                s["settled"] = s["settled"] if s["settled"] is not None else self.now
                if self.now - s["settled"] >= L.rpm_settle_ms:
                    s.update(phase="feed", feed_t0=self.now, rpm_before=rpm, rpm_min=rpm, dip_ms=None)
                    self.last_feed_start = self.now
            else:
                s["settled"] = None
            if s["phase"] == "spin" and self.now - s["t0"] > L.spinup_timeout_ms:
                self._shot_result(s["id"], ShotResult.REJ_AIM_MOVING if rpm_ok else ShotResult.REJ_RPM)
                self.shot = None
            return
        t = self.now - s["feed_t0"]
        if self.jam_next:
            if t > L.feed_timeout_ms:
                self.jam_next = False
                self._finish(ShotResult.JAM)
                self.fault = Fault.FEED
            return
        if t >= RELEASE_MS and self.hopper > 0 and not s.get("hit"):
            s["hit"] = True
            self.hopper -= 1
            self.rpm *= 0.975
            rpm = self.rpm_read = self.rpm
        s["rpm_min"] = min(s["rpm_min"], rpm)
        if s["dip_ms"] is None and rpm <= s["rpm_before"] * (1 - L.dip_fraction):
            s["dip_ms"] = t
        if t >= FEED_MS and (s["dip_ms"] is not None or t >= L.dip_window_ms):
            self._finish(ShotResult.FIRED if s["dip_ms"] is not None else ShotResult.NO_DIP)

    def _abort_shot(self):
        """Disarm or fault mid-shot (R8): spin-up is cancelled; a feed that hasn't released its candy
        reverses to home; one that has released finishes its trip home. Neither counts as NO_DIP."""
        s = self.shot
        if s["phase"] == "spin":
            self._shot_result(s["id"], ShotResult.REJ_NOT_ARMED)
        else:
            t = self.now - s["feed_t0"]
            self.feeder_home_at = self.now + (t if t < RELEASE_MS else max(0, FEED_MS - t))
            result = ShotResult.FIRED if s["dip_ms"] is not None else ShotResult.ABORTED
            self._shot_result(s["id"], result, s["rpm_before"], s["rpm_min"], s["dip_ms"] or 0)
            if result == ShotResult.FIRED:
                self.shots += 1
        self.shot = None

    def _finish(self, result):
        s = self.shot
        self._shot_result(s["id"], result, s["rpm_before"], s["rpm_min"], s["dip_ms"] or 0)
        self.shot = None
        if result == ShotResult.FIRED:
            self.shots += 1
            self.no_dip_run = 0
        elif result == ShotResult.NO_DIP:
            self.no_dip_run += 1
            if self.no_dip_run >= self.lim.max_no_dip:
                self.fault = Fault.FEED
                self.no_dip_run = 0


class SimPort:
    """Looks like a pyserial port; a SimESP32 runs on the other end in a background thread."""

    def __init__(self, sim: SimESP32 | None = None, tick_s: float = 0.002):
        self.sim = sim or SimESP32()
        self.timeout = 0.05
        self._rx = bytearray()
        self._lock = threading.Lock()
        self._cv = threading.Condition()
        self._closed = False
        self._tick_s = tick_s
        self._thread = threading.Thread(target=self._run, name="sim-esp32", daemon=True)
        self._thread.start()

    def _run(self):
        t0 = time.monotonic()
        while not self._closed:
            with self._lock:
                out = self.sim.tick(int((time.monotonic() - t0) * 1000))
            if out:
                with self._cv:
                    self._rx += out
                    self._cv.notify_all()
            time.sleep(self._tick_s)

    def write(self, data: bytes) -> int:
        with self._lock:
            self.sim.feed(data)
        return len(data)

    def read(self, n: int = 1) -> bytes:
        with self._cv:
            if not self._rx:
                self._cv.wait(self.timeout)
            chunk = bytes(self._rx[:n])
            del self._rx[:n]
            return chunk

    @property
    def in_waiting(self) -> int:
        return len(self._rx)

    def close(self):
        self._closed = True
