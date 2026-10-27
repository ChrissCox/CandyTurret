"""Host end of the USB serial link to the ESP32 (docs/PROTOCOL.md).

A writer thread sends SETPOINT at a fixed rate; that stream is the heartbeat the firmware watches.
If the main loop stops calling touch(), the writer keeps the link up but sends a safe setpoint
(flywheel 0, not armed), so a frozen vision loop can never leave the flywheel spinning.
"""
from __future__ import annotations

import math
import queue
import random
import threading
import time

from . import protocol as P
from .protocol import Msg


def open_port(port: str):
    if port == "sim":
        from .sim_esp32 import SimPort
        return SimPort()
    import serial
    if "://" in port:                       # e.g. socket://localhost:7777 for `python -m candyturret sim --tcp`
        return serial.serial_for_url(port, timeout=0.05)
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, 921600, 0.05
    ser.dtr = False                         # don't pulse the ESP32's reset/boot lines when the port opens
    ser.rts = False
    ser.open()
    return ser


class TurretLink:
    def __init__(self, port="sim", setpoint_hz: float = 50, timeout_ms: int = 200, stale_s: float = 0.15):
        self.port_name = port if isinstance(port, str) else "custom"
        self.ser = open_port(port) if isinstance(port, str) else port
        self.period = 1.0 / setpoint_hz
        self.timeout = timeout_ms / 1000.0
        self.stale_s = stale_s
        self.dec = P.FrameDecoder()
        self.host_boot_id = random.randrange(1, 0xFFFF)

        self._lock = threading.Lock()       # setpoint fields
        self._wlock = threading.Lock()      # port writes and reconnects
        self.pan_deg = 0.0
        self.tilt_deg = 0.0
        self.fly_rpm = 0
        self.host_arm = False
        self.paused = False                 # test hook: stop sending entirely, as if the host died
        self._touched = time.monotonic()

        self.seq = 0
        self._sent_at = [0.0] * 256
        self._last_ack = None
        self._last_hello = 0.0
        self.info: P.Packet | None = None
        self.telemetry: P.Packet | None = None
        self.telemetry_at: float | None = None
        self.latency_s: float | None = None
        self._last_t_ms: int | None = None
        self.port_error: str | None = None
        self.writer_error: str | None = None
        self.bad_inputs = 0
        self.reboots = 0
        self.events: queue.Queue[P.Packet] = queue.Queue()   # TRIGGER, SHOT, IMU, LOG

        self._stop = threading.Event()
        self._threads = [threading.Thread(target=self._writer, name="link-writer", daemon=True),
                         threading.Thread(target=self._reader, name="link-reader", daemon=True)]
        for t in self._threads:
            t.start()

    # ---- called by the app ------------------------------------------------------
    def touch(self):
        """Call once per main-loop iteration. Without it the writer falls back to a safe setpoint."""
        self._touched = time.monotonic()

    def set_aim(self, pan_deg: float, tilt_deg: float):
        if not (math.isfinite(pan_deg) and math.isfinite(tilt_deg)):
            self._bad_input()                       # keep the last good aim, drop to safe
            return
        if self.info:
            pan_deg = min(max(pan_deg, self.info.pan_min_cdeg / 100), self.info.pan_max_cdeg / 100)
            tilt_deg = min(max(tilt_deg, self.info.tilt_min_cdeg / 100), self.info.tilt_max_cdeg / 100)
        with self._lock:
            self.pan_deg, self.tilt_deg = pan_deg, tilt_deg

    def set_flywheel(self, rpm: float):
        if not math.isfinite(rpm):
            self._bad_input()
            return
        rpm = max(0, min(65535, int(rpm)))
        if self.info:
            rpm = min(rpm, self.info.rpm_cap)
        with self._lock:
            self.fly_rpm = rpm

    def set_host_arm(self, on: bool):
        with self._lock:
            self.host_arm = bool(on)

    def fire(self, shot_id: int, press_id: int, rpm: int, rpm_tol: int, range_m: float) -> bool:
        """Ask for one shot. Returns False (and sends nothing) if any field is out of range."""
        if not (math.isfinite(range_m) and 0 < press_id <= 0xFFFF and 0 < rpm <= 0xFFFF and 0 <= rpm_tol <= 0xFFFF):
            self.bad_inputs += 1
            return False
        self._send(Msg.FIRE, shot_id=shot_id & 0xFFFF, press_id=press_id, rpm=rpm, rpm_tol=rpm_tol,
                   range_cm=max(0, min(65535, round(range_m * 100))))
        return True

    def _bad_input(self):
        self.bad_inputs += 1
        with self._lock:
            self.fly_rpm = 0
            self.host_arm = False

    @property
    def link_ok(self) -> bool:
        """Telemetry is arriving and our heartbeat thread is alive."""
        return (self.telemetry_at is not None and time.monotonic() - self.telemetry_at <= self.timeout
                and self._threads[0].is_alive())

    @property
    def proto_ok(self) -> bool:
        return self.info is not None and self.info.proto == P.PROTO_VERSION

    def close(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        try:
            self.ser.close()
        except OSError:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ---- threads ----------------------------------------------------------------
    def _send(self, mtype, **fields):
        with self._wlock:
            seq, self.seq = self.seq, (self.seq + 1) & 0xFF
            frame = P.encode(mtype, seq, **fields)
            self._sent_at[seq] = time.monotonic()
            try:
                self.ser.write(frame)
            except OSError as e:
                self.port_error = str(e)

    def _writer(self):
        next_t = time.perf_counter()
        while not self._stop.is_set():
            now = time.monotonic()
            if not self.paused:
                try:
                    if self.info is None and now - self._last_hello > 0.5:
                        self._last_hello = now
                        self._send(Msg.HELLO, proto=P.PROTO_VERSION, host_boot_id=self.host_boot_id)
                    with self._lock:
                        stale = now - self._touched > self.stale_s
                        pan, tilt = self.pan_deg, self.tilt_deg
                        rpm = 0 if stale else self.fly_rpm
                        arm = self.host_arm and not stale
                    self._send(Msg.SETPOINT, pan_cdeg=P.deg_to_cdeg(pan), tilt_cdeg=P.deg_to_cdeg(tilt),
                               fly_rpm=rpm, flags=P.FLAG_HOST_ARM if arm else 0)
                except Exception as e:                  # never let the heartbeat thread die
                    self.writer_error = repr(e)
                    self._bad_input()
            next_t += self.period
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.perf_counter()        # fell behind: skip, never burst

    def _reader(self):
        while not self._stop.is_set():
            try:
                data = self.ser.read(max(1, self.ser.in_waiting))
            except OSError as e:
                self.port_error = str(e)
                self._reconnect()
                continue
            if data:
                for pkt in self.dec.feed(data):
                    self._handle(pkt)

    def _reconnect(self):
        time.sleep(1.0)
        if self.port_name in ("sim", "custom") or self._stop.is_set():
            return
        with self._wlock:
            try:
                self.ser.close()
            except OSError:
                pass
            try:
                self.ser = open_port(self.port_name)
                self.port_error = None
                self.info = None
            except OSError as e:
                self.port_error = str(e)

    def _handle(self, pkt: P.Packet):
        if pkt.type < 0x80:                             # host->ESP32 types are never valid input here
            return
        now = time.monotonic()
        if pkt.type == Msg.TELEMETRY:
            last = self._last_t_ms
            if last is not None and pkt.t_ms < last and last - pkt.t_ms < 2**31:
                self._rebooted()
            self._last_t_ms = pkt.t_ms
            if pkt.ack_seq != self._last_ack:
                self._last_ack = pkt.ack_seq
                self.latency_s = now - self._sent_at[pkt.ack_seq]
            self.telemetry, self.telemetry_at = pkt, now
        elif pkt.type == Msg.INFO:
            if self.info is not None and pkt.boot_id != self.info.boot_id:
                self._rebooted()
            self.info = pkt
        else:
            self.events.put(pkt)

    def _rebooted(self):
        self.reboots += 1
        self._last_t_ms = None
        self.info = None                            # re-HELLO to refresh limits
        self.events.put(P.Packet(Msg.LOG, 0, {"text": "ESP32 rebooted"}))
