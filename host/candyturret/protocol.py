"""Wire format shared with the ESP32. docs/PROTOCOL.md is the spec; this file must match it.

Frame on the wire:   COBS(body) 0x00
body:                type:u8  seq:u8  payload  crc:u16
crc:                 CRC-16/CCITT-FALSE over type|seq|payload, little-endian
All multi-byte fields are little-endian.
"""
from __future__ import annotations

import binascii
import struct
from collections import Counter
from dataclasses import dataclass, field
from enum import IntEnum

PROTO_VERSION = 1
MAX_BODY = 64                      # bytes before COBS; receivers drop anything longer
MAX_ENCODED = MAX_BODY + 2         # COBS adds at most 1 byte per 254 (so 1 here), plus slack


class Msg(IntEnum):
    # host -> ESP32
    HELLO = 0x01
    SETPOINT = 0x02                # sent at a fixed rate; it is also the heartbeat
    FIRE = 0x03
    # ESP32 -> host
    INFO = 0x81
    TELEMETRY = 0x82
    TRIGGER = 0x83
    SHOT = 0x84
    IMU = 0x85
    LOG = 0x86


# type -> (struct format, field names). LOG is variable length UTF-8 text.
FORMATS: dict[int, tuple[str, tuple[str, ...]]] = {
    Msg.HELLO: ("<BH", ("proto", "host_boot_id")),
    Msg.SETPOINT: ("<hhHB", ("pan_cdeg", "tilt_cdeg", "fly_rpm", "flags")),
    Msg.FIRE: ("<HHHHH", ("shot_id", "press_id", "rpm", "rpm_tol", "range_cm")),
    Msg.INFO: ("<BHHHhhhhHHB", ("proto", "fw_version", "boot_id", "rpm_cap",
                                "pan_min_cdeg", "pan_max_cdeg", "tilt_min_cdeg", "tilt_max_cdeg",
                                "min_range_cm", "link_timeout_ms", "features")),
    Msg.TELEMETRY: ("<IBBBBhhHHHHH", ("t_ms", "ack_seq", "state", "fault", "switches",
                                      "pan_cdeg", "tilt_cdeg", "rpm", "rpm_target",
                                      "shots", "press_id", "vbat_cv")),
    Msg.TRIGGER: ("<HI", ("press_id", "t_ms")),
    Msg.SHOT: ("<HBHHH", ("shot_id", "result", "rpm_before", "rpm_min", "dip_ms")),
    Msg.IMU: ("<Ihhhhhh", ("t_ms", "gx", "gy", "gz", "ax", "ay", "az")),
}
LOG_MAX = MAX_BODY - 4

# SETPOINT.flags
FLAG_HOST_ARM = 0x01               # host wants to be armed; the arm switch must also be on

# INFO.features
FEAT_IMU = 0x01
FEAT_RPM_SENSOR = 0x02             # independent hall/optical sensor, not just ESC telemetry
FEAT_POSITION_FEEDBACK = 0x04      # pan/tilt in TELEMETRY are measured, not just commanded


class State(IntEnum):
    BOOT = 0
    DISARMED = 1
    ARMED = 2
    FIRING = 3
    FAULT = 4


class Fault(IntEnum):
    NONE = 0
    LINK_TIMEOUT = 1
    OVERSPEED = 2
    FEED = 3                       # feeder jam, or a shot with no RPM dip
    LIMIT_HIT = 4                  # a limit switch closed that the setpoint should never reach
    RPM_SENSOR = 5                 # throttle up but no RPM reading
    ARM_ON_AT_BOOT = 6
    KILL = 7                       # kill switch opened while armed
    LOW_BATTERY = 8


# TELEMETRY.switches bits
SW_ARM = 0x01
SW_POWER_OK = 0x02                 # flywheel power present (kill switch closed)
SW_TRIGGER = 0x04                  # fire input currently pressed
SW_PAN_LIMIT_LO = 0x08
SW_PAN_LIMIT_HI = 0x10
SW_TILT_LIMIT_LO = 0x20
SW_TILT_LIMIT_HI = 0x40
SW_FEEDER_HOME = 0x80


class ShotResult(IntEnum):
    FIRED = 0                      # feeder cycled and the RPM dip confirmed a shot
    NO_DIP = 1                     # feeder cycled, no dip: empty hopper or stuck before the wheel
    JAM = 2                        # feeder did not reach its next index position
    ABORTED = 3                    # disarm or fault during the feed; no dip seen
    REJ_NOT_ARMED = 10
    REJ_NO_PRESS = 11              # no fresh, unused trigger press with that press_id
    REJ_RPM = 12                   # flywheel did not settle within tolerance in time
    REJ_RANGE = 13                 # range_cm under the firmware minimum
    REJ_RATE = 14                  # too soon after the last shot
    REJ_BUSY = 15                  # a shot is already in progress
    REJ_FIRE_ZONE = 16             # pan/tilt outside the firmware fire window
    REJ_RPM_CAP = 17               # asked for more RPM than the cap
    REJ_AIM_MOVING = 18            # pan/tilt still slewing
    REJ_BAD_REQUEST = 19           # rpm_tol wider than 5% of rpm


@dataclass
class Packet:
    type: int
    seq: int
    data: dict = field(default_factory=dict)

    def __getattr__(self, name):   # pkt.rpm instead of pkt.data["rpm"]
        if name == "data" or name.startswith("__"):
            raise AttributeError(name)
        try:
            return self.data[name]
        except KeyError:
            raise AttributeError(name) from None


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout."""
    return binascii.crc_hqx(data, 0xFFFF)


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_idx, code = 0, 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx, code = len(out), 1
            out.append(0)
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx, code = len(out), 1
                out.append(0)
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        code = data[i]
        if code == 0:
            raise ValueError("zero byte inside COBS frame")
        end = i + code
        if end > n:
            raise ValueError("truncated COBS block")
        block = data[i + 1:end]
        if 0 in block:
            raise ValueError("zero byte inside COBS frame")
        out += block
        i = end
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


def pack_body(mtype: int, seq: int, **fields) -> bytes:
    if mtype == Msg.LOG:
        payload = fields.get("text", "").encode("utf-8")[:LOG_MAX]
    else:
        fmt, names = FORMATS[mtype]
        missing = set(names) - fields.keys()
        if missing:
            raise ValueError(f"{Msg(mtype).name} missing fields {sorted(missing)}")
        try:
            payload = struct.pack(fmt, *(int(fields[n]) for n in names))
        except struct.error as e:
            raise ValueError(f"{Msg(mtype).name}: {e}") from None
    head = bytes([int(mtype), seq & 0xFF]) + payload
    return head + struct.pack("<H", crc16(head))


def encode(mtype: int, seq: int, **fields) -> bytes:
    """One complete wire frame, delimiter included."""
    return cobs_encode(pack_body(mtype, seq, **fields)) + b"\x00"


def parse_body(body: bytes) -> Packet:
    if len(body) < 4:
        raise ValueError("short")
    if len(body) > MAX_BODY:
        raise ValueError("long")
    (crc,) = struct.unpack_from("<H", body, len(body) - 2)
    if crc != crc16(body[:-2]):
        raise ValueError("crc")
    mtype, seq, payload = body[0], body[1], body[2:-2]
    if mtype == Msg.LOG:
        return Packet(mtype, seq, {"text": payload.decode("utf-8", "replace")})
    if mtype not in FORMATS:
        raise ValueError("unknown_type")
    fmt, names = FORMATS[mtype]
    if len(payload) != struct.calcsize(fmt):
        raise ValueError("bad_length")
    return Packet(mtype, seq, dict(zip(names, struct.unpack(fmt, payload))))


class FrameDecoder:
    """Feed it raw bytes from the port; it returns whole packets and counts bad frames in .errors."""

    def __init__(self):
        self.buf = bytearray()
        self.skipping = False      # inside an oversized frame; drop bytes until the next 0x00
        self.errors: Counter[str] = Counter()

    def feed(self, data: bytes) -> list[Packet]:
        self.buf += data
        packets = []
        while True:
            i = self.buf.find(0)
            if i < 0:
                if len(self.buf) > MAX_ENCODED:
                    self.buf.clear()
                    self.skipping = True
                    self.errors["overflow"] += 1
                return packets
            chunk = bytes(self.buf[:i])
            del self.buf[:i + 1]
            if self.skipping:
                self.skipping = False
                continue
            if not chunk:
                continue           # extra delimiters are allowed
            try:
                packets.append(parse_body(cobs_decode(chunk)))
            except ValueError as e:
                self.errors[str(e) if str(e) in ("short", "long", "crc", "unknown_type", "bad_length") else "cobs"] += 1


def deg_to_cdeg(deg: float) -> int:
    return max(-32768, min(32767, round(deg * 100)))


def cdeg_to_deg(cdeg: int) -> float:
    return cdeg / 100.0
