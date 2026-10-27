import random

import pytest

from candyturret import protocol as P
from candyturret.protocol import Msg


def test_crc_check_value():
    assert P.crc16(b"123456789") == 0x29B1          # CRC-16/CCITT-FALSE catalogue check value


@pytest.mark.parametrize("data", [b"", b"\x00", b"\x00\x00", b"\x11\x22\x00\x33", bytes(range(1, 255)),
                                  bytes(range(1, 256)), bytes(300), bytes([1]) * 254, bytes([1]) * 255])
def test_cobs_roundtrip_edges(data):
    enc = P.cobs_encode(data)
    assert 0 not in enc
    assert P.cobs_decode(enc) == data


def test_cobs_roundtrip_random():
    rng = random.Random(1)
    for _ in range(500):
        data = bytes(rng.choice([0, 0, rng.randrange(256)]) for _ in range(rng.randrange(0, 600)))
        assert P.cobs_decode(P.cobs_encode(data)) == data


def test_cobs_rejects_bad_input():
    with pytest.raises(ValueError):
        P.cobs_decode(b"\x05\x01\x02")             # block says 4 bytes follow, only 2 do
    with pytest.raises(ValueError):
        P.cobs_decode(b"\x03\x00\x01")


def sample(mtype):
    fmt, names = P.FORMATS[mtype]
    return {n: 1 for n in names}


@pytest.mark.parametrize("mtype", list(P.FORMATS))
def test_every_message_roundtrips(mtype):
    fields = sample(mtype)
    [pkt] = P.FrameDecoder().feed(P.encode(mtype, 200, **fields))
    assert pkt.type == mtype and pkt.seq == 200 and pkt.data == fields


def test_signed_and_limits():
    f = dict(pan_cdeg=-32768, tilt_cdeg=32767, fly_rpm=65535, flags=255)
    [pkt] = P.FrameDecoder().feed(P.encode(Msg.SETPOINT, 0, **f))
    assert pkt.data == f
    with pytest.raises(ValueError):
        P.encode(Msg.SETPOINT, 0, pan_cdeg=40000, tilt_cdeg=0, fly_rpm=0, flags=0)
    with pytest.raises(ValueError):
        P.encode(Msg.SETPOINT, 0, pan_cdeg=0)       # missing fields


def test_log_text():
    [pkt] = P.FrameDecoder().feed(P.encode(Msg.LOG, 1, text="hello ünïcode"))
    assert pkt.text == "hello ünïcode"


def test_bodies_fit_max():
    for mtype in P.FORMATS:
        assert len(P.pack_body(mtype, 0, **sample(mtype))) <= P.MAX_BODY


def test_decoder_handles_split_garbage_and_errors():
    frames = [P.encode(Msg.TELEMETRY, i, **sample(Msg.TELEMETRY)) for i in range(5)]
    bad = bytearray(frames[2])
    bad[4] ^= 0x40                                 # corrupt one byte -> CRC (or COBS) error
    stream = b"\x00\x00garbage-without-delimiter" + b"\x00" + frames[0] + frames[1] + bytes(bad) + frames[3] + b"\x00" + frames[4]
    dec = P.FrameDecoder()
    got = []
    for i in range(0, len(stream), 3):             # dribble in 3 bytes at a time
        got += dec.feed(stream[i:i + 3])
    assert [p.seq for p in got] == [0, 1, 3, 4]
    assert sum(dec.errors.values()) == 2           # the garbage chunk and the corrupted frame


def test_decoder_drops_oversized_frame_then_recovers():
    dec = P.FrameDecoder()
    assert dec.feed(b"\x01" * 500) == []
    assert dec.errors["overflow"] >= 1
    assert dec.feed(b"\x01\x01\x00") == []         # tail of the oversized frame is skipped
    [pkt] = dec.feed(P.encode(Msg.HELLO, 3, proto=1, host_boot_id=9))
    assert pkt.host_boot_id == 9


def test_unknown_type_and_bad_length_counted():
    dec = P.FrameDecoder()
    body = bytes([0x7E, 0]) + b"\x01\x02"
    body += P.crc16(body).to_bytes(2, "little")
    dec.feed(P.cobs_encode(body) + b"\x00")
    body = bytes([Msg.SETPOINT, 0]) + b"\x01\x02"
    body += P.crc16(body).to_bytes(2, "little")
    dec.feed(P.cobs_encode(body) + b"\x00")
    assert dec.errors["unknown_type"] == 1 and dec.errors["bad_length"] == 1


def test_deg_conversion_clamps():
    assert P.deg_to_cdeg(12.345) == 1234 or P.deg_to_cdeg(12.345) == 1235
    assert P.deg_to_cdeg(1000) == 32767 and P.deg_to_cdeg(-1000) == -32768
