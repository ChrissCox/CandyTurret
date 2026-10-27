"""python -m candyturret <command>   (run from the host/ folder)

  step1        D435 stream with person boxes and distances
  step2        torso aim point, opt-in gesture and target lock (no motion, no firing)
  step3        serial link + manual pan/tilt (no firing); --port sim works without hardware
  sim          pretend ESP32 on a TCP port, for step3 --port socket://localhost:PORT
  ballistics   physics-model table: tilt, flight time and RPM per range
  vectors      protocol test frames (hex) for checking the firmware's encoder/decoder
  temp         list every TEMP placeholder in config/turret.toml
"""
from __future__ import annotations

import argparse
import sys

from . import config


def _camera_args(p):
    p.add_argument("--source", default="realsense",
                   help="'realsense' (default), a video file, an image, or a webcam index")
    p.add_argument("--bag", help="play a RealSense .bag recording instead of the live camera")
    p.add_argument("--record", help="record the live D435 session to this .bag file")
    p.add_argument("--fake-depth", type=float, help="with --source: pretend everything is this many meters away")
    p.add_argument("--frames", type=int, help="stop after this many frames and save the last one to captures/")
    p.add_argument("--headless", action="store_true", help="no window (use with --frames)")


def main(argv=None):
    ap = argparse.ArgumentParser(prog="candyturret", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", help="path to turret.toml (default: ../config/turret.toml)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    _camera_args(sub.add_parser("step1", help="people + distances"))
    _camera_args(sub.add_parser("step2", help="aim point + tracking"))
    s3 = sub.add_parser("step3", help="manual pan/tilt over the serial link")
    s3.add_argument("--port", help="sim, COM5, /dev/ttyACM0, socket://localhost:7777 (default: config link.port)")
    sim = sub.add_parser("sim", help="pretend ESP32 over TCP")
    sim.add_argument("--tcp", type=int, default=7777)
    b = sub.add_parser("ballistics", help="physics-model table")
    b.add_argument("--speeds", type=float, nargs="+", help="exit speeds in m/s (default: 13 and the config cap)")
    sub.add_parser("vectors", help="protocol test frames")
    sub.add_parser("temp", help="list TEMP placeholders")
    args = ap.parse_args(argv)
    cfg = config.load(args.config)

    if args.cmd == "step1":
        from .apps import step1
        step1.run(cfg, args)
    elif args.cmd == "step2":
        from .apps import step2
        step2.run(cfg, args)
    elif args.cmd == "step3":
        from .apps import step3
        step3.run(cfg, args)
    elif args.cmd == "sim":
        from .apps import simserver
        simserver.run(cfg, args)
    elif args.cmd == "ballistics":
        print_ballistics(cfg, args.speeds)
    elif args.cmd == "vectors":
        print_vectors()
    elif args.cmd == "temp":
        rows = config.temp_values(args.config)
        width = max(len(k) for k, _, _ in rows)
        for k, v, note in rows:
            print(f"{k:<{width}}  = {v:<28} {note}")
        print(f"\n{len(rows)} TEMP values. See TEMP.md for how to measure each one.")


def print_ballistics(cfg, speeds=None):
    from .ballistics import Candy, rpm_for_speed, speed_for_rpm, table
    candy = Candy.from_cfg(cfg)
    lc = cfg.launcher
    speeds = speeds or sorted({10.0, float(lc.velocity_cap_mps)})
    d = lc.wheel_diameter_mm / 1000
    print(f"M&M {candy.mass_kg * 1000:.2f} g, {candy.diameter_m * 1000:.1f} x {candy.thickness_m * 1000:.1f} mm, "
          f"Cd {candy.cd}, drag k = {candy.k:.4f} /m   (physics guess, not calibrated)")
    print(f"launcher: {lc.layout}, {lc.wheel_diameter_mm:g} mm wheel, slip {lc.slip}")
    for v in speeds:
        print(f"  {v:4.1f} m/s needs {rpm_for_speed(v, 'single_wheel', d, lc.slip):6.0f} RPM single wheel + backstop, "
              f"{rpm_for_speed(v, 'dual_wheel', d, lc.slip):6.0f} RPM two wheels")
    cap = lc.velocity_cap_mps
    safe_rpm = rpm_for_speed(cap, lc.layout, d, 1.0)
    print(f"  cap-safe RPM_CAP until the chronograph says otherwise: {safe_rpm:.0f} RPM"
          f" (the cap at zero slip, the fastest case)")
    top_rpm = lc.motor_kv * cfg.power.battery_full_v
    top = speed_for_rpm(top_rpm, lc.layout, d, 1.0)
    if top * 1.10 <= cap:
        verdict = "OK: full throttle can't pass the cap, with 10% margin for KV tolerance"
    elif top <= cap:
        verdict = "TOO CLOSE: under the cap but inside the 10% KV tolerance; measure the real KV"
    else:
        verdict = "OVER: only the firmware holds the cap"
    print(f"  hardware ceiling: {lc.motor_kv} KV x {cfg.power.battery_full_v} V = {top_rpm:.0f} RPM no-load"
          f" -> {top:.1f} m/s at zero slip vs cap {cap:g} m/s. {verdict}")
    print(f"  expected full-throttle speed at slip {lc.slip}: {top * lc.slip:.1f} m/s")
    print("\n speed  range   tilt   flight   impact speed   impact energy")
    for v, x, a, t, vi, e in table(candy, speeds):
        if a is None:
            print(f"{v:5.1f}  {x:4.1f} m   out of reach")
        else:
            print(f"{v:5.1f}  {x:4.1f} m  {a:5.1f} deg  {t * 1000:5.0f} ms   {vi:5.1f} m/s     {e:5.3f} J")
    print("\nTilt is for a target level with the muzzle. Real shots will differ; step 7 measures them.")


def print_vectors():
    from . import protocol as P
    examples = [
        ("HELLO", P.Msg.HELLO, 0, dict(proto=1, host_boot_id=0x1234)),
        ("SETPOINT pan +12.34 tilt -5.00 rpm 0, not armed", P.Msg.SETPOINT, 7,
         dict(pan_cdeg=1234, tilt_cdeg=-500, fly_rpm=0, flags=0)),
        ("SETPOINT pan 0 tilt +10.00 rpm 12000, host arm", P.Msg.SETPOINT, 8,
         dict(pan_cdeg=0, tilt_cdeg=1000, fly_rpm=12000, flags=P.FLAG_HOST_ARM)),
        ("FIRE shot 1, press 3, 12000 +-300 rpm, 2.50 m", P.Msg.FIRE, 9,
         dict(shot_id=1, press_id=3, rpm=12000, rpm_tol=300, range_cm=250)),
        ("TELEMETRY", P.Msg.TELEMETRY, 42, dict(t_ms=123456, ack_seq=8, state=2, fault=0, switches=0x83,
                                               pan_cdeg=-250, tilt_cdeg=990, rpm=11950, rpm_target=12000,
                                               shots=5, press_id=3, vbat_cv=1480)),
        ("SHOT FIRED", P.Msg.SHOT, 43, dict(shot_id=1, result=0, rpm_before=12010, rpm_min=11800, dip_ms=64)),
    ]
    print(f"CRC-16/CCITT-FALSE check: crc16(b'123456789') = {P.crc16(b'123456789'):#06x} (must be 0x29b1)\n")
    for name, mtype, seq, fields in examples:
        body = P.pack_body(mtype, seq, **fields)
        print(f"{name}\n  body: {body.hex(' ')}\n  wire: {P.encode(mtype, seq, **fields).hex(' ')}\n")


if __name__ == "__main__":
    sys.exit(main())
