"""Run the pretend ESP32 as a TCP server, so the host connects to it like a real port:

    python -m candyturret sim --tcp 7777
    python -m candyturret step3 --port socket://localhost:7777

Type commands into this window to work the pretend switches: arm, disarm, press, kill, power,
jam, sensor, status, quit.
"""
from __future__ import annotations

import socket
import threading
import time

from .. import protocol as P
from ..sim_esp32 import SimESP32, limits_from_cfg

HELP = "commands: arm, disarm, press, kill, power, jam, sensor, status, quit"


def _console(sim: SimESP32, stop: threading.Event):
    print(HELP)
    while not stop.is_set():
        try:
            cmd = input("> ").strip().lower()
        except EOFError:
            return
        if cmd == "arm":
            sim.arm_switch = True
        elif cmd == "disarm":
            sim.arm_switch = False
        elif cmd == "press":
            sim.press_trigger()
        elif cmd == "kill":
            sim.power_ok = False
        elif cmd == "power":
            sim.power_ok = True
        elif cmd == "jam":
            sim.jam_next = True
        elif cmd == "sensor":
            sim.rpm_sensor_broken = not sim.rpm_sensor_broken
        elif cmd == "status":
            print(f"state {P.State(sim.state).name}  fault {P.Fault(sim.fault).name}  arm switch {sim.arm_switch}"
                  f"  power {sim.power_ok}  rpm {sim.rpm:.0f}/{sim.rpm_target}  pan {sim.pan / 100:+.1f}"
                  f"  tilt {sim.tilt / 100:+.1f}  hopper {sim.hopper}")
        elif cmd in ("quit", "exit"):
            stop.set()
        elif cmd:
            print(HELP)


def run(cfg, args):
    sim = SimESP32(limits_from_cfg(cfg))
    stop = threading.Event()
    threading.Thread(target=_console, args=(sim, stop), daemon=True).start()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", args.tcp))
    srv.listen(1)
    srv.settimeout(0.5)
    print(f"pretend ESP32 listening on socket://localhost:{args.tcp}")
    t0 = time.monotonic()
    try:
        while not stop.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                sim.tick(int((time.monotonic() - t0) * 1000))       # keep time moving with no host
                continue
            print("host connected from", addr)
            conn.setblocking(False)
            with conn:
                while not stop.is_set():
                    try:
                        data = conn.recv(4096)
                        if not data:
                            break
                        sim.feed(data)
                    except BlockingIOError:
                        pass
                    except ConnectionError:
                        break
                    out = sim.tick(int((time.monotonic() - t0) * 1000))
                    if out:
                        try:
                            conn.sendall(out)
                        except (BlockingIOError, ConnectionError):
                            break
                    time.sleep(0.002)
            print("host disconnected")
    finally:
        srv.close()
