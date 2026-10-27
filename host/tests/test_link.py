"""TurretLink against the in-process pretend ESP32, in real time."""
import time

from candyturret import protocol as P
from candyturret.link import TurretLink
from candyturret.sim_esp32 import FirmwareLimits, SimESP32, SimPort


def wait_for(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


def make():
    sim = SimESP32(FirmwareLimits())
    return sim, TurretLink(SimPort(sim), setpoint_hz=50, timeout_ms=200)


def test_link_comes_up_and_aims():
    sim, link = make()
    with link:
        assert wait_for(lambda: link.link_ok and link.proto_ok)
        assert link.info.rpm_cap == sim.lim.rpm_cap
        for _ in range(60):
            link.touch()
            link.set_aim(15.0, 5.0)
            time.sleep(0.01)
        assert wait_for(lambda: abs(link.telemetry.pan_cdeg - 1500) < 5 and abs(link.telemetry.tilt_cdeg - 500) < 5)
        assert link.latency_s is not None and link.latency_s < 0.1
        assert sum(link.dec.errors.values()) == 0


def test_host_clamps_to_firmware_limits():
    sim, link = make()
    with link:
        assert wait_for(lambda: link.info is not None)
        link.set_aim(170.0, -80.0)
        link.set_flywheel(99999)
        assert link.pan_deg == sim.lim.pan_max_cdeg / 100
        assert link.tilt_deg == sim.lim.tilt_min_cdeg / 100
        assert link.fly_rpm == sim.lim.rpm_cap


def test_frozen_main_loop_sends_safe_setpoint():
    sim, link = make()
    with link:
        link.set_flywheel(9000)
        link.set_host_arm(True)
        for _ in range(30):
            link.touch()
            time.sleep(0.01)
        assert wait_for(lambda: sim.sp_rpm == 9000 and sim.host_arm)
        # main loop stops calling touch(): writer must fall back to flywheel 0, not armed
        assert wait_for(lambda: sim.sp_rpm == 0 and not sim.host_arm, timeout=1.0)
        assert link.link_ok                         # ...while the heartbeat itself keeps going


def test_silent_host_trips_firmware_timeout():
    sim, link = make()
    with link:
        assert wait_for(lambda: link.link_ok)
        link.paused = True
        assert wait_for(lambda: sim.fault == P.Fault.LINK_TIMEOUT, timeout=1.0)
        link.paused = False
        assert wait_for(lambda: sim.fault == P.Fault.NONE)   # arm switch is off, so it clears


def test_trigger_events_arrive():
    sim, link = make()
    with link:
        assert wait_for(lambda: link.link_ok)
        sim.press_trigger()
        assert wait_for(lambda: not link.events.empty())
        ev = link.events.get_nowait()
        assert ev.type == P.Msg.TRIGGER and ev.press_id == 1


def test_bad_values_never_kill_the_heartbeat():
    sim, link = make()
    with link:
        assert wait_for(lambda: link.link_ok)
        link.set_flywheel(9000)
        link.set_host_arm(True)
        for bad in (float("nan"), float("inf")):
            link.touch()
            link.set_aim(bad, 0.0)
            link.set_flywheel(bad)
        link.set_flywheel(70000)                     # clamps, doesn't crash
        assert link.fire(1, 0, 9000, 300, 2.0) is False          # press 0 never sent
        assert link.fire(1, 3, 9000, 300, float("nan")) is False
        for _ in range(30):
            link.touch()
            time.sleep(0.01)
        assert link.link_ok and link.writer_error is None
        assert sim.fault == P.Fault.NONE
        assert link.bad_inputs >= 4
        assert not sim.host_arm                      # a bad value drops to safe until re-armed
