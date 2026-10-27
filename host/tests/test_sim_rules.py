"""The firmware safety rules (docs/FIRMWARE_SAFETY.md), checked against the pretend ESP32.
These are the same scenarios the real firmware should be tested with on the bench."""
import pytest

from candyturret import protocol as P
from candyturret.protocol import Fault, Msg, ShotResult, State
from candyturret.sim_esp32 import FirmwareLimits, SimESP32


class Bench:
    """Drives the sim in exact 5 ms steps, like a host sending SETPOINT at 50 Hz."""

    def __init__(self, boot=True, **limits):
        self.sim = SimESP32(FirmwareLimits(**limits))
        self.dec = P.FrameDecoder()
        self.t = 0
        self.seq = 0
        self.packets = []
        self.sp = dict(pan_cdeg=0, tilt_cdeg=0, fly_rpm=0, flags=0)
        self.sending = True
        if boot:
            self.run(5)                                # boot with the arm switch off

    def send(self, mtype, **f):
        self.sim.feed(P.encode(mtype, self.seq, **f))
        self.seq = (self.seq + 1) & 0xFF

    def run(self, ms):
        for _ in range(ms // 5):
            if self.sending and self.t % 20 == 0:
                self.send(Msg.SETPOINT, **self.sp)
            self.packets += self.dec.feed(self.sim.tick(self.t))
            self.t += 5

    def shots(self):
        return [p for p in self.packets if p.type == Msg.SHOT]

    def arm(self, rpm=10000):
        """Host arms first, then the switch goes OFF->ON (arming is edge-triggered)."""
        self.sp.update(flags=P.FLAG_HOST_ARM, fly_rpm=rpm)
        self.sim.arm_switch = False
        self.run(40)
        self.sim.arm_switch = True
        self.run(1500)
        assert self.sim.state == State.ARMED

    def fire(self, rpm=10000, tol=300, range_cm=250, press=True):
        if press:
            self.sim.press_trigger()
            self.run(10)
        self.send(Msg.FIRE, shot_id=len(self.shots()) + 1, press_id=self.sim.press_id, rpm=rpm, rpm_tol=tol, range_cm=range_cm)
        self.run(1000)
        return self.shots()[-1].result


def test_boot_sends_info_and_answers_hello():
    b = Bench()
    b.run(20)
    assert any(p.type == Msg.INFO for p in b.packets)
    n = sum(p.type == Msg.INFO for p in b.packets)
    b.send(Msg.HELLO, proto=P.PROTO_VERSION, host_boot_id=1)
    b.run(20)
    assert sum(p.type == Msg.INFO for p in b.packets) == n + 1


def test_arm_needs_switch_and_host():
    b = Bench()
    b.sp.update(flags=P.FLAG_HOST_ARM)
    b.run(100)
    assert b.sim.state == State.DISARMED               # host alone can't arm
    b.sp.update(flags=0)
    b.sim.arm_switch = True
    b.run(100)
    assert b.sim.state == State.DISARMED               # switch alone can't arm
    b.sp.update(flags=P.FLAG_HOST_ARM)
    b.run(100)
    assert b.sim.state == State.DISARMED               # switch was already on: no OFF->ON edge
    b.sim.arm_switch = False
    b.run(20)
    b.sim.arm_switch = True
    b.run(20)
    assert b.sim.state == State.ARMED


def test_host_arm_drop_needs_switch_cycle():
    b = Bench()
    b.arm()
    b.sp.update(flags=0)
    b.run(60)
    assert b.sim.state == State.DISARMED
    b.sp.update(flags=P.FLAG_HOST_ARM)
    b.run(100)
    assert b.sim.state == State.DISARMED               # the host can't re-arm on its own


def test_arm_switch_on_at_boot_is_a_fault():
    b = Bench(boot=False)
    b.sim.arm_switch = True
    b.sp.update(flags=P.FLAG_HOST_ARM)
    b.run(100)
    assert b.sim.fault == Fault.ARM_ON_AT_BOOT and b.sim.state == State.FAULT
    b.sim.arm_switch = False
    b.run(20)
    b.sim.arm_switch = True
    b.run(20)
    assert b.sim.state == State.ARMED


def test_link_timeout_stops_everything_and_needs_switch_cycle():
    b = Bench()
    b.arm()
    assert b.sim.rpm > 9000
    b.sending = False
    b.run(195)
    assert b.sim.state == State.ARMED                  # 195 ms of silence is still OK
    b.run(30)
    assert b.sim.fault == Fault.LINK_TIMEOUT and b.sim.rpm_target == 0
    pan = b.sim.pan
    b.sending = True
    b.sp.update(pan_cdeg=2000)
    b.run(300)
    assert b.sim.state == State.FAULT                  # host back, still latched
    assert b.sim.pan != pan                            # aiming resumes once the host talks again
    b.sim.arm_switch = False
    b.run(20)
    assert b.sim.fault == Fault.NONE
    b.sim.arm_switch = True
    b.run(20)
    assert b.sim.state == State.ARMED


def test_turret_holds_still_while_link_is_down():
    b = Bench()
    b.sp.update(pan_cdeg=3000)
    b.run(50)
    b.sending = False
    b.run(300)
    held = b.sim.pan
    b.run(300)
    assert b.sim.pan == held


def test_rpm_is_capped():
    b = Bench(rpm_cap=8000)
    b.sp.update(fly_rpm=20000)
    b.arm(rpm=20000)
    assert b.sim.rpm_target == 8000 and b.sim.rpm < 8000 * 1.01


def test_one_press_one_shot():
    b = Bench()
    b.arm()
    assert b.fire() == ShotResult.FIRED
    shots = b.sim.shots
    b.send(Msg.FIRE, shot_id=99, press_id=b.sim.press_id, rpm=10000, rpm_tol=300, range_cm=250)
    b.run(600)
    assert b.shots()[-1].result == ShotResult.REJ_NO_PRESS and b.sim.shots == shots


def test_fire_without_press_or_with_stale_press_is_rejected():
    b = Bench()
    b.arm()
    assert b.fire(press=False) == ShotResult.REJ_NO_PRESS
    b.sim.press_trigger()
    b.run(500)                                         # press is now older than the 400 ms window
    assert b.fire(press=False) == ShotResult.REJ_NO_PRESS


def test_fire_rejections():
    b = Bench(rpm_cap=12000)
    b.arm()
    assert b.fire(range_cm=149) == ShotResult.REJ_RANGE
    assert b.fire(rpm=12001) == ShotResult.REJ_RPM_CAP
    b.sp.update(pan_cdeg=3900)
    b.run(500)
    assert b.fire() == ShotResult.REJ_FIRE_ZONE
    b.sp.update(pan_cdeg=0)
    b.run(500)
    assert b.fire() == ShotResult.FIRED


def test_fire_when_disarmed_is_rejected():
    b = Bench()
    b.run(100)
    assert b.fire() == ShotResult.REJ_NOT_ARMED


def test_shot_rate_limit():
    b = Bench(min_shot_interval_ms=2000)
    b.arm()
    assert b.fire() == ShotResult.FIRED
    assert b.fire() == ShotResult.REJ_RATE


def test_kill_switch_while_armed_latches():
    b = Bench()
    b.arm()
    b.sim.power_ok = False
    b.run(50)
    assert b.sim.fault == Fault.KILL and b.sim.rpm_target == 0
    b.sim.arm_switch = False
    b.run(50)
    assert b.sim.fault == Fault.KILL                   # kill switch still open: fault stays
    b.sim.power_ok = True
    b.run(50)
    assert b.sim.fault == Fault.NONE and b.sim.state == State.DISARMED
    b.run(500)
    assert b.sim.rpm_target == 0                       # resetting the kill switch alone never spins up
    b.sim.arm_switch = True
    b.run(50)
    assert b.sim.state == State.ARMED


def test_kill_reset_with_switch_on_stays_disarmed():
    b = Bench()
    b.arm()
    b.sim.power_ok = False
    b.run(50)
    b.sim.power_ok = True
    b.run(500)
    assert b.sim.state == State.FAULT and b.sim.rpm_target == 0


def test_dead_rpm_sensor_faults_instead_of_running_away():
    b = Bench()
    b.arm()
    b.sim.rpm_sensor_broken = True
    b.run(400)
    assert b.sim.fault == Fault.RPM_SENSOR and b.sim.rpm_target == 0


def test_jam_and_empty_hopper():
    b = Bench()
    b.arm()
    b.sim.jam_next = True
    assert b.fire() == ShotResult.JAM
    assert b.sim.fault == Fault.FEED
    b.sim.arm_switch = False
    b.run(50)
    b.sim.arm_switch = True
    b.run(1500)
    assert b.sim.state == State.ARMED
    b.sim.hopper = 0
    assert b.fire() == ShotResult.NO_DIP
    assert b.sim.fault == Fault.FEED                   # first NO_DIP latches: the next feed could push two


def test_disarm_during_spinup_cancels_shot():
    b = Bench(spinup_timeout_ms=3000)
    b.arm(rpm=2000)
    b.sim.press_trigger()
    b.run(10)
    b.send(Msg.FIRE, shot_id=7, press_id=b.sim.press_id, rpm=12000, rpm_tol=100, range_cm=250)
    b.run(50)
    b.sim.arm_switch = False
    b.run(100)
    assert b.shots()[-1].result == ShotResult.REJ_NOT_ARMED and b.sim.shot is None


@pytest.mark.parametrize("bad", [b"\x00" * 50, b"\x01\x02\x03\x00", bytes(range(1, 200)) + b"\x00"])
def test_garbage_does_not_count_as_heartbeat(bad):
    b = Bench()
    b.arm()
    b.sending = False
    for _ in range(60):
        b.sim.feed(bad)
        b.run(5)
    assert b.sim.fault == Fault.LINK_TIMEOUT


def test_swinging_out_of_fire_window_during_spinup_aborts():
    b = Bench(spinup_timeout_ms=3000)
    b.arm(rpm=2000)
    b.sim.press_trigger()
    b.run(10)
    b.send(Msg.FIRE, shot_id=5, press_id=b.sim.press_id, rpm=12000, rpm_tol=300, range_cm=250)
    b.run(20)
    b.sp.update(pan_cdeg=3990, tilt_cdeg=2990)         # inside the motion limits, outside the fire window
    b.run(1500)
    assert b.shots()[-1].result == ShotResult.REJ_FIRE_ZONE
    assert b.sim.hopper == 200


def test_aim_is_frozen_during_feed():
    b = Bench()
    b.arm()
    b.sim.press_trigger()
    b.run(10)
    b.send(Msg.FIRE, shot_id=1, press_id=b.sim.press_id, rpm=10000, rpm_tol=300, range_cm=250)
    while not (b.sim.shot and b.sim.shot["phase"] == "feed"):
        b.run(5)
    pan = b.sim.pan
    b.sp.update(pan_cdeg=3000)
    b.run(40)
    assert b.sim.pan == pan


def test_press_while_disarmed_never_fires():
    b = Bench()
    b.sp.update(flags=P.FLAG_HOST_ARM, fly_rpm=10000)
    b.run(40)
    b.sim.press_trigger()                              # pressed while the arm switch is off
    b.run(10)
    pid = b.sim.press_id
    b.sim.arm_switch = True
    b.run(100)
    assert b.sim.state == State.ARMED
    b.send(Msg.FIRE, shot_id=1, press_id=pid, rpm=10000, rpm_tol=300, range_cm=250)
    b.run(100)
    assert b.shots()[-1].result == ShotResult.REJ_NO_PRESS


def test_rejected_fire_still_spends_the_press():
    b = Bench()
    b.run(40)
    b.sim.press_trigger()
    b.run(10)
    b.send(Msg.FIRE, shot_id=1, press_id=b.sim.press_id, rpm=10000, rpm_tol=300, range_cm=250)
    b.run(10)
    assert b.shots()[-1].result == ShotResult.REJ_NOT_ARMED and b.sim.press_used


def test_press_zero_is_never_valid():
    b = Bench()
    b.arm()
    b.send(Msg.FIRE, shot_id=1, press_id=0, rpm=10000, rpm_tol=300, range_cm=250)
    b.run(50)
    assert b.shots()[-1].result == ShotResult.REJ_NO_PRESS


def test_wide_rpm_tolerance_is_rejected():
    b = Bench()
    b.arm()
    assert b.fire(rpm=10000, tol=600) == ShotResult.REJ_BAD_REQUEST


def test_only_setpoint_keeps_the_link_alive():
    b = Bench()
    b.arm()
    b.sending = False
    for i in range(60):                                # HELLO and FIRE keep coming, SETPOINT doesn't
        b.send(Msg.HELLO, proto=1, host_boot_id=1)
        b.send(Msg.FIRE, shot_id=100 + i, press_id=0, rpm=10000, rpm_tol=300, range_cm=250)
        b.run(5)
    assert b.sim.fault == Fault.LINK_TIMEOUT


def test_disarm_mid_feed_before_release_aborts_without_a_shot():
    b = Bench()
    b.arm()
    b.sim.press_trigger()
    b.run(10)
    b.send(Msg.FIRE, shot_id=1, press_id=b.sim.press_id, rpm=10000, rpm_tol=300, range_cm=250)
    while not (b.sim.shot and b.sim.shot["phase"] == "feed"):
        b.run(5)
    b.run(20)                                          # 20 ms into the feed, candy not released yet
    b.sim.arm_switch = False
    b.run(5)
    assert b.shots()[-1].result == ShotResult.ABORTED
    assert b.sim.hopper == 200 and b.sim.no_dip_run == 0
    assert not b.sim.feeder_home                       # indexer is still reversing home
    b.run(50)
    assert b.sim.feeder_home
