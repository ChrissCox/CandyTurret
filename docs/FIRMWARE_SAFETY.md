# Firmware safety rules (ESP32-S3)

These rules live in the ESP32 firmware and hold **no matter what the host sends**. The host checks most of them as well, but the host is a big Python program on a general-purpose computer, so it is never the last line of defense.

`host/candyturret/sim_esp32.py` is a Python model of these rules, and `host/tests/test_sim_rules.py` tests that model. Those same scenarios make a good bench checklist for the real firmware (see the end of this page).

**Use unsigned time math for every timer**: `(uint32_t)(now - t) < LIMIT`. Never compare two raw `millis()` values, and never rely on a "long ago" starting value, because `millis()` wraps after 49.7 days.

## Constants (compile-time, not settable from the host)

| Name | Start value | Notes |
|---|---|---|
| `RPM_CAP` | **TEMP**, from the step 5 chronograph test | the RPM that gave the speed cap with a **new, clean wheel on a full battery** (the fastest case) |
| `THROTTLE_MAX` | **TEMP** | the throttle that reaches `RPM_CAP` (not more) on a full battery. A ceiling for when the RPM sensor lies |
| `OVERSPEED` | `RPM_CAP` × 1.05 for 50 ms | → fault OVERSPEED |
| `SENSOR_FAULT` | target > 1000 RPM but reading < 100 RPM for 300 ms | → fault RPM_SENSOR |
| `SENSOR_PLAUSIBLE` | reading ≥ 70% of the RPM expected for this throttle × vbat, and within 10% of DShot eRPM when the ESC reports it, for 300 ms | otherwise → fault RPM_SENSOR. **TEMP**: build the throttle → RPM table in step 5 |
| `LINK_TIMEOUT_MS` | 200 | |
| `MIN_RANGE_CM` | 150 | TEMP: your starting value |
| `PRESS_WINDOW_MS` | 400 | a press is valid while `(now - press_t) < 400` |
| `DEBOUNCE_MS` | 20 | fire and arm inputs |
| `PLUG_SETTLE_MS` | 500 | after a jack is plugged in, ignore that input this long |
| `MIN_SHOT_INTERVAL_MS` | 400 | **feed start to feed start**, checked when a FIRE arrives. TEMP |
| `SPINUP_TIMEOUT_MS` | 1500 | |
| `RPM_SETTLE_MS` | 100 | RPM must stay inside the band this long before feeding |
| `MAX_TOL` | 5% of the FIRE's rpm | a wider `rpm_tol` → REJ_BAD_REQUEST |
| `AIM_TOL` | 0.5° | pan/tilt must be within this of the setpoint before feeding |
| `DIP_FRACTION` | 1.5% | **TEMP**: set from the RPM trace in step 6. The dip may only be 2-3% |
| `DIP_WINDOW_MS` | 300 | |
| `FEED_TIMEOUT_MS` | 300 | **TEMP**: depends on the feeder |
| `MAX_NO_DIP` | 1 | the **first** NO_DIP latches FEED, because the next feed could push two candies through |
| Motion limits | pan ±40°, tilt −10…+30° | **TEMP**: just inside the mechanical hard stops |
| Fire window | pan ±35°, tilt −5…+25° | **TEMP**: stricter than the motion limits. Must never include the rider, the chair or straight up |

## Rules

**R1. Speed cap.**
- `RPM_CAP` is compiled in. SETPOINT `fly_rpm` is clamped to it, and a FIRE asking for more is rejected (REJ_RPM_CAP).
- Close the speed loop on an **independent RPM sensor**: a hall sensor with a magnet, or an optical sensor with a reflective stripe, counted with the S3's PCNT peripheral. When the ESC reports DShot eRPM, **cross-checking it is mandatory**.
- Clamp the controller's integrator and never command more than `THROTTLE_MAX`.
- A zero, stale or implausibly low reading (`SENSOR_FAULT`, `SENSOR_PLAUSIBLE`) is a **sensor fault, never "too slow"**. A sensor that loses a magnet reads half and would otherwise drive the PID to full throttle.
- Overspeed → throttle 0, fault OVERSPEED.
- A candy is only fed while the measured RPM stays at or below `RPM_CAP` for the whole settle time (R9).
- Best of all is picking the motor KV, the full-charge battery voltage and the wheel size so that 100% throttle *can't* pass the cap even with zero slip (DESIGN.md, "Launcher"). Then the firmware cap is a second layer instead of the only one.

**R2. Arming is edge-triggered.**
- The firmware enters ARMED only on an **arm-switch OFF→ON transition** that happens while all of these are already true:
  - `HOST_ARM` is set
  - the link is OK
  - POWER_OK is high
  - no fault is latched
- It leaves ARMED the moment any of those fails, or the switch goes OFF. **Getting back requires the switch to go OFF and ON again**, so neither the host, a reset kill switch nor a returning link can re-arm by itself.
- Anything other than ARMED means throttle 0 (with active brake if the ESC has it) and the feeder at home.
- Operating order: the host arms first, and the switch goes on last.

**R3. Kill switch.**
- It cuts flywheel ESC power **in hardware**, not through the firmware.
- The firmware senses POWER_OK, for example through a divider on the ESC side of the switch. Losing POWER_OK while armed latches fault KILL.
- The kill switch has to be something Chris can operate himself, plus a big one that friends can hit.

**R4. Faults latch.** A fault clears only when the arm switch is OFF **and** that fault's cause is gone:

| Fault | Cause gone when |
|---|---|
| LINK_TIMEOUT | a valid SETPOINT arrives again |
| OVERSPEED | measured RPM is below `RPM_CAP` |
| FEED | the feeder is at home (after someone has checked it) |
| LIMIT_HIT | no limit switch is active |
| RPM_SENSOR | **only a reboot**. The sensor is suspect until the next power-up |
| ARM_ON_AT_BOOT | the switch is OFF |
| KILL | POWER_OK is high again (kill switch reset) |
| LOW_BATTERY | vbat is back above the cutoff plus hysteresis |

Clearing a fault never arms by itself. R2's OFF→ON edge is still needed.

**R5. One press, one shot.**
- The fire input is wired to the ESP32, not to the host.
- Each debounced press increments `press_id` (1..65535, wrapping to 1, **never 0**), records `press_t` at debounce completion (the same value as `TRIGGER.t_ms`), and sends TRIGGER.
- A press made while not ARMED is **born used**.
- At boot the "latest press" is marked used. Every loop, a press whose age reaches `PRESS_WINDOW_MS` is marked used, so expiry latches and a timer wrap can't bring it back.
- FIRE checks run **in this order**, and the first failure is the reply:
  1. If `press_id` is non-zero and equals the latest press, **mark that press used now**, whatever the result below. Note whether it was fresh: unused and younger than the window.
  2. A shot is already in progress → REJ_BUSY
  3. Not ARMED → REJ_NOT_ARMED
  4. The press wasn't fresh (wrong id, 0, used, or old) → REJ_NO_PRESS
  5. `range_cm < MIN_RANGE_CM` → REJ_RANGE
  6. `rpm == 0` or `rpm > RPM_CAP` → REJ_RPM_CAP
  7. `rpm_tol > 5%` of rpm → REJ_BAD_REQUEST
  8. Less than `MIN_SHOT_INTERVAL_MS` since the last feed started → REJ_RATE
  9. Current pan/tilt **or** the current setpoint outside the fire window → REJ_FIRE_ZONE
- **Never retry a feed automatically.**

**R6. Range floor.** A FIRE with `range_cm < MIN_RANGE_CM` is rejected. The host does the real check from the camera; this one catches host bugs.

**R7. Fire window.**
- Pan/tilt must be inside the fire window **for the whole shot**, not just when the FIRE arrives.
- While spinning up, if the current position or the setpoint leaves the window, abort with REJ_FIRE_ZONE (the feeder never moves).
- From feed start until the feeder is home, **the aim is frozen**: ignore new pan/tilt setpoints.
- The window is stricter than the motion limits, and it is set so the barrel can never point at the rider or the chair.
- Back it up with **mechanical hard stops**.

**R8. Feeder.**
- One index per shot.
- When not feeding, the feeder sits at home, closed **by its geometry**: an indexer pocket that only lines up with the outlet mid-rotation. Holding torque doesn't count, because a servo with no power holds nothing.
- **Disarm or fault during a feed:** brake the ESC, and never stop the indexer mid-rotation. If the pocket hasn't reached the outlet yet, **reverse it back to home** (the candy stays in the pocket). If it has already passed the outlet, finish the rotation to home.
- Report ABORTED unless the dip was already seen (then FIRED). ABORTED doesn't count toward `MAX_NO_DIP`.
- Even so, a candy released in the last ~60 ms before a disarm can still meet the coasting wheel. That's physics; the active brake shortens the window.

**R9. Shot confirmation and jams.**
- **Feed only after** the measured RPM has stayed inside `[rpm − tol, min(rpm + tol, RPM_CAP)]` and the aim within `AIM_TOL`, for `RPM_SETTLE_MS`.
- After the feed starts, an RPM drop of `DIP_FRACTION` within `DIP_WINDOW_MS` means FIRED.
- No dip means NO_DIP, and **the first NO_DIP latches fault FEED** (`MAX_NO_DIP = 1`): the hopper is empty or a candy is stuck before the wheel, and the next feed could push two through. Someone looks, then cycles the arm switch.
- If the feeder doesn't reach home within `FEED_TIMEOUT_MS`, that's a JAM: reverse-jog once to free it, then latch FEED.
- Evaluate OVERSPEED and RPM_SENSOR **before** deciding to arm or feed in each loop, so a fault found this loop stops a feed this loop.

**R10. Rate limit.**
- At least `MIN_SHOT_INTERVAL_MS` from one feed start to the next. This is checked when the FIRE arrives (REJ_RATE).
- Only one shot in progress at a time (otherwise REJ_BUSY).

**R11. Link loss.**
- **Only a valid SETPOINT** refreshes the link timer. HELLO and FIRE are processed but don't count, and ESP32→host message types (0x80-0xFF) arriving at the ESP32 are dropped.
- When the link is lost: throttle 0, feeder home, pan/tilt hold where they are, any shot not yet feeding cancelled (a feeding shot follows R8), and LINK_TIMEOUT latches.
- Pan/tilt may follow setpoints again when the link returns. Arming waits for R2's switch cycle.

**R12. Motion.**
- Clamp setpoints to the motion limits and slew-limit them.
- A limit switch closing at a spot the clamped setpoint should never reach means the mechanics or the calibration are wrong. Stop motion and latch LIMIT_HIT.

**R13. Boot and resets.**
- The ESC signal pin has a pull-down and is set to zero throttle before anything else runs.
- Enable the task watchdog and the brownout detector.
- After any reset the firmware starts DISARMED, with the latest press marked used.
- Turn Wi-Fi and Bluetooth **off** (USB is the only link), so no radio is transmitting next to the wheelchair controller.

**R14. Battery.** If `vbat` stays below the pack's cutoff for 1 s, latch LOW_BATTERY.

## Inputs

**Fire and arm inputs use 3.5 mm jacks with a plug-detect (switched) contact.**
- Contact closure to ground, internal pull-up, `DEBOUNCE_MS`, and polarity set per input. Most assistive switches are normally open, but some (one listing for the AbleNet Buddy Button, for example) are normally closed.
- **While a jack is unplugged, and for `PLUG_SETTLE_MS` after it's plugged in, that input is unavailable**: the arm input reads OFF, and no fire presses are counted.
- Sliding a mono plug in briefly shorts the tip to ground, and a normally-closed switch reads "pressed" when its cable is pulled. Without plug-detect, either of those could make a press, or an arm, out of nothing.
- A fire press only counts if the input was idle for at least `PLUG_SETTLE_MS` before it.
- If the arm jack has no plug-detect contact, normally-closed polarity is **not allowed** on the arm input.

Any assistive switch plugs straight in, and so does a QuadStick that has the optional relay-output jack.

**Limit switches and feeder home:** normally closed is better, so a broken wire reads as "at limit" (safe) instead of "clear".

## Bench checklist for the real firmware

Run these with the step 3 panel (`python -m candyturret step3 --port COMx`). For the shot tests, use a scratch script that sends FIRE frames. The Python versions are in `host/tests/test_sim_rules.py`.

1. Boot with the arm switch ON → FAULT ARM_ON_AT_BOOT. Switch OFF then ON with HOST_ARM set → ARMED.
2. HOST_ARM set while the switch is already ON → stays DISARMED until the switch is cycled. HOST_ARM dropped and set again while ARMED → DISARMED, and it stays that way.
3. Freeze the host (`x` in step 3) while armed → LINK_TIMEOUT within 250 ms, flywheel to 0. Unfreeze → still FAULT until the switch is cycled.
4. While frozen, keep sending HELLO/FIRE frames or garbage bytes → still times out (only SETPOINT counts).
5. SETPOINT `fly_rpm` above the cap → telemetry `rpm_target` equals `RPM_CAP`.
6. Unplug the RPM sensor while spinning → RPM_SENSOR, flywheel stops. Then cycling the arm switch must **not** clear it; only a reboot does. Tape over half the sensor's marks → plausibility fault.
7. Open the kill switch while armed → KILL. Switch OFF with the kill switch still open → still KILL. Reset the kill switch → DISARMED, **flywheel stays stopped** until the arm switch goes ON again.
8. Plug and unplug the fire and arm jacks, slowly, while ARMED → no TRIGGER and no arming.
9. FIRE with no press, with press 0, with a press older than 400 ms, with a used press, or with a press made while disarmed → REJ_NO_PRESS. One press → at most one SHOT that isn't a rejection.
10. FIRE with range 149 cm → REJ_RANGE. Above the cap → REJ_RPM_CAP. `rpm_tol` over 5% → REJ_BAD_REQUEST. Pan outside the fire window → REJ_FIRE_ZONE. Two feeds less than `MIN_SHOT_INTERVAL_MS` apart → REJ_RATE.
11. FIRE accepted, then swing the setpoint outside the fire window during spin-up → REJ_FIRE_ZONE, feeder never moves. During a feed, setpoint changes are ignored.
12. Empty hopper → first NO_DIP latches FEED. Block the indexer → JAM, then FEED.
13. Arm switch OFF during spin-up → REJ_NOT_ARMED, feeder never moves. Arm switch OFF early in a feed → indexer reverses home, ABORTED.
