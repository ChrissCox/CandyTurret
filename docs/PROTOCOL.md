# Serial protocol v1: host ⇄ ESP32-S3

This is the contract between the host computer and the ESP32 firmware. Both sides are written against this page. The host implementation is `host/candyturret/protocol.py` and `host/candyturret/link.py`. To check a firmware encoder or decoder, run `python -m candyturret vectors` (or see the vectors at the bottom of this page).

The safety rules the firmware has to enforce are in [FIRMWARE_SAFETY.md](FIRMWARE_SAFETY.md). This page only covers bytes and timing.

## Transport

- USB CDC serial. With the S3's native USB (USB-Serial/JTAG or TinyUSB CDC) the baud rate is ignored. If the board uses a USB-UART bridge chip instead, use 921600 baud, 8N1.
- The host opens the port with **DTR and RTS deasserted** so that opening it doesn't reset the ESP32 or drop it into download mode. Firmware must not depend on DTR being set.
- Bytes can arrive in any chunking. Both sides parse a stream, not packets.

## Framing

```
wire:  COBS(body)  0x00
body:  type:u8  seq:u8  payload[...]  crc:u16
```

- **COBS** (Consistent Overhead Byte Stuffing) removes every 0x00 from the body, so 0x00 only ever means "end of frame". After a dropped byte or a garbage burst, the receiver resyncs at the next 0x00.
- **CRC**: CRC-16/CCITT-FALSE. Poly 0x1021, init 0xFFFF, no input or output reflection, xorout 0x0000. Check value: `crc("123456789") = 0x29B1`. It covers `type | seq | payload` and is stored little-endian.
- **Size**: a body is 4 to 64 bytes. COBS adds 1 byte to that, plus the 0x00 delimiter.
- **seq**: each side keeps its own u8 counter, adds 1 per frame sent, and wraps 255 → 0. Receivers never reject frames because of seq. It is used to count losses and to measure latency (`TELEMETRY.ack_seq`).
- **Byte order**: little-endian everywhere. The ESP32 and the host are both little-endian, so packed structs (or `memcpy` field by field) work.

### Receiver rules (both sides)

1. Collect bytes until a 0x00. Ignore empty frames, since extra 0x00 bytes are allowed.
2. If more than 66 bytes arrive without a 0x00, throw them away and skip to the next 0x00.
3. COBS-decode the frame. Drop it on a decode error.
4. Drop it if the length isn't 4 to 64, if the CRC doesn't match, if the type is unknown, or if the payload length doesn't exactly match that type. Count each kind of drop.
5. **Only a valid SETPOINT refreshes the ESP32's link timer.** HELLO and FIRE are processed but don't count, and garbage never does. The ESP32 drops message types 0x80-0xFF, and the host drops 0x01-0x7F, so an echoed or looped-back frame can't be mistaken for the other side.

## Units and conventions

| Quantity | Encoding | Convention |
|---|---|---|
| Pan | `int16` centidegrees | 0 = straight ahead along the chair. **+ = left** (counterclockwise seen from above) |
| Tilt | `int16` centidegrees | 0 = level with the turret base. **+ = up** |
| Flywheel speed | `uint16` RPM | wheel revolutions per minute, measured on the wheel |
| Time | `uint32` ms | ESP32 milliseconds since boot. Wraps after 49 days |
| Range | `uint16` cm | host's measured distance to the target |
| Battery | `uint16` centivolts | 1480 = 14.80 V. 0 = not measured |

## Host → ESP32

### HELLO `0x01` (3-byte payload)

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u8 | proto | `PROTO_VERSION`, currently 1 |
| 1 | u16 | host_boot_id | random per host start, for logs |

Sent on connect, then every 500 ms until an INFO arrives. The ESP32 answers with INFO.

### SETPOINT `0x02` (7-byte payload), sent at 50 Hz, and also the heartbeat

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | i16 | pan_cdeg | firmware clamps to its pan limits |
| 2 | i16 | tilt_cdeg | firmware clamps to its tilt limits |
| 4 | u16 | fly_rpm | flywheel target while armed; 0 = stop. Firmware clamps to `RPM_CAP` |
| 6 | u8 | flags | bit 0 `HOST_ARM`: the host wants to be armed. Other bits are 0. Arming is edge-triggered: the firmware arms only when the arm switch goes OFF→ON while `HOST_ARM` is already set ([R2](FIRMWARE_SAFETY.md)) |

The host keeps the flywheel pre-spun through `fly_rpm` while it has a target locked, so a FIRE can go right away. `fly_rpm` is ignored unless the ESP32 is ARMED.

### FIRE `0x03` (10-byte payload)

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u16 | shot_id | host's counter; echoed in the SHOT reply |
| 2 | u16 | press_id | the TRIGGER press this shot answers |
| 4 | u16 | rpm | flywheel speed for this shot |
| 6 | u16 | rpm_tol | ± window the measured RPM must sit inside before feeding. At most 5% of `rpm`, otherwise REJ_BAD_REQUEST. The firmware also never feeds above `RPM_CAP` |
| 8 | u16 | range_cm | host's range to the target; the firmware rejects anything under its minimum |

Every FIRE gets exactly one SHOT reply with the same `shot_id`. The first FIRE naming a press uses that press up, whatever the reply. The full check order is in [FIRMWARE_SAFETY.md R5](FIRMWARE_SAFETY.md).

## ESP32 → host

### INFO `0x81` (20-byte payload)

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u8 | proto | firmware's `PROTO_VERSION` |
| 1 | u16 | fw_version | |
| 3 | u16 | boot_id | random per boot; a new value means the ESP32 rebooted |
| 5 | u16 | rpm_cap | the compiled-in `RPM_CAP` |
| 7 | i16 | pan_min_cdeg | motion limits |
| 9 | i16 | pan_max_cdeg | |
| 11 | i16 | tilt_min_cdeg | |
| 13 | i16 | tilt_max_cdeg | |
| 15 | u16 | min_range_cm | compiled-in minimum firing range |
| 17 | u16 | link_timeout_ms | normally 200 |
| 19 | u8 | features | bit 0 IMU present, bit 1 independent RPM sensor, bit 2 pan/tilt are measured (not just commanded) |

Sent once at boot without being asked, and again in reply to every HELLO.

### TELEMETRY `0x82` (22-byte payload), every 20 ms

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u32 | t_ms | |
| 4 | u8 | ack_seq | seq of the last valid host frame received |
| 5 | u8 | state | see State |
| 6 | u8 | fault | latched fault, see Fault |
| 7 | u8 | switches | see switch bits |
| 8 | i16 | pan_cdeg | current pan: measured if feature bit 2 is set, otherwise the firmware's slew-limited setpoint |
| 10 | i16 | tilt_cdeg | same for tilt |
| 12 | u16 | rpm | measured flywheel RPM |
| 14 | u16 | rpm_target | the target actually in use, after clamping and arming; 0 when disarmed |
| 16 | u16 | shots | confirmed shots since boot |
| 18 | u16 | press_id | latest trigger press id (0 = none yet) |
| 20 | u16 | vbat_cv | turret battery voltage |

### TRIGGER `0x83` (6-byte payload)

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u16 | press_id | 1 to 65535, then wraps to 1. Never 0; a FIRE naming 0 is always rejected |
| 2 | u32 | t_ms | when the debounce completed. This is the press time the 400 ms window counts from |

Sent on every debounced press of the fire input. If this frame is lost, the host still sees the new `press_id` in the next TELEMETRY.

### SHOT `0x84` (9-byte payload)

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u16 | shot_id | from the FIRE |
| 2 | u8 | result | see ShotResult |
| 3 | u16 | rpm_before | RPM when the feed started (0 if it never started) |
| 5 | u16 | rpm_min | lowest RPM seen during the feed |
| 7 | u16 | dip_ms | ms from feed start to the dip (0 if there was no dip) |

### IMU `0x85` (16-byte payload), optional, 50 to 100 Hz

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u32 | t_ms | |
| 4 | i16 ×3 | gx, gy, gz | gyro, centidegrees/s, turret base frame (x forward, y left, z up) |
| 10 | i16 ×3 | ax, ay, az | accel, mm/s², same frame |

### LOG `0x86` (0 to 60 bytes of UTF-8)

Free text for debugging. The host prints it and never acts on it.

## Enums

**State**: 0 BOOT, 1 DISARMED, 2 ARMED, 3 FIRING (a shot is in progress), 4 FAULT

**Fault**: 0 NONE, 1 LINK_TIMEOUT, 2 OVERSPEED, 3 FEED (jam, or two shots in a row with no RPM dip), 4 LIMIT_HIT, 5 RPM_SENSOR (throttle up with no RPM reading), 6 ARM_ON_AT_BOOT, 7 KILL (kill switch opened while armed), 8 LOW_BATTERY

**Switch bits**: 0x01 ARM switch on, 0x02 POWER_OK (flywheel power present, so the kill switch is closed), 0x04 TRIGGER pressed now, 0x08 PAN_LIMIT_LO, 0x10 PAN_LIMIT_HI, 0x20 TILT_LIMIT_LO, 0x40 TILT_LIMIT_HI, 0x80 FEEDER_HOME

**ShotResult**:

| Code | Name | Meaning |
|---|---|---|
| 0 | FIRED | fed one, and the RPM dip confirmed a shot |
| 1 | NO_DIP | fed one, no dip: empty hopper, or the candy stuck before the wheel. Latches FEED at once |
| 2 | JAM | feeder didn't reach its next index position |
| 3 | ABORTED | disarm or fault during the feed, before a dip was seen. Doesn't count as NO_DIP |
| 10 | REJ_NOT_ARMED | |
| 11 | REJ_NO_PRESS | no fresh, unused press with that `press_id` |
| 12 | REJ_RPM | flywheel didn't settle inside the tolerance before the spin-up timeout |
| 13 | REJ_RANGE | `range_cm` under the firmware minimum |
| 14 | REJ_RATE | too soon after the last shot |
| 15 | REJ_BUSY | a shot is already in progress |
| 16 | REJ_FIRE_ZONE | pan/tilt (or the setpoint) outside the fire window, at the FIRE or at any point during spin-up |
| 17 | REJ_RPM_CAP | asked for 0 RPM or more than the cap |
| 18 | REJ_AIM_MOVING | pan/tilt didn't reach the setpoint before the spin-up timeout |
| 19 | REJ_BAD_REQUEST | `rpm_tol` wider than 5% of `rpm` |

## Heartbeat and link loss

- The host sends SETPOINT every 20 ms. The ESP32 sends TELEMETRY every 20 ms.
- **ESP32 side:** if no valid SETPOINT arrives for more than 200 ms, it sets throttle to 0, holds pan/tilt where they are, cancels any shot that hasn't started feeding (a shot mid-feed sends the indexer home, [R8](FIRMWARE_SAFETY.md)), and latches `LINK_TIMEOUT`. This only applies if a host has sent a SETPOINT since boot. When the host comes back, pan/tilt follow setpoints again, but the fault stays latched until the arm switch is turned OFF, and arming needs the switch back ON.
- **Host side:** if no TELEMETRY arrives for more than 200 ms, the host treats the link as lost, sends no FIRE, and shows LINK LOST.
- **Frozen host:** if the host's main loop stops updating for 150 ms, its link thread keeps sending SETPOINT but with `fly_rpm = 0` and `HOST_ARM` cleared. A hung vision loop therefore can't leave the flywheel spinning, even though the heartbeat thread is still alive.

## Fire sequence

```
 Chris presses fire ──► ESP32: debounce, press_id += 1, TRIGGER(press_id) ──► host
                                                                             │
             host checks its gate: target locked + opted in, torso aim,      │
             head clearance, range, aim settled. All OK? ◄───────────────────┘
                 │ yes                                    │ no: send nothing; the press expires after 400 ms
                 ▼
 host ──► FIRE(shot_id, press_id, rpm, tol, range_cm)
                 │
 ESP32: checks (FIRMWARE_SAFETY.md R5-R10) ──► spin/settle ──► feed one ──► watch the RPM dip
                 │
 ESP32 ──► SHOT(shot_id, result, rpm_before, rpm_min, dip_ms)
```

## Boot and reboot

- At boot the firmware holds the ESC at zero throttle, sends INFO once, and enters DISARMED. If the arm switch is already on, it enters FAULT `ARM_ON_AT_BOOT` instead.
- The host spots a reboot when INFO arrives with a new `boot_id`, or when TELEMETRY `t_ms` goes backwards. It then sends HELLO again and re-reads the limits.

## Versioning

`PROTO_VERSION` is 1. Any change to an existing payload layout bumps it. The host refuses to run (it shows MISMATCH) when the firmware's version differs. New message types can be added without a bump, because receivers drop unknown types.

## Test vectors

`seq` is the second body byte. "wire" is what goes over USB.

```
CRC-16/CCITT-FALSE check: crc16(b'123456789') = 0x29b1

HELLO  proto 1, host_boot_id 0x1234, seq 0
  body: 01 00 01 34 12 4f 77
  wire: 02 01 06 01 34 12 4f 77 00

SETPOINT pan +12.34 tilt -5.00 rpm 0, not armed, seq 7
  body: 02 07 d2 04 0c fe 00 00 00 1d 88
  wire: 07 02 07 d2 04 0c fe 01 01 03 1d 88 00

SETPOINT pan 0 tilt +10.00 rpm 12000, HOST_ARM, seq 8
  body: 02 08 00 00 e8 03 e0 2e 01 cb f7
  wire: 03 02 08 01 08 e8 03 e0 2e 01 cb f7 00

FIRE shot 1, press 3, 12000 ±300 rpm, 2.50 m, seq 9
  body: 03 09 01 00 03 00 e0 2e 2c 01 fa 00 f3 a3
  wire: 04 03 09 01 02 03 06 e0 2e 2c 01 fa 03 f3 a3 00

TELEMETRY t 123456, ack 8, ARMED, no fault, switches 0x83, pan -2.50, tilt +9.90,
          rpm 11950/12000, shots 5, press 3, 14.80 V, seq 42
  body: 82 2a 40 e2 01 00 08 02 00 83 06 ff de 03 ae 2e e0 2e 05 00 03 00 c8 05 ce 30
  wire: 06 82 2a 40 e2 01 03 08 02 0b 83 06 ff de 03 ae 2e e0 2e 05 02 03 05 c8 05 ce 30 00

SHOT shot 1 FIRED, 12010 → 11800 rpm, dip at 64 ms, seq 43
  body: 84 2b 01 00 00 ea 2e 18 2e 40 00 01 e1
  wire: 04 84 2b 01 01 06 ea 2e 18 2e 40 03 01 e1 00
```

## Testing the firmware without the camera

```bash
python -m candyturret step3 --port COM5
```

This opens a panel that sends SETPOINTs from the keyboard and shows everything the ESP32 reports: link state, latency, bad-frame count, state, fault, switches and RPM. Press `x` to freeze the host. The firmware should report `LINK_TIMEOUT` within 250 ms: 200 ms of silence, plus one telemetry period, plus loop jitter.
