# TEMP values: made up for now, to replace later

Everything that was still blank (`[__]`) when we started has a placeholder, and each placeholder is marked `TEMP`. Config ones are tagged `# TEMP` in `config/turret.toml`. To list them:

```bash
python -m candyturret temp
```

Run it from `host/`. When you replace a value, delete its `TEMP` tag, and strike it off here.

## Hardware you haven't picked yet

| What | Placeholder | Decide or measure by |
|---|---|---|
| Pan/tilt drive | hobby servos for both (`turret.pan_drive`, `tilt_drive`) | before step 3 on hardware. Servos are simpler; steppers need homing |
| Pan range | −40 … +40° | match it to the 69° camera view; measure the hard stops |
| Tilt range | −10 … +30° | measure the hard stops |
| Launcher layout | one wheel + backstop, candy fed flat | DESIGN.md → Launcher |
| Wheel diameter | 40 mm | what you print or buy |
| Flywheel motor | ~650 KV, 2212/2216 class (`launcher.motor_kv`) | pick from the KV table in DESIGN.md; measure the real KV (no-load RPM ÷ volts) |
| ESC | AM32 or Bluejay 35 A, bidirectional DShot | |
| Feeder | servo indexer + home switch | |
| ESP32-S3 board | unknown | note whether it's native USB or a UART bridge (affects PROTOCOL.md transport) |
| Host computer | Windows PC now, Jetson Orin Nano Super for the chair (`host.computer`) | before mounting |
| Battery | 4S Li-ion 21700 with BMS, 14.4 V, 4 Ah; 16.8 V full (`power.battery`, `battery_full_v`) | FLAGS #6; the power budget in DESIGN.md says ~80 Wh for 2 h |
| Fire input | assistive switch in a 3.5 mm jack on the ESP32 (`controls.fire_input`) | Chris picks the switch; does your QuadStick have the "Out" relay jack? |
| Arm input | toggle switch on the turret box (`controls.arm_input`) | must be reachable by Chris |
| IMU | BNO085 (recommended, not yet in config) | DESIGN.md §5 |
| Languages | host = Python (needed for librealsense + YOLO); firmware = your friends' choice | |

## Measurements

| What | Placeholder | How to measure | Step |
|---|---|---|---|
| Camera offset from pan pivot (`camera.offset_*_mm`) | 60 fwd, 80 up, 0 left | tape measure from the pan axis to the D435's left IR lens, then refine with the wall-marker calibration | 4 |
| Camera angles (`camera.pitch/yaw/roll_deg`) | 0, 0, 0 | phone level app on the camera body, then the calibration | 4 |
| Tilt axis height above pan pivot (`turret.tilt_axis_up_mm`) | 40 mm | ruler | 4 |
| Muzzle distance in front of tilt axis (`turret.muzzle_forward_mm`) | 100 mm | ruler | 4 |
| D435 max trusted range (`camera.max_depth_m`) | 6.0 m | stand at taped 2/4/6/8 m marks and compare step 1's number with the tape | 1 |
| M&M mass (`projectile.mass_g`) | 0.86 g (published average) | weigh 20, divide by 20 | 5 |
| M&M size (`projectile.diameter_mm`, `thickness_mm`) | 13.4 × 6.9 mm (published) | calipers on 10 | 5 |
| Slip (`launcher.slip`) | 0.8 | chronograph speed ÷ ideal speed from RPM | 5 |
| Speed cap (`launcher.velocity_cap_mps`) | **13 m/s** (you said ~20; see FLAGS #1) | your decision, then verified on the chronograph | 5 |
| Drag (`projectile.drag_coefficient`) | 0.7 | replaced by the calibration table | 7 |
| Aim point height on the torso (`aim.aim_fraction`) | 0.55 | look at step 2 on a few body types | 2 |
| Head clearance (`aim.head_clearance_m`) | 30 cm | set to measured shot spread at max range + 10 cm head radius | 7 |
| Minimum firing range (`aim.min_fire_range_m`, firmware `MIN_RANGE_CM`) | 1.5 m | your starting value | — |
| Maximum firing range (`aim.max_fire_range_m`) | 4.0 m | where the calibration spread is still under ~15 cm | 7 |
| Serial port (`link.port`) | `sim` | Device Manager → Ports once the ESP32 is plugged in (e.g. COM5) | 3 |

## Firmware constants (in your friends' code, listed in FIRMWARE_SAFETY.md)

| What | Placeholder | How | Step |
|---|---|---|---|
| `RPM_CAP` | ~12,400 (the cap at zero slip, from `python -m candyturret ballistics`) | RPM that gives the speed cap on a new, clean wheel and a full battery | 5 |
| `THROTTLE_MAX` | — | throttle that reaches `RPM_CAP` (not more) on a full battery | 5 |
| Throttle → RPM table | — | for the sensor plausibility check (R1) | 5 |
| `DIP_FRACTION` | 1.5% | RPM trace at 1 kHz during 20 test shots | 6 |
| `FEED_TIMEOUT_MS` | 300 ms | time one index takes, ×2 | 6 |
| `MIN_SHOT_INTERVAL_MS` | 400 ms | | 6 |
| Motion limits / fire window | ±40 / ±35° pan, −10…30 / −5…25° tilt | just inside the hard stops; fire window never toward Chris | 3 |

## Timing (for leading moving targets, step 8)

| What | Placeholder | How |
|---|---|---|
| Camera-to-decision latency | ~60 ms | film the screen and a stopwatch, or use frame timestamps |
| Feed delay (FIRE → candy reaches the wheel) | 60-150 ms | `SHOT.dip_ms` in step 6 |
| Yaw-rate fire hold | 10°/s | needs the IMU |
| Lead limits | 3 m/s, 0.6 m | |
