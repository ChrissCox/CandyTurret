# CandyTurret design

Hardware problems are collected in [FLAGS.md](FLAGS.md), unknowns in [../TEMP.md](../TEMP.md), the serial contract in [PROTOCOL.md](PROTOCOL.md) and the firmware rules in [FIRMWARE_SAFETY.md](FIRMWARE_SAFETY.md). Numbers marked "(research)" were checked against datasheets, standards or papers on 2026-10-26.

## Architecture

Your split is right. The ESP32-S3 can't run the D435 (it needs USB 3 and librealsense) or a pose model, so the host does perception and aiming, and the ESP32 does real-time control and safety. Four changes to the outline:

1. **The fire input is wired to the ESP32, not the host.** That way the firmware can enforce "one press, one shot" by itself. The host can never fire without a real press, even if it crashes into a loop. It goes in as a 3.5 mm assistive-switch jack (see Controls).
2. **The camera is fixed to the chair, not to the turret.** Your camera offsets imply this, and it's the better choice: aiming doesn't shake the camera, and tracking doesn't have to undo the turret's own motion.
3. **Add an IMU to the ESP32.** Yes, you need one; see "Chair motion" below.
4. **The host keeps the flywheel pre-spun** while a target is locked (SETPOINT `fly_rpm`), so a press fires within about 0.1-0.2 s instead of waiting a second for spin-up.

```
 D435 ──USB 3 (locking cable)──► HOST (Python)                      ESP32-S3 firmware (your friends)
                                 color + depth aligned to color       ├─ pan/tilt drive, limit switches, hard stops
                                 YOLO pose + ByteTrack                ├─ flywheel ESC + independent RPM sensor
                                 torso aim, torso range, head check   ├─ feeder indexer + home switch
                                 opt-in, target lock, Kalman          ├─ arm switch, fire jack, kill-switch sense
                                 camera→turret transform, lead,       ├─ IMU (recommended)
                                 ballistics / calibration table       └─ safety rules R1-R14
                                    SETPOINT 50 Hz, FIRE ──USB──►
                                    ◄── TELEMETRY 50 Hz, TRIGGER, SHOT, IMU
 kill switch ── cuts flywheel ESC power directly (hardware, not firmware)
```

### Host computer

| Option | Pose model speed (research) | Catches |
|---|---|---|
| Your Windows PC (RTX 3060) | ~8 ms per frame here | for development only; too big for the chair |
| **Jetson Orin Nano Super (recommended)** | ~5 ms model time with TensorRT FP16, often ~20 fps end to end in Python | JetPack 6 runs Python 3.10, which matches a pyrealsense2 ARM wheel. 7-25 W from a DC input |
| Raspberry Pi 5, CPU only | ~5-12 fps for nano pose at 640 | no pyrealsense2 wheel for Pi OS's Python 3.11 (source build or a 3.12 venv); the Pi isn't an officially supported librealsense platform; USB ports are limited to 600 mA total unless the supply negotiates 5 V / 5 A, and the D435 draws up to 700 mA |
| Pi 5 + AI HAT+ (Hailo) | yolov8s-pose ~100 fps | Ultralytics tracking doesn't run on Hailo, so you'd need your own tracker and model compile |

Develop on the PC now. Everything in `host/` runs the same on a Jetson.

## Pipeline

### 1. Detection and tracking

- **Model:** `yolo11n-pose` (Ultralytics). It gives person boxes **and** 17 body keypoints in one pass, for about the same compute as plain detection. A separate detector plus pose model would cost double. On your 3060 it takes about 8 ms.
- **Tracking:** ByteTrack through `model.track(persist=True)`. The tracker is pinned in config, because newer Ultralytics versions changed the default.
- **ID swaps:** ByteTrack matches on motion and box overlap only, so IDs can swap when people cross. It can also hand a lost ID to someone new who walks into the same spot. The host drops a track's opt-in if the track goes unseen for more than 0.25 s, or if its 3D position jumps further than a person could move (0.8 m, plus 2.5 m/s × up to 0.25 s).
- **Low-confidence boxes:** tracking runs at confidence 0.1, because ByteTrack's second matching pass needs the weak boxes. Anyone under the configured 0.4 is still tracked, but they get a "low confidence" hold.
- **Speed fallbacks** if the host can't keep up: imgsz 320 (about 4× faster), TensorRT or NCNN export, or a Hailo HAT with its own tracker.

### 2. Aim point (torso only)

- **Aim point** = shoulder midpoint + 0.55 × (hip midpoint − shoulder midpoint). That's the chest/belly. `aim.aim_fraction` is TEMP.
- **All four torso keypoints** (shoulders 5/6, hips 11/12) must have confidence ≥ 0.5. Otherwise the person is still tracked, but the host holds fire ("no torso").
- **Head check:**
  - For each visible nose/eye/ear keypoint, take the sideways distance from the aim line, measured at the **nearer** of the head's own depth and the torso depth. A head leaning toward the camera in front of the chest then counts as closer.
  - Hold fire if any head keypoint is within 30 cm (TEMP). That covers about 10 cm of head radius plus shot spread.
  - With `require_head_visible` on, it also holds when no head keypoint is visible: head out of frame, or the person turned away.
  - **Other people's heads count too.** A bystander's face within 30 cm of the line of fire holds ("someone else's head in the way").
  - **Anything in front** holds too: more than 10% of the no-head circle 0.3 m nearer than the torso. That covers an undetected person, a raised hand or a door frame ("something in front").
  - Ultralytics parks keypoints under 0.5 confidence at pixel (0, 0). The host zeroes their confidence so they can't count as a wrist above the head or a phantom head.
- **Consent:** only people who have done the opt-in gesture can be locked. The gesture is both wrists above the **estimated top of the head**, held for 1 s while the person is watched continuously:
  - The top of the head is estimated as half the eye-to-shoulder distance above the face keypoints. Hands resting on the head don't count.
  - With no face visible, the line is 0.7 torso lengths above the shoulders, so a turned person tying their hair doesn't count either.
  - The opt-in is lost if the track goes unseen for more than 0.25 s, or the jump guard fires.

### 3. Range

Depth is the median of the nonzero depth pixels inside the torso quad, shrunk to its middle 60%. That keeps arms, box edges and the wall behind out of the sample.

**Range** is the straight-line distance to the aim point after deprojection, not the camera's z. Someone at the edge of the 69° view is up to about 20% farther than their depth reading. The range gates, the labels and `FIRE.range_cm` all use this distance. `tests/test_perception.py` includes a slim person whose whole-box median lands on the wall while the torso median still reads correctly.

D435 reliability (research):

| | Value |
|---|---|
| Intel's "ideal" range | 0.3-3 m |
| Minimum distance at 848×480 (the most accurate depth mode) | 0.195 m |
| Accuracy spec | ±2% of distance, ≤ 2% RMS noise, at ≤ 2 m |
| Noise per pixel, Intel's formula Z²·0.08/(f·B) with f ≈ 447 px, B = 50 mm | 1.4 cm at 2 m, 3.2 cm at 3 m, 5.7 cm at 4 m, 13 cm at 6 m |
| Color camera field of view | 69° × 42° (rolling shutter). Depth sees 87° × 58°, but detection runs on color |

The torso median averages thousands of pixels, so random noise matters much less than the ±2% bias. Settings:
- Readings outside **0.3-6 m** are rejected.
- Firing is allowed only between **1.5 and 4 m** (both TEMP).
- Depth error barely matters for the aim: a 5 cm range error at 4 m moves the predicted drop by about 1 cm.

### 4. Camera → turret transform

Frames: camera (x right, y down, z forward) and turret base at the pan pivot (x forward, y left, z up).

```
p_base = R_mount · C · p_cam + t
C = [[0,0,1], [-1,0,0], [0,-1,0]]        # camera axes → base axes for a level, forward-facing camera
R_mount = Rz(yaw) · Ry(-pitch) · Rx(roll) # camera mount angles (TEMP: all 0)
t = (forward, left, up) offsets            # TEMP: 60, 0, 80 mm
q = p_base - (0, 0, tilt_axis_up)          # relative to the tilt axis
pan  = atan2(q.y, q.x)
D = hypot(q.x, q.y),  H = q.z             # horizontal distance and height of the target
tilt: solve the ballistics so the shot leaves the muzzle at
      (L·cos θ, L·sin θ), L = muzzle_forward, and passes (D, H). Iterate 3×: θ moves the muzzle.
```

Because the target is a 3D point, the camera-to-barrel parallax comes out right at every range. Step 4 adds a calibration routine: aim by hand (step 3 keys) at a marker on a wall at 2 m and 4 m, record what the camera measured, and solve for the offsets and angles. That replaces the TEMP values with measured ones. **No laser pointer on the turret:** it would sweep across eyes.

### 5. Chair motion: yes, get the IMU

The D435 has no IMU, and the camera can't tell the chair turning from everyone else moving. Four concrete problems:

- **Turning in place.** A chair yawing at 30°/s makes a still person 3 m away look like they're moving at 1.6 m/s. The lead calculation would chase that fake motion.
- **Turning during the shot.** About 0.2 s passes between the decision and the candy leaving: latency plus feeding. At 30°/s the barrel swings 6° in that time, which is a 30 cm miss at 3 m. Pan commands are relative to the chair, so the turret can't correct for this without knowing the turn rate.
- **Slopes and seat tilt.** Ballistics needs to know which way is down. Chairs are rated for 6-7.5° slopes, and seat tilt and elevate move the whole turret. A 6° error in "level" is about 30 cm of vertical miss at 3 m. The accelerometer's gravity vector fixes it.
- **Rolling shutter.** The color camera smears fast pans. Firing only while the chair is steady avoids it.

**Get a BNO085** (about $20, does its own sensor fusion), or an ICM-42688 or MPU-6050 if you're happy to filter it yourselves. Mount it on the turret base and send IMU at 50-100 Hz. The host uses it to:
1. hold fire while |yaw rate| > 10°/s (TEMP);
2. remove the chair's rotation from track velocities, so the Kalman filter runs in a level, non-rotating frame;
3. use measured gravity as "down" in the ballistics.

Driving straight needs no IMU. Tracking measures velocity relative to the chair, which is what the lead needs. The headwind from a 1 m/s chair is too small to matter at 15-20 m/s.

### 6. Leading moving targets

- Each track runs a constant-velocity Kalman filter on the 3D aim point. It's already running in step 2: the yellow arrow shows where the person will be in 0.5 s.
- Time to impact: `T = camera latency (~60 ms, TEMP) + feed delay (FIRE → candy reaches the wheel, measured in step 6 from SHOT.dip_ms) + flight time (from ballistics, which depends on range)`.
- Predict `p(T)`, recompute `T` at the new range, and repeat 2-3 times.
- **Hold fire** if speed > 3 m/s or the lead is over 0.6 m (both TEMP). People change direction faster than a candy flies 4 m.
- The head check is applied to the **predicted** aim point too.

### 7. Ballistics and calibration

The model in `host/candyturret/ballistics.py` is a point mass with quadratic drag:
- M&M 0.86 g, 13.4 × 6.9 mm (research: a published measurement study).
- Drag coefficient about 0.7 on the edge-on area, so CdA ≈ 51 mm², assuming it flies flat with spin.
- No lift.

Run `python -m candyturret ballistics`:

| Exit speed | Range | Tilt above level | Flight time | Impact speed |
|---|---|---|---|---|
| 20 m/s | 1.5 m | 1.1° | 0.08 s | 19.0 m/s |
| 20 m/s | 3 m | 2.3° | 0.16 s | 18.0 m/s |
| 20 m/s | 4 m | 3.1° | 0.22 s | 17.4 m/s |
| 13 m/s | 3 m | 5.4° | 0.25 s | 11.7 m/s |
| 13 m/s | 4 m | 7.4° | 0.33 s | 11.3 m/s |

Drag takes only 10-15% of the speed at these ranges. **For safety, impact speed is effectively the exit speed.**

**Step 7 calibration mode:**
1. Tape a paper grid target at measured distances: 1.5, 2, 2.5, 3 and 4 m.
2. At each distance, fire 5 shots at a fixed RPM with the model's tilt.
3. Mark the hits by clicking them on a photo the camera takes of the board. The host stores the mean vertical and horizontal error and the spread.
4. Adjust the tilt by error ÷ range and re-shoot until the mean is inside 3 cm.
5. Save `range → (tilt, rpm, spread)` to `config/calibration.toml`.

At runtime the host interpolates that table and ignores the model. The spread per range is the measured shot dispersion, which is what `head_clearance_m` should really be based on. Recalibrate when the wheel wears or gets sugary, because slip changes.

### 8. Fire

The full sequence is in [PROTOCOL.md](PROTOCOL.md#fire-sequence).
1. A press arrives as TRIGGER.
2. The host checks its gate: locked target, opted in, all torso keypoints, head clear, range 1.5-4 m, aim settled, chair steady.
3. It sends FIRE with that `press_id`.
4. The ESP32 runs its own checks, settles the RPM, feeds exactly one candy and confirms the shot by the RPM dip.
5. It reports SHOT.

A single NO_DIP latches a FEED fault (empty hopper or stuck candy), because the next feed could push two candies through at once. The firmware never re-feeds on its own.

## Launcher

- **Layout: one wheel against a backstop.** Feed each M&M **flat**, with the wheel's axle **parallel to the candy's short axis**. The candy rolls along the backstop and leaves spinning about its own axis like a tiny frisbee, at about 19,000 rpm at 13 m/s and about 28,000 rpm at 20 m/s (research). That spin holds it flat and stable. Two matched counter-spinning wheels give no spin, so a flat candy tumbles and spreads its shots.
- **The cost of one wheel:** a single wheel with a backstop sends the candy out at **half the rim speed**, so it needs twice the RPM (code: `ballistics.contact_factor`). Real launchers also lose another 20-30% to slip. Measure that with a chronograph; never assume it.
- **Feed channel:** about 8-10 mm tall, between the candy's thickness and its width. That forces every M&M to lie flat. Add a slow agitator in the funnel, because round candies arch and jam over a narrow outlet.
- **Wheel:** soft urethane or foam tire, very little squeeze. Bench-test for cracked shells, since fragments are hard and can be sharp. Fully guard the wheel.
- **Speed drop per shot:** about 2-3% of RPM (research). The RPM sensor needs fine resolution, so log a 1 kHz trace in step 6 to set `DIP_FRACTION`.
- **Make the speed cap physical.** Pick the motor KV, battery and wheel so 100% throttle on a full battery can't pass the cap even with zero slip. The table below includes a 10% margin, because a "700 KV" motor can really be 740.

  For a 20 m/s cap with one wheel:

  | Wheel | Max no-load RPM | Max KV on 3S (12.6 V) | Max KV on 4S (16.8 V) |
  |---|---|---|---|
  | 30 mm | 23,150 | 1,840 | 1,380 |
  | 40 mm | 17,360 | 1,380 | 1,030 |
  | 50 mm | 13,890 | 1,100 | 825 |

  - For two wheels, halve the KV numbers.
  - The TEMP config uses a 40 mm wheel with a ~1000 KV motor on 4S. That tops out at 17.6 m/s with zero slip, so about 14 m/s in practice, because slip costs 20-30%.
  - To actually reach 20 m/s you'd need a faster motor. Then the hardware could pass the cap at zero slip, and only the firmware would hold it. Decide that after the step 5 chronograph test, not before.
  - **Measure the real KV**: no-load RPM ÷ volts.
  - `python -m candyturret ballistics` prints this check, plus the cap-safe `RPM_CAP` (the cap at zero slip) to use until the chronograph result exists.
  - The firmware RPM cap then trims to the chronograph result as a second layer.
- **ESC:** AM32 or Bluejay firmware with bidirectional DShot, so it reports eRPM. BLHeli_32 is discontinued. Bidirectional DShot on the S3 is doable through the RMT peripheral, but the libraries are rough. The independent hall sensor is the reading the firmware trusts; eRPM is the cross-check.

## Power

- **Battery:** its own pack, never the chair's batteries or its charger socket. LiFePO4, or Li-ion with a built-in BMS; no bare hobby LiPo (FLAGS #5). Put a fuse right at the pack's positive terminal.
- **Flywheel kill:** the kill switch interrupts the ESC supply in hardware.
- **Rough budget:**

  | Load | Power |
  |---|---|
  | Jetson | 7-25 W |
  | D435 | ~3.4 W max |
  | ESP32 | ~1 W |
  | Servos | 2-10 W average, 2-3 A peaks each |
  | Flywheel spinning | 20-50 W |
  | Flywheel spin-up | 100 W+ for a moment |

  Call it about 40 W average, so a 2-hour session needs about 80 Wh. A 4S 4 Ah pack (58 Wh) is short; 4S2P 21700 (~115 Wh) fits (TEMP).
- **Converters:** a separate 5-6 V buck for the servos so their current spikes don't brown out the logic, and a 9-19 V input for the Jetson. Tie all the turret grounds together. Nothing connects to the chair's electrics.

## Controls

- **Fire:** a 3.5 mm mono jack on the ESP32. Contact closure to ground, internal pull-up, polarity selectable. Any assistive switch plugs in: lip, sip-puff, chin or button. A **QuadStick** can drive it directly if it has the optional "Out" relay jack: bind the relay to a sip or puff and wire it to the fire jack. Without that option, the QuadStick reaches the host as USB HID, which can't satisfy the firmware's own press rule. In that case use a separate switch for fire, and the QuadStick for "next target" through the host.
- **Arm:** a toggle on the turret box that Chris can reach, plus the host's arm flag.
- **Kill:** one switch Chris can operate himself, plus a big mushroom switch friends can slap.
- **Next target / re-pick:** host keys `n` / `c` for now (QuadStick HID later).

## Build order (each step ends with something to run)

| Step | What runs | Status |
|---|---|---|
| 1 | D435 stream, person boxes + torso distances | **built**, tested on photos with fake depth. Needs a D435 run |
| 2 | Torso aim point, head check, opt-in, target lock, velocity arrow | **built**, same |
| 3 | Serial link + manual pan/tilt panel, against the sim or the real ESP32 | **built**, tested against the sim |
| 4 | Turret follows the locked person (no firing) + offset calibration | next |
| 5 | Flywheel RPM control + chronograph test → `RPM_CAP` | needs the launcher + firmware |
| 6 | Feeder: one candy per command, dip confirm, jam detect | needs the feeder |
| 7 | Calibration mode → range table | |
| 8 | Full auto-aim, arming, press-to-fire | |
