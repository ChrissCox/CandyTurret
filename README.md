# CandyTurret

An M&M launcher turret that mounts on a power wheelchair. A RealSense D435 finds people who have opted in, the turret aims at their torso, and Chris presses a switch to fire one candy at a time.

**Before anything fires, read [docs/FLAGS.md](docs/FLAGS.md):**
- the speed cap starts at 13 m/s;
- everyone in range wears Z87+ glasses;
- the kill switch cuts ESC power in hardware;
- the barrel can't point at the rider.

## How it fits together

- **Host computer** (Python, this repo's `host/`): reads color and depth from the D435, runs YOLO pose and tracking, picks the torso aim point, measures range from the torso only, and talks to the ESP32 over USB serial.
- **ESP32-S3** (firmware by the other half of the team): drives pan/tilt, the flywheel and the feeder, reads the switches, and enforces the safety rules on its own.

| Doc | What's in it |
|---|---|
| [docs/DESIGN.md](docs/DESIGN.md) | architecture, pipeline, math, launcher, power, controls |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | the serial contract: framing, messages, heartbeat, test vectors |
| [docs/FIRMWARE_SAFETY.md](docs/FIRMWARE_SAFETY.md) | rules the ESP32 enforces regardless of the host, plus a bench checklist |
| [docs/FLAGS.md](docs/FLAGS.md) | hardware and safety problems, ranked |
| [TEMP.md](TEMP.md) | every made-up placeholder and how to replace it |

## Setup (host)

```bash
cd host
python -m pip install -r requirements.txt
python -m pytest
```

The first run downloads the small pose model (`yolo11n-pose.pt`, 6 MB) into `host/models/`.

## Steps

Run everything from `host/`. Each step ends with something you can watch working.

### Step 1: people and distances

```bash
python -m candyturret step1
```

**You should see:** a green box around each person, with their distance in meters. The cyan outline on the torso is where the distance is measured. A number with `~` in front means the torso wasn't visible, so it used the middle of the box. Keys: `d` depth view, `s` screenshot, `q` quit.

No camera handy? Run `python -m candyturret step1 --source some_video.mp4 --fake-depth 3`. The distances are fake, but the detection is real.

### Step 2: torso aim point and tracking (nothing moves)

```bash
python -m candyturret step2
```

**You should see:**
- Gray boxes labelled "hands up to join".
- Raise both hands above your head for a second: the bar fills and your box turns green ("IN").
- The turret locks onto one opted-in person (yellow box). It shows a crosshair on the chest, a red circle for the no-head zone, and a yellow arrow showing where they're heading.
- The banner at the bottom says **WOULD FIRE** or **HOLD:** with the reasons: too close, head too close, no torso, and so on.

Keys: `n` next target, `c` re-pick the nearest.

### Step 3: serial link and manual pan/tilt (no firing)

```bash
python -m candyturret step3 --port sim
```

`--port sim` uses a pretend ESP32. Use `--port COM5` (or whatever Device Manager shows) once the real one is flashed.

**You should see:**
- LINK OK, with the latency.
- Dials for pan and tilt. `a`/`d` and `w`/`s` move them one degree at a time; capital letters move five.
- State, fault, switches and RPM from the ESP32.
- Press `x` to freeze the host: the ESP32 should show `LINK_TIMEOUT` within 250 ms.

The flywheel is held at 0 and the host never arms in this step.

To run the pretend ESP32 in its own window (for example, to flip its arm switch or kill switch by typing commands):

```bash
python -m candyturret sim --tcp 7777
```

Then point step 3 at it in another terminal:

```bash
python -m candyturret step3 --port socket://localhost:7777
```

### Steps 4-8 (not built yet)

4. turret follows a person (no firing) + camera offset calibration
5. flywheel RPM control + chronograph → lock in the cap
6. feeder: one candy per command, dip confirmation, jam detection
7. calibration mode → range table
8. full auto-aim with arming and press-to-fire

### Other commands

| Command | What it does |
|---|---|
| `python -m candyturret ballistics` | tilt, flight time and RPM per range from the physics model, plus a check that the motor can't physically exceed the cap |
| `python -m candyturret vectors` | protocol test frames for checking the firmware |
| `python -m candyturret temp` | every TEMP placeholder in the config |

## Recording for later

`step1`/`step2` take `--record session.bag` to save a D435 session and `--bag session.bag` to replay it. Develop against the recording without the camera plugged in.
