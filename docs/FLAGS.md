# Hardware and safety flags

Ranked by how soon each one has to be dealt with.
- **STOP**: before anything fires.
- **CHAIR**: before the turret goes on the wheelchair.
- **BUILD**: affects parts you haven't bought or printed yet.
- **NOTE**: worth knowing.

Research-backed items were checked on 2026-10-26 (see DESIGN.md).

## STOP

1. **The speed cap starts at 20 m/s (Chris's call, 2026-10-27), which is above toy-safety levels.**
   - An M&M is a hard projectile; a Nerf dart is foam.
   - The toy safety standards (ASTM F963, EN 71-1, ISO 8124-1) only allow a projectile above 0.08 J if it has a soft, resilient tip. For a 0.86-0.9 g rigid candy, 0.08 J works out to **13.3-13.6 m/s**.
   - At 20 m/s it carries about 0.15-0.17 J, roughly 2× that limit.
   - Eye-injury research puts a 50% chance of a scratched cornea at about 1,500 J/m². An edge-on M&M reaches that at **~15.6 m/s**. At 20 m/s it's about 1.6× that level, so an eye hit without glasses is likely to injure.
   - Chris chose 20 knowing these numbers, on the condition that everyone wears Z87+ glasses (#2). Verify the real speed on the chronograph before anyone stands downrange.
2. **Everyone in range wears Z87+ safety glasses. At 20 m/s this is a requirement, not a nice-to-have.** Torso-only aim can't be perfect:
   - people duck during the 0.08-0.22 s flight;
   - the detector misses keypoints;
   - candies ricochet off walls.

   Z87+ glasses are tested with a steel ball at about 6× an M&M's energy, so cheap ones are plenty. Even Nerf Rival, with soft balls, tells players and bystanders to wear eyewear.
3. **Don't let software be the only speed limit.**
   - Choose the motor KV, battery and wheel so full throttle can't pass the cap (table in DESIGN.md → Launcher).
   - Add an independent RPM sensor. If the sensor dies, a normal PID controller winds up to full throttle.
   - The firmware must treat "no RPM reading" as a fault (FIRMWARE_SAFETY R1).
   - For scale: a typical ~1950 KV drone motor on 4S with a 50 mm wheel could physically throw a candy at **40+ m/s**.
4. **The kill switch has to actually cut the ESC and be one Chris can work himself.**
   - Most cheap mushroom e-stops are rated around 10 A AC. A brushless ESC can pull 30 A+ DC at spin-up and arc a small switch.
   - Have the e-stop drive a DC contactor or an anti-spark switch module rated for the ESC current, or use an XT60 loop key.
   - Fit two: one Chris can reach with how he controls things, plus a big one for friends.
5. **The barrel must never be able to point at Chris.**
   - Put mechanical hard stops on the pan/tilt mount, plus the firmware fire window (R7).
   - Check the pan range against where his head actually is in the chair, including when the seat is tilted.

## CHAIR

6. **Battery fire risk.**
   - Someone who can't get away quickly shouldn't sit on a bare hobby LiPo.
   - Use LiFePO4 (safest chemistry), or a Li-ion pack with a built-in BMS, ideally UL 2271 listed.
   - Put a fuse at the pack's positive terminal, sized and interrupt-rated for lithium.
   - Charge it **off the chair**, in the daytime, while someone is around, and not on an escape route. That's UK fire service and FDNY advice for people with limited mobility.
7. **Balance.**
   - The turret, battery and mount count toward the chair's maximum user weight (Pride and Permobil both say so).
   - Mount it low and near the centerline: not high on the backrest, not on a swing-out arm. Keep the whole thing under ~3 kg (TEMP).
   - Tilt, recline and seat elevate all reduce stability. Chairs are rated for only 6-7.5° slopes.
   - Ask your chair supplier before clamping anything on; modifications can affect the warranty.
8. **Wheelchair electronics and EMI.**
   - FDA documented powered wheelchairs moving on their own from radio interference, with wheel movement seen at about 10 V/m. Chairs are tested to 20 V/m, but only as they left the factory.
   - Turn the ESP32's Wi-Fi and Bluetooth off; the USB link is enough.
   - Keep motor and ESC wires short, twisted and at least 20-30 cm from the joystick, the controller and their cables. Add ferrites or a metal enclosure.
   - **Before riding:** lift the drive wheels off the ground, power the chair on, then run the flywheel at full speed and fire. Watch for any wheel movement or brake release.
   - Never power anything from the chair's batteries or its charger port. Permobil says that port can damage the controller or affect EMC.
9. **Reach, sight and transfers.**
   - The mount must not block the drive control (joystick or mouth or head control) or its swing-away bracket.
   - It must not block Chris's view.
   - It must not get in the way of transfers.
   - It must not hit anything when the seat tilts or reclines.
   - Put it on the side away from the drive control.
10. **Take it off before riding in a vehicle.**
    - In a crash test (30 mph, 20 g), a 3 kg turret pulls about 590 N on its mount.
    - Accessory mounts (Rehadapt, Daessy, Mount'n Mover) are rated 3-6.8 kg *static*, not for crashes, slewing or vibration.
    - Use a tool-free quick release, and stow the turret separately for transport.

## BUILD

11. **D435 cable.** It needs real USB 3: over USB 2, depth drops to 10 fps. Use a **screw-locking USB-C cable** (the datasheet recommends one; Newnex makes them) with strain relief, because chair vibration unseats plain USB-C plugs. The host prints a warning at startup if the camera comes up on USB 2.
12. **Host computer.** Get a Jetson Orin Nano Super. A Pi 5 needs a 5 V / 5 A supply or a powered hub for the D435, a pyrealsense2 source build, and still runs pose at only 5-12 fps.
13. **The camera sees 69° × 42° in color.**
    - Panning past about ±35° is pointless unless the camera pans too.
    - At 1.5 m a tall adult's head can be out of the top of the frame. The host then holds fire with "head not visible", which is correct but annoying.
    - Tilt the camera up a few degrees (TEMP pitch).
14. **Feeder geometry.**
    - It must be closed by its shape when unpowered, not by holding torque (R8).
    - An 8-10 mm feed channel forces the candies flat.
    - Add an agitator against bridging.
    - Chocolate smears when warm and sugar dust coats the wheel. Keep the hopper shaded and clean the wheel. Slip changes over time, so recalibrate.
15. **Candy shells crack** under too much wheel squeeze, and the fragments are hard. Use a soft tire with very little compression, and test it.
16. **Get the IMU** (BNO085 or similar) for chair turning, slopes and seat tilt (DESIGN.md §5).
17. **The RPM dip per shot is small, about 2-3%.** The RPM sensor needs good resolution (hall sensor on PCNT, or eRPM).

## NOTE

18. **Track ID swaps.** People crossing can swap tracker IDs, and a lost ID can be handed to whoever walks into the same spot. The host drops opt-in when a track jumps or goes unseen for 0.25 s, but two people standing close together can still swap. **Chris's press is the last check: look before you press.**
19. **License.** Ultralytics (YOLO) is AGPL-3.0. A public repo that uses it should be AGPL-3.0 as a whole. The other route is swapping to an Apache-2.0 pose model (RTMPose/MMPose, MediaPipe). Your call.
20. **An M&M is a choking hazard** if it lands in an open mouth, which is another reason for the head exclusion. Take extra care with kids. Their heads sit closer to their chest in pixels, and the 30 cm clearance doesn't scale down.
21. **Camera temperature and sun.** The D435 is validated for 0-35 °C. Outdoors, shade it and add lens baffles against glare. Stereo depth itself works fine in sunlight.
22. **The ESP32 can reset when the port opens** (DTR/RTS). The host already opens with both off, and the firmware shouldn't depend on them.
23. **Assistive-switch jacks need plug detection.** Sliding a mono plug in shorts the tip to ground for a moment, and a normally-closed switch reads "pressed" when its cable is pulled. Either could make a fire press or an arm out of nothing. Use 3.5 mm jacks with a switched plug-detect contact (FIRMWARE_SAFETY.md, Inputs).
