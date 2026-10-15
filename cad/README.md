# CAD Spec (locked from the sim)

The MuJoCo model is the source of truth for geometry. This folder holds what CAD must match, generated from it, and
the real dimensions of every purchased part. Regenerate after any model change:

```bash
python3 tools/export_cad_spec.py        # hardpoints.json / hardpoints.md / purchased_parts.md
python3 tools/check_cad_clearances.py   # clearance_report.md (exit 1 on a real-part layout clash)
python3 tools/export_design_loads.py    # design_loads.json / design_loads.md (~30 s)
tools/fetch_vendor_cad.sh               # vendor drawings + STEP into cad/vendor/ (git-ignored)
```

| File | What it is |
| :--- | :--- |
| [`hardpoints.md`](hardpoints.md) / `.json` | Every joint anchor, axis and range, body frame, part placement, camera, site and light, in mm at the `home_balanced` pose, plus key dimensions and the leg poses the structure must clear. Build the CAD skeleton from this. |
| [`purchased_parts.json`](purchased_parts.json) → [`.md`](purchased_parts.md) | Real envelopes, hole patterns, masses and load limits of purchased parts, each with its source and a confidence level. Edit the JSON. |
| [`clearance_report.md`](clearance_report.md) | Interference over all operating poses and full joint ranges, packing inside the trunk and head, ground clearance, scrub torque, model-vs-real envelope check. |
| [`design_loads.md`](design_loads.md) / `.json` | Force and moment through every joint (axial, radial, about and across the axis) for standing, driving, crouching, jumps, ±40 N pushes and a fall. Size brackets and bolts from these. |

Frame everywhere: floor at z = 0, x forward, y left, z up, millimetres.

## Key numbers (gim6010 build, 5.48 kg)

| | |
| :--- | :--- |
| Wheel Ø / tire width / track | 150 / 32 / 224 mm |
| Thigh / shin (axis to axis, in the leg plane) | 118 / 118 mm |
| Hip-yaw axis spacing | 168 mm |
| Wheel offset inboard of the knee (wheel under hip roll) | 48 mm; scrub offset 4 mm → 0.11 N·m standing roll torque |
| Height at home pose | 650 mm incl. antennas |
| Hip-pitch axis above the wheel axle | 236 (jump take-off) / 207 (home) / 174 mm (deepest crouch), wheel always directly below |
| Eye camera baseline | 70 mm |
| Ground clearance (lowest non-wheel part, any operating pose) | 28.7 mm (wheel motor housing, deep crouch + hip roll) |

## What the lock-down found and changed

- **GIM4305-10 is 33 mm long, not 26.** Listings quote the housing only. The SteadyWin drawing and STEP add a 6 mm
  driver boss and a 1 mm output step. The model's motor envelopes are now Ø53 × 33.
- **The 6S2P pack is 73 × 89 × 68 mm, not 70 × 84 × 63.** 12 Molicel P45B cells (Ø21.55 × 70.15 max) lying 4 × 3,
  plus strip and wrap. It now sits 2 mm below the trunk shell stand-in and 2 mm under the Pi: the trunk needs about
  5 mm more interior height, or the Pi has to move up.
- **The Teensy carrier and IMU were outside the trunk** (x = -95 / -108 mm, the scaled Open Duck Mini IMU spot). Both
  moved inside, onto the upper rear wall. The controller only uses IMU orientation and gyro, which don't depend on
  where the IMU sits on the trunk.
- **The anti-spark switch overlapped the full-size battery** by 0.5 mm; moved 3 mm forward.
- **The eye cameras' field of view was the wide lens's** (67° vertical). The standard Camera Module 3 is 41°.

## Open issues for CAD (details in the reports)

1. **Hip-yaw output bearing, axial: 117 N peak on jump landings vs. 125 N limit (93%).** Carry the trunk's weight on
   the hip-yaw joint through a thrust bearing or bushing in the bracket, so it doesn't pass through the motor's
   output bearing. Confirm the limit with SteadyWin.
2. **Bending moments are large for these bearings**: ~6 N·m across the hip yaw and hip roll (GIM4305), ~9 N·m across
   the hip pitch and knee (GIM6010). Neither limit is published: ask SteadyWin, and support the far side of each
   joint (a bearing on the bracket opposite the motor) where you can.
3. **Wheel landing: 260 N contact, 245 N radial through the wheel motor (49% of 500 N)**, plus a torque spike from
   the spinning rotor being stopped at touchdown. The wheel hub and tire need to take it.
4. **Neck pitch reaches 5.4 N·m on landings**, above what the STS3215 can hold, so the neck hits its joint limit.
   Give it a real hard stop that isn't the servo.
5. **Printed parts must clear the head and neck motion**: head roll ±26°, neck pitch −17°…+52°, head pitch and yaw
   at their controller clips. The shell stand-ins clash by up to 13 mm (`clearance_report.md`, CAD constraints).
6. **The hip-yaw bracket has no room to turn.** Its stand-in already overlaps the trunk floor at the home pose, and
   the overlap grows from about 3° of yaw; on the right side it also reaches the main fuse. The controller holds hip
   yaw at 0, but CAD must give the rotating bracket clearance under the trunk floor for whatever yaw range you keep
   (±30° in the model), or fit hard stops. Keep the fuse out of the bracket's swept circle.
7. **Space still to reserve**: the e-stop needs ~45 mm behind the rear panel; the speaker depth depends on the part
   (the model assumes 6 mm, with 0.5 mm to the battery); two buck converters and the bus servo adapter aren't in
   the model yet.
8. **Measure on arrival**: GIM6010-8 radial protrusion (its STEP is Ø86.6 across, likely the connector), micro servo
   tab holes, the speaker, BMS, anti-spark switch and e-stop.
