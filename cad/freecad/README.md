# FreeCAD Assembly (first pass)

`build_super_duck.py` builds the whole robot in FreeCAD from `../hardpoints.json` at the `home_balanced` pose:
the printed structure, every purchased part as a model (vendor STEP for the GIM motors and the Pi 5, parametric
models for everything else, in `duck_parts.py`), and an interference check.

```bash
tools/fetch_vendor_cad.sh                      # once: vendor STEP + ODM STLs into cad/vendor/ (else stand-ins are used)
freecadcmd cad/freecad/build_super_duck.py     # headless
```

Or live in the FreeCAD GUI (Python console, or the FreeCAD MCP `execute_code` tool):

```python
p = "<repo>/cad/freecad/build_super_duck.py"; exec(open(p).read(), {"__file__": p, "__name__": "__main__"})
```

ODM meshes are converted to solids once and cached in `cad/vendor/odm/cache/`.

Outputs: `super_duck.FCStd` (git-ignored: it embeds the vendor STEP), `printed/*.step` (git-ignored, ~46 MB; each printed part in its
body frame), and [`fit_report.md`](fit_report.md) (interferences at home + printed-part volumes). Re-run after any
model change, the same as the other `cad/` generators.

## Document structure

One `App::Part` per MJCF body, nested like the MJCF tree (`SuperDuck > Trunk > L_HipMount > L_HipRoll > L_Thigh >
L_Shin > L_Wheel`, `Trunk > NeckPitch > HeadPitch > HeadYaw > Head > L/R_Antenna`). Each part's placement holds
the joint angle, so you can pose a leg or the head by editing the rotation angle of that `App::Part`.
`Sim_Standins` (hidden) holds every MJCF geom, for comparison with the sim's shapes.

## Layout decisions in this pass (differ from the sim; update the MJCF if you keep them)

| Change | Why |
| :--- | :--- |
| **Charge port**: GX16-2 aviation socket on the rear wall (y −44, z 20), mirroring the e-stop | Keyed, latching, and can't short like a barrel jack. Wire it to the BMS charge port (C−/P−) and pack +, before the anti-spark switch, so it charges with the robot off. Use a 25.2 V CC/CV charger at ≤ 5 A |
| E-stop at y +44, z 30, with its 45 mm contact block clear of the Pi (2 mm) | Open issue 7 |
| Teensy carrier (42 × 30) + ICM-42688-P on a shelf on the upper rear wall (z 34–37) | The sim boxes were outside / overlapping |
| Amp on the left side wall; fuse, anti-spark switch and peripheral buck moved to the floor | Clashes with the battery and the hip-mount bolts |
| 5.1 V Pi buck and the Waveshare bus servo adapter on the inside of the front wall; 7.4 V servo buck on the floor ahead of the pack | Open issue 7 (these weren't in the model) |
| Chest USB camera in the printed breast cover (lens x 95) | |
| **Neck and head are the Open Duck Mini v2's, 1:1** (same STS3215 servos): its own STLs and placements from its `robot.xml` (`odm.py`), hung from our neck-pitch joint. Joints in the trunk frame: neck pitch (36, 0, 60) as in the sim, head pitch (36, 0, 126), head yaw (36, 0, 183), head roll (77, 0, 163.9); the sim has 117.3 / 139.3 / (54.7, 177.3) | The sim's neck spacing (22 mm pitch→yaw) can't fit STS3215s. **Update the MJCF**: neck body positions, head mass / inertia, the ODM's tilted antenna axes, camera positions |
| Parts used unchanged: `neck_left/right_sheet`, `head_pitch_to_yaw`, `head_yaw_to_roll`, `head_roll_mount`, `head_bot_sheet`, `head` (shell), `roll_bearing`, `left/right_antenna_holder`, `antenna`, `sg90`, the STS3215 case + horn meshes (decimated for speed), and the servo pocket cut from `trunk_top` (`neck_servo_pocket`, bolts into our lid opening) | Closest possible copy of the ODM. Not used: the ODM's head Pi Zero and its board (our compute is in the trunk) |
| Head shell = ODM `head.stl` (197 W × 200 D × 59 H) with our additions: Ø36 eye windows for the Ø37 WS2812 rings, eye cups for the Camera Module 3s on the ODM's 80 mm baseline (sim: 70), mic port between the eyes, projector holes | Our expression package in the ODM head |
| **Projector is the ODM flashlight pod**: `flash_light_module.stl` + `flash_reflector_interface.stl` unchanged, at the ODM position on the left cheek, 4 mm higher up the cheek (ODM holes plugged, new ones drilled), shining forward. LED at (40.3, 97.8, 19.3) in the head frame | Move the sim's `projector` light there (dir +x). The LED is a 1 W emitter on a **15 × 12 mm board** (the insert's pocket) |
| Trunk floor at z −78 / −82 (sim trunk box −58); battery centre z −44 (sim −26); BMS, fuse, anti-spark switch and both floor bucks 12 mm lower | Fits the Pi stack under the ODM neck-pitch servo pocket (which reaches z 14.8) and closes the old battery/Pi clash; lowers the CoM |
| Pi 5 on 2 mm standoffs on the battery-clamp bars, PCB bottom z −4.5; AI HAT+ on its 16 mm header (PCB bottom z 12.9); sim has the Pi at 10 and the HAT at 39 | Under the servo pocket with 1.5 mm to spare; USB / Ethernet still open at the front |
| Lid opening x 4–68, y ±26 for the neck-pitch servo and the neck sheets through their sweep | |

## Neck range of motion

`sweep_neck.py` (full geometry, 108 poses) and `neck_envelope.py` (conservative head proxy, fine grid) pose the
four neck joints over the MJCF ranges (neck pitch −20…65°, head pitch ±45°, yaw ±94.5°, roll ±30°):

- The neck mechanism itself (STS3215s, sheets, brackets, servo pocket, lid opening) is clear everywhere.
- The **head** hits the trunk when both pitch joints go to their extremes in the same direction, as it would on the ODM
  (the ODM's sim simply excludes head/trunk contact). Clash-free envelope ([`neck_envelope.md`](neck_envelope.md)):
  - looking up: **neck_pitch + head_pitch ≥ −30°**
  - looking down: **head_pitch ≤ +30°** and **neck_pitch + head_pitch ≤ +60°** (the down limits only bind with the head
    also turned and rolled; straight ahead there is more room)
- **To do in the controller / sim:** clip head commands to this envelope (a coupled limit; MJCF joint ranges can't express
  it), and enable head/trunk contact in the sim so violations show up.

## Joint structure (as modeled)

- **Hip mount** (fixed): plate on the GIM4305 rear face (6x M2.5 on Ø47.5, Ø41 boss hole: use M2.5 × 8 in the 5 mm
  plate, since more than 3 mm of thread engagement damages the motor), flange on the trunk side wall (4x M4), shelf under the floor (3x M4).
- **Hip-roll yoke**: GIM4305 output (3x M3 on Ø27) → GIM6010 hip-pitch housing inboard face (8x M3 on Ø74, Ø60 cover clearance).
- **Thigh**: hip-pitch output hub (6x M3 on Ø25 + 3 dowels) → knee housing outboard face via an 11 mm ring over the driver cover.
- **Shin**: knee output (inboard) → jogs 9.5 mm outboard → wheel GIM4305 rear face.
- **Wheel**: hub web on the GIM4305 output, Ø138 rim with 12 tread key holes and grooves, VytaFlex tread (`../tires.md`).

## Not done yet

- Controller: clip head commands to the neck envelope above (TODO in `simulate_wheeled_duck.py`).
- No far-side bearing supports at the hip roll / hip pitch / knee yet (open issue 2), and no FEM of the printed parts against `design_loads.md`.
- Swept clearance for the legs (only the neck has a sweep; `check_cad_clearances.py` covers the sim shapes).
- Cable routing, fasteners, and an access hatch for the battery.
- The "Ø86.6" GIM6010 envelope in `purchased_parts.md` is OpenCASCADE's loose bounding box. The tight box of the
  STEP is Ø80.6, so the motor is Ø80 with no radial protrusion.
