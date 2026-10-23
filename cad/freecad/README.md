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
| **Charge port**: GX16-2 aviation socket on the tail end face (z −38), under the e-stop | Keyed, latching, and can't short like a barrel jack. Wire it to the BMS charge port (C−/P−) and pack +, before the anti-spark switch, so it charges with the robot off. Use a 25.2 V CC/CV charger at 3–4 A (GX16-2 contacts are rated ~5 A), with a 10 A fuse in the charge lead |
| E-stop on the tail end face (y 0, z −8), its contact block reaching into the tail | Open issue 7 |
| Teensy carrier (42 × 30) + ICM-42688-P on a shelf cantilevered off the rear bulkhead (z 34–37) | The sim boxes were outside / overlapping |
| Amp on the rear bulkhead; fuse, anti-spark switch and peripheral buck on a shelf in the tail (z −60); BMS at y 50; speaker under a grille on the tail's top face | The ODM body's bottom chamfers took the floor corners they used |
| **Ventilation**: rounded 4 mm slots. Low inlet through the floor down the middle of the underside, between the hip-mount shelves and under the servo buck (4 slots, x 20…68, y ±12, about 7.5 cm²); exhaust high on both side walls beside the Pi, under the lid (x −40…8, z 22…46, 9 a side, about 17 cm²); a 40 × 10 mm fan on the rear bulkhead's tail face (y −38, z 6, Ø38 bore, 4x M3 on 32 mm) blowing out of the tail's tapered sides (6 slots a side, about 12 cm²). The intake slots first put low on the side walls were moved to the underside (user choice) | ~10 W typical / ~20 W peak inside (Pi 5, AI HAT+, bucks); sealed, the air inside would reach ~45–50 °C at peak. It breathes by convection with the fan off |
| 5.1 V Pi buck and the Waveshare bus servo adapter on the front bulkhead (the old front wall); 7.4 V servo buck on the floor ahead of the pack | Open issue 7 (these weren't in the model) |
| USB nav camera (the sim's `trunk_nav_cam`) behind the front cap, its axis 20° below horizontal like the sim's, lens at (86.8, 0, −13.7), 1 mm inside the cap (sim: 93, 0, −14). Tilted, the 32 mm board needs 12.4 mm of the 10.75 mm gap in front of the front bulkhead, so its lower edge and USB plug pass through a window in the bulkhead (also the screw access). 4 posts off the cap: M2 self-tapping at the bottom corners, locating pins at the top. The 5.1 V Pi buck moved 3 mm outboard (y 36) so its lower boss clears the window | Sees the floor from ~0.2–0.3 m ahead of the axle to the horizon, about ±0.35 m wide at the near edge, nothing of the robot in view (sim render, ≥ 100° / 75° fov). Covers the ~0.6–0.75 m stopping distance from 1.5 m/s |
| **Neck and head are the Open Duck Mini v2's, 1:1** (same STS3215 servos): its own STLs and placements from its `robot.xml` (`odm.py`), hung from our neck-pitch joint. Joints in the trunk frame: neck pitch (36, 0, 60) as in the sim, head pitch (36, 0, 126), head yaw (36, 0, 183), head roll (77, 0, 163.9); the sim has 117.3 / 139.3 / (54.7, 177.3) | The sim's neck spacing (22 mm pitch→yaw) can't fit STS3215s. **Update the MJCF**: neck body positions, head mass / inertia, the ODM's tilted antenna axes, camera positions |
| Parts used unchanged: `neck_left/right_sheet`, `head_pitch_to_yaw`, `head_yaw_to_roll`, `head_roll_mount`, `head_bot_sheet`, `head` (shell), `roll_bearing`, `left/right_antenna_holder`, `antenna`, `sg90`, the STS3215 case + horn meshes (decimated for speed), and the servo pocket cut from `trunk_top` (`neck_servo_pocket`, bolts into our lid opening) | Closest possible copy of the ODM. Not used: the ODM's head Pi Zero and its board (our compute is in the trunk) |
| Head shell = ODM `head.stl` (197 W × 200 D × 59 H) with our additions: Ø36 eye windows for the Ø37 WS2812 rings, eye cups for the Camera Module 3s on the ODM's 80 mm baseline (sim: 70), mic port between the eyes, projector holes | Our expression package in the ODM head |
| **Projector is the ODM flashlight pod**: `flash_light_module.stl` + `flash_reflector_interface.stl` unchanged, at the ODM position on the left cheek, 4 mm higher up the cheek (ODM holes plugged, new ones drilled), shining forward. LED at (40.3, 97.8, 19.3) in the head frame | Move the sim's `projector` light there (dir +x). The LED is a 1 W emitter on a **15 × 12 mm board** (the insert's pocket) |
| Trunk floor top at z −78, body underside −86.9 (sim trunk box −58); battery centre z −44 (sim −26); BMS, fuse, anti-spark switch and both floor bucks 12 mm lower | Fits the Pi stack under the ODM neck-pitch servo pocket (which reaches z 14.8) and closes the old battery/Pi clash; lowers the CoM |
| Pi 5 on 2 mm standoffs on the battery-clamp bars, PCB bottom z −4.5; AI HAT+ on its 16 mm header (PCB bottom z 12.9); sim has the Pi at 10 and the HAT at 39 | Under the servo pocket with 1.5 mm to spare; USB / Ethernet still open at the front |
| Lid opening x 4–68, y ±26 for the neck-pitch servo and the neck sheets through their sweep | |
| **Knees bend backward** (home hip pitch +28.6°, knee −57.3°; was −28.6 / +57.3). Applied to the MJCF too (hip-pitch range −30…+70°, keyframes, controller `nominal_*`, trim refit) | User choice. The legs are fore-aft symmetric, so only the signs change |
| **Leg styling after the Impulse robot** (`~/Downloads/Impulse.zip`, reference only, no parts copied): slotted motor cans, twin thigh side plates, rimmed lower link (the Warren-truss triangular windows were removed, user choice) | User choice |
| Impulse details kept: slotted **end caps** over the hip-roll and wheel motors' rear bosses (cable notch). The Impulse trunk chamfers, vents, carry handle and breast cover went with the old body | User choice. (Moving the legs under the trunk, Impulse-style, was tried and rejected: it needed the hips 64 mm lower and a 186 mm track) |
| **Body is the Open Duck Mini v2's shape, ×1.15** (its width then matches ours, 126 mm): planes measured from the ODM `body_front` / `body_back` / `body_middle_*` meshes (`tools/fetch_vendor_cad.sh` now fetches them), scaled about the neck-pitch axis and 3 mm lower. 230 long (x −142…88) × 126 × 164 (z −87…78): octagonal section, front cap, 45° tapered tail, closed sides (the ODM's side windows, where its legs come out, are left out: our hip motors are outside) and the opening round the neck (x −49…66, y ±29). Split like the ODM at z 12.3 into the lower tub (solid floor, the old front and rear walls kept as bulkheads, a cover locating lip) and the top cover (4 screws down through posts into the lid and the bulkheads). Differences from the ODM: the underside starts sloping up at x −63.5, not −27 (our battery runs back to −62.5), so the slope is 23.6°, not 16.6°; the interior is ours (the ODM's floor is 13 mm thick and 7 mm higher than ours) | User choice. The sim's trunk box and mass are unchanged; the printed trunk is now 617 cm³ (was 395) |
| **Legs connect like the ODM's**: the hip mount hangs under the body (shelf screwed into the floor from below, web hugging the bottom chamfer, rib out to the GIM4305 plate), so the leg arm comes out from under the body. The thigh and shin are twin sheets of one outline, like the ODM's leg links; front and back walls close the thigh into one piece from the hip to the knee (the ODM's thigh cover). The hip-roll motors stay outside the body (the ODM's are inside): with the wheels at y ±112 that would put a 72 mm lever on a 1 N·m GIM4305 | User choice |

## Neck range of motion

`sweep_neck.py` (full geometry, 108 poses) and `neck_envelope.py` (conservative head proxy, fine grid) pose the
four neck joints over the MJCF ranges (neck pitch −20…65°, head pitch ±45°, yaw ±94.5°, roll ±30°):

- The neck mechanism itself (STS3215s, sheets, brackets, servo pocket, lid opening) is clear everywhere.
- The **head** hits the trunk when both pitch joints go to their extremes in the same direction, as it would on the ODM
  (the ODM's sim simply excludes head/trunk contact). Clash-free envelope ([`neck_envelope.md`](neck_envelope.md)):
  - looking up: **neck_pitch + head_pitch ≥ −15°**
  - looking down: **head_pitch ≤ +15°** and **neck_pitch + head_pitch ≤ +65°** (the limits mostly bind with the head
    also turned and rolled; straight ahead there is more room)
  - With the ODM-shaped body (top cover 18 mm above the neck axis, like the ODM's) this is tighter than with the old
    lid-topped box (−30° / +30° / +60°): the head's corners reach the cover rim at full yaw and roll.
- **To do in the controller / sim:** clip head commands to this envelope (a coupled limit; MJCF joint ranges can't express
  it), and enable head/trunk contact in the sim so violations show up.

## Joint structure (as modeled)

- **Hip mount** (fixed): shelf under the body floor (3x M4 into heat inserts, counterbored), a web hugging the bottom
  chamfer 0.5 mm off the skin, a rib out to the GIM4305 rear-face plate (6x M2.5 on Ø47.5, Ø41 boss hole: use
  M2.5 × 8 in the 5 mm plate, since more than 3 mm of thread engagement damages the motor). The plate's outboard
  end is round, R 29.5 about the roll axis, and its top edge runs into that round through an R 10 bend, so the hip-pitch motor cover clears it at ±25° roll.
- **Hip casing (hip-roll yoke)**: GIM4305 output (3x M3 on Ø27, round Ø66 plate) → GIM6010 hip-pitch housing inboard
  face (8x M3 on Ø74, Ø60 cover clearance). A slotted Ø88 can encloses the hip-pitch housing and a slotted Ø66 wrap
  turns with the roll output around the GIM4305 (2.5 mm radial clearance).
- **Thigh**: twin sheets, one outline (hull of R32 at the hip, R46 at the knee, 2 mm edge fillets). The outer sheet
  (7 mm) bolts to the hip-pitch output hub (6x M3 on Ø25 + 3 dowels) and the knee housing's cover-side face (8x M3
  on Ø74 through an 11 mm ring); the inner sheet (5 mm) to the housing's output-side face, scalloped R47 round the
  hip so it clears the yoke. Front and back walls between the motors close it. Six windows over the knee motor
  cover, the knee in a slotted Ø88 can.
- **Shin**: twin sheets, one outline (R28 knee, R32 wheel): the outer one (6 mm) on the knee output, scalloped R28.5
  round the wheel motor; the inner one (6 mm) on the wheel GIM4305 rear face, scalloped R54 (clears the thigh at ±90° knee).
  Walls join them.
- **Wheel**: hub web on the GIM4305 output, Ø138 rim with 12 tread key holes and grooves, VytaFlex tread (`../tires.md`).

## Not done yet

- Controller: clip head commands to the neck envelope above (TODO in `simulate_wheeled_duck.py`).
- No far-side bearing supports at the hip roll / hip pitch / knee yet (open issue 2), and no FEM of the printed parts against `design_loads.md`.
- Leg sweep: `sweep_legs.py` poses the left leg over its joint ranges and writes `leg_sweep_report.md`.
- Cable routing and fasteners. Battery access: lift the top cover (the lid and neck come out with it, or unscrew the lid first).
- The "Ø86.6" GIM6010 envelope in `purchased_parts.md` is OpenCASCADE's loose bounding box. The tight box of the
  STEP is Ø80.6, so the motor is Ø80 with no radial protrusion.
