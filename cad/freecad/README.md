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
model change, the same as the other `cad/` generators. Then export the assembly into the sim (`freecadcmd cad/freecad/export_sim.py`, ~1 min): it writes every body's meshes
(`assets/cad/`, one per colour), the collision hulls, each body's mass / CoM / inertia (`mass_properties.json`, from
the walls + infill mass model), the neck body positions and antenna axes, the cameras, projector light and IMU site
into `open_duck_wheeled_v2_gim6010.xml`, so the sim is the CAD. Then re-run `tools/export_cad_spec.py`,
`tools/check_cad_clearances.py`, `tools/derive_lqr_params.py` and `tools/fit_balance_trim.py gim6010`, and the tests.
What stays out of the export: the vendor STEP shapes (motors, Pi 5; their meshes use the `duck_parts` envelope models,
the vendor files are not redistributed; their masses and inertia still come from the real shapes), and the tires (MuJoCo
cylinders Ø150 × 32, without the CAD's R4 crown, so rolling contact stays analytic).

## Document structure

One `App::Part` per MJCF body, nested like the MJCF tree (`SuperDuck > Trunk > L_HipMount > L_HipRoll > L_Thigh >
L_Shin > L_Wheel`, `Trunk > NeckPitch > HeadPitch > HeadYaw > Head > L/R_Antenna`). Each part's placement holds
the joint angle, so you can pose a leg or the head by editing the rotation angle of that `App::Part`.
`Sim_Standins` (hidden) holds the MJCF's primitive geoms (the tires); everything else in the sim is exported from this document.

## Layout decisions in this pass (in the sim through `export_sim.py`; "sim" below = the old hand-made stand-ins)

| Change | Why |
| :--- | :--- |
| **Charge port**: GX16-2 aviation socket on the tail end face (z −38), under the e-stop | Keyed, latching, and can't short like a barrel jack. Wire it to the BMS charge port (C−/P−) and pack +, before the anti-spark switch, so it charges with the robot off. Use a 25.2 V CC/CV charger at 3–4 A (GX16-2 contacts are rated ~5 A), with a 10 A fuse in the charge lead |
| E-stop on the tail end face (y 0, z −8), its contact block reaching into the tail | Open issue 7 |
| Teensy carrier (42 × 30) + ICM-42688-P on a shelf cantilevered off the rear bulkhead (z 34–37) | The sim boxes were outside / overlapping |
| Amp on the rear bulkhead; fuse, anti-spark switch and peripheral buck on a shelf in the tail (z −60); BMS on edge across the bay just ahead of the pack (x 12…20); speaker under a grille on the tail's top face | The ODM body's bottom chamfers took the floor corners they used |
| **Ventilation**: rounded 4 mm slots. Low inlet through the floor down the middle of the underside, between the hip-mount shelves and under the servo buck (4 slots, x 20…68, y ±12, about 7.5 cm²); exhaust high on both side walls beside the Pi, under the lid (x −40…8, z 22…46, 9 a side, about 17 cm²); a 40 × 10 mm fan on the rear bulkhead's tail face (y −38, z 6, Ø38 bore, 4x M3 on 32 mm) blowing out of the tail's tapered sides (6 slots a side, about 12 cm²). The intake slots first put low on the side walls were moved to the underside (user choice) | ~10 W typical / ~20 W peak inside (Pi 5, AI HAT+, bucks); sealed, the air inside would reach ~45–50 °C at peak. It breathes by convection with the fan off |
| 5.1 V Pi buck and the Waveshare bus servo adapter on the front bulkhead (the old front wall); 7.4 V servo buck on the floor ahead of the pack | Open issue 7 (these weren't in the model) |
| USB nav camera (the sim's `trunk_nav_cam`) behind the front cap, its axis 20° below horizontal like the sim's, lens at (86.8, 0, −13.7), 1 mm inside the cap (sim: 93, 0, −14). Tilted, the 32 mm board needs 12.4 mm of the 10.75 mm gap in front of the front bulkhead, so its lower edge and USB plug pass through a window in the bulkhead (also the screw access). 4 posts off the cap: M2 self-tapping at the bottom corners, locating pins at the top. The 5.1 V Pi buck moved 3 mm outboard (y 36) so its lower boss clears the window | Sees the floor from ~0.2–0.3 m ahead of the axle to the horizon, about ±0.35 m wide at the near edge, nothing of the robot in view (sim render, ≥ 100° / 75° fov). Covers the ~0.6–0.75 m stopping distance from 1.5 m/s |
| **Neck and head are the Open Duck Mini v2's, 1:1** (same STS3215 servos): its own STLs and placements from its `robot.xml` (`odm.py`), hung from our neck-pitch joint. Joints in the trunk frame: neck pitch (36, 0, 60) as in the sim, head pitch (36, 0, 126), head yaw (36, 0, 183), head roll (77, 0, 163.9); the sim has 117.3 / 139.3 / (54.7, 177.3) | The sim's neck spacing (22 mm pitch→yaw) can't fit STS3215s. Exported to the MJCF (2026-10-26): neck body positions, head mass / inertia, the ODM's tilted antenna axes, camera positions |
| Parts used unchanged: `neck_left/right_sheet`, `head_pitch_to_yaw`, `head_yaw_to_roll`, `head_roll_mount`, `head_bot_sheet`, `head` (shell), `roll_bearing`, `left/right_antenna_holder`, `antenna`, `sg90`, the STS3215 case + horn meshes (decimated for speed), and the servo pocket cut from `trunk_top` (`neck_servo_pocket`, bolts into our lid opening) | Closest possible copy of the ODM. Not used: the ODM's head Pi Zero and its board (our compute is in the trunk) |
| Head shell = ODM `head.stl` (197 W × 200 D × 59 H) with our additions: Ø36 eye windows for the Ø37 WS2812 rings, eye cups for the Camera Module 3s on the ODM's 80 mm baseline (sim: 70), mic port between the eyes, projector holes | Our expression package in the ODM head |
| **Projector is the ODM flashlight pod**: `flash_light_module.stl` + `flash_reflector_interface.stl` unchanged, at the ODM position on the left cheek, 4 mm higher up the cheek (ODM holes plugged, new ones drilled), shining forward. LED at (40.3, 97.8, 19.3) in the head frame | The sim's `projector` light is there (dir +x). The LED is a 1 W emitter on a **15 × 12 mm board** (the insert's pocket) |
| Trunk floor top at z −78, body underside −86.9 (sim trunk box −58); battery centre z −44 (sim −26); BMS, fuse, anti-spark switch and both floor bucks 12 mm lower | Fits the Pi stack under the ODM neck-pitch servo pocket (which reaches z 14.8) and closes the old battery/Pi clash; lowers the CoM |
| Pi 5 on 2 mm standoffs on the battery-clamp bars, PCB bottom z −4.5; AI HAT+ on its 16 mm header (PCB bottom z 12.9); sim has the Pi at 10 and the HAT at 39 | Under the servo pocket with 1.5 mm to spare; USB / Ethernet still open at the front |
| Lid opening x 4–68, y ±26 for the neck-pitch servo and the neck sheets through their sweep | |
| **Knees bend backward** (home hip pitch +28.6°, knee −57.3°; was −28.6 / +57.3). Applied to the MJCF too (hip-pitch range −30…+70°, keyframes, controller `nominal_*`, trim refit) | User choice. The legs are fore-aft symmetric, so only the signs change |
| **Leg styling after the Impulse robot** (`~/Downloads/Impulse.zip`, reference only, no parts copied): slotted motor cans, twin thigh side plates, rimmed lower link (the Warren-truss triangular windows were removed, user choice) | User choice |
| Impulse details kept: slotted **end caps** over the hip-roll and wheel motors' rear bosses (cable notch). The Impulse trunk chamfers, vents, carry handle and breast cover went with the old body | User choice. (Moving the legs under the trunk, Impulse-style, was tried and rejected: it needed the hips 64 mm lower and a 186 mm track) |
| **Body is the Open Duck Mini v2's shape, ×1.15** (its width then matches ours, 126 mm): planes measured from the ODM `body_front` / `body_back` / `body_middle_*` meshes (`tools/fetch_vendor_cad.sh` now fetches them), scaled about the neck-pitch axis and 3 mm lower. 230 long (x −142…88) × 126 × 164 (z −87…78): octagonal section, front cap, 45° tapered tail, closed sides (the ODM's side windows, where its legs come out, are left out: our hip motors are outside) and the opening round the neck (x −49…66, y ±29). Split like the ODM at z 12.3 into the lower tub (solid floor, the old front and rear walls kept as bulkheads, a cover locating lip) and the top cover (4 screws down through posts into the lid and the bulkheads). Differences from the ODM: the underside starts sloping up at x −63.5, not −27 (our battery runs back to −62.5), so the slope is 23.6°, not 16.6°; the interior is ours (the ODM's floor is 13 mm thick and 7 mm higher than ours) | User choice. The sim's trunk box and mass are unchanged; the printed trunk is now 617 cm³ (was 395) |
| **Legs connect like the ODM's**: the hip mount hangs under the body (shelf screwed into the floor from below, web hugging the bottom chamfer, rib out to the GIM4305 plate), so the leg arm comes out from under the body. The thigh and shin are twin sheets of one outline, like the ODM's leg links; front and back walls close the thigh from the hip to the knee (the ODM's thigh cover; printed as an outer sheet and an inner tray). The hip-roll motors stay outside the body (the ODM's are inside): with the wheels at y ±112 that would put a 72 mm lever on a 1 N·m GIM4305 | User choice |
| **Leg harness** (one bundle per leg: 2× 16 AWG power + the CAN twisted pair, ~6 mm, in 6 mm expandable PET braid, modelled Ø8; Ø6 sleeved drops to each motor). The size is one setting, `CABLE_D` in `build_super_duck.py`: the clip / post holes (+0.8 mm), the grommet (hole `CABLE_D`+6, bore +1), the run's offsets and the twist-point gaps all follow it; measure the sleeved bundle, set it, rebuild and re-run `sweep_legs.py`. Enters the body through a grommet in each side wall at (28, ±62, −35), Ø14 hole, into the free front corner of the bay, then up and back over the Pi deck (it ends at x −35 in the model: there the CAN pair goes to the Teensy terminals and the power pair down through the rear bulkhead hole to the fuse / anti-spark switch). Outside: back along the body side over the hip-mount plate, down behind the end cap to a zip-tie clip on the rib's end, **across the hip-roll axis 20 mm behind the hip mount**, up a post on the yoke, over the hip-pitch can (clip), **across the hip-pitch axis on the thigh's outer face**, down that face in 3 clips, **across the knee axis**, then in a free span round the thigh's knee end (no holder; leave ~15 mm of slack) to one clip on the shin's inner sheet, and down that sheet to the wheel motor's end-cap notch. Drops: hip roll from its end-cap notch, hip pitch from the back of its driver cover, knee through the 30° hub window. The hip-output screws under the hip crossing are countersunk into a 3 mm aluminium clamp ring on the thigh's outer face, the knee-housing ones under the knee crossing into the sheet | Crossing each joint on its axis means the joint only twists the cable (±25°, −30…+70°, ±90°) and never bends or drags it: no service loops to snag. Every crossing is modelled as a ball on one side and the other side's run stopping 7 mm short, so `sweep_legs.py` checks the harness through the whole range. Adds about 15 mm a side (the thigh's clips are the widest point): overall width 407 mm at `CABLE_D` 8 (was 378 without the harness) |

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

- **Hip mount** (fixed): shelf under the body floor (3x M4 into heat inserts, counterbored), its top face flush on the
  floor; a web hugging the bottom chamfer 0.5 mm off the skin; a rib out to the GIM4305 rear-face plate (7 mm; 6x M2.5
  on Ø47.5, Ø41 boss hole: use M2.5 × 10, since more than 3 mm of thread engagement damages the motor). Between the
  body and the end cap the rib and plate are one block down to the plate's bottom, ramped at its outboard end, so the
  hip-pitch reaction twists a closed section instead of an open cross (R4 fillet in the plate/rib corner beyond it).
  The plate's outboard end is round, R 29.5 about the roll axis, and its top edge runs into that round through an R 10
  bend, so the hip-pitch motor cover clears it at ±25° roll.
- **Hip casing (hip-roll yoke)**: GIM4305 output (3x M3 on Ø27, round Ø66 plate) → GIM6010 hip-pitch housing inboard
  face (8x M3 on Ø74, Ø60 cover clearance). A slotted Ø88 can encloses the hip-pitch housing and a slotted Ø66 wrap
  turns with the roll output around the GIM4305 (2.5 mm radial clearance). Top and bottom walls 1 mm clear of the Ø60
  bore, 3 mm gussets where they meet the motor plates. The 3x M3 into the GIM4305 output carry ~10–13 N·m of bending
  at a landing: steel washers, threadlocker, and torque them to ~1 N·m (≥ 350 N preload each).
- **Thigh**: twin sheets, one outline (hull of R32 at the hip, R46 at the knee, 2 mm edge fillets). The wheel is 74 mm
  inboard of the outer sheet and the hip-pitch motor has no far-side bearing, so from the hip to the walls the outer
  sheet alone carries the leg's side bending: it is **13 mm thick** (y 16…29) outside the motor's output hub. It bolts to
  the hip-pitch output hub (6x M3 × 14 countersunk on Ø25 + 3 dowels, through a 3 mm aluminium clamp ring Ø56 on its
  outer face) and to the knee housing's cover-side face (8x M3 on Ø74). The inner sheet (5 mm) bolts to the housing's
  output-side face (8x M3 countersunk: the shin passes 2 mm off it), scalloped R47 round the hip so it clears the yoke. Front and back walls from r 47 off the hip
  axis to the knee can. **Two prints**, each flat with nothing to support inside: the outer sheet (on its y 16 face,
  clips up) and the inner tray (inner sheet, walls, the slotted Ø88 knee can and the ring the housing bolts to, on
  its inner sheet). The knee-housing screws clamp them together; 4x M3 × 14 socket heads into heat inserts in wall
  bosses join them along the length.
- **Shin**: one outline (R28 knee, R32 wheel). The outer sheet (6 mm) on the knee output, the inner sheet (6 mm) on the
  wheel GIM4305 rear face, scalloped R54 (tapered into the outer sheet) so it clears the thigh at ±90° knee. The gap between them
  is filled, so below the scallop the shin is one 15.5 mm slab, pocketed Ø57 for the wheel motor. From the knee down,
  wherever it clears the tyre by 3.5 mm, the outer sheet is 14 mm (y −36…−22); the knee-output screws (M3 × 14) sit
  in counterbores in it.
- **Wheel**: hub web on the GIM4305 output (3x M3 with washers: the drive torque reverses), Ø138 rim with 12 Ø3 tread
  key holes and grooves, VytaFlex tread (`../tires.md`).

## Strength (FEM)

Voxel FEM (1 mm hexes, PETG E 2.0 GPa, motors as rigid patches, the hip-mount shelf in one-sided contact with the
floor and held by its 3 M4s) against `design_loads.md` plus actuator stall. A = landing peak (250 N at the wheel,
roll at its 3.47 N·m limit), B = hip pitch and knee at the 11 N·m stall, D = driving (p99.5), E = standing.
"Nominal" is the 99.9th-percentile principal stress, "hot spot" the worst sharp-corner value (mesh-dependent,
indicative). PETG: ~45 MPa along the layers, ~25 across, fatigue ~10 MPa at 10⁶ cycles, creep below ~5–7 MPa.
Before → after this redesign:

| Part | A nominal / hot spot (MPa) | B hot spot | D hot spot | Stiffness |
| :--- | :--- | :--- | :--- | :--- |
| Thigh (two pieces, analysed bonded) | 46 / 102 → **24 / 43** | 25 | 8 | wheel tilt standing 1.27° → **0.29°** |
| Thigh outer sheet alone | **26 / 44** | 27 | 8 | 0.39° standing |
| Shin | 29 / 71 → **21 / 46** | | 14 → **9** | wheel tilt standing 1.13° → **0.30°** |
| Hip mount | 20 / 57 → **9 / 21** (B: 28 / 81 → **12 / 31**) | | 17.5 → **6.5** | pitch at A 2.4° → **0.9°** |
| Hip-roll yoke | **9 / 18** | 25 → **21** | 4.5 | |

Remaining peaks: the thigh's is in the 7 mm zone round the hip-output hub, under the aluminium clamp ring the FEM
leaves out; the shin's at its edges where the tyre clearance ends (x ±28, z −43). Both are about 2× under PETG's
strength along the layers at the landing peak and under its fatigue limit when driving. PETG-CF / PA6-CF for the
thigh outer sheet and the shin would add margin. Hip-mount M4 inserts: ≤ 180 N (B), against ~1 kN pull-out.
The FEM scripts (`fem.py`, `loads.py`, ~300 lines of numpy / pyamg) aren't in the repo yet.

## Printing (Bambu P1S, 256³)

Every part fits the bed unsplit (largest: the tub, 230 × 126.5 × 141). PETG, 0.2 mm layers; structural parts (hip
mount, yoke, thigh, shin, hub) 6 walls and 40–50% gyroid, shells 3 walls and 15–20%. No PLA anywhere structural: the
bay reaches 45–50 °C and the motors run warm. PETG-CF or PA6-CF (hardened nozzle) is worth it for the yoke and hip
mounts. Orientation:

- **Thigh outer sheet** on its inner (y 16) face, clips up; **thigh inner tray** on its inner sheet. The ring the
  knee housing bolts to overhangs 7.75 mm into the can: print slowly there or add a small support ring.
- **Shin** on the inboard face of its knee end (y −36); support under the slab's tyre-side face.
- **Hip-roll yoke** on the GIM4305 output plate, so the loaded corners stay along the layers.
- **Hip mount** standing on its x −58 end with a 5–8 mm brim; support under the rib and block.
- **Trunk tub** upright: the wall holes are teardrops and the tail shelf's pockets are closed (no support there:
  block it in the slicer; the 5 ribs keep the bridges ~18 mm); tree support under the tail slope.
- **Wheel hub** on its web; optionally split the rim at the web to print both halves flat.
- About 2.6 kg of PETG with supports (three 1 kg spools).

## Not done yet

- Controller: clip head commands to the neck envelope above (TODO in `simulate_wheeled_duck.py`).
- No far-side bearing supports at the hip roll / hip pitch / knee (open issue 2). At the hip pitch there's no room for one: the hip-roll motor's wrap is 3 mm inboard of the yoke plate on that axis. The thigh's 13 mm outer sheet carries the bending instead, which leaves 15–22 N·m on the GIM6010's own bearing at a landing peak: ask SteadyWin for its moment rating.
- `design_loads.md` gives bending only at the joint anchors, and has no sideways wheel impacts, side falls, one-wheel landings, motor-stall case, cycle counts or temperatures.
- Leg sweep: `sweep_legs.py` poses the left leg over its joint ranges and writes `leg_sweep_report.md`.
- Fasteners, the in-body harness breakout (CAN to the Teensy, power to the tail) and the head / neck wiring. Battery access: lift the top cover (the lid and neck come out with it, or unscrew the lid first).
- The "Ø86.6" GIM6010 envelope in `purchased_parts.md` is OpenCASCADE's loose bounding box. The tight box of the
  STEP is Ø80.6, so the motor is Ø80 with no radial protrusion.
