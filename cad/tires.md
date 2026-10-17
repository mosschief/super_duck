# Cast Urethane Tires

Soft cast urethane treads on printed hubs. Grip limits push recovery (`GEMINI.md`, "Push-recovery limit is
traction"), and casting gives the exact size and hardness the model assumes.

## Spec

| | |
| :--- | :--- |
| Tread OD / width | **150 / 32 mm** (model: `wheel_tire`, radius 75, half-width 16) |
| Tread thickness | **6 mm** → hub rim OD 138 mm. Keeps the wheel at the model's 255 g / 0.00072 kg·m² spin inertia; a 10 mm tread adds ~50 g per wheel |
| Material | **Smooth-On VytaFlex 40** (Shore 40A, 1:1 by volume, 30 min pot life, 16 h demold). ReoFlex 40 is equivalent. Go to 30A for more grip, 50A for more wear life |
| Tread volume | ~87 cm³ per wheel; ~90 g at a density of ~1.0 (estimate: check the datasheet) |
| Profile | **Crowned**, edge radius ~4 mm: hip roll tilts the wheel up to ±17° (`roll_comp` clip), which moves the contact patch toward the tread edge |
| Hub | Printed PETG or PA-CF dish bolted to the GIM4305-10 output: **3x M3 on Ø27 PCD**, centred on the **Ø37 output step** (`purchased_parts.md`). The motor's 125 N axial / 500 N radial bearing limits apply; landing contact peaks at ~250 N radial (`design_loads.md`) |
| Tread retention | Urethane doesn't bond to PLA/PETG: lock it mechanically. 12x Ø5 axial through-holes in the rim, 3 mm below the rim surface, plus 1 mm circumferential grooves; the urethane flows through and keys in |

## Casting

1. Print the hub, then a two-part mold that closes around the rim and leaves the 6 mm tread cavity (crowned
   profile), with a pour gate and a vent at the top. PETG, 0.12-0.16 mm layers, sanded or coated with a sealer: the
   mold's surface finish is the tread's.
2. Seal and release: brush a sealer on porous prints, then a urethane-compatible mold release (e.g. Smooth-On
   Ease Release 200) on the mold, **not** on the hub rim (the urethane should fill the keys).
3. Mix A:B 1:1 by volume, scrape sides and bottom. Degas in a vacuum chamber if you have one; otherwise pour in a thin
   stream from high up, into the lowest point, and let air escape through the vent.
4. Demold after 16 h (check the datasheet for full-cure time before hard use). Trim the gate.
5. Cast spares: a trial-size kit covers both tires and a spare set. Keep the mold.

## Measure grip, then update the sim

On each floor the robot will use, put the robot (or a weighted wheel) on a board, raise one end slowly until it
slides, and record the angle: **μ = tan(angle)**. The sim currently uses μ = 1.25 (`wheel_tire`) and randomizes
0.8-1.25 in `--realism` (`RealismConfig.friction`). Put the measured range in both, then rerun
`python3 test_scenarios.py` and `--realism 12`: the ±40 N push results show what the grip buys.
