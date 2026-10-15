# GEMINI.md — Gemini CLI Setup Guide & Project Context

> **How to use this file**: Save this file as `GEMINI.md` inside the same local folder as `open_duck_wheeled_v2.xml` and `simulate_wheeled_duck.py`. Whenever you run `gemini` inside that folder, [Gemini CLI](https://www.google.com/search?q=https%3A%2F%2Fgeminicli.com%2F) automatically loads this file as persistent project memory so you can continue right where our web conversation left off.

## 1. Installing & Authenticating Gemini CLI

1. **Create Your Project Folder & Save the Files**:

   * Create a local directory (e.g., `mkdir ~/wheeled_duck && cd ~/wheeled_duck`).

   * Save `open_duck_wheeled_v2.xml`, `simulate_wheeled_duck.py`, and this `GEMINI.md` file into `~/wheeled_duck/`.

2. **Install Gemini CLI**:

   * Ensure Node.js (v20+) is installed, then install the official [Gemini CLI package](https://www.google.com/search?q=https%3A%2F%2Fgithub.com%2Fgoogle-gemini%2Fgemini-cli) globally:
     `npm install -g @google/gemini-cli`

3. **Authenticate Your Account or API Key**:

   * Run `gemini` in your terminal inside `~/wheeled_duck/`.

   * Select **Sign in with Google** (opens a browser prompt) or set a Gemini API key via `export GEMINI_API_KEY="your_api_key"` ([authentication docs](https://www.google.com/search?q=https%3A%2F%2Fgeminicli.com%2Fdocs%2Fget-started%2Fauthentication%2F)).

4. **Verify Context & Resume Iteration**:

   * Because `GEMINI.md` is in the working directory, Gemini CLI automatically ingests the full robot specification below ([GEMINI.md documentation](https://www.google.com/search?q=https%3A%2F%2Fgithub.com%2Fgoogle-gemini%2Fgemini-cli%2Fblob%2Fmain%2Fdocs%2Fcli%2Fgemini-md.md)).

   * You can verify it loaded by typing `/memory show` inside Gemini CLI, or immediately ask:
     `Read @open_duck_wheeled_v2.xml and @simulate_wheeled_duck.py and add a jump keybind on 'C'.`

## 2. Project Overview: 1.35x Scaled Wheeled Open Duck Mini v2

This project models and controls a **1.35x scaled, wheeled bipedal version** of the [Open Duck Mini v2](https://www.google.com/search?q=https%3A%2F%2Fgithub.com%2Fapirrone%2FOpen_Duck_Mini) (`56.7 cm` standing height vs. `42 cm` original), retaining the authentic BDX droid look, two-tone off-white/charcoal palette, and 16-body kinematic tree while scaling the chassis to fit a [Raspberry Pi 5](https://www.google.com/search?q=https%3A%2F%2Fwww.raspberrypi.com%2Fproducts%2Fraspberry-pi-5%2F), three [Raspberry Pi Camera Module 3](https://www.google.com/search?q=https%3A%2F%2Fwww.raspberrypi.com%2Fproducts%2Fcamera-module-3%2F) sensors, and [SteadyWin GIM4305-10](https://www.google.com/search?q=https%3A%2F%2Frcdrone.top%2Fproducts%2Fsteadywin-gim4305-planetary-reducer-servo-motor) quasi-direct-drive (QDD) planetary actuators.

## 3. Files in This Workspace

* `open_duck_wheeled_v2_gim6010.xml`: **Default model = the build configuration.** SteadyWin GIM6010-8 on hip-pitch and knees (GIM4305-10 on hip-yaw, hip-roll and wheels), 6S2P 21700 battery, AI HAT+, PJRC Teensy 4.0 Dual-CAN (or Feather M4 CAN) + ICM-42688-P SPI IMU, Open Duck Mini expression package (antennas, eye LEDs, projector, speaker, mic) and battery protection: **5.48 kg**.

* `open_duck_wheeled_v2.xml`: The original standalone MuJoCo MJCF robot and scene model (`open_duck_mini_v2_scaled_wheeled`), all GIM4305-10. Selected with `--robot gim4305`.

* `simulate_wheeled_duck.py`: 500 Hz (`dt = 0.002 s`) full-state LQR wheel-balancing, 4-DOF leg posture, stereo-camera horizon leveling, and live keyboard driving controller. `--robot gim6010|gim4305` picks a `RobotProfile` (model file + per-robot tuning; default `gim6010`). `--realism SEED` runs it on a randomized robot with noisy sensors and CAN latency (`SimToReal`).

* `tools/`: `upgrade_motor.py` (swap a joint to GIM6010-8), `battery_pack.py` (set pack mass/size), `add_trunk_electronics.py` (AI HAT+, Teensy), `add_expression_hardware.py` (antennas, eye LEDs, projector, speaker, mic, battery protection), `derive_lqr_params.py` (profile body params from an MJCF), `fit_balance_trim.py` (profile trim/posture fit), `power_budget.py`. After any mass change: run `derive_lqr_params.py`, then `fit_balance_trim.py`, paste both into the profile, then re-run `test_scenarios.py`.

* `test_scenarios.py`: Scripted scenario tests scored against MuJoCo ground truth (stand, drive/turn/brake, brake settle, crouch, pushes ±20–40 N, jumps). `--robot` as above; `--realism N` runs each scenario on N randomized robots and reports pass rates; `-k name` filters scenarios; `thermal` compares RMS joint torque to each motor's continuous rating.

## 4. Locked-In Hardware & Simulation Specifications

| Subsystem | Specification | MuJoCo Modeling Details | 
| ----- | ----- | ----- | 
| **Scale Factor & Mass** | `1.35x` linear scale (`567 mm` tall, `3.895 kg` total mass: `3.385 kg` body + `2 x 0.255 kg` wheels) | Preserves exact `open_duck_mini_v2.xml` body names (`base`, `trunk_assembly`, `hip_roll_assembly`, `knee_and_ankle_assembly`, `foot_assembly`, etc.) | 
| **Upper Leg Joints (8 DOFs)** | 4 joints per leg: `hip_yaw`, `hip_roll`, `hip_pitch`, `knee` using [SteadyWin GIM4305-10](https://www.google.com/search?q=https%3A%2F%2Frcdrone.top%2Fproducts%2Fsteadywin-gim4305-planetary-reducer-servo-motor) (`Ø53 x 26 mm`, `150 g`, `10:1` planetary gear, `1.0 N·m` nominal / `3.47 N·m` peak torque, `38.3 rad/s` max speed) | `class="gim4305"` position/impedance actuators (`kp = 32.0`, `kv = 4.0` — controller drops it to `1.4` for the jump wind-up/thrust via MIT-mode kd scheduling, `forcerange = ±3.47 N·m`, `armature = 0.0038 kg·m²`, `damping = 0.12`, `frictionloss = 0.025`) | 
| **Driven Wheels (2 DOFs)** | 2x [SteadyWin GIM4305-10](https://www.google.com/search?q=https%3A%2F%2Faifitlab.com%2Fproducts%2Fsteadywin-gim4305-planetary-reducer-servo-motor) direct-torque hub motors at `foot_assembly` / `foot_assembly_2` driving `75 mm`-radius TPU tires (`324 mm` track width) | `class="gim4305_wheel"` `<motor>` torque actuators (`ctrlrange = ±3.47 N·m`) | 
| **Neck & Head Gimbal (4 DOFs)** | `neck_pitch`, `head_pitch`, `head_yaw`, `head_roll` using lightweight `55 g` [Feetech STS3215](https://www.google.com/search?q=https%3A%2F%2Fshop.babsco.com%2Ffeetech-sts3215-19-5kg-smart-servo-c001) servos to keep CoM low (`L = 0.192 m`) | `class="sts3215"` (`kp = 13.37`, `kv = 0.8`, `forcerange = ±3.23 N·m`, `armature = 0.027`, `damping = 0.56`, `frictionloss = 0.068`) | 
| **Compute & Power Bay** | `148 x 126 x 116 mm` `trunk_assembly` (`1.099 kg`) housing a [Raspberry Pi 5](https://www.google.com/search?q=https%3A%2F%2Fwww.ldlc.com%2Fen%2Fproduct%2FPB00574323.html) (`86 x 56 x 16 mm`), active cooler, Raspberry Pi AI HAT+ 26 TOPS (Hailo-8, `65 x 56.5 mm`, ~41 g incl. standoffs, on 16 mm standoffs above the Pi), PJRC Teensy 4.0 (`600 MHz` Cortex-M7, real-time 500 Hz balance loop + dual 1 Mbps CAN buses `CAN1` left / `CAN2` right via 2x `SN65HVD230`/`MCP2562FD` transceivers, ~8 g; or single-bus Adafruit Feather M4 CAN Express alternative), 6S2P (24V) Li-Ion battery at `z = -0.026 m`, and fast raw ICM-42688-P IMU wired over 24 MHz SPI to the Teensy (`pos="-0.108 0 0.0675"`). Added to both MJCFs by `tools/add_trunk_electronics.py`. | 3 voltage rails required for physical build: `24V (6S)` for GIM4305 CAN bus, `7.4V` buck for STS3215 TTL bus, `5.1V/5A` buck for Raspberry Pi 5 | 
| **Cameras (3x Camera Module 3)** | Stereo [Raspberry Pi Camera Module 3](https://www.google.com/search?q=https%3A%2F%2Fwww.raspberrypi.com%2Fproducts%2Fcamera-module-3%2F) pair (`left_eye_cam`, `right_eye_cam`, `70 mm` baseline, `67°` fovy) in the BDX binocular head + 1 chest nav cam (`trunk_nav_cam`, `75°` fovy, pitched `-70°` down) | Defined as `<camera>` elements in `head_assembly` and `trunk_assembly` | 

## 5. Active Control Architecture (`simulate_wheeled_duck.py`)

* **LQR Wheel Balancing**: Linearizes the 4-state Wheeled Inverted Pendulum (`[x_pos, x_vel, pitch, pitch_rate]`) via `scipy.linalg.solve_continuous_are`, with `Q = diag(15, 60, 280, 32)` (`LQR_Q`), yielding `K = [-3.873, -10.205, -38.707, -8.345]`. Wheel odometry adds the measured leg fold (`hip_pitch + knee`), since the wheel encoder measures rotation relative to the foot.

* **Balance trim**: Balance lean = learned trim (slow integrator, starts at `-0.0758 rad`, frozen while settling) + posture feedforward `posture_trim(fold)`, fitted against the *measured* leg fold (legs sag ~0.02–0.05 rad under load). Each 1° of trim error parks the robot ~25 cm off its hold point.

* **Posture & Turn Coordination**: Nominal crouch `hip_pitch = -0.50 rad`, `knee = +1.00 rad` (`base z = 0.3561 m`). Crouch range `-0.25 … +0.24` (`MAX_CROUCH`: keeps the sagging knee ~0.09 rad clear of its 90° joint limit). Includes active hip-roll leveling, centrifugal turn banking (`turn_bank`) and horizon-stabilized stereo head camera tracking. (The old hip-pitch balance assist was removed: it caused a saturated 2.5 Hz limit cycle.)

* **Live Keyboard Controls**:

  * `Up/Down Arrow` or `I/K` or `W/S`: Forward/reverse speed (`±0.15 m/s` steps, ramped at `1.2 m/s²`)

  * `Left/Right Arrow` or `J/L` or `A/D`: Yaw steering (`±15°` heading steps)

  * `Spacebar`: Full brake & station hold (S-curve decel; hold point follows the robot until it has stopped)

  * `C`: Jump (crouch → thrust → flight with wheels as reaction wheel → soft landing)

  * `Z` / `X`: Crouch deeper / stand taller (`Δknee = -2 * Δhip_pitch`)

  * `U` / `O`: Pan BDX head cameras left / right

  * `Q` / `E`: Raise / lower the antennas (9 g micro servos, `ctrl[14:16]`)

  * `P`: Toggle the head projector light

  * `R`: Reset to `home_balanced` keyframe

## 6. Next Engineering Milestones

1. Optionally download `.stl` meshes from `apirrone/Open_Duck_Mini` into `./assets/` and uncomment `<mesh ... scale="1.35 1.35 1.35"/>` in `open_duck_wheeled_v2.xml`.

2. ~~Add sim-to-real domain randomization (IMU noise, `2-4 ms` CAN bus latency, push-recovery benchmarking) or a jumping trajectory.~~ Done (2026-10-14): `SimToReal`, `test_scenarios.py --realism N`, `C` jump. Benchmark on 12 randomized robots: stand 12/12, drive/turn/brake 12/12, brake settle 7/12, crouch 5/12, push ±20 N 12/12, ±30 N 9/12 (falls only on μ < 0.93 floors), ±40 N 0/12, jump 12/12, jump while driving 8/12.

   * **Push-recovery limit is traction, not control**: max tire force ≈ μ·m·g ≈ 48 N at μ = 1.25, and a 40 N shove uses most of it. Stronger hips, traction control, torque slew limits and a leg-extension reflex were all tried and didn't help; only more grip (μ ≥ 1.6 with damped legs) survived 40 N. Realistic target on wood/tile (μ ≈ 0.8–1.0) is ~30 N. Hardware levers: grippier/softer tires, lower CoM.
   * **Knee heat (`thermal` scenario)**: stock GIM4305 knees run at 80% of their 1.0 N·m continuous rating standing and 115% crouched — they would overheat.
   * **GIM6010-8 hip-pitch + knee upgrade evaluated (2026-10-14)**: model in `open_duck_wheeled_v2_gim6010.xml` (+0.95 kg, CoM 13 mm lower). Specs from the SteadyWin GIM6010-8 manual rev1.2: Ø80 × 44 mm incl. driver, 8× M3 on Ø74 bolt circle both faces, output 6× M3 + 3× Ø4 pins, Ø47 spigot; 5 N·m rated @120 rpm, 11 N·m stall, 0.47 N·m/A; MIT mode supported (kp in N·m/turn). Rotor inertia unpublished — armature 0.0048 from the GB6010's 748 g·cm² × 8². Sim should apply the torque-speed curve T ≤ min(11, (285 − rpm)/17): the jump drives joints to ~185 rpm. Needs its own tuning: derived LQR body params, trim `-0.0952` + posture fit `(-0.1588, -0.0566)`, `THRUST_WHEEL_MODE="zero"`, `JUMP_THRUST_TIME=0.05`. Results vs stock on 12 randomized robots: knee heat 20–29% of rating, ±30 N pushes 12/12 (stock 9/12), jump 27 mm with the torque-speed curve (stock 16–17 mm, also curve-free so optimistic) 12/12, crouch 3/12 (stock 9/12, mostly 8.5–9.6 cm vs an 8 cm limit).
   * **Control architecture (2026-10-14)**: `--realism` now models the Feather M4 CAN running the balance loop (`--arch mcu`, 0.3–0.8 ms) vs Linux on the Pi (`--arch pi`, 2–4 ms + scheduling stalls), the IMU (`--imu bno055` = fused orientation and gyro at 100 Hz, assumed from the BNO055 fusion-mode rate — unverified; `--imu fast` = raw 1 kHz-class IMU filtered on the MCU) and the single classic-CAN bus (`--leg-rate`; GIM drivers are classic CAN, 1 Mbps max). Bus load: wheels 500 Hz + legs 100 Hz = 50% (legs 200 Hz = 72%, all at 500 Hz = 136%). Results (gim6010, 12 randomized robots): MCU vs Pi — drive/turn/brake 10/12 vs 2/12, brake settle 7/12 vs 0/12; fast IMU on the MCU — drive 12/12, brake 9/12, ±30 N 12/12; legs at 200 Hz gave no gain over 100 Hz. Stock gim4305 with the 100 Hz BNO055 gyro: jump while driving drops to 8/12.
   * **Pre-order checks (2026-10-14)**:
     * *Motor drivers*: SteadyWin ships two driver families — ODrive-based (GIM6010-8 manual: `odrivetool`, CAN ID = node_id×32 + cmd) and SteadyWin's own (Motor Wizard: CAN ID + master ID, SteadyWin or MIT protocol). Order all 10 GIM motors with the same family; mixing works on one bus only with non-overlapping IDs.
     * *Heat (`thermal`, all GIM motors, sustained windows)*: hip roll (GIM4305) is the problem — 68% of rated just standing, 72–83% while weaving, because the wheel contact sits 60 mm outboard of the roll axis. Fix in CAD (roll axis above the contact patch) or upgrade hip roll to GIM6010-8 (+476 g). Wheels 40–49%, knees 29%, hip yaw 14%, hip pitch 4%.
     * *Power (`tools/power_budget.py`)*: standing ~35 W, cruising ~52 W, mixed use ~43 W → a 90 Wh 6S pack gives ~100 min. Jump thrust pulls ~80 A for ~50 ms (4 knees/hips near stall): use high-drain cells / 6S2P, bus capacitance, or cap the jump thrust current. 5 V rail peak ~3.8 A (Pi 5 + Hailo-8 at max) → 5.1 V/5 A buck is OK.
     * *IMU*: fast raw IMU (ICM-42688-P: 2.8 mdps/√Hz, up to 32 kHz ODR, SPI 24 MHz; or BMI088) on SPI to the Feather, filtered there; the sim's fast-IMU noise (0.3 °/s RMS) is conservative.
     * *Tires*: grip is the push-recovery limit. Soft cast urethane (≈Shore 45A) treads on printed 150 mm hubs; vendor μ on MDF ≈1.75 urethane vs 1.30 silicone vs 1.05 rubber (unverified). Measure on your floors (incline test, μ = tan θ) and set `RealismConfig.friction`.
   * **Build configuration (2026-10-14)**: hip roll also moved to GIM6010-8 (fixes its 72–83% heat: now 21%) and the battery to 6S2P (+0.42 kg vs the assumed 6S1P baseline). 5.79 kg total. Ideal sensors: every scenario passes, incl. ±40 N pushes (22° / 15° peak) and a 30–32 mm jump. 12 randomized robots: stand, drive, ±20–30 N pushes, jumps, thermal 12/12; ±40 N 9/12; brake settle 9/12; crouch 3/12. Heat: knees 37%, wheels 44%, hip roll 21%, hip yaw 16%. Power: ~40 W mixed use → ~214 min on a 180 Wh pack; jump thrust still ~80 A for ~50 ms. Note: at full crouch the heavier robot's knees sag to ~0.04 rad from their joint stop (was ~0.09).
   * **Open issues**: (a) on robots with a ±6–8 mm CoM offset the trim starts wrong and the learner (τ ≈ 7 s; faster gains oscillate) leaves 7–22 cm offsets after braking/crouching — on hardware, persist the learned trim between runs; (b) jump while driving still falls on 4/12 low-grip/latency robots (lands with ~60° lean).

3. Fork the Open Duck Mini v2 Onshape CAD, scale shells by `1.35x`, and adapt leg servo pockets to `Ø53 x 26 mm` GIM4305-10 flanges (`6x M2.5` bolt circle).