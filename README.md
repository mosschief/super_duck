# Super Duck (`mosschief/super_duck`)

**Super Duck** is a `1.35x` scaled (`56.7 cm` tall, `7.03 kg` CAD-ready build configuration), wheeled bipedal robot based on the [Open Duck Mini v2](https://github.com/apirrone/Open_Duck_Mini) BDX architecture. It combines **4x SteadyWin GIM6010-8** high-torque QDD actuators (`hip_pitch`, `knee`) and **4x [SteadyWin GIM4305-10](https://rcdrone.top/products/steadywin-gim4305-planetary-reducer-servo-motor)** actuators (zero-scrub `hip_roll` and driven `75 mm`-radius wheels; the hips are fixed mounts, with no hip yaw: steering is by wheel torque difference) with a split-rate compute stack: a **[Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/) + [Raspberry Pi AI HAT+ (26 TOPS)](https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html)** for vision/AI and a real-time **[PJRC Teensy 4.0](https://www.pjrc.com/store/teensy40.html)** running the deterministic `500 Hz` LQR balance loop over dual `1 Mbps` CAN buses. It carries the full Open Duck Mini expression package: moving antennas, LED eyes, head projector, speaker and microphone.

## Repository Structure & Tooling

| File / Script | Purpose |
| :--- | :--- |
| `open_duck_wheeled_v2_gim6010.xml` | **Default CAD-ready build model (`--robot gim6010`, `7.03 kg`, exported from the FreeCAD assembly: every part's mesh, mass and inertia, collision hulls, neck, cameras, IMU; `cad/freecad/export_sim.py`)**: 4x GIM6010-8 (`hip_pitch`, `knee`), 4x GIM4305-10 (inline zero-scrub `hip_roll`, `left/right_wheel`), fixed hip mounts (no hip yaw), 4x STS3215 (`neck/head`), 2x 9 g antenna servos, `6S2P` 21700 battery pack with BMS / fuse / anti-spark switch / e-stop, AI HAT+, Teensy 4.0 dual-CAN module, and the expression package |
| `open_duck_wheeled_v2.xml` | **Baseline model (`--robot gim4305`, `3.87 kg`)**: Original all-GIM4305-10 leg/wheel configuration with a `6S1P` battery pack (also with fixed hip mounts) |
| `simulate_wheeled_duck.py` | Interactive viewer / CLI entry point (`--robot`, `--realism`, `--headless`); re-exports the `duck/` package for the tests and tools |
| `duck/robot.py` | Physical parameters, `RobotProfile`s (`gim6010`, `gim4305`), wheeled-inverted-pendulum model and LQR gain synthesis |
| `duck/plant.py` | Simulation side: `SimToReal` domain randomization (`--realism SEED --arch mcu\|feather\|pi --imu fast\|bno055`), sensor readout, motor torque-speed curves, `step_robot` |
| `duck/controller.py` | Pure-Python controller (no MuJoCo; the part to port to the Teensy): `500 Hz` full-state LQR wheel-balancing, measured leg-fold odometry compensation, online pitch-trim learner (parked and cruising: payload, IMU mounting; plus a fast acceleration-based balance-lean estimate after leg moves, for trunk CoM offsets), a slope observer (momentum observer on wheel torque and odometry) feeding forward a slope's wheel torque and lean, running gyro-bias estimate, turn-rate limit, `2.5 m/s` top speed with a voltage-based speed cap (`2.43 m/s` at `22.2 V`, `2.30 m/s` at the `21 V` cutoff), a virtual wheel damper, low-battery parking at `21 V`, free-fall detection with a braced landing, bump detection (driving into a wall: IMU acceleration spike + trunk pitching back -> stop and back off `10 cm`), a fall cutoff (wheels off past `40°` of lean, re-armed once stood back up), quadratic posture feedforward (lean and lean rate), `C`-key jump state machine (with MIT-mode `kd` scheduling & airborne reaction-wheel attitude control) |
| `test_scenarios.py` | Automated ground-truth benchmark suite: `stand_still`, `drive_turn_brake`, `brake_settle`, `crouch_stand`, `push` (`±20 N`, `±30 N`, `±40 N`, 0.1 s, on the back of the trunk at `0.43 m`; it recovers `100 N` standing, `40 N` at `1.5 m/s`, `30 N` at `1.8 m/s`), `jump`, `jump_driving`, and `thermal` (sustained RMS torque vs. continuous motor rating) |
| `test_robustness.py` | Robustness suite (~100 scenarios, ~1.5 min on all cores, `--report` -> `reports/robustness.md`): slopes held and driven, ramps, door thresholds and curbs, uneven floor, low grip, turns and shoves at speed, battery sag (motor speed-torque curves at the pack voltage), e-stop at speed, CAN frame loss / driver timeouts / stale feedback, IMU mounting error / gyro drift / vibration, falls, driving into a wall (must stop and stand off it), drops, bad landings, a full head-range sweep, and a 20 min duty cycle (thermal, plus joint load cycles for fatigue -> `cad/fatigue_cycles.md`) |
| `tools/sensitivity_sweep.py` | One-factor-at-a-time sweep of the sim-to-real factors on `crouch_stand` / `brake_settle` -> `reports/sensitivity.md` (`-j N` workers) |
| `tools/export_fea_loads.py` | The worst joint wrenches of the hard scenarios as vectors in each printed leg part's frame -> `cad/fea_loads.json` |
| `tools/fea_printed_parts.py` | Linear static FEA of the printed leg parts under those loads (gmsh + scikit-fem, in `~/.venvs/fea`: `python3 -m venv ~/.venvs/fea && ~/.venvs/fea/bin/pip install -r requirements-fea.txt`; needs `cad/freecad/printed/` from a CAD build; ~1 h; `--printed DIR -k part` tries out STEP files from elsewhere and only prints the results) -> `cad/fea_report.md` |
| `tools/sysid.py` | Bench system identification (torque chirp log -> inertia, damping, Coulomb friction, gravity) to set the MJCF's motor parameters from the real robot; `selftest` checks it on synthetic logs from the sim |
| `tools/upgrade_motor.py` | Upgrades specified leg joints in an MJCF from `gim4305` (`150 g`, `3.47 N·m`) to `gim6010` (`388 g`, `11 N·m` stall / `5 N·m` rated) with exact cylindrical inertia shifts |
| `tools/battery_pack.py` | Configures trunk battery pack geometry and parallel-axis inertia (`6s1p` `~0.48 kg` `90 Wh` vs. `6s2p` `~0.90 kg` `180 Wh`) |
| `tools/add_trunk_electronics.py` | Folds the `41 g` AI HAT+ (26 TOPS) and `8 g` Teensy 4.0 Dual-CAN + IMU (or Feather M4 CAN) into `trunk_assembly` |
| `tools/add_expression_hardware.py` | Adds the Open Duck Mini expression package (hinged antennas on 9 g servos, WS2812B eye rings, projector LED + spot light, I2S mic, speaker + amp) and the battery protection parts (BMS, fuse, anti-spark switch, e-stop, peripheral 5 V buck), folding `37 g` into `head_assembly` and `130 g` into `trunk_assembly`; adds wheel `touch` sensors |
| `tools/derive_lqr_params.py` | Computes lumped body mass, CoM height above wheel axle, and pitch inertia (excluding wheels) from any MJCF for `RobotProfile.params` |
| `tools/fit_balance_trim.py` | Runs parallel `80 s` holds across crouch depths (`-0.24..+0.24 rad`) to fit `initial_pitch_trim` and `posture_coeffs` against measured leg sag |
| `tools/check_cad_clearances.py` | Sweeps every operating pose and full joint range for self-contact of the collision hulls (the contacts the sim will produce), ground clearance and hip-roll scrub torque → `cad/clearance_report.md` |
| `cad/freecad/export_sim.py` | Exports the FreeCAD assembly into the `gim6010` MJCF (run with `freecadcmd` after `build_super_duck.py`): every part as a mesh (`assets/cad/`), collision hulls, each body's mass / CoM / inertia from a walls + infill print model, neck body positions, antenna axes, cameras, projector light and IMU site. The mass tools above (`upgrade_motor`, `battery_pack`, `add_*`) predate the CAD and only apply to the `gim4305` baseline now |
| `tools/export_cad_spec.py` | Exports the CAD hardpoints (joint anchors/axes/ranges, body frames, part placement, cameras, key dimensions) → `cad/hardpoints.md/.json`, and renders `cad/purchased_parts.md` |
| `tools/export_design_loads.py` | Force and moment through every joint (axial, radial, about/across the axis) for driving, jumps, pushes and a fall, with GIM4305 bearing checks → `cad/design_loads.md/.json` |
| `tools/fetch_vendor_cad.sh` | Downloads the SteadyWin, Raspberry Pi and camera drawings / STEP models into `cad/vendor/` (git-ignored) |
| `docs/design_log.md` | Dated history of design decisions and evaluations (what was tried, what was measured, open issues at the time) |
| `cad/` | **CAD spec locked from the sim**: hardpoints, real purchased-part dimensions with sources, clearance report, design loads, and the open issues for CAD ([`cad/README.md`](cad/README.md)) |
| `tools/power_budget.py` | Simulates copper ($1.5 I^2 R$) + mechanical power across standing, weaving cruise, crouching, and jumping (`~35 W` mixed average, `~251 min` runtime on `180 Wh` `6S2P`, `~80 A` `50 ms` jump peak) |

## Hardware Shopping List & Bill of Materials (Build Configuration: `gim6010`)

### 1. Compute, Real-Time Control & Sensing

| Qty | Component | Key Specs & Role in Super Duck | Reference Link |
| :--- | :--- | :--- | :--- |
| **1x** | **Raspberry Pi 5 (8GB or 16GB)** | `86 x 56 x 16 mm`, `46 g`; runs high-level navigation, stereo vision, and telemetry | [Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/) |
| **1x** | **Raspberry Pi Active Cooler** | Anodized aluminum heatsink + PWM blower fan (fits under the AI HAT+ on `16 mm` standoffs) | [Raspberry Pi 5 Accessories](https://www.raspberrypi.com/products/raspberry-pi-5/) |
| **1x** | **40 x 40 x 10 mm 5 V PWM fan** *(e.g. Noctua NF-A4x10 5V PWM)* | Body exhaust fan on the rear bulkhead, blowing out through the tail's side vents (body heat ~10 W typical / ~20 W peak). 5 V from the peripheral buck; `25 kHz` PWM from Teensy pin `5` through a `2N7002` (the fan's PWM input is open-drain, pulled up to 5 V, and the Teensy isn't 5 V tolerant), tach on pin `6` with a `10 kΩ` pull-up to `3.3 V` | |
| **1x** | **Raspberry Pi AI HAT+ (26 TOPS)** | Hailo-8 NPU accelerator (`65 x 56.5 mm`, `~41 g` w/ standoffs) over PCIe Gen 3 ribbon (`2.5 W` typ / `8.65 W` max) | [Raspberry Pi AI HAT+ (26 TOPS)](https://www.sparkfun.com/raspberry-pi-ai-hat-26-tops.html) |
| **1x** | **PJRC Teensy 4.0** *(or Teensy 4.1)* | **Recommended Real-Time MCU (`--arch mcu`)**: `600 MHz` ARM Cortex-M7 w/ 64-bit FPU (`35 x 18 mm`, `~5 g`); runs the `500 Hz` LQR loop with `0.25-0.55 ms` latency and **splits the 8 GIM motors across 2 independent 1 Mbps CAN buses (`CAN1` & `CAN2`)**, cutting bus load from `44%` to `22%` at `100 Hz` legs (or `55%` with every motor at `500 Hz`) | [PJRC Teensy 4.0](https://www.pjrc.com/store/teensy40.html) / [Adafruit](https://www.adafruit.com/product/4323) |
| **2x** | **3.3V CAN Transceiver Breakouts (`SN65HVD230`)** | Keep the board's `120 Ω` terminator: the Teensy is one end of each bus. (An `MCP2562FD` also works but needs `5 V` on `VDD` and `3.3 V` on `VIO`, and its own `120 Ω`.) Converts Teensy `CAN1` (pins `23/22`, Left Leg + Wheel) and `CAN2` (pins `0/1`, Right Leg + Wheel) logic into differential `CAN_H / CAN_L` (or use a pre-assembled [Copperhill Teensy Triple CAN-FD carrier](https://copperhilltech.com/blog/the-ultimate-guide-to-teensy-microcontrollers-history-specs-can-bus-applications/)) | [PJRC Teensy CAN Specs](https://www.pjrc.com/teensy/techspecs.html) |
| **1x** | **ICM-42688-P (or BMI088) 6-Axis SPI IMU Breakout** | **Recommended Fast Raw IMU (`--imu fast`)**: Up to `32 kHz` ODR, `2.8 mdps/√Hz` noise, wired to Teensy `SPI0` (`24 MHz`) for `500 Hz-1 kHz` on-MCU complementary/Mahony filtering (solves the `100 Hz` BNO055 fusion lag during jumping & braking) | [SimToReal IMU Benchmark (`GEMINI.md`)](./GEMINI.md) |
| **2x** | **Raspberry Pi Camera Module 3** *(Standard)* | `25 x 24 x 11.5 mm`, 12 MP Sony IMX708; stereo eyes (`70 mm` baseline in BDX head) on the Pi 5's two CSI/DSI ports (`CAM/DISP 0` and `1`), which keeps the pair hardware-synchronizable | [Raspberry Pi Camera Module 3](https://www.raspberrypi.com/products/camera-module-3/) |
| **1x** | **[Arducam UB0234](https://welectron.com/Arducam-UB0234-2MP-Wide-Angle-USB20-Camera-Board-with-M12-Lens) USB UVC Wide-Angle Camera** *(32 × 32 mm board, 105° FOV)* | Downward chest nav cam (`trunk_nav_cam`). The Pi 5 has only two camera connectors, both used by the eyes; a USB camera avoids a CSI multiplexer, which would break synchronized stereo capture | e.g. Arducam wide-angle USB camera modules |
| **1x** | **microSD card, 64 GB+, high-endurance A2** | Pi 5 boot and storage (the AI HAT+ takes the PCIe port, so no NVMe) | |
| **1x** | **USB cables: short right-angle USB-A to micro-USB (Teensy), short USB lead (nav cam)** | Teensy on the rear shelf and the nav cam behind the front cap, both to the Pi's USB ports | |
| **1x** | **Waveshare Bus Servo Adapter (A)** | UART/USB-to-TTL serial bus interface for the 4x neck/head [Feetech STS3215](https://shop.babsco.com/feetech-sts3215-19-5kg-smart-servo-c001) gimbal servos | [Feetech STS3215](https://shop.babsco.com/feetech-sts3215-19-5kg-smart-servo-c001) |

| **1x** | **Xbox Wireless Controller (Bluetooth)** | Teleop, paired to the Pi 5's built-in Bluetooth; same controller and button layout as Open Duck Mini (see [Gamepad Mapping](#gamepad-mapping)) | |

> **Single-Bus No-Solder Alternative (`--arch feather`)**: If you want an all-in-one board with an onboard CAN transceiver and screw terminals (no external transceiver breakouts), you can use the **[Adafruit Feather M4 CAN Express (ATSAME51)](https://thepihut.com/products/adafruit-feather-m4-can-express-with-atsame51)** (`120 MHz` Cortex-M4F, `50.8 x 22.8 mm`, `5.5 g`). All 8 GIM motors share its single `1 Mbps` classic CAN bus (`44%` bus utilization at `500 Hz` wheels + `100 Hz` legs).

### 2. Actuators & Drivetrain (14 Active DOFs — `gim6010` Build Config)

> **Important Ordering Note**: SteadyWin ships GIM motors with two driver families: **ODrive-based** (`odrivetool`, `CAN ID = node_id * 32 + cmd`) and **SteadyWin MIT/Motor Wizard** (`CAN ID + master ID`). **Order all 8 GIM motors (4x GIM6010-8 + 4x GIM4305-10) with the same driver family** so they share the same CAN protocol on the bus.

| Qty | Motor Model | Assigned Joints | Specs & Thermal Benchmark (`test_scenarios.py -k thermal`) |
| :--- | :--- | :--- | :--- |
| **4x** | **SteadyWin GIM6010-8 QDD Planetary Actuator** | `left/right_hip_pitch` (2x) + `left/right_knee` (2x) | `Ø80 x 44 mm` incl. driver, `388 g`, `8:1` gear, `5.0 N·m` continuous / `11.0 N·m` stall torque, `0.47 N·m/A`, `8x M3` on `Ø74 mm` bolt circle; keeps sustained knee RMS torque at **`34%` of rating** |
| **4x** | **SteadyWin GIM4305-10 QDD Planetary Actuator** | `left/right_hip_roll` (2x), `left/right_wheel` (2x) | `Ø53 x 33 mm` overall incl. driver (`26 mm` housing + `6 mm` driver boss + `1 mm` output step; listings quote only the housing), `150 g`, `10:1` gear, `1.0 N·m` continuous / `3.47 N·m` stall torque, `300-366 RPM` ([RCDrone](https://rcdrone.top/products/steadywin-gim4305-planetary-reducer-servo-motor)); inboard wheel routing (`4 mm` scrub offset) drops standing `hip_roll` load to **`10%` (`0.10 N·m`)**, saving **`476 g`** and eliminating hip canister clashes |
| **4x** | **Feetech STS3215 Serial Bus Servo (7.4V)** | `neck_pitch`, `head_pitch`, `head_yaw`, `head_roll` | `45.2 x 24.7 x 35 mm`, `55 g`, `19.5 kg·cm` (`1.91 N·m`), 12-bit magnetic encoder ([BABSCO](https://shop.babsco.com/feetech-sts3215-19-5kg-smart-servo-c001)); keeps upper head mass light (`0.742 kg` total neck+head incl. expression package) |
| **2x** | **9 g Micro Servo (SG90 / MG90S class, 5V)** | `left_antenna`, `right_antenna` | `23 x 12 x 29 mm`, `9 g`, `~0.18 N·m` stall, `±45°` via `1-2 ms` PWM at `50 Hz` (Open Duck Mini `antennas.py`); `class="micro_servo_9g"` position actuators at `ctrl[14:16]` |

### 3. Battery, Power Tree & High-Grip Tires

| Qty | Component | Specification & Sizing Rationale (`tools/power_budget.py`) |
| :--- | :--- | :--- |
| **1x** | **6S2P 21700 High-Drain Li-Ion Pack (`21.6V` nom / `25.2V` max, `~180-200 Wh`)** | `12x` high-drain 21700 cells (e.g., Molicel P42A / P45B, `45A` continuous per cell = `90A` pack continuous), `73 x 89 x 68 mm` (4 x 3 cells + wrap, `cad/purchased_parts.md`), `~0.90 kg` mounted at `z = -0.026 m` in `trunk_assembly`. Provides **`~251 min` runtime** at `35 W` mixed load and handles the **`~80 A` (`50 ms`) jump thrust current** without voltage sag |
| **1x** | **[Pololu D36V50F5](https://www.pololu.com/product/4091) 5 V step-down (Pi)** | Powers Raspberry Pi 5 (`5.0 W` typ / `8.8 W` peak) + AI HAT+ (`2.5 W` typ / `8.65 W` peak) + cameras + Teensy 4.0 (`~3.8 A` peak on 5V rail). `25.4 × 25.4 × 9.5 mm`, 3× M2; `~7.3 A` at `24 V` in at room temperature (Pololu graph). It gives `5.0 V`, not `5.1`: keep the Pi's `20 AWG` feed short |
| **1x** | **[Pololu D36V50F7](https://www.pololu.com/product/4093) 7.5 V step-down (head servos)** | Powers the 4x Feetech STS3215 neck/head gimbal servos (`2.0 W` typ / `6.0 W` peak). `~6.4 A` at `24 V` in at room temperature: all four servos stalled at once (`~11 A`) would trip its thermal shutdown |
| **1x** | **[Pololu D36V50F5](https://www.pololu.com/product/4091) 5 V step-down (peripherals)** | Separate rail for the antenna servos, eye LED rings, projector, audio amp and body fan, so servo and LED current spikes stay off the Pi 5's supply (already at `~3.8 A` peak). Peak `~13 W` (`~2.7 A`) with both antenna servos stalled |
| | **All three bucks** | No published ambient rating, and the bay reaches `45–50 °C` at peak: expect less than the room-temperature figures (thermal shutdown protects them) |
| **1x** | **[Heltec 6S/7S 80 A split-port BMS](https://heltec-bms.com/product/6s-7s-bms-30a-60a-80a-150a-200a-300a-350a-balance-battery-protection-board/) (3.7 V Li-ion, set for 6S)** | Over-discharge / over-current / short protection on the discharge path and cell balancing (`83 mA`). `80 A` continuous, `200 A` current limit, `83 × 55 × 8 mm`, on edge across the bay just ahead of the pack. **Ask Heltec for the over-current delay** (it must pass the `~80 A`, `50 ms` jump bursts) and the charge-port rating (`≥ 4 A`). Separate-port: charge through `C−`, discharge through `P−`. Balancing in the BMS means the charger needs no balance lead |
| **1x** | **GX16-2 Aviation Panel Socket + Mating Plug** | Charge input on the tail end face (keyed, latching; contacts ~5 A). Wire the plug onto the charger's output lead, observing polarity |
| **1x** | **6S Li-Ion Charger, `25.2 V` CC/CV, `3–4 A`** | External brick. `3–4 A` keeps the GX16 contacts below their rating; `~2.5–3 h` from empty for the `180 Wh` pack. The BMS balances the cells |
| **1x** | **5 A Blade Fuse + Inline Holder (charge lead)** | Between the GX16 socket's `+` pin and pack `+`, matched to the GX16 contacts' `~5 A`. The `40 A` main fuse is too large to protect the thin charge wiring. The charge lead joins before the anti-spark switch, so the robot charges switched off |
| **1x** | **Schottky Diode, ≥ 10 A, ≥ 40 V (e.g. SR1040 / MBR1045), in the charge lead** | Without it the GX16's exposed pins sit at pack voltage whenever the robot is unplugged. It drops `~0.4 V`, so the charger ends at `~24.8 V` (about 95% charge); for a full charge, use an ideal-diode module (LM74700 class) instead |
| **1x** | **40 A Blade Fuse + Inline Holder** | Main fuse on the battery positive, before everything else. A slow-blow `40 A` fuse rides through `80 A` for `50 ms`; it is there for wiring shorts |
| **1x** | **[Flipsky Antispark Switch Pro V3.0](https://flipsky.net/collections/anti-spark-switch/products/flipsky-antispark-switch-pro-with-aluminum-pcb-case-200a-for-electric-skateboard-ebike-scooter-robots), heatsink version (`ANSPFS14SH`)** | Main on/off. Avoids the inrush spark from the 8 GIM drivers' input capacitors. `42.2 × 33 × 26.6 mm`, 3–14S, `60 A` continuous (`200 A` max), 12 AWG leads; on the tail shelf. It switches the **battery negative**, so nothing downstream may reach pack `−` another way. Its latching button (S common, `+` = NO, `−` = NC; pressed = on) is replaced by the e-stop's two contacts: join one side of each to `S`, the e-stop's **NC** to `+` and its **NO** to `−`. E-stop released = on, pressed = off. Bench-test that before trusting it |
| **1x** | **Power distribution: 30 × 10 mm copper bus bars (+ / −), or a small PDB, ~$10** | On the tail shelf after the anti-spark switch. Feeds both leg harnesses, the 3 bucks and the bulk capacitors |
| **1x** | **12 AWG silicone wire + XT60 pair (pack lead)** | Pack → BMS → fuse → anti-spark switch → bus bars, sized for the `~80 A` bursts. Each leg's harness is `16 AWG` (`~40 A` for `50 ms` per leg) |
| **1x** | **Branch fuses: 5 A mini blade on the buck feed, 20 A on each leg feed** | The `40 A` main fuse won't blow for a short in a thin buck lead or one leg harness |
| **2x** | **1000 µF 35 V low-ESR electrolytic capacitors** | Across the bus bars. Braking and landing regen pumps the bus up when nothing absorbs it (BMS charge FET open, or the BMS tripped), and the GIM4305 drivers are rated `36 V` max. Also check that the BMS passes reverse (charge) current through `P−`; a common-port BMS does. The GIM6010 drivers have a brake-resistor port (manual §2.4.7) if testing shows overshoot |
| **1x** | **[RAFI LUMOTAST 22](https://www.digikey.com/en/product-highlight/r/rafi/lumotast-22-emergency-stop-push-buttons) latching e-stop, `1.15.105.032/0000` (NC + NO; `.031` adds a status light)** | On the back of the trunk: `Ø22.3` cutout, `17.9 mm` behind the panel, 0.8 mm tabs, twist to release. Confirm on the Digi-Key page that it has the NO contact (the switch wiring above needs NC + NO); Digi-Key showed 0 in stock (Oct 2026). Cuts all power, the Pi included: shut the Pi down from the gamepad first (an e-stop cut can corrupt the microSD; a later option is to let the e-stop cut motor power only) (the robot will drop: use the gamepad for a soft stop). Firmware to do: have the Teensy read battery voltage from the GIM drivers over CAN, and port the sim's low-battery logic: below `3.5 V/cell` (`21 V`, filtered over 2 s) the robot stops, crouches and refuses to drive or jump, before the BMS cuts power. In the sim a `40 N` shove still passes at `21.1 V` and used to knock it over at `19.8 V` |
| **2x** | **Cast Urethane Tires (Smooth-On VytaFlex 40, Shore 40A) on `150 mm` (`75 mm` radius) Printed Hubs** | Push recovery is traction-limited ($F_{\max} \approx \mu m g$), so the tires are soft cast urethane: a `6 mm` crowned tread, `32 mm` wide, cast in a printed mold onto a PETG / PA-CF hub that bolts to the GIM4305 output (`3x M3` on `Ø27`). Hub, mold, casting steps and the grip test: [`cad/tires.md`](cad/tires.md). Also buy a urethane mold release (e.g. Smooth-On Ease Release 200) |

### 3a. Wiring & Connectors

| Qty | Component | Role & Notes |
| :--- | :--- | :--- |
| **8x** | **Amass XT30(2+2)-F pigtails** | The GIM6010 drivers' power + CAN connector (XT30PB(2+2)-M on the driver); one per motor drop. The GIM4305's connector is a different 6-pin "DC IN & CAN" part: confirm it on arrival and buy its mate |
| **1x** | **Leg harness wire: 16 AWG silicone (red / black), 24–26 AWG twisted pair (CAN), 6 mm expandable PET braid, heat-shrink** | One daisy-chained bundle per leg (`cad/freecad/README.md`, "Leg harness"): solder T-splices at each motor's drop, heat-shrunk, inside the braid |
| **2x** | **Rubber grommets, Ø14 panel hole, Ø9 bore** | Where each leg harness enters the body side wall |
| **2x** | **Camera Cable "Standard–Mini", 500 mm** | Eye cameras (15-pin) to the Pi 5's 22-pin mini CSI ports; 200 mm doesn't reach through the neck |
| **1x** | **Head harness: 12-core 28 AWG high-flex cable, ~0.6 m, + JST-GH connectors** | Pi / Teensy to the head: 5 V, GND, eye-ring data, projector gate, 2 antenna PWM lines, mic I2S `SCK` / `WS` / `SD` + `3.3 V`. The STS3215 bus runs beside it. Leave a service loop round the yaw joint (`±94.5°`, so no slip ring). `33 Ω` series resistors on the I2S lines at the Pi end, each paired with a ground |

### 3b. Recommended Extras

| Qty | Component | Role & Notes |
| :--- | :--- | :--- |
| **1x** | **INA226 (or INA228) current / voltage monitor + 2 mΩ shunt** | On the pack lead, read by the Teensy over `I2C0` (pins `18/19`): battery voltage and current even with the motors off (today the only reading is the GIM bus voltage over CAN) |
| **1x** | **Piezo buzzer (3.3 V, active)** | On Teensy pin `21`: low-battery and fault alarms while the Pi is down |
| **1x** | **USB Bluetooth / Wi-Fi dongle (optional)** | Only if the Xbox controller link is poor: the Pi 5's antenna is under the HAT, above the pack |

### 4. Expression Package (Open Duck Mini Parity)

Everything in the Open Duck Mini expression package (its BOM and [`Open_Duck_Mini_Runtime`](https://github.com/apirrone/Open_Duck_Mini_Runtime) `expression_features`: eyes, projector, antennas, speaker, microphone, camera), scaled to this robot. Added to both MJCFs by `tools/add_expression_hardware.py`.

| Qty | Component | Role & Notes |
| :--- | :--- | :--- |
| **2x** | **WS2812B 12-LED Ring (37 mm OD)** | Eye LEDs behind the existing eye bezels, around the camera lenses (Open Duck Mini uses plain LEDs; RGB rings add blink plus status colors such as low battery). Cap brightness at ~35% in firmware: full white is `7.2 W` |
| **1x** | **1 W LED (15 × 12 mm board) + Small Reflector + AO3400 MOSFET + 6.8 Ω 1 W resistor** | Head projector below the eyes (Open Duck Mini: LED + reflector on one GPIO). The MOSFET must be fully on at a `3.3 V` gate (AO3400 class, not IRLZ44N); `6.8 Ω` sets `~270 mA` from `5 V` (or use a `350 mA` constant-current driver). Modeled as the `projector` spot light; `P` toggles it in the sim |
| **1x** | **MAX98357A I2S Class-D Amplifier** | Same amp as Open Duck Mini; the Pi 5 has no analog audio output |
| **1x** | **[Adafruit 3968](https://www.adafruit.com/product/3968) 40 mm 4 Ω speaker (3 W, 5 W max)** | `Ø40 × 20 mm`, no tabs: clamped by its rim under the grille on the tail's top face. Plays the Open Duck Mini sound set |
| **1x** | **INMP441 I2S MEMS Microphone** | Top front of the head; shares the Pi's I2S bus with the amp (separate data pin) |
| **1x** | **74AHCT125 Quad Level Shifter** | Lifts the Teensy's `3.3 V` WS2812B data line to `5 V` (WS2812B needs `≥ 3.5 V` high at `5 V` supply) |
| **1x** | **10 kΩ Resistor + 330 Ω Resistor** | Projector MOSFET gate pull-down (keeps it off while the Teensy boots) and LED data-line series resistor |
| **1x** | **2N7002 MOSFET + 2x 10 kΩ resistors** | Fan PWM driver and the fan tach pull-up (pinout below) |

### Gamepad Mapping

Open Duck Mini's Xbox layout, with the walking-specific buttons reassigned to the wheeled robot's functions:

| Input | Open Duck Mini | Super Duck | Sim key |
| :--- | :--- | :--- | :--- |
| Left stick | Walk / turn | Drive speed / heading | Arrows, `WASD` |
| Right stick | Head (`Y` to toggle) | Head pan / tilt | `U` / `O` |
| Left / right trigger | Left / right antenna | Left / right antenna | `Q` / `E` (both) |
| `A` | Pause / unpause | Brake and hold position (soft stop) | `Space` |
| `B` | Play a random sound | Play a random sound | |
| `X` | Projector on / off | Projector on / off | `P` |
| `Y` | Head control mode | Jump | `C` |
| `LB` | Sprint (walk frequency) | Unassigned | |
| D-pad up / down | | Stand taller / crouch | `X` / `Z` |

## Teensy 4.0 Wiring Pinout (CAN, IMU, Head Servos, Expression)

The Teensy runs the antennas, eye LEDs and projector as well as the balance loop: their timing is hardware-driven,
and they keep showing status (low battery, fallen, Pi not connected) even when the Pi is down. Audio stays on the Pi.

- **Raspberry Pi 5 <-> Teensy 4.0 (`50-100 Hz` High-Level Teleop & Vision Link)**:
  - USB-A to Micro-USB cable (`480 Mbps` native USB CDC serial, `< 0.2 ms` latency).
  - The Teensy is powered from the Pi buck's `5 V`: **cut its VUSB–VIN pad** so the USB `5 V` from the Pi doesn't back-feed it.
- **Pi 5 power**: `5.1 V` from its buck into the GPIO `5 V` / `GND` pins (`20 AWG`, through the HAT's stacking header). Without USB-PD the Pi limits USB to `600 mA` and warns about the supply: set `PSU_MAX_CURRENT=5000` in the bootloader EEPROM (`rpi-eeprom-config --edit`) and `usb_max_current_enable=1` in `config.txt`.
- **Teensy 4.0 <-> 8x GIM6010-8 / GIM4305-10 Motors (`500 Hz` Dual 1 Mbps Classic CAN)**:
  - **`CAN1` (`CRX1` Pin 23, `CTX1` Pin 22 -> Transceiver #1)**: Left Leg (`left_hip_roll`, `left_hip_pitch`, `left_knee`) + `left_wheel` (CAN IDs `1-4`, `120 Ω` termination at both ends: the transceiver board at the Teensy and `left_wheel`).
  - **`CAN2` (`CRX2` Pin 0, `CTX2` Pin 1 -> Transceiver #2)**: Right Leg (`right_hip_roll`, `right_hip_pitch`, `right_knee`) + `right_wheel` (CAN IDs `5-8`, `120 Ω` termination at both ends: the Teensy and `right_wheel`).
  - Mid-bus motors unterminated: turn the GIM6010 driver's `120 R` switch **off** on both hip-pitch and knee motors. The GIM4305s at the bus ends may need an inline `120 Ω` across `CAN_H / CAN_L` if their drivers have none: check on arrival.
- **Teensy 4.0 <-> ICM-42688-P Raw IMU (`500 Hz - 1 kHz` SPI Attitude Loop)**:
  - **`SPI0`**: `CS` Pin 10, `MOSI` Pin 11, `MISO` Pin 12, `SCK` Pin 13 + data-ready interrupt (`INT1` on Pin 9). *(If using a BNO055 instead, wire to `I2C0` on `SDA0` Pin 18 / `SCL0` Pin 19).*
- **Teensy 4.0 <-> 4x Neck/Head STS3215 Servos (`100 Hz` Camera Horizon Gimbal)**:
  - **`Serial3` (`RX3` Pin 15, `TX3` Pin 14)** -> Waveshare Bus Servo Adapter (A).
- **Teensy 4.0 <-> Expression Package (peripheral `5 V` rail, common ground with the Teensy)**:
  - **Antenna servos**: Pin `2` (left), Pin `3` (right) -> servo signal, `50 Hz`, `1-2 ms` pulses (`PWMServo` or `analogWriteFrequency` + `analogWrite`; Open Duck Mini `antennas.py` mapping, right side mirrored). Servo power from the peripheral `5 V` buck, not the Teensy.
  - **Eye rings**: Pin `8` (`TX2`) -> 74AHCT125 (`5 V` supplied) -> `330 Ω` -> left ring `DIN`; left ring `DOUT` -> right ring `DIN` (24 LEDs, one chain). Drive with **`WS2812Serial`** (DMA over `Serial2`): FastLED / Adafruit NeoPixel disable interrupts for `~0.7 ms` per update, which would jitter the `500 Hz` balance loop.
  - **Projector**: Pin `4` -> logic-level N-MOSFET gate (`10 kΩ` pull-down to GND) switching the 1 W LED's low side from the `5 V` rail (with its current-limit resistor). PWM on pin 4 dims it.
  - **Body fan**: Pin `5` -> `2N7002` gate (`10 kΩ` pull-down) pulling the fan's PWM line low, `analogWriteFrequency(5, 25000)`; tach on pin `6` (`10 kΩ` pull-up to `3.3 V`).
  - **Recommended extras**: INA226 on `I2C0` (`SDA0` `18` / `SCL0` `19`), buzzer on pin `21`.
  - **Free for later**: Pins `7`, `16`, `17`, `20` (`I2C1` on `16/17`).
- **Pi 5 -> Teensy expression commands** (same USB serial link as teleop, `~50 Hz`): antenna targets, eye mode / color / blink, projector on / off / brightness. The Teensy overrides them for status: eyes red on low battery (GIM bus voltage over CAN), amber blink when the Pi link times out (`> 0.5 s`), antennas back on a fall.

## Raspberry Pi 5 Audio Wiring (I2S)

- **MAX98357A amplifier**: `BCLK` -> GPIO `18`, `LRC` -> GPIO `19`, `DIN` -> GPIO `21`, `VIN` -> peripheral `5 V`, `GND` -> GND; speaker on `+`/`-`.
- **INMP441 microphone**: `SCK` -> GPIO `18`, `WS` -> GPIO `19`, `SD` -> GPIO `20`, `L/R` -> GND (left channel), `VDD` -> `3.3 V`.
- Reach the header through the AI HAT+'s GPIO stacking header. Enable both with one I2S overlay in `/boot/firmware/config.txt` (e.g. `dtoverlay=googlevoicehat-soundcard`, a common choice for a MAX98357A + INMP441 pair).

## Running the Simulation & Benchmark Suite

```bash
# 1. Install dependencies
pip install -r requirements.txt            # pinned mujoco / numpy / scipy (FEA: requirements-fea.txt)

# 2. Launch interactive 3D simulation (default `gim6010` 7.03 kg build configuration)
python3 simulate_wheeled_duck.py

# 3. Launch with SimToReal domain randomization (Teensy 4.0 dual-CAN + fast SPI IMU)
python3 simulate_wheeled_duck.py --realism 3 --arch mcu --imu fast

# 4. Run the full ground-truth scenario & thermal test suite (ideal + 12 randomized robots)
python3 test_scenarios.py
python3 test_scenarios.py --realism 12 --arch mcu --imu fast
python3 test_robustness.py --report          # terrain, faults, impacts, endurance (reports/robustness.md)

# 5. Run the battery & motor power budget analysis
python3 tools/power_budget.py --battery-wh 180
```
