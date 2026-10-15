# Super Duck (`mosschief/super_duck`)

**Super Duck** is a `1.35x` scaled (`56.7 cm` tall, `5.48 kg` CAD-ready build configuration), wheeled bipedal robot based on the [Open Duck Mini v2](https://github.com/apirrone/Open_Duck_Mini) BDX architecture. It combines **4x SteadyWin GIM6010-8** high-torque QDD actuators (`hip_pitch`, `knee`) and **6x [SteadyWin GIM4305-10](https://rcdrone.top/products/steadywin-gim4305-planetary-reducer-servo-motor)** actuators (`hip_yaw`, zero-scrub `hip_roll`, and driven `75 mm`-radius wheels) with a split-rate compute stack: a **[Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/) + [Raspberry Pi AI HAT+ (26 TOPS)](https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html)** for vision/AI and a real-time **[PJRC Teensy 4.0](https://www.pjrc.com/store/teensy40.html)** running the deterministic `500 Hz` LQR balance loop over dual `1 Mbps` CAN buses. It carries the full Open Duck Mini expression package: moving antennas, LED eyes, head projector, speaker and microphone.

## Repository Structure & Tooling

| File / Script | Purpose |
| :--- | :--- |
| `open_duck_wheeled_v2_gim6010.xml` | **Default CAD-ready build model (`--robot gim6010`, `5.48 kg`)**: 4x GIM6010-8 (`hip_pitch`, `knee`), 6x GIM4305-10 (`hip_yaw`, inline zero-scrub `hip_roll`, `left/right_wheel`), 4x STS3215 (`neck/head`), 2x 9 g antenna servos, `6S2P` 21700 battery pack with BMS / fuse / anti-spark switch / e-stop, AI HAT+, Teensy 4.0 dual-CAN module, and the expression package |
| `open_duck_wheeled_v2.xml` | **Baseline model (`--robot gim4305`, `4.11 kg`)**: Original all-GIM4305-10 leg/wheel configuration with a `6S1P` battery pack |
| `simulate_wheeled_duck.py` | `500 Hz` full-state LQR wheel-balancing, measured leg-fold odometry compensation, online pitch-trim learner, quadratic posture feedforward, `C`-key jump state machine (with MIT-mode `kd` scheduling & airborne reaction-wheel attitude control), and `SimToReal` domain randomization (`--realism SEED --arch mcu|feather|pi --imu fast|bno055`) |
| `test_scenarios.py` | Automated ground-truth benchmark suite: `stand_still`, `drive_turn_brake`, `brake_settle`, `crouch_stand`, `push` (`±20 N`, `±30 N`, `±40 N`), `jump`, `jump_driving`, and `thermal` (sustained RMS torque vs. continuous motor rating) |
| `tools/upgrade_motor.py` | Upgrades specified leg joints in an MJCF from `gim4305` (`150 g`, `3.47 N·m`) to `gim6010` (`388 g`, `11 N·m` stall / `5 N·m` rated) with exact cylindrical inertia shifts |
| `tools/battery_pack.py` | Configures trunk battery pack geometry and parallel-axis inertia (`6s1p` `~0.48 kg` `90 Wh` vs. `6s2p` `~0.90 kg` `180 Wh`) |
| `tools/add_trunk_electronics.py` | Folds the `41 g` AI HAT+ (26 TOPS) and `8 g` Teensy 4.0 Dual-CAN + IMU (or Feather M4 CAN) into `trunk_assembly` |
| `tools/add_expression_hardware.py` | Adds the Open Duck Mini expression package (hinged antennas on 9 g servos, WS2812B eye rings, projector LED + spot light, I2S mic, speaker + amp) and the battery protection parts (BMS, fuse, anti-spark switch, e-stop, peripheral 5 V buck), folding `37 g` into `head_assembly` and `130 g` into `trunk_assembly`; adds wheel `touch` sensors |
| `tools/derive_lqr_params.py` | Computes lumped body mass, CoM height above wheel axle, and pitch inertia (excluding wheels) from any MJCF for `RobotProfile.params` |
| `tools/fit_balance_trim.py` | Runs parallel `80 s` holds across crouch depths (`-0.24..+0.24 rad`) to fit `initial_pitch_trim` and `posture_coeffs` against measured leg sag |
| `tools/check_cad_clearances.py` | Sweeps every operating pose and full joint range for part interference (real-part layout errors vs. printed-part CAD constraints), checks packing inside the trunk/head, ground clearance, hip-roll scrub torque, and model envelopes vs. real parts → `cad/clearance_report.md` |
| `tools/export_cad_spec.py` | Exports the CAD hardpoints (joint anchors/axes/ranges, body frames, part placement, cameras, key dimensions) → `cad/hardpoints.md/.json`, and renders `cad/purchased_parts.md` |
| `tools/export_design_loads.py` | Force and moment through every joint (axial, radial, about/across the axis) for driving, jumps, pushes and a fall, with GIM4305 bearing checks → `cad/design_loads.md/.json` |
| `tools/fetch_vendor_cad.sh` | Downloads the SteadyWin, Raspberry Pi and camera drawings / STEP models into `cad/vendor/` (git-ignored) |
| `cad/` | **CAD spec locked from the sim**: hardpoints, real purchased-part dimensions with sources, clearance report, design loads, and the open issues for CAD ([`cad/README.md`](cad/README.md)) |
| `tools/power_budget.py` | Simulates copper ($1.5 I^2 R$) + mechanical power across standing, weaving cruise, crouching, and jumping (`~37 W` mixed average, `~236 min` runtime on `180 Wh` `6S2P`, `~80 A` `50 ms` jump peak) |

## Hardware Shopping List & Bill of Materials (Build Configuration: `gim6010`)

### 1. Compute, Real-Time Control & Sensing

| Qty | Component | Key Specs & Role in Super Duck | Reference Link |
| :--- | :--- | :--- | :--- |
| **1x** | **Raspberry Pi 5 (8GB or 16GB)** | `86 x 56 x 16 mm`, `46 g`; runs high-level navigation, stereo vision, and telemetry | [Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/) |
| **1x** | **Raspberry Pi Active Cooler** | Anodized aluminum heatsink + PWM blower fan (fits under the AI HAT+ on `16 mm` standoffs) | [Raspberry Pi 5 Accessories](https://www.raspberrypi.com/products/raspberry-pi-5/) |
| **1x** | **Raspberry Pi AI HAT+ (26 TOPS)** | Hailo-8 NPU accelerator (`65 x 56.5 mm`, `~41 g` w/ standoffs) over PCIe Gen 3 ribbon (`2.5 W` typ / `8.65 W` max) | [Raspberry Pi AI HAT+ (26 TOPS)](https://www.sparkfun.com/raspberry-pi-ai-hat-26-tops.html) |
| **1x** | **PJRC Teensy 4.0** *(or Teensy 4.1)* | **Recommended Real-Time MCU (`--arch mcu`)**: `600 MHz` ARM Cortex-M7 w/ 64-bit FPU (`35 x 18 mm`, `~5 g`); runs the `500 Hz` LQR loop with `0.25-0.55 ms` latency and **splits the 10 GIM motors across 2 independent 1 Mbps CAN buses (`CAN1` & `CAN2`)**, cutting bus load from `50%` to `25%` at `100 Hz` legs (or `68%` at `500 Hz` legs) | [PJRC Teensy 4.0](https://www.pjrc.com/store/teensy40.html) / [Adafruit](https://www.adafruit.com/product/4323) |
| **2x** | **3.3V CAN Transceiver Breakouts (`SN65HVD230` or `MCP2562FD`)** | Converts Teensy `CAN1` (pins `23/22`, Left Leg + Wheel) and `CAN2` (pins `0/1`, Right Leg + Wheel) logic into differential `CAN_H / CAN_L` (or use a pre-assembled [Copperhill Teensy Triple CAN-FD carrier](https://copperhilltech.com/blog/the-ultimate-guide-to-teensy-microcontrollers-history-specs-can-bus-applications/)) | [PJRC Teensy CAN Specs](https://www.pjrc.com/teensy/techspecs.html) |
| **1x** | **ICM-42688-P (or BMI088) 6-Axis SPI IMU Breakout** | **Recommended Fast Raw IMU (`--imu fast`)**: Up to `32 kHz` ODR, `2.8 mdps/√Hz` noise, wired to Teensy `SPI0` (`24 MHz`) for `500 Hz-1 kHz` on-MCU complementary/Mahony filtering (solves the `100 Hz` BNO055 fusion lag during jumping & braking) | [SimToReal IMU Benchmark (`GEMINI.md`)](./GEMINI.md) |
| **2x** | **Raspberry Pi Camera Module 3** *(Standard)* | `25 x 24 x 11.5 mm`, 12 MP Sony IMX708; stereo eyes (`70 mm` baseline in BDX head) on the Pi 5's two CSI/DSI ports (`CAM/DISP 0` and `1`), which keeps the pair hardware-synchronizable | [Raspberry Pi Camera Module 3](https://www.raspberrypi.com/products/camera-module-3/) |
| **1x** | **USB UVC Wide-Angle Camera Module** *(~25 x 25 mm board, ≥ 100° FOV)* | Downward chest nav cam (`trunk_nav_cam`). The Pi 5 has only two camera connectors, both used by the eyes; a USB camera avoids a CSI multiplexer, which would break synchronized stereo capture | e.g. Arducam wide-angle USB camera modules |
| **1x** | **Waveshare Bus Servo Adapter (A)** | UART/USB-to-TTL serial bus interface for the 4x neck/head [Feetech STS3215](https://shop.babsco.com/feetech-sts3215-19-5kg-smart-servo-c001) gimbal servos | [Feetech STS3215](https://shop.babsco.com/feetech-sts3215-19-5kg-smart-servo-c001) |

| **1x** | **Xbox Wireless Controller (Bluetooth)** | Teleop, paired to the Pi 5's built-in Bluetooth; same controller and button layout as Open Duck Mini (see [Gamepad Mapping](#gamepad-mapping)) | |

> **Single-Bus No-Solder Alternative (`--arch feather`)**: If you want an all-in-one board with an onboard CAN transceiver and screw terminals (no external transceiver breakouts), you can use the **[Adafruit Feather M4 CAN Express (ATSAME51)](https://thepihut.com/products/adafruit-feather-m4-can-express-with-atsame51)** (`120 MHz` Cortex-M4F, `50.8 x 22.8 mm`, `5.5 g`). All 10 GIM motors share its single `1 Mbps` classic CAN bus (`50%` bus utilization at `500 Hz` wheels + `100 Hz` legs).

### 2. Actuators & Drivetrain (16 Active DOFs — `gim6010` Build Config)

> **Important Ordering Note**: SteadyWin ships GIM motors with two driver families: **ODrive-based** (`odrivetool`, `CAN ID = node_id * 32 + cmd`) and **SteadyWin MIT/Motor Wizard** (`CAN ID + master ID`). **Order all 10 GIM motors (4x GIM6010-8 + 6x GIM4305-10) with the same driver family** so they share the same CAN protocol on the bus.

| Qty | Motor Model | Assigned Joints | Specs & Thermal Benchmark (`test_scenarios.py -k thermal`) |
| :--- | :--- | :--- | :--- |
| **4x** | **SteadyWin GIM6010-8 QDD Planetary Actuator** | `left/right_hip_pitch` (2x) + `left/right_knee` (2x) | `Ø80 x 44 mm` incl. driver, `388 g`, `8:1` gear, `5.0 N·m` continuous / `11.0 N·m` stall torque, `0.47 N·m/A`, `8x M3` on `Ø74 mm` bolt circle; keeps sustained knee RMS torque at **`34%` of rating** |
| **6x** | **SteadyWin GIM4305-10 QDD Planetary Actuator** | `left/right_hip_yaw` (2x), `left/right_hip_roll` (2x), `left/right_wheel` (2x) | `Ø53 x 33 mm` overall incl. driver (`26 mm` housing + `6 mm` driver boss + `1 mm` output step; listings quote only the housing), `150 g`, `10:1` gear, `1.0 N·m` continuous / `3.47 N·m` stall torque, `300-366 RPM` ([RCDrone](https://rcdrone.top/products/steadywin-gim4305-planetary-reducer-servo-motor)); inboard wheel routing (`4 mm` scrub offset) drops standing `hip_roll` load to **`10%` (`0.10 N·m`)**, saving **`476 g`** and eliminating hip canister clashes |
| **4x** | **Feetech STS3215 Serial Bus Servo (7.4V)** | `neck_pitch`, `head_pitch`, `head_yaw`, `head_roll` | `45.2 x 24.7 x 35 mm`, `55 g`, `19.5 kg·cm` (`1.91 N·m`), 12-bit magnetic encoder ([BABSCO](https://shop.babsco.com/feetech-sts3215-19-5kg-smart-servo-c001)); keeps upper head mass light (`0.742 kg` total neck+head incl. expression package) |
| **2x** | **9 g Micro Servo (SG90 / MG90S class, 5V)** | `left_antenna`, `right_antenna` | `23 x 12 x 29 mm`, `9 g`, `~0.18 N·m` stall, `±45°` via `1-2 ms` PWM at `50 Hz` (Open Duck Mini `antennas.py`); `class="micro_servo_9g"` position actuators at `ctrl[14:16]` |

### 3. Battery, Power Tree & High-Grip Tires

| Qty | Component | Specification & Sizing Rationale (`tools/power_budget.py`) |
| :--- | :--- | :--- |
| **1x** | **6S2P 21700 High-Drain Li-Ion Pack (`21.6V` nom / `25.2V` max, `~180-200 Wh`)** | `12x` high-drain 21700 cells (e.g., Molicel P42A / P45B, `45A` continuous per cell = `90A` pack continuous), `~70 x 84 x 63 mm`, `~0.90 kg` mounted at `z = -0.026 m` in `trunk_assembly`. Provides **`~236 min` runtime** at `37 W` mixed load and handles the **`~80 A` (`50 ms`) jump thrust current** without voltage sag |
| **1x** | **24V-to-5.1V (5A) Buck Converter** | Powers Raspberry Pi 5 (`5.0 W` typ / `8.8 W` peak) + AI HAT+ (`2.5 W` typ / `8.65 W` peak) + cameras + Teensy 4.0 (`~3.8 A` peak on 5V rail) |
| **1x** | **24V-to-7.4V (10A) Buck Converter** | Powers the 4x Feetech STS3215 neck/head gimbal servos (`2.0 W` typ / `6.0 W` peak) |
| **1x** | **24V-to-5V (3A) Peripheral Buck Converter** | Separate rail for the antenna servos, eye LED rings, projector and audio amp (`1.7 A` peak), so servo and LED current spikes stay off the Pi 5's supply (already at `~3.8 A` peak) |
| **1x** | **6S Li-Ion BMS (≥ 100 A discharge, with balancing)** | Over-discharge / over-current / short protection and cell balancing; must pass the `~80 A`, `50 ms` jump bursts without tripping (check its over-current delay). `~40 g` beside the pack |
| **1x** | **40 A Blade Fuse + Inline Holder** | Main fuse on the battery positive, before everything else. A slow-blow `40 A` fuse rides through `80 A` for `50 ms`; it is there for wiring shorts |
| **1x** | **Anti-Spark Electronic Power Switch (6S, ≥ 120 A)** | Main on/off. Avoids the inrush spark from the 10 GIM drivers' input capacitors; its low-current control loop is wired through the e-stop below |
| **1x** | **22 mm Latching Mushroom E-Stop (NC contact)** | On the back of the trunk. Opens the anti-spark switch's control loop, cutting all power (the robot will drop: use the gamepad for a soft stop). Firmware to do: have the Teensy read battery voltage from the GIM drivers over CAN, warn below `3.4 V/cell` (`20.4 V`) and sit the robot down below `3.3 V/cell` (`19.8 V`), before the BMS cuts power |
| **2x** | **Shore 45A Cast Urethane Tires on `150 mm` (`75 mm` radius) Printed Hubs** | Push recovery is traction-limited ($F_{\max} \approx \mu m g$); soft cast urethane treads increase floor grip ($\mu \approx 1.3-1.7$) so the robot survives `±30-40 N` shoves |

### 4. Expression Package (Open Duck Mini Parity)

Everything in the Open Duck Mini expression package (its BOM and [`Open_Duck_Mini_Runtime`](https://github.com/apirrone/Open_Duck_Mini_Runtime) `expression_features`: eyes, projector, antennas, speaker, microphone, camera), scaled to this robot. Added to both MJCFs by `tools/add_expression_hardware.py`.

| Qty | Component | Role & Notes |
| :--- | :--- | :--- |
| **2x** | **WS2812B 12-LED Ring (37 mm OD)** | Eye LEDs behind the existing eye bezels, around the camera lenses (Open Duck Mini uses plain LEDs; RGB rings add blink plus status colors such as low battery). Cap brightness at ~35% in firmware: full white is `7.2 W` |
| **1x** | **1 W LED + Small Reflector + Logic-Level MOSFET** | Head projector below the eyes (Open Duck Mini: LED + reflector on one GPIO). Modeled as the `projector` spot light; `P` toggles it in the sim |
| **1x** | **MAX98357A I2S Class-D Amplifier** | Same amp as Open Duck Mini; the Pi 5 has no analog audio output |
| **1x** | **40 mm 3 W 4 Ω Speaker** | Rear wall of the trunk. Plays the Open Duck Mini sound set |
| **1x** | **INMP441 I2S MEMS Microphone** | Top front of the head; shares the Pi's I2S bus with the amp (separate data pin) |
| **1x** | **74AHCT125 Quad Level Shifter** | Lifts the Teensy's `3.3 V` WS2812B data line to `5 V` (WS2812B needs `≥ 3.5 V` high at `5 V` supply) |
| **1x** | **10 kΩ Resistor + 330 Ω Resistor** | Projector MOSFET gate pull-down (keeps it off while the Teensy boots) and LED data-line series resistor |

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
- **Teensy 4.0 <-> 10x GIM6010-8 / GIM4305-10 Motors (`500 Hz` Dual 1 Mbps Classic CAN)**:
  - **`CAN1` (`CRX1` Pin 23, `CTX1` Pin 22 -> Transceiver #1)**: Left Leg (`left_hip_yaw`, `left_hip_roll`, `left_hip_pitch`, `left_knee`) + `left_wheel` (CAN IDs `1-5`, `120 Ω` termination at `left_wheel`).
  - **`CAN2` (`CRX2` Pin 0, `CTX2` Pin 1 -> Transceiver #2)**: Right Leg (`right_hip_yaw`, `right_hip_roll`, `right_hip_pitch`, `right_knee`) + `right_wheel` (CAN IDs `6-10`, `120 Ω` termination at `right_wheel`).
- **Teensy 4.0 <-> ICM-42688-P Raw IMU (`500 Hz - 1 kHz` SPI Attitude Loop)**:
  - **`SPI0`**: `CS` Pin 10, `MOSI` Pin 11, `MISO` Pin 12, `SCK` Pin 13 + data-ready interrupt (`INT1` on Pin 9). *(If using a BNO055 instead, wire to `I2C0` on `SDA0` Pin 18 / `SCL0` Pin 19).*
- **Teensy 4.0 <-> 4x Neck/Head STS3215 Servos (`100 Hz` Camera Horizon Gimbal)**:
  - **`Serial3` (`RX3` Pin 15, `TX3` Pin 14)** -> Waveshare Bus Servo Adapter (A).
- **Teensy 4.0 <-> Expression Package (peripheral `5 V` rail, common ground with the Teensy)**:
  - **Antenna servos**: Pin `2` (left), Pin `3` (right) -> servo signal, `50 Hz`, `1-2 ms` pulses (`PWMServo` or `analogWriteFrequency` + `analogWrite`; Open Duck Mini `antennas.py` mapping, right side mirrored). Servo power from the peripheral `5 V` buck, not the Teensy.
  - **Eye rings**: Pin `8` (`TX2`) -> 74AHCT125 (`5 V` supplied) -> `330 Ω` -> left ring `DIN`; left ring `DOUT` -> right ring `DIN` (24 LEDs, one chain). Drive with **`WS2812Serial`** (DMA over `Serial2`): FastLED / Adafruit NeoPixel disable interrupts for `~0.7 ms` per update, which would jitter the `500 Hz` balance loop.
  - **Projector**: Pin `4` -> logic-level N-MOSFET gate (`10 kΩ` pull-down to GND) switching the 1 W LED's low side from the `5 V` rail (with its current-limit resistor). PWM on pin 4 dims it.
  - **Free for later**: Pins `5`, `6`, `16-21` (`I2C1` on `16/17`, `I2C0` on `18/19`).
- **Pi 5 -> Teensy expression commands** (same USB serial link as teleop, `~50 Hz`): antenna targets, eye mode / color / blink, projector on / off / brightness. The Teensy overrides them for status: eyes red on low battery (GIM bus voltage over CAN), amber blink when the Pi link times out (`> 0.5 s`), antennas back on a fall.

## Raspberry Pi 5 Audio Wiring (I2S)

- **MAX98357A amplifier**: `BCLK` -> GPIO `18`, `LRC` -> GPIO `19`, `DIN` -> GPIO `21`, `VIN` -> peripheral `5 V`, `GND` -> GND; speaker on `+`/`-`.
- **INMP441 microphone**: `SCK` -> GPIO `18`, `WS` -> GPIO `19`, `SD` -> GPIO `20`, `L/R` -> GND (left channel), `VDD` -> `3.3 V`.
- Reach the header through the AI HAT+'s GPIO stacking header. Enable both with one I2S overlay in `/boot/firmware/config.txt` (e.g. `dtoverlay=googlevoicehat-soundcard`, a common choice for a MAX98357A + INMP441 pair).

## Running the Simulation & Benchmark Suite

```bash
# 1. Install dependencies
pip install mujoco numpy scipy

# 2. Launch interactive 3D simulation (default `gim6010` 5.48 kg build configuration)
python3 simulate_wheeled_duck.py

# 3. Launch with SimToReal domain randomization (Teensy 4.0 dual-CAN + fast SPI IMU)
python3 simulate_wheeled_duck.py --realism 3 --arch mcu --imu fast

# 4. Run the full ground-truth scenario & thermal test suite (ideal + 12 randomized robots)
python3 test_scenarios.py
python3 test_scenarios.py --realism 12 --arch mcu --imu fast

# 5. Run the battery & motor power budget analysis
python3 tools/power_budget.py --battery-wh 180
```
