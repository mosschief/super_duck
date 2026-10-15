# Super Duck (`mosschief/super_duck`)

**Super Duck** is a `1.35x` scaled (`56.7 cm` tall, `5.32 kg` CAD-ready build configuration), wheeled bipedal robot based on the [Open Duck Mini v2](https://github.com/apirrone/Open_Duck_Mini) BDX architecture. It combines **4x SteadyWin GIM6010-8** high-torque QDD actuators (`hip_pitch`, `knee`) and **6x [SteadyWin GIM4305-10](https://rcdrone.top/products/steadywin-gim4305-planetary-reducer-servo-motor)** actuators (`hip_yaw`, zero-scrub `hip_roll`, and driven `75 mm`-radius wheels) with a split-rate compute stack: a **[Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/) + [Raspberry Pi AI HAT+ (26 TOPS)](https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html)** for vision/AI and a real-time **[PJRC Teensy 4.0](https://www.pjrc.com/store/teensy40.html)** running the deterministic `500 Hz` LQR balance loop over dual `1 Mbps` CAN buses.

## Repository Structure & Tooling

| File / Script | Purpose |
| :--- | :--- |
| `open_duck_wheeled_v2_gim6010.xml` | **Default CAD-ready build model (`--robot gim6010`, `5.32 kg`)**: 4x GIM6010-8 (`hip_pitch`, `knee`), 6x GIM4305-10 (`hip_yaw`, inline zero-scrub `hip_roll`, `left/right_wheel`), 4x STS3215 (`neck/head`), `6S2P` 21700 battery pack, AI HAT+, and Teensy 4.0 dual-CAN module |
| `open_duck_wheeled_v2.xml` | **Baseline model (`--robot gim4305`, `3.94 kg`)**: Original all-GIM4305-10 leg/wheel configuration with a `6S1P` battery pack |
| `simulate_wheeled_duck.py` | `500 Hz` full-state LQR wheel-balancing, measured leg-fold odometry compensation, online pitch-trim learner, quadratic posture feedforward, `C`-key jump state machine (with MIT-mode `kd` scheduling & airborne reaction-wheel attitude control), and `SimToReal` domain randomization (`--realism SEED --arch mcu|feather|pi --imu fast|bno055`) |
| `test_scenarios.py` | Automated ground-truth benchmark suite: `stand_still`, `drive_turn_brake`, `brake_settle`, `crouch_stand`, `push` (`±20 N`, `±30 N`, `±40 N`), `jump`, `jump_driving`, and `thermal` (sustained RMS torque vs. continuous motor rating) |
| `tools/upgrade_motor.py` | Upgrades specified leg joints in an MJCF from `gim4305` (`150 g`, `3.47 N·m`) to `gim6010` (`388 g`, `11 N·m` stall / `5 N·m` rated) with exact cylindrical inertia shifts |
| `tools/battery_pack.py` | Configures trunk battery pack geometry and parallel-axis inertia (`6s1p` `~0.48 kg` `90 Wh` vs. `6s2p` `~0.90 kg` `180 Wh`) |
| `tools/add_trunk_electronics.py` | Folds the `41 g` AI HAT+ (26 TOPS) and `8 g` Teensy 4.0 Dual-CAN + IMU (or Feather M4 CAN) into `trunk_assembly` |
| `tools/derive_lqr_params.py` | Computes lumped body mass, CoM height above wheel axle, and pitch inertia (excluding wheels) from any MJCF for `RobotProfile.params` |
| `tools/fit_balance_trim.py` | Runs parallel `80 s` holds across crouch depths (`-0.24..+0.24 rad`) to fit `initial_pitch_trim` and `posture_coeffs` against measured leg sag |
| `tools/check_cad_clearances.py` | Verifies 3D motor canister clearances (`hip_yaw`, `hip_roll`, `hip_pitch`, `knee`, deep-crouch wheel tire) and lateral wheel scrub torque before CAD |
| `tools/power_budget.py` | Simulates copper ($1.5 I^2 R$) + mechanical power across standing, weaving cruise, crouching, and jumping (`~40 W` mixed average, `~214 min` runtime on `180 Wh` `6S2P`, `~80 A` `50 ms` jump peak) |

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
| **3x** | **Raspberry Pi Camera Module 3** *(2x Standard, 1x Wide)* | `25 x 24 x 11.5 mm`, 12 MP Sony IMX708; 2x stereo eyes (`70 mm` baseline in BDX head) + 1x downward chest nav cam (`trunk_nav_cam`) | [Raspberry Pi Camera Module 3](https://www.raspberrypi.com/products/camera-module-3/) |
| **1x** | **Waveshare Bus Servo Adapter (A)** | UART/USB-to-TTL serial bus interface for the 4x neck/head [Feetech STS3215](https://shop.babsco.com/feetech-sts3215-19-5kg-smart-servo-c001) gimbal servos | [Feetech STS3215](https://shop.babsco.com/feetech-sts3215-19-5kg-smart-servo-c001) |

> **Single-Bus No-Solder Alternative (`--arch feather`)**: If you want an all-in-one board with an onboard CAN transceiver and screw terminals (no external transceiver breakouts), you can use the **[Adafruit Feather M4 CAN Express (ATSAME51)](https://thepihut.com/products/adafruit-feather-m4-can-express-with-atsame51)** (`120 MHz` Cortex-M4F, `50.8 x 22.8 mm`, `5.5 g`). All 10 GIM motors share its single `1 Mbps` classic CAN bus (`50%` bus utilization at `500 Hz` wheels + `100 Hz` legs).

### 2. Actuators & Drivetrain (14 Active DOFs — `gim6010` Build Config)

> **Important Ordering Note**: SteadyWin ships GIM motors with two driver families: **ODrive-based** (`odrivetool`, `CAN ID = node_id * 32 + cmd`) and **SteadyWin MIT/Motor Wizard** (`CAN ID + master ID`). **Order all 10 GIM motors (4x GIM6010-8 + 6x GIM4305-10) with the same driver family** so they share the same CAN protocol on the bus.

| Qty | Motor Model | Assigned Joints | Specs & Thermal Benchmark (`test_scenarios.py -k thermal`) |
| :--- | :--- | :--- | :--- |
| **4x** | **SteadyWin GIM6010-8 QDD Planetary Actuator** | `left/right_hip_pitch` (2x) + `left/right_knee` (2x) | `Ø80 x 44 mm` incl. driver, `388 g`, `8:1` gear, `5.0 N·m` continuous / `11.0 N·m` stall torque, `0.47 N·m/A`, `8x M3` on `Ø74 mm` bolt circle; keeps sustained knee RMS torque at **`34%` of rating** |
| **6x** | **SteadyWin GIM4305-10 QDD Planetary Actuator** | `left/right_hip_yaw` (2x), `left/right_hip_roll` (2x), `left/right_wheel` (2x) | `Ø53 x 26 mm` incl. driver, `150 g`, `10:1` gear, `1.0 N·m` continuous / `3.47 N·m` stall torque, `300-366 RPM` ([RCDrone](https://rcdrone.top/products/steadywin-gim4305-planetary-reducer-servo-motor)); inboard wheel routing (`4 mm` scrub offset) drops standing `hip_roll` load to **`10%` (`0.10 N·m`)**, saving **`476 g`** and eliminating hip canister clashes |
| **4x** | **Feetech STS3215 Serial Bus Servo (7.4V)** | `neck_pitch`, `head_pitch`, `head_yaw`, `head_roll` | `45.2 x 24.7 x 35 mm`, `55 g`, `19.5 kg·cm` (`1.91 N·m`), 12-bit magnetic encoder ([BABSCO](https://shop.babsco.com/feetech-sts3215-19-5kg-smart-servo-c001)); keeps upper head mass light (`0.705 kg` total neck+head) |

### 3. Battery, Power Tree & High-Grip Tires

| Qty | Component | Specification & Sizing Rationale (`tools/power_budget.py`) |
| :--- | :--- | :--- |
| **1x** | **6S2P 21700 High-Drain Li-Ion Pack (`21.6V` nom / `25.2V` max, `~180-200 Wh`)** | `12x` high-drain 21700 cells (e.g., Molicel P42A / P45B, `45A` continuous per cell = `90A` pack continuous), `~70 x 84 x 63 mm`, `~0.90 kg` mounted at `z = -0.026 m` in `trunk_assembly`. Provides **`~214 min` runtime** at `40 W` mixed load and handles the **`~80 A` (`50 ms`) jump thrust current** without voltage sag |
| **1x** | **24V-to-5.1V (5A) Buck Converter** | Powers Raspberry Pi 5 (`5.0 W` typ / `8.8 W` peak) + AI HAT+ (`2.5 W` typ / `8.65 W` peak) + cameras + Teensy 4.0 (`~3.8 A` peak on 5V rail) |
| **1x** | **24V-to-7.4V (10A) Buck Converter** | Powers the 4x Feetech STS3215 neck/head gimbal servos (`2.0 W` typ / `6.0 W` peak) |
| **2x** | **Shore 45A Cast Urethane Tires on `150 mm` (`75 mm` radius) Printed Hubs** | Push recovery is traction-limited ($F_{\max} \approx \mu m g$); soft cast urethane treads increase floor grip ($\mu \approx 1.3-1.7$) so the robot survives `±30-40 N` shoves |

## Teensy 4.0 Dual-CAN & SPI IMU Wiring Pinout

- **Raspberry Pi 5 <-> Teensy 4.0 (`50-100 Hz` High-Level Teleop & Vision Link)**:
  - USB-A to Micro-USB cable (`480 Mbps` native USB CDC serial, `< 0.2 ms` latency).
- **Teensy 4.0 <-> 10x GIM6010-8 / GIM4305-10 Motors (`500 Hz` Dual 1 Mbps Classic CAN)**:
  - **`CAN1` (`CRX1` Pin 23, `CTX1` Pin 22 -> Transceiver #1)**: Left Leg (`left_hip_yaw`, `left_hip_roll`, `left_hip_pitch`, `left_knee`) + `left_wheel` (CAN IDs `1-5`, `120 Ω` termination at `left_wheel`).
  - **`CAN2` (`CRX2` Pin 0, `CTX2` Pin 1 -> Transceiver #2)**: Right Leg (`right_hip_yaw`, `right_hip_roll`, `right_hip_pitch`, `right_knee`) + `right_wheel` (CAN IDs `6-10`, `120 Ω` termination at `right_wheel`).
- **Teensy 4.0 <-> ICM-42688-P Raw IMU (`500 Hz - 1 kHz` SPI Attitude Loop)**:
  - **`SPI0`**: `CS` Pin 10, `MOSI` Pin 11, `MISO` Pin 12, `SCK` Pin 13 + data-ready interrupt (`INT1` on Pin 9). *(If using a BNO055 instead, wire to `I2C0` on `SDA0` Pin 18 / `SCL0` Pin 19).*
- **Teensy 4.0 <-> 4x Neck/Head STS3215 Servos (`100 Hz` Camera Horizon Gimbal)**:
  - **`Serial3` (`RX3` Pin 15, `TX3` Pin 14)** -> Waveshare Bus Servo Adapter (A).

## Running the Simulation & Benchmark Suite

```bash
# 1. Install dependencies
pip install mujoco numpy scipy

# 2. Launch interactive 3D simulation (default `gim6010` 5.32 kg build configuration)
python3 simulate_wheeled_duck.py

# 3. Launch with SimToReal domain randomization (Teensy 4.0 dual-CAN + fast SPI IMU)
python3 simulate_wheeled_duck.py --realism 3 --arch mcu --imu fast

# 4. Run the full ground-truth scenario & thermal test suite (ideal + 12 randomized robots)
python3 test_scenarios.py
python3 test_scenarios.py --realism 12 --arch mcu --imu fast

# 5. Run the battery & motor power budget analysis
python3 tools/power_budget.py --battery-wh 180
```
