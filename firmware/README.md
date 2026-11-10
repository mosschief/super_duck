# Firmware

The Teensy 4.0 balance computer's code. So far it is only the controller itself; the hardware layer comes next.

## `controller/`: the balance controller in C++

| File | What it is |
| :--- | :--- |
| `duck_controller.hpp` / `.cpp` | `duck::Controller`, a line-for-line port of `duck/controller.py`: balance (LQR), trim and slope learners, posture, jump, drop / bump / fall handling, driving. Plain C++17: no heap, exceptions or Arduino code, so it builds the same on a PC and on the Teensy. |
| `config_gim6010.hpp` | **Generated** by `tools/export_firmware_config.py`: every tunable, robot parameter and LQR gain for the build, taken from the Python controller. Don't edit it; re-run the script. |
| `duck_capi.cpp` | C API the sim loads with ctypes (`duck/cpp_controller.py`). Not part of the firmware. |

The Python controller is the reference. To change the controller:

1. Change `duck/controller.py` and get the test suites passing.
2. Mirror the change in `duck_controller.cpp` (same order of operations; a new constant goes in `DUCK_CONFIG_FIELDS`,
   a new state variable in `DUCK_STATE_FIELDS`).
3. Re-run `python3 tools/export_firmware_config.py`.
4. Check: `python3 test_scenarios.py --controller compare` (CI runs this), and for the full set,
   `python3 test_robustness.py --controller compare`.

`--controller compare` runs both controllers on the same sensor readings every tick and stops at the first command
or state field that differs (tolerance 1e-9). With `-ffp-contract=off` they currently agree bit for bit. The
compare run also fails if a Python class constant has no field in the C++ `Config`.

Other ways to run it: `--controller cpp` runs the sim on the C++ controller alone (`simulate_wheeled_duck.py`,
`test_scenarios.py`, `test_robustness.py`), and `DUCK_CONTROLLER=cpp` does the same for any script. The shared
library is built into `build/` on first use (needs a C++17 compiler; set `CXX` to pick one).

## Next: the hardware layer

To be written around `duck::Controller`, ticking it at 500 Hz:

- **CAN to the GIM motors** (`FlexCAN_T4`, CAN1 = left leg + wheel, CAN2 = right). Depends on the driver firmware
  the motors ship with: ODrive-based (configured with `odrivetool`; CAN ID = node_id x 32 + command) or
  SteadyWin's own (Motor Wizard; SteadyWin or MIT protocol). Check which tool talks to them before writing this.
- **IMU** (ICM-42688-P over SPI): quaternion, gyro, acceleration magnitude for `Inputs`.
- **Safety**: e-stop input, CAN watchdog (motors to zero torque if the loop stops), the controller's fall cutoff.
- **Persist the learned trim** between runs (EEPROM), as `docs/design_log.md` recommends.

On the Teensy the libm differs from the PC's, so its results will match the sim to rounding, not bit for bit.
