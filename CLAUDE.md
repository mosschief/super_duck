# Super Duck — agent notes

1.35x wheeled Open Duck Mini v2 (MuJoCo sim + FreeCAD assembly). `README.md` is the source of truth for hardware,
BOM, wiring and every script's purpose; `docs/design_log.md` is the dated history of design decisions.

## Layout

- `duck/robot.py` — `ScaledWheeledDuckParams`, `RobotProfile` / `PROFILES` (`gim6010` = the build, default;
  `gim4305` = original design), WIP model and LQR gains (`LQR_Q`). No MuJoCo.
- `duck/controller.py` — `ScaledWheeledDuckController`: balance, trim/slope learners, posture, jump, driving.
  Sensor values in, actuator commands out; no MuJoCo. This is what gets ported to the Teensy firmware.
- `duck/plant.py` — sim side: `SimToReal` realism layer, `read_sensors`, motor torque-speed curves, `step_robot`.
- `firmware/controller/` — the controller ported to C++ for the Teensy. `duck/controller.py` stays the reference:
  mirror every change there in `duck_controller.cpp` (same order of operations), re-run
  `tools/export_firmware_config.py`, and check with `--controller compare` (see `firmware/README.md`).
- `simulate_wheeled_duck.py` — viewer / CLI; re-exports the above so `import simulate_wheeled_duck as swd` works.
- `open_duck_wheeled_v2_gim6010.xml` is **generated** by `cad/freecad/export_sim.py` from the CAD; change the CAD
  (or the export) rather than hand-editing it.
- Generated docs: `reports/*.md`, `cad/*.md` / `cad/*.json` are written by scripts in `tools/` and the test suites
  (each says which at the top). Regenerate, don't hand-edit.

## Checks

```bash
pip install -r requirements.txt
python3 test_scenarios.py --controller compare   # ~35 s, must stay all PASS, C++ == Python (CI runs this)
python3 test_scenarios.py --realism 12       # randomized robots
python3 test_robustness.py --report          # ~100 scenarios -> reports/robustness.md (3 known fails at 2.5 m/s / mu 0.3)
```

After any controller or model change, run `test_scenarios.py` and `test_robustness.py` and update the reports and
README numbers they feed. If the trunk mass or CoM moves, re-fit the trim with `tools/fit_balance_trim.py`.
