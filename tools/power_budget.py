#!/usr/bin/env python3
"""
Power budget: battery bus power/current per activity, from simulated motor torques and speeds plus electronics.

Per GIM motor and 2 ms step:
  current      I = tau / Kt                       (output-side torque constant)
  copper loss  P_cu = 1.5 * I^2 * R_phase         (FOC, amplitude-invariant)
  mechanical   P_mech = max(tau * omega, 0) / eta (no credit for regenerative braking: conservative)
Electronics are added as constant loads behind ~90%-efficient buck converters.

Usage:
  python3 tools/power_budget.py                  # default robot profile
  python3 tools/power_budget.py --robot gim4305 --battery-wh 90
"""
import argparse
import contextlib
import io
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import mujoco  # noqa: E402
import simulate_wheeled_duck as swd  # noqa: E402
import test_scenarios as T  # noqa: E402

# Motor electrical data by peak torque (= which motor an actuator's forcerange says it is)
MOTORS = {
    11.0: dict(name="GIM6010-8", kt=0.47, r=0.48),   # SteadyWin GIM6010-8 manual rev1.2, sec 1.2
    # GIM4305-10: listings give 0.62 N*m/A & 1.15 ohm or 0.39 N*m/A & 1.11 ohm depending on driver variant;
    # use the lower Kt (more current per N*m) with the higher R: worst case.
    3.47: dict(name="GIM4305-10", kt=0.39, r=1.15),
}
ETA_MECH = 0.85            # gearbox x driver efficiency for positive mechanical work (estimate)
DRIVER_IDLE_W = 0.5        # per GIM driver, powered and holding (estimate)
BUCK_EFF = 0.90
BUS_V = 21.6               # 6S Li-ion nominal (3.6 V/cell)
# Electronics (W at the load): (typical, peak). Pi 5: ~5 W mixed / 8.8 W all cores; Hailo-8: 2.5 W typical /
# 8.65 W max (Waveshare); cameras / servos / Teensy / expression parts are estimates.
ELECTRONICS = {
    "Raspberry Pi 5": (5.0, 8.8),
    "AI HAT+ 26 TOPS (Hailo-8)": (2.5, 8.65),
    "2x Camera Module 3 + USB wide chest cam (est.)": (1.2, 1.6),
    "Teensy 4.0 + Dual CAN + ICM-42688-P IMU (est.)": (0.5, 0.6),
    "4x STS3215 head servos (est.)": (2.0, 6.0),
    # Expression package (on the peripheral 5 V buck). Eye rings capped at ~35% brightness in firmware
    # (24 WS2812B at full white would be 7.2 W).
    "2x 9 g antenna servos (est.)": (0.1, 2.0),
    "2x WS2812B 12-LED eye rings (est.)": (0.5, 2.5),
    "Projector 1 W LED (est.)": (0.3, 1.0),
    "MAX98357A amp + 3 W speaker + INMP441 mic (est.)": (0.2, 3.0),
}
PERIPHERAL_5V = [k for k in ELECTRONICS if k.startswith(("2x 9 g", "2x WS2812B", "Projector", "MAX98357A"))]

ACTIVITIES = {
    "stand": ([], 12.0, (4, 12)),
    "cruise 0.6 m/s weaving": (T.DRIVE_EVENTS, 24.0, (4, 24)),
    "crouch / stand cycles": ([(1.0 + 3.0 * k + 0.01 * i, "z" if k % 2 == 0 else "x") for k in range(6) for i in range(3)],
                              20.0, (1, 20)),
    "jump": ([(2.0, "c")], 5.0, (2.0, 3.0)),
}


def motor_power(tq: np.ndarray, model) -> np.ndarray:
    """Per-step battery-side power (W) of all GIM motors. tq rows: [t, forces(nu), velocities(nu)]."""
    nu = model.nu
    force, vel = tq[:, 1:1 + nu], tq[:, 1 + nu:1 + 2 * nu]
    total = np.zeros(len(tq))
    for a in [*range(8), 12, 13]:
        mot = MOTORS[round(float(model.actuator_forcerange[a, 1]), 2)]
        current = force[:, a] / mot["kt"]
        total += 1.5 * current ** 2 * mot["r"] + np.maximum(force[:, a] * vel[:, a], 0.0) / ETA_MECH
    return total + 10 * DRIVER_IDLE_W


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", choices=sorted(swd.PROFILES), default=swd.DEFAULT_ROBOT)
    ap.add_argument("--battery-wh", type=float, default=None, help="pack energy for a runtime estimate")
    args = ap.parse_args()
    T.PROFILE = swd.PROFILES[args.robot]
    T.XML = swd.profile_xml(T.PROFILE)
    model = mujoco.MjModel.from_xml_path(T.XML)

    elec_typ = sum(v[0] for v in ELECTRONICS.values()) / BUCK_EFF
    elec_pk = sum(v[1] for v in ELECTRONICS.values()) / BUCK_EFF
    print(f"Robot '{args.robot}', bus {BUS_V} V. Electronics: {elec_typ:.1f} W typical, {elec_pk:.1f} W peak "
          f"(incl. {100 * (1 - BUCK_EFF):.0f}% buck loss); 5 V rail peak "
          f"{(ELECTRONICS['Raspberry Pi 5'][1] + ELECTRONICS['AI HAT+ 26 TOPS (Hailo-8)'][1] + 1.6) / 5.0:.1f} A, "
          f"peripheral 5 V rail peak {sum(ELECTRONICS[k][1] for k in PERIPHERAL_5V) / 5.0:.1f} A\n")
    print(f"{'activity':26s} {'motors avg':>10s} {'motors peak':>11s} {'total avg':>10s} {'bus avg':>8s} {'bus peak':>9s}")
    avgs = {}
    for name, (events, duration, (t0, t1)) in ACTIVITIES.items():
        torques = []
        with contextlib.redirect_stdout(io.StringIO()):
            _, fell, _ = T.run(events, duration, torques=torques)
        tq = np.array(torques)
        w = (tq[:, 0] >= t0) & (tq[:, 0] < t1)
        p = motor_power(tq, model)[w]
        peak = np.convolve(p, np.ones(5) / 5, mode="valid").max()   # 10 ms average: what the pack actually sees
        total_avg = p.mean() + elec_typ
        avgs[name] = total_avg
        print(f"{name:26s} {p.mean():9.1f}W {peak:10.1f}W {total_avg:9.1f}W {total_avg / BUS_V:7.2f}A "
              f"{(peak + elec_pk) / BUS_V:8.1f}A" + ("  (FELL)" if fell else ""))
    mix = 0.5 * avgs["stand"] + 0.4 * avgs["cruise 0.6 m/s weaving"] + 0.1 * avgs["crouch / stand cycles"]
    print(f"\nMixed use (50% standing, 40% driving, 10% posture changes): {mix:.1f} W average")
    for wh in ([args.battery_wh] if args.battery_wh else [90.0, 180.0]):
        print(f"  {wh:.0f} Wh pack -> ~{60 * 0.8 * wh / mix:.0f} min (using 80% of capacity)")


if __name__ == "__main__":
    main()
