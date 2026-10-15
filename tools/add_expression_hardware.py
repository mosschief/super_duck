#!/usr/bin/env python3
"""
Adds the Open Duck Mini "expression package" and the battery-protection parts to an MJCF model, in place.

Open Duck Mini (apirrone/Open_Duck_Mini BOM + Open_Duck_Mini_Runtime) has eyes, a projector, two antennas, a speaker,
a microphone and a camera. This adds the ones the 1.35x model lacked, plus the 6S pack protection it needs:

  Head (`head_assembly`):
    2x 9 g micro servo (23 x 12 x 29 mm) driving the antennas, now hinged (`left/right_antenna` joints + position
        actuators; Open Duck Mini `antennas.py` maps 1-2 ms PWM to +/-45 deg). Antenna stalk + horn: 3 g each.
    2x 12-LED WS2812B ring (37 mm OD, ~2.5 g) behind the existing eye bezels, around the camera lenses.
    Projector: 1 W LED + reflector + MOSFET (~6 g), below the eyes; a MuJoCo spot light (`projector`, off at start).
    INMP441 I2S MEMS microphone breakout (14 mm, ~2 g), top front.
  Trunk (`trunk_assembly`):
    40 mm 3 W 4 ohm speaker (~18 g), rear wall; MAX98357A I2S amplifier (~2 g).
    6S 100 A BMS (~40 g) beside the pack; 40 A main fuse + holder (~10 g); 120 A anti-spark power switch (~25 g);
    24 V -> 5 V 3 A peripheral buck for servos/LEDs/amp (~10 g); 22 mm latching e-stop on the back (~25 g),
    wired into the anti-spark switch's control loop.
  Wheels: `touch` sensors (`left/right_wheel_touch`) for landing / lift-off detection.

Masses are folded into the explicit <inertial> of each body (see add_trunk_electronics.py). Adds two actuators at
ctrl[14:16] (after the wheels, so existing indices are unchanged) and two joints after `head_roll` in qpos.
Idempotent: skips a model that already has them. Re-run tools/derive_lqr_params.py and tools/fit_balance_trim.py after.

Usage: python3 tools/add_expression_hardware.py open_duck_wheeled_v2.xml open_duck_wheeled_v2_gim6010.xml
"""
import re
import sys

from add_trunk_electronics import fold

HEAD_PARTS = [  # name, mass (kg), position in head frame (m), half-size (m) or None, material
    ("left_antenna_servo_9g", 0.009, (-0.032, 0.046, 0.020), (0.0115, 0.006, 0.0145), "gim4305_housing"),
    ("right_antenna_servo_9g", 0.009, (-0.032, -0.046, 0.020), (0.0115, 0.006, 0.0145), "gim4305_housing"),
    ("left_eye_ws2812_ring", 0.0025, (0.053, 0.035, 0.002), None, ""),     # the existing left_eye_led_ring geom
    ("right_eye_ws2812_ring", 0.0025, (0.053, -0.035, 0.002), None, ""),   # the existing right_eye_led_ring geom
    ("projector_led_reflector", 0.006, (0.053, 0.0, -0.018), None, ""),    # cylinder added below
    ("microphone_inmp441", 0.002, (0.030, 0.0, 0.032), (0.007, 0.007, 0.001), "rpi_green"),
]
TRUNK_PARTS = [
    ("speaker_40mm_3w", 0.018, (-0.066, 0.0, 0.012), None, ""),             # cylinder added below
    ("max98357a_i2s_amp", 0.002, (-0.060, 0.035, 0.035), (0.009, 0.0095, 0.0015), "rpi_green"),
    ("bms_6s_100a", 0.040, (-0.026, 0.053, -0.026), (0.030, 0.004, 0.020), "gim4305_housing"),
    ("main_fuse_40a", 0.010, (-0.026, -0.053, -0.040), (0.012, 0.005, 0.010), "foot_bottom_tpu_material"),
    ("antispark_switch_120a", 0.025, (0.033, -0.048, -0.030), (0.020, 0.010, 0.008), "gim4305_housing"),
    ("peripheral_5v_buck", 0.010, (0.030, 0.048, -0.030), (0.011, 0.008, 0.006), "rpi_green"),
    ("estop_22mm", 0.025, (-0.082, 0.0, 0.015), None, ""),                  # cylinders added below
]

ANTENNA_GEOMS = re.compile(r'\n *<!-- BDX Twin Rear Antenna Ears -->\n(?: *<geom name="(?:left|right)_antenna_\w+"[^>]*/>\n){4}')
ANTENNA = '''
                <body name="{side}_antenna" pos="-0.032 {y} 0.038">
                  <joint name="{side}_antenna" class="micro_servo_9g"/>
                  <inertial pos="-0.007 {iy} 0.032" mass="0.003" diaginertia="0.0000009 0.0000009 0.0000001"/>
                  <geom name="{side}_antenna_base" class="visual" type="cylinder" size="0.008 0.008" material="foot_bottom_tpu_material"/>
                  <geom name="{side}_antenna_stalk" class="visual" type="capsule" fromto="0 0 0.004 -0.014 {sy} 0.060" size="0.0035" material="foot_bottom_tpu_material"/>
                </body>'''
ANTENNAS = ('''
                <!-- BDX Twin Rear Antenna Ears, each swung fore/aft by a 9 g micro servo (servo bodies folded into the
                     head inertial above; Open Duck Mini `antennas.py`: 1-2 ms PWM = +/-45 deg) -->'''
            + ANTENNA.format(side="left", y="0.046", iy="0.004", sy="0.008")
            + ANTENNA.format(side="right", y="-0.046", iy="-0.004", sy="-0.008") + "\n")
HEAD_EXTRA = '''
                <geom name="projector_lens" class="visual" type="cylinder" size="0.007 0.003" pos="0.055 0 -0.018" euler="0 1.5708 0" material="bdx_eye_led"/>
                <light name="projector" pos="0.059 0 -0.018" dir="1 0 -0.25" cutoff="20" exponent="8" diffuse="0.9 0.85 0.7" specular="0.2 0.2 0.2" castshadow="false" active="false"/>'''
TRUNK_EXTRA = '''
        <geom name="speaker_40mm_3w" class="visual" type="cylinder" size="0.020 0.003" pos="-0.066 0 0.012" euler="0 1.5708 0" material="foot_bottom_tpu_material"/>
        <geom name="estop_bezel" class="visual" type="cylinder" size="0.012 0.004" pos="-0.080 0 0.015" euler="0 1.5708 0" material="estop_yellow"/>
        <geom name="estop_mushroom" class="visual" type="cylinder" size="0.016 0.004" pos="-0.087 0 0.015" euler="0 1.5708 0" material="estop_red"/>'''
DEFAULT = '''

      <!-- 9 g micro servo (SG90/MG90S class, 4.8-6 V): ~0.18 N*m stall, 0.1 s/60 deg. Armature: rotor x ~300:1 gear.
           Range = Open Duck Mini's +/-1 command (1-2 ms PWM). -->
      <default class="micro_servo_9g">
        <joint type="hinge" axis="0 1 0" range="-0.785398 0.785398" damping="0.002" frictionloss="0.002" armature="0.0001" limited="true"/>
        <position kp="0.3" kv="0.008" forcerange="-0.18 0.18" inheritrange="1"/>
      </default>'''
MATERIALS = '''
    <material name="estop_red" rgba="0.80 0.08 0.06 1" specular="0.40" shininess="0.5"/>
    <material name="estop_yellow" rgba="0.95 0.78 0.10 1"/>'''
TOUCH_SITE = ('\n{ind}<site name="{side}_wheel_touch" type="cylinder" size="0.077 0.017" euler="1.5707963 0 0" '
              'group="5" rgba="0 0 0 0"/>')
SENSORS = '''

    <!-- Wheel touch (normal force inside the tire envelope): landing / lift-off detection -->
    <touch site="left_wheel_touch" name="left_wheel_touch"/>
    <touch site="right_wheel_touch" name="right_wheel_touch"/>'''
ACTUATORS = '''

    <!-- Antenna 9 g micro servos (Open Duck Mini expression package), ctrl[14:16] -->
    <position name="left_antenna" joint="left_antenna" class="micro_servo_9g"/>
    <position name="right_antenna" joint="right_antenna" class="micro_servo_9g"/>'''


def insert_after(s: str, pattern: str, text: str) -> str:
    m = re.search(pattern, s, re.S)
    assert m, pattern
    return s[:m.end()] + text + s[m.end():]


def add(path: str) -> None:
    s = open(path).read()
    if HEAD_PARTS[0][0] in s:
        print(f"{path}: already has expression hardware, skipped")
        return
    print(path)
    s = fold(s, "head_assembly", HEAD_PARTS, indent=" " * 16)
    s = fold(s, "trunk_assembly", TRUNK_PARTS)
    s, n = ANTENNA_GEOMS.subn(ANTENNAS, s)
    assert n == 1, "antenna geoms not found"
    s = insert_after(s, r'<geom name="right_rpi_cam3_pcb"[^>]*/>', HEAD_EXTRA)
    s = insert_after(s, r'<geom name="battery_pack_6s"[^>]*/>', TRUNK_EXTRA)
    s = insert_after(s, r'<default class="sts3215">.*?</default>', DEFAULT)
    s = insert_after(s, r'<material name="camera_lens"[^>]*/>', MATERIALS)
    for side, site in (("left", "left_foot"), ("right", "right_foot")):
        m = re.search(rf'\n( *)<site name="{site}"[^>]*/>', s)
        s = s[:m.end()] + TOUCH_SITE.format(ind=m.group(1), side=side) + s[m.end():]
    s = insert_after(s, r'<jointvel joint="right_wheel" name="right_wheel_vel"/>', SENSORS)
    s = insert_after(s, r'<motor name="right_wheel_motor"[^>]*/>', ACTUATORS)

    # Keyframe: antenna joints follow the 4 neck/head joints in qpos (after the 7 free-joint values); ctrl appends 2
    def key(m):
        q = m.group(1).split()
        q[11:11] = ["0", "0"]
        return f'qpos="{" ".join(q)}"\n         ctrl="{m.group(2)} 0 0"'
    s, n = re.subn(r'qpos="([^"]+)"\s+ctrl="([^"]+)"', key, s)
    assert n == 1, "keyframe not found"
    open(path, "w").write(s)


if __name__ == "__main__":
    for p in sys.argv[1:]:
        add(p)
