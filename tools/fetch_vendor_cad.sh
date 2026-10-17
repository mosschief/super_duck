#!/usr/bin/env bash
# Downloads the vendor drawings / STEP models cited in cad/purchased_parts.json into cad/vendor/ (git-ignored:
# vendor files are not redistributed). Usage: tools/fetch_vendor_cad.sh
set -euo pipefail
cd "$(dirname "$0")/../cad"
mkdir -p vendor
SW=https://cdn.shopify.com/s/files/1/0673/6848/5000/files
while read -r name url; do
  [ -z "$name" ] && continue
  echo "-> vendor/$name"
  curl -fsSL -o "vendor/$name" "$url"
done <<LIST
GIM6010-8_Installation_Diagram.pdf $SW/6010-8_Installation_Diagram.pdf
GIM6010-8_Output_Shaft_Mounting.pdf $SW/6010-8_Output_Shaft_Mounting_Dimensions_Drawing.pdf
GIM6010-8.step $SW/SteadyWin_GIM6010_8.step
GIM6010-8_Manual_rev1.pdf https://sdalal1.github.io/assets/pdf/SteadyWin_GIM6010-8_Manual_rev1.pdf
GIM4305-10SHS_Installation_Diagram.pdf $SW/GIM4305-10SHS_Installation_Diagram.pdf
GIM4305_Motor_Interface_Schematic.pdf $SW/GIM4305_Motor_Interface_Schematic_Diagram.pdf
GIM4305-10.step $SW/SteadyWin_GIM4305_10.step
Raspberry_Pi_5_Mechanical_Drawing.pdf https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-mechanical-drawing.pdf
Raspberry_Pi_5.step https://www.step.parts/step/raspberry_pi_5.step
Camera_Module_3_Product_Brief.pdf https://cdn.sparkfun.com/assets/c/b/3/a/1/Camera_Module_3_Product_Brief.pdf
LIST
# Open Duck Mini parts (Apache-2.0, github.com/apirrone/Open_Duck_Mini), pinned to a commit
mkdir -p vendor/odm
ODM=https://raw.githubusercontent.com/apirrone/Open_Duck_Mini/b23317a485b3cec7d8417f352478778b3475173c
R=mini_bdx/robots/open_duck_mini_v2
for f in $R/robot.xml $R/trunk_top.stl $R/head.stl $R/head_bot_sheet.stl $R/head_roll_mount.stl $R/head_pitch_to_yaw.stl \
         $R/head_yaw_to_roll.stl $R/neck_left_sheet.stl $R/neck_right_sheet.stl $R/roll_bearing.stl \
         $R/sg90.stl $R/antenna.stl $R/left_antenna_holder.stl $R/right_antenna_holder.stl \
         $R/wj-wk00-0122topcabinetcase_95.stl $R/wj-wk00-0123middlecase_56.stl $R/wj-wk00-0124bottomcase_45.stl \
         $R/drive_palonier.stl $R/passive_palonier.stl \
         print/flash_light_module.stl print/flash_reflector_interface.stl; do
  echo "-> vendor/odm/$(basename "$f")"
  curl -fsSL -o "vendor/odm/$(basename "$f")" "$ODM/$f"
done
echo "Done. STS3215, Camera Module 3 and Pi 5 models are also in the Open Duck Mini Onshape CAD."
