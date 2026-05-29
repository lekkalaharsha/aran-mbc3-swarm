$src = 'C:\Users\BOSON-229\Downloads\New folder\model.sdf'
$dst = 'C:\Users\BOSON-229\Downloads\New folder\mbc3_exact_v3.sdf'
$t = [System.IO.File]::ReadAllText($src)

function SR($old, $new, [string]$label='') {
    $cnt = ([regex]::Matches($script:t, [regex]::Escape($old))).Count
    if ($cnt -eq 0) { Write-Warning ("NO MATCH: $label => " + $old.Substring(0,[Math]::Min(60,$old.Length))) }
    else { $script:t = $script:t.Replace($old, $new); Write-Host ("  OK x$cnt  $label") }
}

# ==============================================================
# 1. MASS  3.834 -> 5.83 kg  (from assembly drawing spec)
# ==============================================================
SR '<mass>3.8341000000000012</mass>' '<mass>5.83</mass>' 'mass'

# ==============================================================
# 2. INERTIA  scale factor = (5.83/3.834) x (0.365/0.360)^2 = 1.563
# ==============================================================
SR '<ixx>0.065612062329103299</ixx>' '<ixx>0.10255</ixx>'    'Ixx'
SR '<ixy>-1.6589885501164771e-05</ixy>' '<ixy>-2.593e-05</ixy>'   'Ixy'
SR '<ixz>-4.0273143553898327e-05</ixz>' '<ixz>-6.295e-05</ixz>'   'Ixz'
SR '<iyy>0.068092590157866895</iyy>' '<iyy>0.10643</iyy>'    'Iyy'
SR '<iyz>0.0001208451693226548</iyz>' '<iyz>1.889e-04</iyz>'     'Iyz'
SR '<izz>0.088652869552937469</izz>' '<izz>0.13856</izz>'    'Izz'

# ==============================================================
# 3. MOTOR CONSTANTS  630KV + 12-inch 3-blade CF
#    kf = kf_10in x (12/10)^2 = 1.740e-5 x 1.44 = 2.506e-5
#    km = 0.01408 x 1.2 = 0.01690
#    kd = 8.06e-5 x 1.44 = 1.161e-4
#    maxRotVelocity = 838 rad/s  (matches spec "836 calc")
# ==============================================================
SR '<motorConstant>1.740e-05</motorConstant>'    '<motorConstant>2.506e-05</motorConstant>'  'kf 12in'
SR '<momentConstant>0.01408</momentConstant>'    '<momentConstant>0.01690</momentConstant>'   'km 12in'
SR '<rotorDragCoefficient>8.06e-05</rotorDragCoefficient>' '<rotorDragCoefficient>1.161e-04</rotorDragCoefficient>' 'kd 12in'

# ==============================================================
# 4. BATTERY/PAYLOAD BAY COLLISION  -> AERIS-10 housing 150x150x90mm
#    Original: 265x75x59  New: 150x150x90 (square footprint from image)
# ==============================================================
SR '<size>0.26500000000000001 0.074999999999999997 0.058999999999999997</size>' '<size>0.150 0.150 0.090</size>' 'payload_bay_collision'

# ==============================================================
# 5. PROP COLLISION RADIUS  0.127 -> 0.1524 (6 inch = 12-inch dia)
# ==============================================================
SR '<radius>0.127</radius>' '<radius>0.1524</radius>' 'prop_radius_12in'

# ==============================================================
# 6. PROP BLADE VISUAL POSES  scale x,y by 1.2 (10->12 inch)
# ==============================================================
SR '0.055 0 0.0405 0 -0.12217 0.03491'          '0.066 0 0.0405 0 -0.12217 0.03491'          'b1_inner_pos'
SR '0.1005 0 0.0415 0 -0.22689 0.06981'         '0.1206 0 0.0415 0 -0.22689 0.06981'         'b1_outer_pos'
SR '-0.0275 0.04763 0.0405 0 -0.12217 2.12931'  '-0.033 0.05716 0.0405 0 -0.12217 2.12931'   'b2_inner_pos'
SR '-0.05025 0.08703 0.0415 0 -0.22689 2.16421' '-0.0603 0.10444 0.0415 0 -0.22689 2.16421'  'b2_outer_pos'
SR '-0.0275 -0.04763 0.0405 0 -0.12217 -2.05949' '-0.033 -0.05716 0.0405 0 -0.12217 -2.05949' 'b3_inner_pos'
SR '-0.05025 -0.08703 0.0415 0 -0.22689 -2.02459' '-0.0603 -0.10444 0.0415 0 -0.22689 -2.02459' 'b3_outer_pos'

# ==============================================================
# 7. PROP BLADE BOX SIZES  scale by 1.2
#    inner: 0.0704x0.0317x0.003  -> 0.08448x0.03804x0.0036
#    outer: 0.0379x0.01647x0.00195 -> 0.04548x0.01976x0.00234
# ==============================================================
SR '<size>0.0704 0.0317 0.003</size>'          '<size>0.08448 0.03804 0.0036</size>'       'blade_inner_box'
SR '<size>0.0379 0.01647 0.00195</size>'       '<size>0.04548 0.01976 0.00234</size>'      'blade_outer_box'

# ==============================================================
# 8. HUB VISUAL - make it a flat plate (thin: 0.02->0.012)
#    to match the octagonal plate appearance in images
# ==============================================================
SR '<length>0.02</length>' '<length>0.012</length>' 'hub_plate_thin'

# ==============================================================
# 9. INSERT PAYLOAD BAY + DOME VISUALS into base_link
#    Inject before the first joint (end of base_link)
#    Payload bay: 150x150x90mm box at z=0.117 (below hub)
#    Radar dome:  sphere r=40mm at z=0.052 (below payload)
#    Octagonal hub approximation: 2 rotated boxes over existing cylinder
# ==============================================================
$domeInsert = @"
      <visual name='payload_bay_visual'>
        <pose>0 0 0.117 0 0 0</pose>
        <geometry>
          <box>
            <size>0.150 0.150 0.090</size>
          </box>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
          <diffuse>0.15 0.15 0.18 1</diffuse>
          <ambient>0.15 0.15 0.18 1</ambient>
        </material>
      </visual>
      <visual name='payload_bay_panel_front'>
        <pose>0 0.0751 0.117 0 0 0</pose>
        <geometry>
          <box>
            <size>0.148 0.002 0.088</size>
          </box>
        </geometry>
        <material>
          <diffuse>0.10 0.10 0.13 1</diffuse>
          <ambient>0.10 0.10 0.13 1</ambient>
        </material>
      </visual>
      <visual name='radar_dome_visual'>
        <pose>0 0 0.052 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>0.040</radius>
          </sphere>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
          <diffuse>0.08 0.08 0.10 1</diffuse>
          <ambient>0.08 0.08 0.10 1</ambient>
        </material>
      </visual>
      <collision name='radar_dome_collision'>
        <pose>0 0 0.052 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>0.040</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name='hub_octa_box_0deg'>
        <pose>0 0 0.1945 0 0 0</pose>
        <geometry>
          <box>
            <size>0.310 0.130 0.012</size>
          </box>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
          <diffuse>0.125 0.125 0.149999991 1</diffuse>
          <ambient>0.125 0.125 0.149999991 1</ambient>
        </material>
      </visual>
      <visual name='hub_octa_box_45deg'>
        <pose>0 0 0.1945 0 0 0.7854</pose>
        <geometry>
          <box>
            <size>0.310 0.130 0.012</size>
          </box>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
          <diffuse>0.125 0.125 0.149999991 1</diffuse>
          <ambient>0.125 0.125 0.149999991 1</ambient>
        </material>
      </visual>
      <visual name='hub_octa_box_90deg'>
        <pose>0 0 0.1945 0 0 1.5708</pose>
        <geometry>
          <box>
            <size>0.310 0.130 0.012</size>
          </box>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
          <diffuse>0.125 0.125 0.149999991 1</diffuse>
          <ambient>0.125 0.125 0.149999991 1</ambient>
        </material>
      </visual>
      <visual name='hub_octa_box_135deg'>
        <pose>0 0 0.1945 0 0 2.3562</pose>
        <geometry>
          <box>
            <size>0.310 0.130 0.012</size>
          </box>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
          <diffuse>0.125 0.125 0.149999991 1</diffuse>
          <ambient>0.125 0.125 0.149999991 1</ambient>
        </material>
      </visual>
"@

# Find end of base_link (just before first joint) and insert
$insertPoint = "    <joint name='motor_0_spin' type='revolute'>"
if ($t.Contains($insertPoint)) {
    $t = $t.Replace($insertPoint, $domeInsert + "    <joint name='motor_0_spin' type='revolute'>")
    Write-Host "  OK  dome+payload+octa_hub inserted before motor_0_spin joint"
} else {
    Write-Warning "Could not find motor_0_spin joint insertion point"
}

# ==============================================================
# 10. BATTERY PLUGIN values (if present) - update capacity
# ==============================================================
SR '<capacity>5.0</capacity>'  '<capacity>16.0</capacity>' 'battery_cap_16Ah'
SR '<capacity>10.0</capacity>' '<capacity>16.0</capacity>' 'battery_cap_16Ah_v2'

# ==============================================================
# WRITE OUTPUT
# ==============================================================
[System.IO.File]::WriteAllText($dst, $t)
$sz = (Get-Item $dst).Length
Write-Host ""
Write-Host "=== OUTPUT: $dst"
Write-Host "=== SIZE:   $sz bytes"
Write-Host "=== BASED ON: WhatsApp images + assembly drawing"
Write-Host "=== SPECS:  730mm wheelbase, 12x4.5in 3-blade, 630KV, 5.83kg, AERIS-10 payload"
