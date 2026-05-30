# MBC-3 Migration Guide — HP Pavilion to Native Ubuntu 24.04

> Target machine: HP Pavilion Gaming  
> CPU: AMD Ryzen 7 5800H | RAM: 16 GB | SSD: 512 GB | GPU: RTX 3050 4 GB  
> Decision: **Dual-boot Ubuntu 24.04** (NOT WSL2 — native USB for AERIS-10, full 16 GB RAM, better Gazebo performance)  
> Do this AFTER 31 May Phase 0 submission.

---

## STEP 0 — Backup project (do on current WSL2 machine first)

```bash
# On current WSL2 machine
cd ~
tar czf aran_mbc_backup_$(date +%Y%m%d).tar.gz aran_mbc/
cp aran_mbc_backup_*.tar.gz /mnt/c/Users/BOSON-229/Desktop/

# Also backup ros2_ws built packages (optional — can rebuild from source)
tar czf ros2_ws_backup_$(date +%Y%m%d).tar.gz ros2_ws/
cp ros2_ws_backup_*.tar.gz /mnt/c/Users/BOSON-229/Desktop/
```

Verify backup exists on Windows Desktop before proceeding.

---

## STEP 1 — Windows prep (on HP Pavilion)

### 1.1 Disable Fast Startup
```
Control Panel → Power Options → Choose what power buttons do
→ Uncheck "Turn on fast startup"
→ Save changes
```
Why: Fast Startup leaves Windows partition locked → Ubuntu can't mount it.

### 1.2 Disable Secure Boot
```
Restart → press F10 repeatedly (HP BIOS key)
→ Security tab → Secure Boot → Disabled
→ F10 to save and exit
```
Why: Secure Boot blocks unsigned Ubuntu kernel modules (NVIDIA driver).

### 1.3 Shrink Windows partition
```
Right-click Start → Disk Management
→ Right-click C: → Shrink Volume
→ Shrink by: 250000 MB (250 GB)
→ Shrink
```
Result: 250 GB unallocated space for Ubuntu.

### 1.4 Download Ubuntu 24.04 LTS ISO
```
https://ubuntu.com/download/desktop
→ Ubuntu 24.04.x LTS → Download
```
Flash to USB (8 GB+) using Rufus (Windows) or Balena Etcher.

---

## STEP 2 — Install Ubuntu 24.04

1. Insert USB, restart HP Pavilion
2. Press **F9** for boot menu (HP) → select USB
3. Choose **Try or Install Ubuntu**
4. Select language → **Install Ubuntu**
5. Installation type → **Install Ubuntu alongside Windows Boot Manager**
6. Allocate the 250 GB unallocated space to Ubuntu
7. Set timezone, username, password
8. Install → reboot → remove USB

On reboot: GRUB menu shows Ubuntu and Windows. Default → Ubuntu.

---

## STEP 3 — First boot Ubuntu setup

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install base tools
sudo apt install -y git curl wget build-essential python3-pip python3-venv \
    cmake ninja-build ccache lsb-release gnupg2 software-properties-common \
    net-tools openssh-server htop tmux
```

---

## STEP 4 — NVIDIA driver (RTX 3050, Optimus)

```bash
# Auto-install recommended driver
sudo ubuntu-drivers autoinstall
sudo reboot
```

After reboot:
```bash
# Verify RTX 3050 visible
nvidia-smi
# Expected: NVIDIA RTX 3050, driver version 535.x or newer

# Switch to NVIDIA (performance mode for Gazebo simulation)
sudo prime-select nvidia
sudo reboot

# Verify
prime-select query   # → nvidia
glxinfo | grep renderer   # → NVIDIA GeForce RTX 3050
```

If `ubuntu-drivers` fails:
```bash
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update
sudo apt install nvidia-driver-535
sudo reboot
```

---

## STEP 5 — ROS2 Jazzy

```bash
# Add ROS2 apt repo
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# Full desktop install (rviz2, rqt, demos included)
sudo apt install -y ros-jazzy-desktop

# ROS2 tools
sudo apt install -y ros-dev-tools python3-colcon-common-extensions \
    python3-rosdep python3-argcomplete

# Required sensor + TF packages for radar_fusion
sudo apt install -y ros-jazzy-sensor-msgs-py ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs ros-jazzy-ros-gz-bridge

# Auto-source in new terminals
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify
ros2 --version   # ros2cli 0.x.x (jazzy)
```

---

## STEP 6 — Gazebo Harmonic

```bash
# Add Gazebo apt repo
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable \
    $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install -y gz-harmonic

# Verify
gz sim --version   # Gazebo Harmonic x.x.x
```

---

## STEP 7 — PX4 Autopilot

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Install PX4 deps (Ubuntu script handles all)
bash Tools/setup/ubuntu.sh
sudo reboot

# First build (takes 10-15 min — downloads SITL models)
cd ~/PX4-Autopilot
make px4_sitl_default

# Verify Gazebo SITL works (opens window with x500 quadcopter)
make px4_sitl gz_x500
# Wait for "Ready for takeoff" → Ctrl+C to stop
```

---

## STEP 8 — Python dependencies

```bash
pip3 install --user \
    mavsdk flask flask-socketio requests \
    numpy scipy scikit-learn joblib \
    pyusb pyserial \
    ollama anthropic

# Verify key imports
python3 -c "import mavsdk, flask, numpy, sklearn; print('all OK')"
python3 -c "import usb; print('pyusb OK')"
```

---

## STEP 9 — Restore MBC-3 project

```bash
# Copy backup from USB drive (or Windows partition)
# Windows partition mounts at /media/$USER/... or:
ls /media/$USER/

# Or copy from USB backup drive
cp /media/$USER/YOUR_DRIVE/aran_mbc_backup_*.tar.gz ~/

# Extract
cd ~
tar xzf aran_mbc_backup_*.tar.gz

# Verify
ls ~/aran_mbc/
# Should show: src/ radar_fusion/ aeris10_driver/ new_drone/ worlds/ etc.
```

---

## STEP 10 — Install MBC-3 drone model in PX4

```bash
cd ~/aran_mbc
bash new_drone/install_px4_model.sh

# Verify model installed
ls ~/PX4-Autopilot/Tools/simulation/gz/models/mbc3_radar_drone/
# Should show: model.config  model.sdf

# Verify world files installed
ls ~/PX4-Autopilot/Tools/simulation/gz/worlds/mbc3_*.sdf
```

---

## STEP 11 — Build ROS2 workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
ln -sfn ~/aran_mbc/radar_fusion    ~/ros2_ws/src/radar_fusion
ln -sfn ~/aran_mbc/aeris10_driver  ~/ros2_ws/src/aeris10_driver

# Install Python deps for packages
pip3 install --user pyusb numpy scikit-learn joblib

# Build
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select radar_fusion aeris10_driver --symlink-install

# Source workspace (add to .bashrc)
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify executables
ros2 pkg executables radar_fusion
# → radar_fusion detection_node
# → radar_fusion fusion_node

ros2 pkg executables aeris10_driver
# → aeris10_driver driver_node
```

---

## STEP 12 — AERIS-10 USB setup (when hardware arrives)

```bash
# Find VID/PID from device
lsusb   # look for AERIS-10 or STMicroelectronics entry
# Example: Bus 001 Device 005: ID 0483:ae10 STMicroelectronics AERIS-10

# Update aeris10_usb.py
nano ~/aran_mbc/aeris10_driver/aeris10_driver/aeris10_usb.py
# Change: AERIS10_VID = 0x0483
# Change: AERIS10_PID = 0xAE10   ← update to real PID from lsusb

# Allow non-root USB access
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="ae10", MODE="0666"' \
    | sudo tee /etc/udev/rules.d/99-aeris10.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Rebuild
cd ~/ros2_ws && colcon build --packages-select aeris10_driver
```

---

## STEP 13 — Verification tests

### Test 1: Single drone flight
```bash
# Delete stale PX4 params
rm -f ~/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson

bash ~/aran_mbc/launch.sh
# Expected:
#   Step 3: PX4 ready (match: "Ready for takeoff")
#   Step 3.5: AERIS-10 sim running at 10 Hz
#   Step 4: GCS live → http://localhost:5000
#   Step 5: mission executing, waypoints progressing
```

### Test 2: ROS2 radar pipeline
```bash
# Terminal 1
ros2 run aeris10_driver driver_node --ros-args -p sim_mode:=true

# Terminal 2
ros2 run radar_fusion detection_node
# Expected: "[INFO] Detected 1 target(s): TGT_01 R=200m Az=xxx°"
```

### Test 3: Swarm 5-drone
```bash
rm -f ~/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson
MBC3_MODE=1 bash ~/aran_mbc/swarm_launch.sh
# Browser → http://localhost:5000
# Expected: 5 drone markers on map, radar polar panel, phase updates
```

### Test 4: Failure redistribution
```bash
# While swarm running (after drones at altitude):
bash ~/aran_mbc/tools/kill_drone_sim.sh 2 30
# Expected: "[REDISTRIB] DRONE-2 failed WP X/Y → DRONE-1:N, DRONE-3:M"
```

---

## Performance expectations (native Ubuntu vs WSL2)

| Metric | WSL2 (current) | Native Ubuntu | Improvement |
|--------|----------------|---------------|-------------|
| Gazebo render | ~15–20 FPS | ~45–60 FPS | 3× |
| PX4 SITL CPU | 85–95% | 50–60% | 1.5× |
| 5-drone swarm | occasional stutter | smooth | significant |
| AERIS-10 USB | not accessible | direct access | ✓ |
| RAM available | 14 GB (wslconfig) | 16 GB full | +2 GB |
| RTX 3050 | indirect (HW accel) | direct CUDA | ✓ |

---

## Known issues & fixes

| Issue | Fix |
|-------|-----|
| Gazebo blank window on NVIDIA Optimus | `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia gz sim ...` |
| PX4 make fails: `ninja: error` | `make distclean` then rebuild |
| colcon: `SetupToolsDeprecationWarning` | `pip install --upgrade setuptools` |
| `ros2: command not found` | `source /opt/ros/jazzy/setup.bash` missing in .bashrc |
| MAVSDK: `grpc channel closed` | stale mavsdk_server — `pkill -f mavsdk_server` |
| RTX 3050 not shown in nvidia-smi | `sudo prime-select nvidia && sudo reboot` |
| PX4 arm fail: sensor missing | check SDF sensor names (baro=air_pressure_sensor, gps=navsat_sensor) |
| AERIS-10 USB permission denied | add udev rule (see Step 12) |

---

## .bashrc additions (paste at bottom)

```bash
# ROS2 Jazzy
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# MBC-3 shortcuts
alias swarm='cd ~/aran_mbc && rm -f ~/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson && MBC3_MODE=1 bash swarm_launch.sh'
alias launch1='cd ~/aran_mbc && rm -f ~/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson && bash launch.sh'
alias killdrone='bash ~/aran_mbc/tools/kill_drone_sim.sh'
alias gcs='sensible-browser http://localhost:5000'

# PX4 GPU (RTX 3050 for Gazebo)
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

---

## Estimated setup time

| Step | Time |
|------|------|
| Windows prep + Ubuntu install | 45 min |
| NVIDIA driver + reboot | 15 min |
| ROS2 Jazzy | 20 min |
| Gazebo Harmonic | 10 min |
| PX4 first build | 15–20 min |
| Python deps | 5 min |
| Project restore + ROS2 ws build | 15 min |
| Model install + verification | 20 min |
| **Total** | **~2.5 hours** |

---

*Generated 2026-05-29 | Target: HP Pavilion Ryzen 7 5800H / RTX 3050 / 16GB*
