# Road Surface Repair Machine (RSRM) - Software Documentation

## Overview

The Road Surface Repair Machine is an autonomous robotic system designed to scan and repair road surface defects (potholes). It uses ROS2 (Jazzy) for coordination and micro-ROS for real-time hardware control.

**Key Features:**
- Multi-axis stepper motor control (XY scanning, Z filling)
- Real-time state management
- Time-of-Flight (ToF) sensor-based hole detection
- Automated filling sequence with optimal path planning
- Pump and camera control via micro-ROS
- Foxglove Studio integration for visualization

---

## System Architecture

### Software Components

```
┌─────────────────────────────────────────────────────────┐
│  Main State Repeater (State Manager)                    │
│  - Publishes /main_state                                │
│  - Listens to /change_main_state                        │
│  - Enforces state transition rules                      │
└─────────────────────────────────────────────────────────┘
              ↓         ↓         ↓         ↓
    ┌─────────┴─────────┴─────────┴─────────┐
    ↓         ↓         ↓         ↓         ↓
┌────────┐ ┌───────┐ ┌──────────┐ ┌──────┐ ┌───────────┐
│Stepper │ │Z-Axis │ │Pump/Cam  │ │Wheel │ │Custom Cam │
│  (XY)  │ │Motor  │ │ Control  │ │Motor │ │ Publisher │
└────────┘ └───────┘ └──────────┘ └──────┘ └───────────┘
    ↓         ↓         ↓
┌──────────────────────────────────────────────────┐
│  Processing Nodes                                │
├──────────────────────────────────────────────────┤
│ • ToF Costmap Node (Scanning)                    │
│ • ToF Visualiser (Hole Detection & Analysis)     │
│ • Filling Control (Path Planning & Execution)    │
└──────────────────────────────────────────────────┘
    ↓         ↓         ↓
┌──────────────────────────────────────────────────┐
│  Visualization (Foxglove Studio)                 │
└──────────────────────────────────────────────────┘
```

### State Machine

```
States: homing → free → scanning → wait_for_fill → filling
                 ↑                                      ↓
                 └──────────────────────────────────────┘
                 wait_for_fill_after_homing (if not at home)
```

**State Descriptions:**
- **homing**: Robot homing all axes to origin
- **free**: Ready for next command
- **scanning**: Scanning surface with ToF sensors
- **wait_for_fill**: Waiting after scanning before filling
- **wait_for_fill_after_homing**: Special state after homing completes
- **filling**: Executing hole filling sequence
- **wheel_moving**: Wheel movement (platform repositioning)
- **cam_moving**: Camera adjustment

---

## Hardware Components

### Microcontroller Units (ESP32)

1. **Stepper Controller** (`/dev/esp_stepper`)
   - Controls XY scanning stage
   - A4988 driver (X and Y axes)
   - Endstops for homing

2. **Z-Axis Motor** (`/dev/esp_zaxis`)
   - Controls Z-axis height adjustment
   - A4988 driver
   - For pump positioning

3. **Pump & Camera Controller** (`/dev/esp_cam`)
   - PWM pump control
   - TB6612FNG motor driver for cameras
   - Bidirectional motor control

4. **Wheel Motor** (`/dev/esp_wheel`)
   - Platform movement
   - Velocity control via `/cmd_vel`

### Sensors

- **ToF (Time-of-Flight)**: Published by tof_publisher node
- **Camera**: Custom camera node for visual feedback

---

## Installation & Setup

### Prerequisites

```bash
# Ubuntu 22.04 + ROS2 Jazzy
sudo apt update && sudo apt upgrade

# Install ROS2 Jazzy (if not already installed)
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions
```

### Serial Device Setup

```bash
# Add current user to dialout group (one-time setup)
sudo usermod -a -G dialout $USER
# Log out and back in for permissions to apply

# Create USB device aliases (optional, for consistent naming)
sudo nano /etc/udev/rules.d/99-rsrm-usb.rules

# Add these lines:
# SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY", SYMLINK+="esp_stepper"
# SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="ZZZZ", SYMLINK+="esp_zaxis"
# etc.

sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Build the Project

```bash
cd /home/rsrmstayhard/Road_Surface_Repair_Machine/Software/ros2_jazzy

# Clean (if needed)
rm -rf build install log && colcon clean all

# Build all packages
colcon build --symlink-install

# Source the environment
source install/setup.bash
```

---

## Running Different Functions

### Quick Start: All Systems

```bash
# Terminal 1: Start all micro-ROS agents
ros2 launch rsrm_bringup robot_start.launch.py

# Terminal 2: Start Foxglove visualization
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
# Open http://localhost:8080

# Terminal 3: Monitor state
ros2 topic echo /main_state
ros2 topic echo /warning
```

### Individual System Tests

#### 1. **Homing All Axes**

```bash
ros2 topic pub /change_main_state std_msgs/String "data: homing"
# Wait ~5-10 seconds for homing to complete
# Check: /current_xy_pos should be "0, 0" and /current_z_pos should be "0"
```

#### 2. **Scanning Operation**

```bash
# Prerequisites: Must be homed first
ros2 topic pub /change_main_state std_msgs/String "data: homing"
sleep 5

# Start scanning
ros2 topic pub /change_main_state std_msgs/String "data: scanning"
# tof_costmap_node will control stepper via /target_xy
# Watch /tof_costmap for scan results
sleep 15  # Wait for scan to complete

# View results
ros2 topic echo /tof_result
```

#### 3. **Filling Operation**

```bash
# Prerequisites: Must have completed scanning
ros2 topic pub /change_main_state std_msgs/String "data: homing"
sleep 5

ros2 topic pub /change_main_state std_msgs/String "data: scanning"
sleep 15

ros2 topic pub /change_main_state std_msgs/String "data: wait_for_fill"
sleep 1

ros2 topic pub /change_main_state std_msgs/String "data: filling"
# filling_control node will:
# 1. Home XYZ
# 2. Move to each hole centroid
# 3. Lower Z axis
# 4. Pump material
# 5. Raise Z and move to next hole
```

#### 4. **Manual Motor Control**

```bash
# Move XY stepper
ros2 topic pub /target_xy geometry_msgs/Point "{x: 100, y: 50, z: 0}"
ros2 topic pub /target_xy geometry_msgs/Point "{x: -100, y: -50, z: 0}"

# Move Z axis
ros2 topic pub /target_z std_msgs/Float32 "data: 15.0"
ros2 topic pub /target_z std_msgs/Float32 "data: 0.0"

# Control pump (duration in milliseconds)
ros2 topic pub /pump_cmd std_msgs/Int32 "data: 3000"  # Pump for 3 seconds

# Control camera
ros2 topic pub /cam_control std_msgs/String "data: up"
ros2 topic pub /cam_control std_msgs/String "data: down"

# Move wheels
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## ROS2 Topics Reference

### Publishers (System → User)

| Topic | Type | Description |
|-------|------|-------------|
| `/main_state` | `std_msgs/String` | Current machine state (homing, free, scanning, etc.) |
| `/warning` | `std_msgs/String` | Warning/error messages with timestamps |
| `/current_xy_pos` | `geometry_msgs/PointStamped` | Current XY position (steps) |
| `/current_z_pos` | `std_msgs/Float32` | Current Z position (meters) |
| `/current_pos` | `geometry_msgs/PointStamped` | Combined position with timestamp |
| `/tof_costmap` | `Custom Message` | Raw ToF data with XY coordinates |
| `/tof_result` | `Custom Message` | Detected holes (centroid, volume, depth) |
| `/xy_status` | `std_msgs/String` | Stepper system status |
| `/xy_minmax` | `std_msgs/String` | XY axis limits |

### Subscribers (User → System)

| Topic | Type | Description |
|-------|------|-------------|
| `/change_main_state` | `std_msgs/String` | Request state change |
| `/target_xy` | `geometry_msgs/Point` | Target XY position |
| `/target_z` | `std_msgs/Float32` | Target Z position |
| `/pump_cmd` | `std_msgs/Int32` | Pump duration (ms), 0 to stop |
| `/cam_control` | `std_msgs/String` | Camera command ("up"/"down") |
| `/cmd_vel` | `geometry_msgs/Twist` | Wheel velocity command |

---

## State Transition Rules

### Allowed Transitions

| From | To | Condition |
|------|----|---------  |
| Any | `homing` | Always allowed |
| `free` | `scanning` | Allowed |
| `scanning` | `wait_for_fill` | Allowed |
| `wait_for_fill` | `filling` | Must be at home position (0,0,0) |
| `wait_for_fill_after_homing` | `filling` | At home position after homing |
| `free` | `wheel_moving` | Allowed |
| `free` | `cam_moving` | Allowed |
| Any non-free | Any | **DENIED** - Warning published |

### Warning Messages

| Scenario | Warning |
|----------|---------|
| Try to change state while busy | `"[state] is going on, cannot change state"` |
| Try filling without scanning | `"Please do scanning first"` |
| Try filling not at home | `"Homing will be done first"` |
| Request wheel/cam move while busy | `"[state] is going on, cannot change state"` |

---

## Troubleshooting

### Common Issues

**Issue:** Micro-ROS agents not connecting
```bash
# Check serial ports
ls /dev/tty*
dmesg | tail -20

# Check permissions
groups $USER  # Should include 'dialout'

# Restart udev
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Issue:** State transitions rejected
```bash
# Check warning messages
ros2 topic echo /warning

# Verify current state
ros2 topic echo /main_state

# Check prerequisites for the target state
```

**Issue:** Motors not responding
```bash
# Verify micro-ROS agent is running
ros2 node list

# Check node connections
ros2 node info /xy_stepper_node
ros2 node info /z_axis_motor_node

# Test manual command
ros2 topic pub /target_xy geometry_msgs/Point "{x: 10, y: 10, z: 0}"
```

**Issue:** Position not updating
```bash
# Check if homing was completed
ros2 topic echo /main_state  # Should be "free", not "homing"

# Verify stepper is responding
ros2 topic pub /change_main_state std_msgs/String "data: homing"
```

---

## Development Notes

### Node Descriptions

**main_state_repeater**: Central state manager
- Enforces state transitions
- Publishes current state at 10Hz
- Validates position-based transitions
- Publishes warnings for invalid requests

**stepper** (XY): Scans the road surface
- Homes X/Y to center (0, 0)
- Responds to `/target_xy` commands
- Publishes current position with timestamps
- Auto-publishes `/change_main_state = "free"` after homing

**z_axis_motor** (Z): Controls pump height
- Homes to Z = 0
- Responds to `/target_z` commands
- Publishes current Z position at 10Hz

**tof_costmap**: Scanning coordinator
- Listens to `/main_state` for "scanning"
- Controls stepper via `/target_xy`
- Combines ToF data with position
- Publishes `/tof_costmap`

**tof_visualiser**: Hole detection
- Processes `/tof_costmap`
- Detects hole centroids, volumes, depths
- Publishes `/tof_result` with optimal fill order

**filling_control**: Automated filling
- Listens for `/main_state = "filling"`
- Reads hole data from `/tof_result`
- Executes path-planned filling sequence
- Controls stepper, Z-axis, and pump

### Adding New Nodes

1. Create package: `ros2 pkg create my_node --build-type ament_python`
2. Implement node logic
3. Update `rsrm_bringup/launch/robot_start.launch.py`
4. Build: `colcon build --packages-select my_node`
5. Test: `ros2 run my_node my_node`

---

## Safety Features

✅ **State Lock**: Prevents conflicting operations (only one state active)
✅ **Position Validation**: Checks home position before critical operations
✅ **Timeout Protection**: Motors automatically stop if command not acknowledged
✅ **Emergency Stop**: Can transition to "free" from any state
✅ **Warning System**: All violations logged and published

---

## References

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [micro-ROS Documentation](https://micro.ros.org/)
- [Foxglove Studio](https://app.foxglove.dev/)

---

## License

Apache License 2.0

---

## Support

For issues or questions, check:
1. `/warning` topic for error messages
2. ROS2 node status: `ros2 node list`
3. Topic connections: `ros2 topic info [topic_name]`
4. Build logs: `colcon build --packages-select [package] --event-handlers console_direct+`
