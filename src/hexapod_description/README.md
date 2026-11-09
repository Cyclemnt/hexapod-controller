# hexapod_description

ROS 2 robot description package for a biomimetic 18-DOF hexapod robot powered by Feetech STS3215-C018 serial bus smart servos.

## Overview

- **Robot Type:** 18-DOF Hexapod (6 legs $\times$ 3 joints: Coxa, Femur, Tibia)
- **Actuators:** Feetech STS3215-C018 (Stall Torque: ~3.0 N·m @ 7.4V, Max Speed: ~5.8 rad/s)
- **Build System:** `ament_cmake`
- **Target Environments:** ROS 2 Jazzy, Gazebo Sim (Harmonic / Sim 8), RViz2, Reinforcement Learning (Isaac Gym, Isaac Lab, MuJoCo)

---

## Kinematic & Naming Conventions

All joints and links follow standard international biomimetic robotics conventions in English `snake_case`:

### Leg Identification
| Leg Index | Location | Coxa Joint (Yaw) | Femur Joint (Pitch) | Tibia Joint (Pitch) |
| :--- | :--- | :--- | :--- | :--- |
| **Leg 1** | Front-Left (LF) | `leg_1_coxa_joint` | `leg_1_femur_joint` | `leg_1_tibia_joint` |
| **Leg 2** | Middle-Left (LM) | `leg_2_coxa_joint` | `leg_2_femur_joint` | `leg_2_tibia_joint` |
| **Leg 3** | Rear-Left (LR) | `leg_3_coxa_joint` | `leg_3_femur_joint` | `leg_3_tibia_joint` |
| **Leg 4** | Rear-Right (RR) | `leg_4_coxa_joint` | `leg_4_femur_joint` | `leg_4_tibia_joint` |
| **Leg 5** | Middle-Right (RM) | `leg_5_coxa_joint` | `leg_5_femur_joint` | `leg_5_tibia_joint` |
| **Leg 6** | Front-Right (RF) | `leg_6_coxa_joint` | `leg_6_femur_joint` | `leg_6_tibia_joint` |

### Link Hierarchy per Leg $i \in \{1..6\}$
- `base_footprint` (Root frame on ground / REP-120 compliant)
  - `base_link` (Main chassis with real mass and inertia)
    - `leg_i_servo_1` (Fixed to `base_link`)
      - `leg_i_coxa` (Rotates yaw around Z-axis)
        - `leg_i_servo_2` (Pitch servo mounted on Coxa)
          - `leg_i_femur` (Main leg limb)
            - `leg_i_servo_3` (Pitch servo mounted on Femur)
              - `leg_i_tibia` (Lower limb touching the ground)

---

## Package Structure

```
hexapod_description/
├── CMakeLists.txt              # ament_cmake build rules
├── package.xml                 # Package manifest & dependencies
├── README.md                   # Documentation
├── config/
│   ├── display.rviz            # RViz2 visualization setup
│   ├── gazebo.rviz             # RViz2 setup for Gazebo
│   ├── hexapod_controllers.yaml # ros2_control configuration (joint_trajectory_controller)
│   └── ros_gz_bridge_gazebo.yaml # Bridge config (/clock and /joint_states)
├── launch/
│   ├── display.launch.py       # Robot State Publisher + RViz2 + Joint GUI sliders
│   └── gazebo.launch.py        # Gazebo Sim + RSP + Bridge + Auto Controller Spawners
├── meshes/                     # 37 clean, snake_case STL files
└── urdf/
    ├── hexapod.xacro           # Main robot description (links, joints, limits, footprint)
    ├── hexapod.gazebo          # Gazebo Sim plugins & physical properties
    ├── hexapod.ros2_control.xacro # ros2_control interfaces (18 joints)
    └── materials.xacro         # RViz color definitions
```

---

## Build & Usage

### 1. Build the Workspace
From the root of your colcon workspace:
```bash
colcon build --packages-select hexapod_description
source install/setup.bash
```

### 2. Real-Time RViz2 Visualization & Joint Sliders
Inspect the kinematics and move the legs interactively:
```bash
ros2 launch hexapod_description display.launch.py
```

### 3. Gazebo Sim Simulation
Spawn the robot in Gazebo Sim and automatically load the ROS 2 controllers:
```bash
ros2 launch hexapod_description gazebo.launch.py
```

Check loaded controllers:
```bash
ros2 control list_controllers
```

---

## Next Steps / TODOs

- [ ] **Physical Mass & Inertia:** Measure the real mass and center of gravity of the chassis to update `base_link` in `urdf/hexapod.xacro`.
- [ ] **Mechanical Angle Limits:** Refine `coxa_limit_lower/upper`, `femur_limit_lower/upper`, and `tibia_limit_lower/upper` in `urdf/hexapod.xacro` based on mechanical stops.
- [ ] **Foot Contact Points:** Add virtual tip/foot frames (`leg_i_foot`) at the bottom of each tibia for inverse kinematics and ground contact detection.
- [ ] **Reinforcement Learning:** Train locomotion policies directly in Isaac Gym, Isaac Lab or MuJoCo using the clean URDF and DART collision primitives.
