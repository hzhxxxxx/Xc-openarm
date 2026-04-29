# OpenArmX ROS 2 Core — v2.0

English | [简体中文](README_CN.md)

Core ROS 2 packages for OpenArmX. Combined with `openarmx_description` (URDF/xacro/meshes), they provide the baseline hardware bringup and motion control stack for OpenArmX V10.

> **This is the v2.0 branch**, which adds the Commander node, Coriolis compensation, acceleration limits, and the `getpose` tool on top of the original 6.0 codebase.

---

## Package Contents

| Package | Description |
|---------|-------------|
| `openarmx` | Metapackage that pulls the core components |
| `openarmx_hardware` | `ros2_control` hardware plugin (`OpenArmX_v10HW`) driving the arm/gripper via CAN |
| `openarmx_bringup` | Launch files, RViz config, gripper control guides |
| `openarmx_bimanual_moveit_config` | MoveIt config for bimanual setup |
| `openarmx_gravity_comp` | Real-time gravity + Coriolis feedforward compensation node |
| `openarmx_commander` | *(v2.0 new)* MoveIt API wrapped as ROS topic interface |
| `openarmx_interfaces` | *(v2.0 new)* Custom message definitions (`JointCommand`, `PoseCommand`) |
| `openarmx_preview_bringup` | Joint motion control package |

---

## v2.0 Changes

### 1. Commander Node (`openarmx_commander`)

Wraps `MoveGroupInterface` as a topic-based API so the arm can be controlled by publishing ROS messages without writing C++ code.

**Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `/left_arm_named_target` | `String` | Named pose (e.g. `"home"`, `"hands_up"`) |
| `/left_arm_joint_target` | `JointCommand` | 7-DOF joint angles (rad) |
| `/left_arm_pose_target` | `PoseCommand` | End-effector pose (x, y, z, roll, pitch, yaw) in link0 frame |
| `/left_gripper_open` | `Bool` | true = open, false = close |
| `/right_arm_*` / `/right_gripper_open` | — | Same for right arm |

> All pose targets use the **gripper TCP** frame (`openarmx_left/right_hand_tcp`), not link7. Camera-detected object positions can be sent directly without gripper offset compensation.

**Launch:**
```bash
ros2 launch openarmx_bringup moveit_commander.launch.py
```

**Example commands:**
```bash
# Named pose
ros2 topic pub --once /left_arm_named_target example_interfaces/msg/String "{data: 'home'}"

# Joint target
ros2 topic pub --once /left_arm_joint_target openarmx_interfaces/msg/JointCommand \
  "{joint_positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# Pose target
ros2 topic pub --once /left_arm_pose_target openarmx_interfaces/msg/PoseCommand \
  "{x: 0.3, y: 0.2, z: 0.4, roll: 0.0, pitch: 0.0, yaw: 0.0, cartesian_path: false}"

# Gripper
ros2 topic pub --once /left_gripper_open example_interfaces/msg/Bool "{data: true}"
```

**`getpose` tool** — quickly print current arm pose and joint states:
```bash
getpose
```
Add to `~/.bashrc` if not already present:
```bash
alias getpose='python3 ~/openarmx_ws/src/openarmx_ros2/openarmx_bringup/launch/get_pose_jointstates.py'
```

---

### 2. Coriolis Compensation (`openarmx_gravity_comp`)

The original `gravity_comp_node` only compensated gravity `g(q)`. v2.0 also compensates the Coriolis/centrifugal term `C(q,q̇)q̇`:

```
τ_ff = g_scale × τ_gravity + c_scale × τ_coriolis
```

New runtime-tunable parameters (no restart needed):
```bash
ros2 param set /gravity_comp_node enable_coriolis true   # enable/disable
ros2 param set /gravity_comp_node c_scale 1.0            # scaling factor
```

> **Note:** Coriolis compensation improves joint tracking smoothness. For chassis vibration caused by arm motion, the dominant source is the inertia term `M(q)q̈`, which is not yet compensated.

---

### 3. Acceleration Limits (`openarmx_bimanual_moveit_config`)

Acceleration limits are now enabled in `joint_limits.yaml`. Values were derived by recording `/joint_states` during real motion and computing peak accelerations via velocity differentiation (×1.2 safety factor).

| Joint | max_acceleration (rad/s²) |
|-------|--------------------------|
| joint1 | 11.0 |
| joint2 | 10.0 |
| joint3 | 25.0 |
| joint4 | 23.0 |
| joint5 | 11.0 |
| joint6 | 15.0 |
| joint7 | 16.0 |

**Recommended MoveIt scaling** (RViz Planning panel):
- Velocity Scaling: `0.2 ~ 0.3`
- Acceleration Scaling: `0.2 ~ 0.3`

> These limits only apply to MoveIt-planned trajectories. `forward_position_controller` (VR teleoperation) bypasses the planning pipeline entirely and is unaffected.

---

## Requirements

- Ubuntu 22.04, ROS 2 Humble
- Build: `colcon`, `ament_cmake`, C++17
- ROS deps: `rclcpp`, `pluginlib`, `hardware_interface`, `ros2_control`, MoveIt, KDL (`orocos_kdl`, `kdl_parser`)
- System: SocketCAN enabled (`can-utils`)

## Workspace Setup

```bash
sudo apt-get install python3-vcstool -y
mkdir -p ~/openarmx_ws/src && cd ~/openarmx_ws/src
git clone -b 2.0 https://github.com/hzhxxxxx/Xc-openarm.git openarmx_ros2
vcs import < openarmx_ros2/openarmx_minimal.repos
rosdep install --from-paths . --ignore-src -r -y
```

## Install OpenArmX CAN Driver

```bash
sudo dpkg -i openarmx-can_1.0.0_amd64.deb
```

## Build

```bash
cd ~/openarmx_ws
colcon build
source install/setup.bash
```

## Run

```bash
# Real robot with MoveIt + gravity compensation + commander
ros2 launch openarmx_bimanual_moveit_config demo.launch.py \
    control_mode:=mit \
    enable_forward_effort:=true

# Launch commander node (in a separate terminal)
ros2 launch openarmx_bringup moveit_commander.launch.py

# VR teleoperation (forward_position_controller)
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can0 \
    left_can_interface:=can1 \
    control_mode:=mit \
    robot_controller:=forward_position_controller \
    enable_forward_effort:=true
```

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.
