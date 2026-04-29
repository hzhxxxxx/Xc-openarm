# OpenArmX ROS 2 核心库 — v2.0

[English](README.md) | 简体中文

OpenArmX 的基础 ROS 2 包，搭配 `openarmx_description`（URDF/xacro/mesh）提供双臂机器人的硬件驱动与运动控制。

> **这是 v2.0 分支**，在原版 6.0 代码基础上新增了 Commander 节点、科里奥利力补偿、关节加速度限制和 `getpose` 工具。

---

## 包含内容

| 包名 | 说明 |
|------|------|
| `openarmx` | 元包，聚合核心组件 |
| `openarmx_hardware` | `ros2_control` 硬件插件，通过 CAN 驱动机械臂与夹爪 |
| `openarmx_bringup` | 启动文件、RViz 配置、夹爪控制指南 |
| `openarmx_bimanual_moveit_config` | 双臂 MoveIt 配置 |
| `openarmx_gravity_comp` | 实时重力 + 科里奥利力前馈补偿节点 |
| `openarmx_commander` | *(v2.0 新增)* MoveIt API 封装为话题接口 |
| `openarmx_interfaces` | *(v2.0 新增)* 自定义消息定义（`JointCommand`、`PoseCommand`） |
| `openarmx_preview_bringup` | 关节运动控制包 |

---

## v2.0 新增内容

### 1. Commander 节点（`openarmx_commander`）

将 `MoveGroupInterface` 封装为话题接口，通过发布 ROS 消息即可控制双臂，无需编写 C++ 代码。

**话题接口：**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/left_arm_named_target` | `String` | 命名位姿（如 `"home"`、`"hands_up"`） |
| `/left_arm_joint_target` | `JointCommand` | 7个关节角度（弧度） |
| `/left_arm_pose_target` | `PoseCommand` | 末端位姿（x, y, z, roll, pitch, yaw），基于 link0 坐标系 |
| `/left_gripper_open` | `Bool` | true=打开，false=关闭 |
| `/right_arm_*` / `/right_gripper_open` | — | 右臂同理 |

> 所有位姿控制均以**夹爪 TCP**（`openarmx_left/right_hand_tcp`）为参考，而非 link7。相机识别到的物体位置可直接发送，无需手动补偿夹爪偏移。

**启动：**
```bash
ros2 launch openarmx_bringup moveit_commander.launch.py
```

**控制命令示例：**
```bash
# 命名位姿
ros2 topic pub --once /left_arm_named_target example_interfaces/msg/String "{data: 'home'}"

# 关节角度控制
ros2 topic pub --once /left_arm_joint_target openarmx_interfaces/msg/JointCommand \
  "{joint_positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# 末端位姿控制
ros2 topic pub --once /left_arm_pose_target openarmx_interfaces/msg/PoseCommand \
  "{x: 0.3, y: 0.2, z: 0.4, roll: 0.0, pitch: 0.0, yaw: 0.0, cartesian_path: false}"

# 夹爪控制
ros2 topic pub --once /left_gripper_open example_interfaces/msg/Bool "{data: true}"
```

**`getpose` 工具** — 快速查看当前双臂位姿和关节状态：
```bash
getpose
```

若命令不可用，在 `~/.bashrc` 中添加：
```bash
alias getpose='python3 ~/openarmx_ws/src/openarmx_ros2/openarmx_bringup/launch/get_pose_jointstates.py'
```

---

### 2. 科里奥利力补偿（`openarmx_gravity_comp`）

原版 `gravity_comp_node` 只补偿了重力 `g(q)`，v2.0 额外补偿了科里奥利/离心力项 `C(q,q̇)q̇`：

```
τ_ff = g_scale × τ_gravity + c_scale × τ_coriolis
```

新增参数，运行时可调，无需重启节点：
```bash
ros2 param set /gravity_comp_node enable_coriolis true   # 开启/关闭
ros2 param set /gravity_comp_node c_scale 1.0            # 缩放系数
```

> **说明：** 科里奥利补偿改善了关节跟踪平滑度。对于底盘晃动问题，主要来源是惯性项 `M(q)q̈`（需要关节加速度），该项已尝试补偿但效果不佳，见下方说明。

---

### ~~3. 惯性前馈补偿（已尝试，已放弃）~~

**思路：** 对 `/joint_states` 速度字段做数值微分估算 q̈，乘以惯量矩阵对角元素：

```text
τ_m = diag(M(q)) × q̈_filtered
```

**实测结果：失败。** 开启后产生明显冲过头现象，关闭后反而更好。

**原因：** 低通滤波引入相位延迟，补偿力矩总是比实际加速度晚到，在错误时机叠加了额外力矩，加剧过冲而非消除。这是数值微分方案的根本缺陷，无法通过调参解决。

**正确做法（暂未实现）：** 从规划轨迹直接获取无噪声的 q̈_d，但 `forward_position_controller`（VR 遥操）没有轨迹信息，此方案对遥操模式无效。**当前代码已还原，不含惯性补偿。**

---

### 3. 关节加速度限制（`openarmx_bimanual_moveit_config`）

在 `joint_limits.yaml` 中为所有关节开启了加速度限制，数值来源于录制真实运动 `/joint_states` 数据，对速度微分估算峰值加速度（×1.2 安全系数）。

| 关节 | max_acceleration (rad/s²) |
|------|--------------------------|
| joint1 | 11.0 |
| joint2 | 10.0 |
| joint3 | 25.0 |
| joint4 | 23.0 |
| joint5 | 11.0 |
| joint6 | 15.0 |
| joint7 | 16.0 |

**推荐 MoveIt Scaling 参数**（RViz Planning 面板）：
- Velocity Scaling：`0.2 ~ 0.3`
- Acceleration Scaling：`0.2 ~ 0.3`

默认值 0.1 偏保守，调高后动作明显流畅，对底盘冲击感也更均匀。

> 以上限制**只对 MoveIt 规划的轨迹生效**。VR 遥操作使用 `forward_position_controller` 直接透传位置指令，完全绕过 MoveIt 规划管线，加速度限制对其无效。

---

## 环境要求

- Ubuntu 22.04，ROS 2 Humble
- 编译：`colcon`、`ament_cmake`、C++17
- ROS 依赖：`rclcpp`、`pluginlib`、`hardware_interface`、`ros2_control`、MoveIt、KDL（`orocos_kdl`、`kdl_parser`）
- 系统：SocketCAN（建议安装 `can-utils`）

## 工作区准备

```bash
sudo apt-get install python3-vcstool -y
mkdir -p ~/openarmx_ws/src && cd ~/openarmx_ws/src
git clone -b 2.0 https://github.com/hzhxxxxx/Xc-openarm.git openarmx_ros2
vcs import < openarmx_ros2/openarmx_minimal.repos
rosdep install --from-paths . --ignore-src -r -y
```

## 安装 OpenArmX CAN 驱动

```bash
sudo dpkg -i openarmx-can_1.0.0_amd64.deb
```

## 编译

```bash
cd ~/openarmx_ws
colcon build
source install/setup.bash
```

## 运行

```bash
# 实机启动：MoveIt + 重力补偿 + Commander
ros2 launch openarmx_bimanual_moveit_config demo.launch.py \
    control_mode:=mit \
    enable_forward_effort:=true

# 启动 Commander 节点（另开终端）
ros2 launch openarmx_bringup moveit_commander.launch.py

# VR 遥操作模式（forward_position_controller）
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can0 \
    left_can_interface:=can1 \
    control_mode:=mit \
    robot_controller:=forward_position_controller \
    enable_forward_effort:=true
```

## 许可证

本作品采用知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司
