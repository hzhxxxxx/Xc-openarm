# MoveIt API 封装成 Commander 节点说明

## 需要的文件

在原版 openarmx_ros2 基础上，需要创建以下文件：

### 1. 消息接口包
```
openarmx_ros2/
└── openarmx_interfaces/
    ├── msg/
    │   ├── JointCommand.msg
    │   └── PoseCommand.msg
    ├── CMakeLists.txt
    └── package.xml
```

### 2. Commander 节点包
```
openarmx_ros2/
└── openarmx_commander/
    ├── src/
    │   └── moveit_commander.cpp
    ├── CMakeLists.txt
    └── package.xml
```

### 3. 启动文件和工具脚本
```
openarmx_ros2/
└── openarmx_bringup/
    └── launch/
        ├── moveit_commander.launch.py
        └── get_pose_jointstates.py
```

所有文件路径：
- `/home/hzh/openarmx_ws/src/openarmx_ros2/openarmx_interfaces/`
- `/home/hzh/openarmx_ws/src/openarmx_ros2/openarmx_commander/`
- `/home/hzh/openarmx_ws/src/openarmx_ros2/openarmx_bringup/launch/moveit_commander.launch.py`
- `/home/hzh/openarmx_ws/src/openarmx_ros2/openarmx_bringup/launch/get_pose_jointstates.py`

## 整体思路

将 MoveIt 的 `MoveGroupInterface` API 封装成 ROS 话题接口，通过发布话题消息来控制双臂机器人，而不需要直接编写 C++ 代码调用 MoveIt API。

**核心组件：**
- `openarmx_interfaces` - 自定义消息包（JointCommand, PoseCommand）
- `openarmx_commander` - Commander 节点包（订阅话题并调用 MoveIt API）
- `moveit_commander.cpp` - 主程序源文件
- `moveit_commander.launch.py` - 启动文件

**设计模式：**
- 4个 MoveGroupInterface 对象：`left_arm`, `right_arm`, `left_gripper`, `right_gripper`
- 8个订阅者：对应左右臂的3种控制方式 + 左右夹爪
- 共享功能函数 + 回调函数传参区分左右

## 数据类型

### JointCommand.msg
```
float64[7] joint_positions  # 7个关节角度（弧度）
```

### PoseCommand.msg
```
float64 x, y, z              # 位置
float64 roll, pitch, yaw     # 姿态（欧拉角）
bool cartesian_path          # 是否笛卡尔直线运动
```

### 标准消息
- `example_interfaces/msg/String` - 命名位姿
- `example_interfaces/msg/Bool` - 夹爪控制

## 话题接口

### 左臂控制
- `/left_arm_named_target` (String) - 命名位姿（如 "home", "hands_up"）
- `/left_arm_joint_target` (JointCommand) - 关节角度控制
- `/left_arm_pose_target` (PoseCommand) - 末端位姿控制

### 右臂控制
- `/right_arm_named_target` (String)
- `/right_arm_joint_target` (JointCommand)
- `/right_arm_pose_target` (PoseCommand)

### 夹爪控制
- `/left_gripper_open` (Bool) - true=打开, false=关闭
- `/right_gripper_open` (Bool)

## 基坐标系与末端执行器

- **左臂基坐标系**：`openarmx_left_link0`
- **右臂基坐标系**：`openarmx_right_link0`
- **左臂末端执行器（TCP）**：`openarmx_left_hand_tcp`
- **右臂末端执行器（TCP）**：`openarmx_right_hand_tcp`

**重要说明：**
- 所有位姿控制都已适配为**夹爪 TCP 坐标**（而非机械臂末端 link7）
- 发送 pose_target 时，MoveIt 会让夹爪中心到达目标位置
- 这样可以直接用相机识别的物体位置作为目标，无需手动补偿夹爪偏移

## 使用流程

### 1. 启动 Commander 节点
```bash
ros2 launch openarmx_bringup moveit_commander.launch.py
```

### 2. 发送控制命令

**命名位姿控制：**
```bash
ros2 topic pub --once /left_arm_named_target example_interfaces/msg/String "{data: 'home'}"
ros2 topic pub --once /right_arm_named_target example_interfaces/msg/String "{data: 'hands_up'}"
```

**关节角度控制：**
```bash
ros2 topic pub --once /left_arm_joint_target openarmx_interfaces/msg/JointCommand "{joint_positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

**末端位姿控制：**
```bash
ros2 topic pub --once /left_arm_pose_target openarmx_interfaces/msg/PoseCommand "{x: 0.3, y: 0.2, z: 0.4, roll: 0.0, pitch: 0.0, yaw: 0.0, cartesian_path: false}"
```

**夹爪控制：**
```bash
ros2 topic pub --once /left_gripper_open example_interfaces/msg/Bool "{data: true}"   # 打开
ros2 topic pub --once /right_gripper_open example_interfaces/msg/Bool "{data: false}"  # 关闭
```

## 命名位姿（SRDF 中定义）

**手臂：**
- `home` - 归零位姿
- `hands_up` - 举手位姿

**夹爪：**
- `open` - 完全打开 (0.044)
- `half_closed` - 半闭合 (0.022)
- `closed` - 完全闭合 (0.0)

## 获取当前位姿和关节状态

使用 `getpose` 命令快速查看当前双臂状态：

```bash
getpose
```

**输出示例：**
```
以下pose都是夹爪tcp在各自link0基坐标系下的坐标(moveit中posetarget也做了适配,其实也是夹爪tcp的坐标)

=== LEFT ARM ===
pose: [0.301000, 0.221000, 0.267000, -3.141593, -0.000000, -0.000000]
joint_states: [0.000000, 0.500000, 0.000000, 1.570000, 0.000000, 0.000000, 0.000000]

pose_command:
ros2 topic pub --once /left_arm_pose_target openarmx_interfaces/msg/PoseCommand "{x: 0.301000, y: 0.221000, z: 0.267000, roll: -3.141593, pitch: -0.000000, yaw: -0.000000, cartesian_path: false}"

=== RIGHT ARM ===
pose: [0.301000, -0.221000, 0.267000, -3.141593, -0.000000, -0.000000]
joint_states: [0.000000, 0.500000, 0.000000, 1.570000, 0.000000, 0.000000, 0.000000]

pose_command:
ros2 topic pub --once /right_arm_pose_target openarmx_interfaces/msg/PoseCommand "{x: 0.301000, y: -0.221000, z: 0.267000, roll: -3.141593, pitch: -0.000000, yaw: -0.000000, cartesian_path: false}"
```

**说明：**
- `pose` - 夹爪 TCP 在基坐标系下的 6D 位姿 [x, y, z, roll, pitch, yaw]
- `joint_states` - 7 个关节的角度（弧度）
- `pose_command` - 可直接复制执行的命令，让机械臂回到当前位姿

### 设置别名

如果 `getpose` 命令不可用，在 `~/.bashrc` 中添加：
```bash
alias getpose='python3 /home/hzh/openarmx_ws/src/openarmx_ros2/openarmx_bringup/launch/get_pose_jointstates.py' 
```

然后执行：
```bash
source ~/.bashrc
```

## 注意事项

1. 启动 Commander 前需要确保 MoveIt 配置已加载
2. 关节角度单位为弧度
3. 位姿的坐标系参考对应臂的 link0，但控制的是夹爪 TCP 位置
4. `cartesian_path=true` 时为直线运动，只有 100% 规划成功才执行
