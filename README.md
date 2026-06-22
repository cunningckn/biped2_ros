# Biped2 摆腿 Demo

基于 **ROS 2 Humble + Gazebo Classic + Pinocchio** 的双足机器人摆腿演示。机器人在 Gazebo 中悬空固定，通过逆运动学驱动双腿沿摆线轨迹交替摆动。

---

## 1. 框架介绍

### 整体架构

```
biped2_ros/
├── biped2_description   # 机器人模型（URDF/xacro、mesh、Gazebo 插件）
├── biped2_kinematics    # Pinocchio 正/逆运动学、摆腿 demo 节点
└── biped2_gazebo        # Gazebo 世界与 launch 入口
```

| 包 | 作用 |
|----|------|
| `biped2_description` | 提供 biped2 URDF；构建时将 mesh 转为 `file://` 绝对路径供 Gazebo 加载；内置关节状态发布与轨迹跟踪插件 |
| `biped2_kinematics` | Pinocchio FK/IK；`foot_lift_demo` 生成摆线足端轨迹并求解关节角；`foot_fk_publisher` 发布足端位姿 |
| `biped2_gazebo` | 启动 Gazebo、spawn 机器人、拉起 demo 节点 |

### 控制流程

1. **Gazebo 插件**订阅 `/joint_trajectory_controller/joint_trajectory`，驱动 10 个腿部关节（无需 `controller_manager`）。
2. **`foot_lift_demo`** 以固定频率计算左右腿相位（相差半个周期），沿矢状面摆线向前抬脚，并通过 IK 转为关节目标。
3. **`foot_fk_publisher`** 将当前足端位姿发布到 `/biped2/left_foot_pose`、`/biped2/right_foot_pose` 供调试。

### Demo 行为概要

- 对称站立：两脚同高，脚底板水平（`AnklePitch = 0` + 踝关节补偿）。
- 摆线轨迹：前向 `-X` 摆腿，可配置抬脚高度与步幅。
- 半周期交错：左右腿相位差 0.5，一腿抬起时另一腿支撑，类似步行节奏。

### 主要参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `base_height`（xacro） | `0.37` m | 机身悬空高度（`biped2.urdf.xacro` 顶部） |
| `stand_foot_z` | `0.0` m | 站立时脚的目标高度 |
| `lift_height` | `0.10` m | 摆线峰值抬脚高度 |
| `step_length` | `0.08` m | 摆线前向步幅 |
| `cycle_duration` | `2.0` s | 单腿完整摆线周期 |
| `leg_phase_offset` | `0.5` | 左右腿相位差（0.5 = 半周期交错） |

---

## 2. 快速上手

### 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic

### 安装依赖

```bash
cd biped2_ros
./install_deps.sh
```

### 编译

```bash
source /opt/ros/humble/setup.bash
cd biped2_ros
colcon build --symlink-install
source install/setup.bash
```

若 Python 节点报 `ModuleNotFoundError: biped2_kinematics`：

```bash
colcon build --symlink-install --packages-select biped2_kinematics
source install/setup.bash
```

### 运行 Demo

先清理残留 Gazebo / demo 进程（避免端口占用或旧节点干扰）：

```bash
./scripts/stop_gazebo.sh
```

启动：

```bash
ros2 launch biped2_gazebo biped2_gazebo.launch.py
```

**时间线（约）：**

| 时间 | 事件 |
|------|------|
| 0 s | 清理旧进程，启动 Gazebo |
| ~7 s | 机器人在世界中 spawn |
| ~12 s | `foot_lift_demo` 开始，双腿交替摆线摆动 |

### 验证（可选）

```bash
ros2 topic echo /joint_states
ros2 topic echo /biped2/right_foot_pose
ros2 topic hz /joint_trajectory_controller/joint_trajectory
```

### 调整参数示例

```bash
ros2 launch biped2_gazebo biped2_gazebo.launch.py

# 或单独运行 demo 节点并改参：
ros2 run biped2_kinematics foot_lift_demo --ros-args \
  -p lift_height:=0.12 \
  -p step_length:=0.10 \
  -p cycle_duration:=1.5
```

修改整机高度：编辑 `src/biped2_description/urdf/biped2.urdf.xacro` 顶部 `base_height`，重新 `colcon build --packages-select biped2_description` 后 launch。

### 常见问题

| 现象 | 处理 |
|------|------|
| `gzserver exit 255` / 端口占用 | `./scripts/stop_gazebo.sh` 后重试 |
| Gazebo 只有地面、没有机器人 | 等待约 7 s；确认已 `source install/setup.bash` |
| `Package 'biped2_gazebo' not found` | 在工作空间执行 `source install/setup.bash` |
| 模型不显示 | `colcon build --packages-select biped2_description` 重建 mesh 路径 |
| `frame_id` 报错 | `./scripts/stop_gazebo.sh` 清理残留 `foot_lift_demo` 进程 |

`colcon build` 时 `packaging>=22` 警告可忽略。
