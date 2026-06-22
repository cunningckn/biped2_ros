# Biped2 Gazebo + Pinocchio Demo

ROS 2 Humble workspace for simulating the biped2 robot in Gazebo Classic with Pinocchio FK/IK control.

## 1. Install dependencies

```bash
cd /home/ws0/Workspace/ckn/biped2_ros
./install_deps.sh
```

## 2. Build

```bash
source /opt/ros/humble/setup.bash
cd /home/ws0/Workspace/ckn/biped2_ros
colcon build --symlink-install
source install/setup.bash
```

If Python nodes fail with `ModuleNotFoundError: biped2_kinematics`, rebuild only the kinematics package:

```bash
colcon build --symlink-install --packages-select biped2_kinematics
source install/setup.bash
```

## 3. Run

先确保没有残留 Gazebo 进程（否则会报 `Address already in use`，`gzserver` exit 255）：

```bash
./scripts/stop_gazebo.sh
```

再启动：

```bash
ros2 launch biped2_gazebo biped2_gazebo.launch.py
```

## 4. Verify topics

```bash
ros2 topic echo /biped2/right_foot_pose
ros2 topic echo /joint_states
ros2 topic hz /joint_trajectory_controller/joint_trajectory
```

## Architecture

- `biped2_description`: URDF/xacro、meshes、Gazebo 关节插件
- `biped2_kinematics`: Pinocchio FK publisher + alternating foot-lift IK demo
- `biped2_gazebo`: Gazebo world and launch file

关节控制使用 Gazebo 插件（`joint_state_publisher` + `joint_pose_trajectory`），无需 `controller_manager`。

## Troubleshooting

| 现象 | 原因 | 处理 |
|------|------|------|
| Gazebo 只有地面、没有机器人 | `gzserver` 启动失败（exit 255，端口被占用）或 spawn 未完成 | 重新 `colcon build` 后 launch 会自动清理残留进程；约 **7 秒** 后机器人才 spawn |
| 终端报 `Package 'biped2_gazebo' not found` | 未 source 工作空间 | `source /home/ws0/Workspace/ckn/biped2_ros/install/setup.bash` |
| 机器人 spawn 成功但看不见模型 | mesh 路径 `package://` 未被 Gazebo 解析 | 已改为构建时写入 `file://` 绝对路径，需 `colcon build --packages-select biped2_description` |
| `spawn_entity.py: error: unrecognized arguments: -J ...` | 旧版 launch 文件 | `colcon build --symlink-install && source install/setup.bash` |
| `ModuleNotFoundError: biped2_kinematics` | Python 包未正确安装 | `colcon build --symlink-install --packages-select biped2_kinematics` |
| `gzserver ... exit code 255` / `Address already in use` | 已有 Gazebo 在运行 | `./scripts/stop_gazebo.sh` 后重试 |
| `Could not contact service /controller_manager/...` | 已改用 Gazebo 关节插件，不再使用 controller_manager；请重新 `colcon build` |

`colcon build` 开头的 `packaging>=22` 警告可忽略，不影响编译。

## Notes

- 机器人 spawn 高度 z=0.65 m，`base_link` 关闭重力以保持 demo 稳定。
- IK 仅调整每条腿的 `HipPitch / KneePitch / AnklePitch`。
- Demo 周期：stand -> 抬右脚 -> stand -> 抬左脚。
