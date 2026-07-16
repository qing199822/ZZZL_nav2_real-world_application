# COD Gazebo Simulator — RM2026 导航仿真环境

## 概述

本包为 COD RM2026 导航项目提供 Gazebo 仿真测试环境，用于在无需实际硬件的情况下验证导航栈。

## 依赖

- **Ignition Gazebo Fortress** (version 6) — `sudo apt install ignition-fortress`
- **ros_gz** — ROS2-Gazebo bridge (`ros-humble-ros-gz-bridge`, `ros-humble-ros-gz-sim`)
- **xacro** — URDF 模型处理
- **rmu_gazebo_simulator** — 提供比赛场地 world 模型（如需比赛场地仿真）
- **cod_bringup** — COD 导航启动包
- **fake_vel_transform** — 速度坐标变换
- **pointcloud_to_laserscan** — 点云转激光扫描

## 快速开始

### 1. 安装 Ignition Gazebo
```bash
sudo apt install ignition-fortress ros-humble-ros-gz-bridge ros-humble-ros-gz-sim
```

### 2. 编译
```bash
cd <workspace>
colcon build --symlink-install --packages-select cod_gazebo_simulator
source install/setup.bash
```

### 3. 启动仿真 (空世界 + 机器人)
```bash
ros2 launch cod_gazebo_simulator sim_standalone.launch.py
```

### 4. 启动仿真 (比赛场地 — 需要 rmu_gazebo_simulator)
```bash
ros2 launch cod_gazebo_simulator gazebo_sim.launch.py world:=rmul_2025
```

### 5. 在 RViz 中设置导航目标
- 点击 "2D Goal Pose" 工具
- 在地图上点击目标位置并拖拽方向
- 机器人应开始导航

## 文件结构
```
cod_gazebo_simulator/
├── launch/
│   ├── gazebo_sim.launch.py       # 完整仿真 (需要 rmu_gazebo_simulator)
│   └── sim_standalone.launch.py   # 独立仿真 (空世界, 自包含)
├── config/
│   ├── sim_world.yaml             # 世界配置
│   ├── gz_bridge.yaml             # ROS-Gazebo 话题桥接
│   ├── sim_nav2_params.yaml       # 仿真用 Nav2 参数
│   └── gz_ros2_control.yaml       # Gazebo ROS2 控制参数
├── resource/
│   ├── cod_robot.urdf.xacro       # ★ 机器人模型 (替换此文件导入你的模型)
│   └── empty_world.sdf            # 空世界 SDF
├── rviz/
│   └── gazebo_nav.rviz            # RViz 配置
└── maps/
    └── *.pgm + *.yaml             # 比赛场地地图 (RMUL/RMUC 2024/2025)
```

## 导入你的机器人模型

`resource/cod_robot.urdf.xacro` 是占位模型。要替换为你的模型：

1. **保持这些 link/joint 名称不变**（外部代码依赖）：
   - `base_link` — 机体基座 (Nav2 使用)
   - `livox_frame` — 雷达安装位
   - `livox_frame_joint` — `base_link → livox_frame` 固定关节

2. **保持这些传感器名称不变**（bridge config 依赖）：
   - `front_mid360_lidar` — LiDAR 传感器 (→ `/livox/lidar`)
   - `front_mid360_imu` — IMU 传感器 (→ `/livox/imu`)

3. **更新 bridge config** 如果你的传感器 Gz 话题路径不同

4. **可以自由修改**：
   - 底盘尺寸、重量、转动惯量
   - 轮子类型、尺寸、位置
   - 视觉外观 (颜色、mesh 文件)
   - 额外传感器

## 架构

```
Gazebo Ignition                     ROS 2
┌──────────────────┐              ┌─────────────────────────┐
│  World SDF       │              │                         │
│  └─ Robot Model  │              │  /Odometry (ground truth)│
│     ├─ LiDAR ────┼─ros_gz──────▶│  /livox/lidar           │
│     ├─ IMU ──────┼─bridge──────▶│  /livox/imu             │
│     ├─ Joints ───┼─────────────▶│  /joint_states          │
│     └─ Pose ─────┼─────────────▶│  /tf                    │
└──────────────────┘              │                         │
                                  │  pointcloud_to_laserscan│
                                  │  └─ /livox/lidar → /scan│
                                  │                         │
                                  │  fake_vel_transform     │
                                  │  └─ cmd_vel → aft_cmd_vel│
                                  │                         │
                                  │  Nav2 Stack             │
                                  │  └─ /plan, /cmd_vel     │
                                  └─────────────────────────┘
```

在仿真中，`/Odometry` 由 Gazebo 提供（ground truth），替代了真实雷达的 `small_point_lio`。
