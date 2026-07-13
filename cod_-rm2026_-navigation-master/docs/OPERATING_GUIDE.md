# COD RM2026 导航系统 — 操作指南

> 适用于真机部署和 Gazebo 仿真。最后更新: 2026-07-06

## 目录

1. [项目结构]
1.5 [环境准备 — Livox MID-360 雷达](#15-环境准备--livox-mid-360-雷达)(#1-项目结构)
2. [真机导航](#2-真机导航)
3. [Gazebo 仿真](#3-gazebo-仿真)
4. [缓存清理](#4-缓存清理)
5. [关键参数参考](#5-关键参数参考)
6. [TF 坐标树](#6-tf-坐标树)
7. [常用调试命令](#7-常用调试命令)
8. [故障排查](#8-故障排查)

---

## 1. 项目结构

```
cod_-rm2026_-navigation-master/
├── src/
│   ├── cod_bringup/              # 导航启动文件、参数、地图、航点
│   │   ├── launch/               # ★ 真机启动文件
│   │   │   ├── singlenav_launch.py     # 单点导航 (定位模式)
│   │   │   ├── multiplenav_launch.py   # 多点 SLAM 导航
│   │   │   ├── navigation_launch.py   # Nav2 核心栈
│   │   │   ├── localization_launch.py # 地图服务 + 生命周期
│   │   │   ├── rviz_launch.py         # RViz 启动
│   │   │   └── auto_save_map.launch.py # 定时保存地图
│   │   ├── params/               # ★ Nav2 参数
│   │   │   ├── singlenav2_params.yaml     # 单点导航参数
│   │   │   ├── multiplenav2_params.yaml   # 多点 SLAM 参数
│   │   │   └── mapper_params_online_async.yaml # SLAM 参数
│   │   ├── maps/                 # 地图文件
│   │   ├── behavior_trees/       # 行为树 XML
│   │   └── wps/                  # 多点航点 CSV
│   │
│   ├── cod_gazebo_simulator/     # ★ Gazebo 仿真包
│   │   ├── launch/               # ★ 仿真启动文件
│   │   │   ├── gazebo_slam.launch.py    # SLAM 仿真
│   │   │   ├── gazebo_sim.launch.py     # 定位仿真
│   │   │   └── sim_standalone.launch.py # 空世界测试
│   │   ├── config/               # ★ 仿真参数
│   │   │   ├── sim_nav2_params.yaml     # 仿真 Nav2 参数
│   │   │   ├── gz_bridge.yaml          # ROS-Gazebo 桥接
│   │   │   ├── sim_world.yaml          # 世界配置
│   │   │   └── gz_ros2_control.yaml    # Gazebo 控制
│   │   ├── resource/             # 机器人模型 + 世界文件
│   │   │   ├── cod_robot.urdf.xacro    # ★ 机器人 URDF 模型
│   │   │   ├── worlds/                 # 比赛场地 SDF
│   │   │   └── models/                 # 场地 3D 模型
│   │   ├── rviz/                 # RViz 显示配置
│   │   └── maps/                 # 仿真地图
│   │
│   ├── small_point_lio/          # Point-LIO 里程计
│   ├── fake_vel_transform/       # 速度坐标变换
│   ├── serial_def_sdk/           # 串口通信 (Seasky 协议)
│   ├── pointcloud_to_laserscan/  # 3D点云 - 2D激光
│   ├── cpp_lidar_filter/         # 点云裁剪 (去车身)
│   ├── goal_approach_controller/ # MPPI 目标接近控制器
│   ├── pb_nav2_plugins/          # 自定义 Nav2 插件
│   ├── pb_omni_pid_pursuit_controller/ # PID 追踪控制器
│   ├── waypoint_editor/          # Rviz 航点编辑器
│   ├── def_msg/                  # 自定义消息
│   ├── vision_msg/               # 视觉消息
│   └── pb_rm_interfaces/         # 裁判系统消息
│
└── docs/
    └── OPERATING_GUIDE.md        # 本文档
```

---

---

## 1.5 环境准备 — Livox MID-360 雷达

### 硬件连接
1. MID-360 雷达使用 12V DC 供电
2. 网线直连雷达到计算机的以太网口 (enp5s0)
3. 雷达默认通过 DHCP 获取 IP (通常 192.168.1.1xx)

### 网络配置 (已配置)
```bash
# 激活雷达网络连接
sudo nmcli connection up Livox-MID360  # 设置本机 IP 为 192.168.1.50/24

# 验证连接
ping 192.168.1.181  # 雷达 IP (在 MID360_config.json 中配置)
```

### 雷达驱动安装 (已安装)
| 组件 | 位置 |
|------|------|
| Livox SDK2 | /usr/local/lib/liblivox_lidar_sdk_shared.so |
| livox_ros_driver2 | src/livox_ros_driver2/ |
| MID360_config.json | src/livox_ros_driver2/config/MID360_config.json |

### 首次安装步骤 (供参考)
```bash
# 1. Livox SDK2
cd ~ && git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig

# 2. 依赖
sudo apt-get install -y libapr1-dev

# 3. livox_ros_driver2 (在项目工作空间中)
cd src/ && git clone https://github.com/Livox-SDK/livox_ros_driver2.git
# 编辑 config/MID360_config.json 设置雷达和主机 IP

# 4. 构建
cd .. && source /opt/ros/humble/setup.bash
cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml
cp -rf src/livox_ros_driver2/launch_ROS2/ src/livox_ros_driver2/launch/
colcon build --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
```

### Livox Viewer 2 GUI (已安装)
```bash
~/bin/livox-viewer  # v2.5.9, UE5 应用
# 安装位置: ~/LivoxViewer2/Viewer2_2.5.9_Linux/
```

## 2. 真机导航

### 2.1 前置条件

```bash
# 1. 启动 Livox MID-360 雷达驱动 (必须先于导航)
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 2. 确保串口可用
ls /dev/ttyACM0
sudo usermod -a -G dialout $USER  # 首次配置需重启

# 3. 雷达 TF 必须已发布 (livox_frame -> base_link)
# 可在雷达驱动中配置或单独发布静态 TF
```

### 2.2 单点导航 (定位模式，预建地图)

适用场景: 已有地图，精确导航到单个目标点。

```bash
cd ~/COD26/cod_-rm2026_-navigation-master
colcon build --symlink-install
source install/setup.bash
ros2 launch cod_bringup singlenav_launch.py
```

启动的节点: cpp_lidar_filter, small_point_lio, fake_vel_transform, serial_def_sdk/uart, realsense, Nav2(定位模式), RViz

关键文件: `src/cod_bringup/params/singlenav2_params.yaml`, `src/cod_bringup/maps/rmul2026.yaml`

### 2.3 多点 SLAM 导航 (边建图边导航)

适用场景: 无地图环境，同时建图和导航多个目标。

```bash
ros2 launch cod_bringup multiplenav_launch.py
```

额外节点: pointcloud_to_laserscan (/livox/lidar -> /scan), slam_toolbox, auto_save_map

关键文件: `src/cod_bringup/params/multiplenav2_params.yaml`, `src/cod_bringup/params/mapper_params_online_async.yaml`

### 2.4 真机模式对比

| | 单点导航 | 多点 SLAM |
| --- | --- | --- |
| 启动命令 | `singlenav_launch.py` | `multiplenav_launch.py` |
| 地图来源 | 预建 .pgm 地图 | slam_toolbox 实时建图 |
| Nav2 参数 | `singlenav2_params.yaml` | `multiplenav2_params.yaml` |
| 需要 /scan | 否 | 是 |
| robot_base_frame | base_link_fake | base_link_fake |
| 定位方式 | AMCL + 预建地图 | SLAM 同时定位 |
| 雷达→串口 | 真机全链路 | 真机全链路 |

---

## 3. Gazebo 仿真

### 3.1 安装依赖

```bash
sudo apt install ignition-fortress ros-humble-ros-gz-bridge ros-humble-ros-gz-sim
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-bringup
```

### 3.2 SLAM 仿真 (建图 + 导航)

```bash
source install/setup.bash

# 默认 rmul_2025 场地
ros2 launch cod_gazebo_simulator gazebo_slam.launch.py world:=rmul_2025

# 可选 world: rmul_2024, rmuc_2024, rmuc_2025, empty
# 启用自动保存地图
ros2 launch cod_gazebo_simulator gazebo_slam.launch.py world:=rmul_2025 auto_save:=true
```

数据流: `Gazebo LiDAR -> /livox/lidar -> pointcloud_to_laserscan -> /scan -> slam_toolbox -> /map`

真机vs仿真差异: 里程计来自 Gazebo DiffDrive (而非 LIO), cmd_vel 通过 ros_gz_bridge 直接发给 Gazebo (而非串口)

### 3.3 定位仿真 (预建地图 + 导航)

```bash
ros2 launch cod_gazebo_simulator gazebo_sim.launch.py world:=rmul_2025
```

### 3.4 空世界快速测试

```bash
ros2 launch cod_gazebo_simulator sim_standalone.launch.py
```
仅启动 Gazebo + 机器人 + 传感器 + RViz，不启动 Nav2。

### 3.5 仿真模式对比

| | SLAM 仿真 | 定位仿真 | 空世界 |
| --- | --- | --- | --- |
| Launch | `gazebo_slam.launch.py` | `gazebo_sim.launch.py` | `sim_standalone.launch.py` |
| 地图 | slam_toolbox 实时 | 预建 .pgm | 无 |
| Nav2 | 有 | 有 | 无 |
| 场地 | 5个可选 | 5个可选 | 空世界 |

### 3.6 机器人模型替换

编辑 `src/cod_gazebo_simulator/resource/cod_robot.urdf.xacro`

必须保持的 link/joint 名称 (外部代码依赖):
- base_link (Nav2 robot_base_frame)
- livox_frame + livox_frame_joint (雷达安装位)
- front_left_wheel_joint / front_right_wheel_joint / rear_left_wheel_joint / rear_right_wheel_joint (DiffDrive)
- front_mid360_lidar / front_mid360_imu (传感器, bridge 依赖)

可自由修改: 底盘尺寸/质量/惯量, 轮子半径间距, 外观颜色/mesh, 额外传感器

---

## 4. 缓存清理

### 4.1 colcon 构建缓存

```bash
cd ~/COD26/cod_-rm2026_-navigation-master
rm -rf build/ install/ log/
colcon build --symlink-install
```

### 4.2 ROS2 日志

```bash
rm -rf ~/.ros/log/*        # 可能积累数 GB
du -sh ~/.ros/log/
```

### 4.3 Gazebo 模型缓存

```bash
rm -rf ~/.ignition/fuel/    # fuel.ignitionrobotics.org 下载的模型
du -sh ~/.ignition/
```

### 4.4 SLAM 中间文件

```bash
rm -f ~/.ros/slam_toolbox_*.posegraph
rm -f ~/.ros/slam_toolbox_*.data
```

### 4.5 一键清理脚本

```bash
#!/bin/bash
# clean_all.sh
rm -rf build/ install/ log/
rm -rf ~/.ros/log/*
rm -rf ~/.ignition/fuel/
rm -f ~/.ros/slam_toolbox_*
colcon build --symlink-install
echo "Done"
```

---

## 5. 关键参数参考

### 5.1 MPPI 控制器

文件: `src/cod_bringup/params/singlenav2_params.yaml` L84-145

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| time_steps | 80 | 预测时域步数 (80x0.02=1.6s) |
| model_dt | 0.02 | 每步时长(s) |
| batch_size | 1500 | 采样轨迹数 (CPU 开销来源) |
| temperature | 0.25 | 越低越果断, 过高会摇摆 |
| vx_max/vy_max | 2.5 | 最大线速度 (m/s) |
| wz_max | 1.5 | 最大角速度 (rad/s) |
| ax_max/ay_max | 1.5 | 最大加速度 (m/s^2) |
| ax_min/ay_min | -3.5 | 最大减速度 (必须对称) |
| motion_model | Omni | 全向运动模型 |

### 5.2 GoalApproachController (接近目标限速)

文件: `src/cod_bringup/params/singlenav2_params.yaml` L85-90

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| approach_distance | 2.5m | 距目标多远开始限速 |
| approach_velocity | 0.2 m/s | 接近速度上限 |
| direct_approach_distance | 2.0m | P控制驱动距离 |
| direct_approach_kp | 3.0 | P增益 |
| xy_goal_tolerance | 0.2m | 到达判定半径 |

### 5.3 代价地图

文件: `src/cod_bringup/params/singlenav2_params.yaml` L184-326

| 参数 | 单点默认 | 多点默认 | 说明 |
| --- | --- | --- | --- |
| resolution | 0.05 | 0.04-0.05 | 栅格分辨率 (m) |
| local_width/height | 10x10 | 10x10 | 局部代价地图尺寸 |
| inflation_radius | 0.55 | 0.75 | 膨胀半径 (m) |
| update_frequency | 5.0 Hz | 5.0 Hz | 更新频率 |

### 5.4 速度平滑器

文件: `src/cod_bringup/params/singlenav2_params.yaml` L470-483

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| max_velocity | [2.5, 2.5, 2.5] | 最大速度 |
| max_accel | [2.5, 2.5, 1.5] | 最大加速度 |
| max_decel | [-3.5, -3.5, -3.0] | 最大减速度 |
| smoothing_frequency | 30.0 Hz | 平滑频率 |
| feedback | OPEN_LOOP | 开环 (LIO不输出速度) |

### 5.5 SLAM (slam_toolbox)

文件: `src/cod_bringup/params/mapper_params_online_async.yaml`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| mode | lifelong | SLAM模式 |
| resolution | 0.05 | 地图分辨率 |
| max_laser_range | 10.0 | 激光最大距离 (m) |
| minimum_travel_distance | 1.0m | 最小移动后更新 |
| do_loop_closing | true | 回环检测 |
| map_update_interval | 1.0s | 地图更新间隔 |

### 5.6 fake_vel_transform (坐标变换)

文件: `src/fake_vel_transform/src/fake_vel_transform.cpp` L14-19

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| robot_base_frame | base_link | 真实机体帧 |
| fake_robot_base_frame | base_link_fake | 航向归零虚拟帧 |
| odom_topic | Odometry | 里程计来源 |
| input_cmd_vel_topic | cmd_vel | 输入(map系速度) |
| output_cmd_vel_topic | aft_cmd_vel | 输出(机体系速度) |
| spin_speed | 0.0 | 小陀螺速度 (rad/s) |

### 5.7 串口 (serial_def_sdk)

文件: `src/serial_def_sdk/launch/serial.launch.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| serial_port | /dev/ttyACM0 | 串口设备路径 |
| topic_timeout_ms | 100 | 话题超时(ms) |

话题重映射: `/hardware/cmd_vel_api` -> `/aft_cmd_vel`

### 5.8 仿真机器人模型

文件: `src/cod_gazebo_simulator/resource/cod_robot.urdf.xacro`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| chassis_size | 0.40x0.35x0.20m | 底盘尺寸 |
| chassis_mass | 15.0 kg | 底盘质量 |
| wheel_radius | 0.06 m | 轮子半径 |
| wheel_separation | 0.36 m | 左右轮间距 |
| lidar_hz | 10 Hz | LiDAR 更新频率 |
| lidar_range | 0.1-40.0 m | LiDAR 测距范围 |
| imu_hz | 200 Hz | IMU 更新频率 |
| lidar_z | 0.15 m | 雷达安装高度 |

### 5.9 仿真桥接

文件: `src/cod_gazebo_simulator/config/gz_bridge.yaml`

| ROS 话题 | Gazebo 话题 | 方向 |
| --- | --- | --- |
| /Odometry | /model/cod_robot/odom | GZ->ROS |
| /cmd_vel | /model/cod_robot/cmd_vel | ROS->GZ |
| /livox/lidar | .../front_mid360_lidar/scan/points | GZ->ROS |
| /livox/imu | .../front_mid360_imu/imu | GZ->ROS |
| /joint_states | .../joint_state | GZ->ROS |
| /clock | /clock | GZ->ROS |

---

## 6. TF 坐标树

### 6.1 真机

```
map -> odom -> base_link -> base_link_fake (Nav2使用)
 |       |         |
静态TF  LIO动态   +-- livox_frame (雷达)
(slam时  (100Hz)  +-- camera_link (深度相机)
覆盖)
```

| 变换 | 来源 | 频率 |
| --- | --- | --- |
| map->odom | static TF + slam_toolbox覆盖 | ~10Hz(SLAM模式) |
| odom->base_link | small_point_lio | ~100Hz |
| base_link->base_link_fake | fake_vel_transform | 随/Odometry |
| base_link->livox_frame | 用户提供静态TF | 静态 |

### 6.2 仿真

```
map -> odom -> base_link -> livox_frame
 |       |         |
SLAM   Gazebo    +-- 4个轮子
或静态  DiffDrive
```

---

## 7. 常用调试命令

### 话题

```bash
ros2 topic list
ros2 topic hz /Odometry
ros2 topic echo /cmd_vel --once
ros2 topic bw /livox/lidar
```

### 节点

```bash
ros2 node list
ros2 node info /hardware_serial
```

### TF

```bash
ros2 run tf2_tools view_frames       # 生成 frames.pdf
ros2 run tf2_ros tf2_echo map base_link
```

### 参数

```bash
ros2 param get /hardware_serial serial_port
ros2 param get /hardware_serial topic_timeout_ms
```

### 仿真专用

```bash
ign model --list                    # Gazebo 模型列表
ign topic --list                    # Gazebo 话题列表
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}" -r 10
```

---

## 8. 故障排查

### 真机

| 问题 | 原因 | 解决 |
| --- | --- | --- |
| LIO 无法初始化 | 缺少 livox_frame->base_link TF | 添加静态 TF |
| 串口无数据 | 权限不足 | dialout 组 + 重启 |
| Nav2 不规划 | costmap 无数据 | 检查 /livox/lidar_filtered |
| slam_toolbox 无地图 | /scan 未生成 | 检查 pointcloud_to_laserscan |
| CRC 校验失败 | MCU 固件未更新 | 编译时定义 NO_CRC_MODE |

### 仿真

| 问题 | 原因 | 解决 |
| --- | --- | --- |
| Gazebo 闪退 | GPU 驱动 | nvidia-smi, 确认 Ogre2 |
| 机器人不生成 | xacro 错误 | xacro 手动测试 |
| 传感器无数据 | 桥接路径错误 | ign topic -l 对比 yaml |
| /scan 为空 | pcl2scan 参数 | 检查 height 范围 |
| 机器人不动 | cmd_vel 未桥接 | 检查 ROS_TO_GZ 方向 |

---

## 附录: 参数修改速查

| 要修改的内容 | 文件 | 参数名 |
| --- | --- | --- |
| 最大速度 | params/singlenav2_params.yaml | vx_max / max_velocity |
| 目标精度 | params/singlenav2_params.yaml | xy_goal_tolerance |
| MPPI 预测范围 | params/singlenav2_params.yaml | time_steps x model_dt |
| 串口设备 | launch/serial.launch.py | serial_port |
| 话题超时 | launch/serial.launch.py | topic_timeout_ms |
| 雷达安装位 | resource/cod_robot.urdf.xacro | livox_frame_joint origin |
| 轮距/轮径 | resource/cod_robot.urdf.xacro | wheel_separation/radius |
| LiDAR 频率 | resource/cod_robot.urdf.xacro | sensor update_rate |
| 仿真初始位姿 | launch/gazebo_slam.launch.py | robot_x/y/z/yaw |
| SLAM 闭环 | params/mapper_params_online_async.yaml | do_loop_closing |
| 膨胀半径 | params/singlenav2_params.yaml | inflation_radius |
| 加减速 | params/singlenav2_params.yaml | max_accel / max_decel |
