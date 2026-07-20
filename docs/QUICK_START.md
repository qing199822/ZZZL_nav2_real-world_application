# COD 导航系统 — 快速启动指南

For Stage 0 SLAM dynamic-filter evidence collection, see [MAPPING_EVIDENCE_RUNBOOK.md](../cod_-rm2026_-navigation-master/docs/MAPPING_EVIDENCE_RUNBOOK.md).

For the pragmatic move-stop mapping filter, see [PRAGMATIC_MAPPING_FILTER.md](../cod_-rm2026_-navigation-master/docs/PRAGMATIC_MAPPING_FILTER.md).

## 前置条件

- Ubuntu 22.04 + ROS2 Humble
- Livox SDK2 已安装 (`/usr/local/lib/liblivox_lidar_sdk_shared.so`)
- 项目已构建: `cod_-rm2026_-navigation-master/` 下 `colcon build --symlink-install` 通过
- 当前已验证: **雷达感知-定位-建图管线** (MCU 未连接时串口节点仅发零速)

## 硬件连接检查

```bash
# 1. 雷达网络 (MID-360 直连 enp5s0)
sudo nmcli connection up Livox-MID360   # 若未自动连接
ping 192.168.1.181                      # 应 <2ms

# 2. 串口 (MCU, 可选)
ls /dev/cod_mcu                         # udev 规则创建的软链接
```

---

## 方式一：多点导航 (含 SLAM 实时建图) ✅ 已验证

**适用场景：** 未知环境，边建图边导航

```bash
# 终端 1 — 雷达驱动
source /opt/ros/humble/setup.bash
source ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 终端 2 — 导航系统 (SLAM + Nav2 + RViz)
source /opt/ros/humble/setup.bash
source ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
ros2 launch cod_bringup multiplenav_launch.py
```

**启动后检查：**

```bash
source ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash

# 确认数据流畅通
ros2 topic hz /livox/lidar        # ~10Hz
ros2 topic hz /Odometry            # ~10Hz
ros2 topic hz /scan                # ~3Hz
ros2 topic info /map               # 应有 1 publisher

# 确认 TF 树
ros2 run tf2_ros tf2_echo map base_link   # 帧链: map→odom→base_link
```

---

## 方式二：单点导航 (纯定位，需预建地图)

**适用场景：** 已知地图环境，仅定位不建图

```bash
# 终端 1 — 雷达驱动
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 终端 2 — 导航系统 (AMCL 定位 + Nav2)
ros2 launch cod_bringup singlenav_launch.py
```

---

## 方式三：仿真 (Gazebo Ignition Fortress)

**适用场景：** 无硬件时测试导航算法

```bash
source /opt/ros/humble/setup.bash
source ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash

# 空世界测试
ros2 launch cod_gazebo_simulator sim_standalone.launch.py

# 比赛场地 + SLAM
ros2 launch cod_gazebo_simulator gazebo_slam.launch.py world:=rmul_2025

# 比赛场地 + 定位 (需地图)
ros2 launch cod_gazebo_simulator gazebo_sim.launch.py world:=rmul_2025
```

---

## 当前已知限制

| 项 | 状态 | 说明 |
|----|------|------|
| MCU 串口 | ⚠️ 未连接 | `serial_def_sdk` 发零速，不影响感知管线 |
| RealSense 摄像头 | ⚠️ 未安装 | launch 文件中已注释 |
| 导航闭环 | ⚠️ 未测试 | 需 MCU 连接后设置 goal 验证运动控制 |
| Gazebo | ❌ 未安装 | 需 `apt install ignition-fortress` |

---

## 数据管线 (真机)

```
MID-360 (192.168.1.181)
  │  /livox/lidar (PointCloud2, 10Hz)
  │  /livox/imu   (sensor_msgs/Imu, 200Hz)
  │
  ├─→ cpp_lidar_filter      → /livox/lidar_filtered
  ├─→ pointcloud_to_laserscan → /scan → slam_toolbox → /map
  └─→ small_point_lio         → /Odometry + odom→base_link TF
  
TF 树:  map → odom → base_link → livox_frame (pitch=45°)
                                → base_link_fake (fake_vel_transform)
```

## 话题速查

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/livox/lidar` | PointCloud2 | 10Hz | 雷达原始点云 |
| `/livox/imu` | Imu | 200Hz | 雷达内置 IMU |
| `/livox/lidar_filtered` | PointCloud2 | 10Hz | 自滤波后点云 |
| `/Odometry` | Odometry | 10Hz | LIO 里程计 |
| `/cloud_registered` | PointCloud2 | 10Hz | 配准后点云 |
| `/scan` | LaserScan | ~3Hz | 2D 激光扫描 |
| `/map` | OccupancyGrid | - | SLAM 地图 |
| `/cmd_vel` | Twist | - | 平滑后速度指令 |
| `/aft_cmd_vel` | Twist | 50Hz | 机体坐标系速度 |
