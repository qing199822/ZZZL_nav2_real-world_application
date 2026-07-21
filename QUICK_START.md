# COD 导航系统 — 快速启动指南

> LiDAR: Livox MID-360, 45° 前倾安装 (pitch=0.7854 rad)
> TF: base_link → livox_frame (z=0.46m, pitch=45°)
> LIO: small_point_lio → /Odometry
> 最低障碍物目标: 离地 0.10m（软件高度下限 0.05m）

## 前置条件

```bash
# 1. 确认雷达网络通畅
ping -c 1 192.168.1.181 || sudo nmcli connection up Livox-MID360

# 2. 确认 MCU 串口
ls /dev/cod_mcu || sudo ln -sf /dev/ttyACM0 /dev/cod_mcu
```

## 启动步骤

推荐在工作区中运行 `bash start_nav.sh`。脚本会检查雷达网络、MCU、实际点云，并在导航退出时清理雷达驱动；任一必需硬件未就绪都会拒绝启动。

### 终端 1 — 雷达驱动（先启动，保持运行）
```bash
source /opt/ros/humble/setup.bash
source ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```
确认看到: `livox/lidar publish use PointCloud2 format`

### 终端 2 — 导航（等终端 1 就绪后启动）
```bash
source /opt/ros/humble/setup.bash
source ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
ros2 launch cod_bringup singlenav_launch.py
```

### 验证里程计
```bash
source ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
ros2 topic echo /Odometry --once --field pose.pose.position
# 期望输出: x/y/z 在米级别 (< 1m)
```

## RViz 操作

| 操作 | 按钮 | 说明 |
|------|------|------|
| 发送目标点 | Nav2 Goal | 机器人自主导航到目标 |
| 保存地图 | — | `ros2 run nav2_map_server map_saver_cli -f ~/map_name` |

真机单点模式不启动 AMCL，`2D Pose Estimate` 不会进行真实重定位。`map→odom` 为固定变换，多点模式的 slam_toolbox 也不发布校正 TF；短程运行接受 LIO 漂移，但上电前必须确认地图对齐。

速度命令依次经过速度平滑、0.55m Collision Monitor StopZone、0.3s LiDAR 新鲜度看门狗和 `fake_vel_transform`，再送往串口。

## 常见问题

| 问题 | 解决 |
|------|------|
| 雷达 ping 不通 | `sudo nmcli connection up Livox-MID360` |
| `/dev/cod_mcu` 不存在 | `sudo ln -sf /dev/ttyACM0 /dev/cod_mcu` |
| `ros2 launch` 找不到包 | 确认 source 了 install/setup.bash |
| TF 报 extrapolation 错误 | 旧 ROS2 节点残留，`pkill -9 -f ros2` 后重启 |
| 里程计无输出 | 确保雷达驱动先于导航启动，等 15 秒以上 |
| `/Odometry` 发散到百万米 | 雷达驱动未启动或未就绪 |
| 始终零速 | 检查 `/livox/lidar_filtered` 是否持续更新，以及 Collision Monitor StopZone 内是否有点 |

正式运动前必须用最低点离地 `0.10m` 的实体障碍在多个方向验证 costmap 与停车/绕行，并在运动中断开雷达数据验证约 `0.3s + 一个控制周期` 内持续零速。MCU 下位机还应有独立通信 watchdog。
