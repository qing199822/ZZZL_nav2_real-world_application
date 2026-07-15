# COD 导航系统 — 快速启动指南

> LiDAR: Livox MID-360, 45° 前倾安装 (pitch=0.7854 rad)
> TF: base_link → livox_frame (z=0.15m, pitch=45°)
> LIO: small_point_lio → /Odometry

## 前置条件

```bash
# 1. 确认雷达网络通畅
ping -c 1 192.168.1.181 || sudo nmcli connection up Livox-MID360

# 2. 确认 MCU 串口
ls /dev/cod_mcu || sudo ln -sf /dev/ttyACM0 /dev/cod_mcu
```

## 启动步骤

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
| 设定初始位姿 | 2D Pose Estimate | 在地图上标注当前位置 |
| 发送目标点 | Nav2 Goal | 机器人自主导航到目标 |
| 保存地图 | — | `ros2 run nav2_map_server map_saver_cli -f ~/map_name` |

## 常见问题

| 问题 | 解决 |
|------|------|
| 雷达 ping 不通 | `sudo nmcli connection up Livox-MID360` |
| `/dev/cod_mcu` 不存在 | `sudo ln -sf /dev/ttyACM0 /dev/cod_mcu` |
| `ros2 launch` 找不到包 | 确认 source 了 install/setup.bash |
| TF 报 extrapolation 错误 | 旧 ROS2 节点残留，`pkill -9 -f ros2` 后重启 |
| 里程计无输出 | 确保雷达驱动先于导航启动，等 15 秒以上 |
| `/Odometry` 发散到百万米 | 雷达驱动未启动或未就绪 |
