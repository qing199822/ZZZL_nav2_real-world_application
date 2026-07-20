# 建图动态单位会话记忆过滤器

该过滤器只处理送给 `slam_toolbox` 的二维扫描。Nav2 costmap 继续直接使用
`/livox/lidar_filtered`，所以被建图过滤器排除的人或机器人仍会参与实时避障。

## 构建

```bash
cd /home/wangtao/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master
source /opt/ros/humble/setup.bash
colcon build --packages-select slam_dynamic_filter cod_bringup --symlink-install
source install/setup.bash
```

## 启动顺序

终端 1 启动 MID-360 驱动：

```bash
source /opt/ros/humble/setup.bash
source /home/wangtao/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

终端 2 首先使用观察模式：

```bash
source /opt/ros/humble/setup.bash
source /home/wangtao/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
ros2 launch cod_bringup multiplenav_launch.py slam_filter_mode:=observe
```

`observe` 会运行完整跟踪器并发布掩码，但送给 SLAM 的延迟扫描不应用掩码。
确认静态环境没有误报、移动单位能生成动态轨迹后，在封闭安全区域使用：

```bash
ros2 launch cod_bringup multiplenav_launch.py slam_filter_mode:=enforce
```

恢复原有建图路径：

```bash
ros2 launch cod_bringup multiplenav_launch.py slam_filter_mode:=disabled
```

`disabled` 是默认值，不写参数时行为与修改前相同。

## 观察状态

```bash
ros2 topic echo /slam_dynamic_filter/diagnostics --once
ros2 topic hz /scan_slam_filtered
ros2 topic echo /slam_dynamic_filter/ready --once
```

正常状态应满足：

- `message: healthy`
- `ready: true`
- `input_errors: 0`
- `capacity_exceeded: false`
- 静态环境中 `dynamic_tracks: 0`
- `/scan_slam_filtered` 约 10 Hz

轨迹可在 RViz 添加 `MarkerArray`：`/slam_dynamic_filter/tracks`。红色为已确认动态，
黄色为尚未确认的候选轨迹。任何输入时间、里程计跳变或轨迹容量异常都会停止
SLAM 扫描输出，不会回退到原始扫描。

## 保存地图

`observe` 和 `enforce` 模式不会启动项目原有的无条件自动保存。检查地图后手动保存：

```bash
mkdir -p /home/wangtao/maps
ros2 run nav2_map_server map_saver_cli -f /home/wangtao/maps/cod_filtered_map
```

生成的 PGM 可以继续人工修整。正常导航时使用修整后的固定地图、AMCL 和 Nav2，
不要继续运行在线 SLAM。

## 当前参数

参数文件：`slam_dynamic_filter/config/pragmatic_filter.yaml`。

主要判定条件：

- 10 帧缓冲和运动确认，约 1 秒。
- 目标累计位移至少 0.30 m。
- 平均速度至少 0.12 m/s，单步速度不超过 1.20 m/s。
- 连续观测间隔不超过 0.15 秒。
- 运动方向一致性至少 0.90。
- 只跟踪直径约 0.18-1.50 m 的独立扫描段。
- 最多 32 条轨迹；目标一旦确认移动，本次节点会话内不恢复为静态。

这些值针对现有静态真机 bag 做过防误报校准，但仍需使用真实 move-stop 场景继续调整。

## 明确限制

- 从建图开始到结束始终不动的单位无法仅靠激光几何识别。
- 目标第一次运动前已经写入地图的部分不会被在线过滤器追溯删除，可人工修图或离线重放。
- 目标与墙体合并成一个大扫描段时，过滤器优先保护墙，不做整段删除。
- 节点重启会开始新会话，之前记住的动态身份不会跨会话保留。
