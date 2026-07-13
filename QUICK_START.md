# COD 导航系统 — 快速启动指南

> 适用场景: 每次开机后启动真机导航与 SLAM 建图

## 前置条件

- 雷达 USB 网卡已插入
- MCU USB 已连接
- 雷达已上电

## 启动步骤

### 一键启动

```bash
bash ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/start_nav.sh
```

或设置别名后直接输入 `nav`。

### 手动启动 (分步)

**终端 1 — 雷达驱动**
```bash
source /opt/ros/humble/setup.bash
source ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```
确认看到: `livox/lidar publish use PointCloud2 format`

**终端 2 — 导航 + SLAM + RViz**
```bash
source /opt/ros/humble/setup.bash
source ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
ros2 launch cod_bringup singlenav_launch.py
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
| RViz 无代价地图 | 添加 Map 显示，订阅 `/local_costmap/costmap` |
| LIO 无输出 | 移动雷达使其初始化（静止状态下可能不输出） |
