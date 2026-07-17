# Livox MID-360 雷达调试指南

> 工作空间: `cod_-rm2026_-navigation-master/`
> 目标: 验证真实雷达和仿真两条链路，支撑 COD 导航项目真机调试
>
> 独立调试工作空间（来源）: `~/livoxviewer/livox_ws`

## 环境要求

| 组件 | 版本 | 状态 |
|------|------|------|
| Ubuntu | 22.04.5 LTS | ✅ |
| ROS2 | Humble | ✅ |
| Ignition Gazebo | 6.16.0 (Fortress) | ✅ |
| Livox SDK2 | master (liblivox_lidar_sdk_shared.so) | ✅ |
| livox_ros_driver2 | GitHub master | ✅ |
| livox_sim | 本地包 | ✅ |

## 快速启动

### 真实雷达链路

```bash
# 1. 激活雷达网络
sudo nmcli connection up Livox-MID360

# 2. 启动驱动
source /opt/ros/humble/setup.bash
source ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 3. 查看数据
source /opt/ros/humble/setup.bash
ros2 topic hz /livox/lidar
```

### 仿真链路

```bash
# 终端 1: Gazebo
source ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
ign gazebo $(ros2 pkg prefix livox_sim)/share/livox_sim/worlds/livox_test.sdf

# 终端 2: Bridge
source /opt/ros/humble/setup.bash
source ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /livox/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked

# 终端 3: RViz2
source /opt/ros/humble/setup.bash
rviz2
# Add → By topic → /livox/points → PointCloud2
# Fixed Frame → livox_frame
```

## 网络配置

- 接口: enp5s0
- Host IP: 192.168.1.50/24
- Radar IP: 192.168.1.181
- 连接名: Livox-MID360 (nmcli)
- 激活: `sudo nmcli connection up Livox-MID360`

## 雷达参数

- lidar_type: 8 (MID-360)
- 数据端口: cmd=56100, push_msg=56200, point_data=56300, imu=56400
- xfer_format: 1 (custom pointcloud)
- pattern_mode: 0

## 仿真传感器参数

- 类型: Ignition Gazebo gpu_lidar
- 更新率: 10 Hz
- 水平采样: 360 pts/rev
- 距离: 0.1m - 40m

> ⚠️ 注意: 此为单环扫描近似，非 MID-360 真实非重复扫描 pattern。
> 如需真实 pattern 用于算法开发，推荐 rosbag 录制真实雷达数据后回放。

## 故障排查

| 症状 | 检查 |
|------|------|
| `/livox/lidar` 无数据 | `nmcli connection show Livox-MID360` 确认连接 |
| | `ping 192.168.1.181` 确认网络通 |
| | 雷达是否上电（12V） |
| Gazebo 模型加载失败 | `echo $GZ_SIM_RESOURCE_PATH` 确认包含 livox_sim/models |
| `/livox/points` 无数据 | 检查 gpu_lidar sensor `always_on=1` |
| | `ign topic -l` 确认 Gazebo 侧有数据 |
| ros_gz_bridge 报错 | `dpkg -l | grep ros-humble-ros-gz-bridge` 确认已安装 |
| 驱动启动后 SIGSEGV | 这是 Livox Viewer 2 的问题，与本驱动无关 |

## 迁移到宿主项目

验证通过。以下内容已迁入本项目:

1. `models/livox_mid360/` — 雷达仿真模型
2. `config/livox_bridge.yaml` — ros_gz_bridge 配置
3. 本文档 — 雷达调试指南
4. `scripts/record_bag.sh` — rosbag 录制脚本
