# COD 导航系统 — 新上位机环境搭建清单

> 适用场景: 在新 Jetson/工控机上从零部署本项目  
> 前置条件: Ubuntu 22.04, x86_64 或 aarch64  
> 预计耗时: 首次约 60-90 分钟

---

## 快速重建 (迁移到新电脑)

> 已拷贝整个项目目录到新机器后，按以下步骤重建。

### 一、安装 ROS2 Humble

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
```

### 二、安装项目依赖

```bash
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-realsense2-camera \
  python3-rosdep

# 初始化 rosdep
sudo rosdep init
rosdep update

# 进入工作空间, 自动补全缺失依赖
cd ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master
rosdep install --from-paths src --ignore-src -r -y
```

### 三、清理系统干扰

```bash
sudo apt purge -y modemmanager brltty libbrlapi0.8 python3-brlapi
sudo usermod -a -G dialout $USER
# ⚠ 重新登录使 dialout 组生效
```

### 四、构建

```bash
cd ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master
source /opt/ros/humble/setup.bash
colcon build --symlink-install
# 期望输出: 16 packages finished
```

### 五、环境变量

在 `~/.bashrc` 末尾追加:

```bash
# COD 导航项目
source /opt/ros/humble/setup.bash
source ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/src/cod_gazebo_simulator/resource
```

然后 `source ~/.bashrc`。

### 六、验证

```bash
# ROS2 环境
ros2 --version

# 包列表 (应包含 serial_def_sdk, cod_gazebo_simulator, cod_bringup 等)
ros2 pkg list | grep -E 'serial_def_sdk|cod_gazebo|cod_bringup|small_point_lio'

# 串口设备 (MCU 需已连接)
ls -la /dev/cod_mcu || ls -la /dev/ttyACM*
```

### 真机模式 — 额外步骤

```bash
# Livox SDK2 (系统级)
cd ~ && git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig

# livox_ros_driver2 (工作空间内, 已随项目拷贝, 需重新构建)
cd ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master
colcon build --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble

# 雷达网络
sudo nmcli connection add type ethernet ifname enp5s0 con-name Livox-MID360 ip4 192.168.1.50/24
sudo nmcli connection modify Livox-MID360 ipv4.never-default yes
sudo nmcli connection up Livox-MID360

# udev 规则 (MCU 串口固定名)
sudo cp config/99-cod-mcu-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 仿真模式 — 额外步骤

```bash
sudo apt install -y ignition-fortress
ign gazebo --version
```

### 工作空间包清单 (16 个)

| 包名 | 用途 |
|------|------|
| `cod_bringup` | 导航启动 (launch + config) |
| `cod_gazebo_simulator` | Gazebo 仿真 (worlds + models + maps + bridge) |
| `cpp_lidar_filter` | Livox 点云裁剪 |
| `def_msg` | 自定义消息定义 |
| `fake_vel_transform` | 坐标系速度变换 (base_link→base_link_fake) |
| `goal_approach_controller` | Nav2 目标逼近控制器 |
| `livox_ros_driver2` | Livox MID-360 ROS2 驱动 |
| `pb_nav2_plugins` | Nav2 行为树插件 |
| `pb_omni_pid_pursuit_controller` | 全向 PID 追踪控制器 |
| `pb_rm_interfaces` | 比赛接口定义 |
| `pointcloud_to_laserscan` | 3D 点云→2D 激光 |
| `serial_def_sdk` | 新 Seasky 串口通信 (核心) |
| `slam_dynamic_filter` | SLAM 动态障碍过滤 |
| `small_point_lio` | 激光惯性里程计 (LIO) |
| `vision_msg` | 视觉消息定义 |
| `waypoint_editor` | 航点编辑器 |

---
| 3 | 用户权限 | ✅ | 1min |
| 4 | udev 规则 (MCU 串口固定名) | ✅ | 3min |
| 5 | Livox 雷达网络 | ✅ | 5min |
| 6 | Livox SDK2 + ROS 驱动 | ✅ | 15min |
| 7 | 项目依赖 | ✅ | 10min |
| 8 | 工作空间构建 | ✅ | 5min |
| 9 | 环境变量 | ✅ | 1min |
| 10 | Gazebo 仿真 (可选) | 可选 | 10min |
| 11 | 验证 | ✅ | 5min |

---

## 1. ROS2 Humble 安装

```bash
# 添加 ROS2 仓库
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 (推荐 desktop 版本, 包含 RViz)
sudo apt update && sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
```

---

## 2. 系统服务清理

某些服务会占用串口或干扰 USB 设备识别，**必须先清理**：

```bash
# 移除 ModemManager (会抢占 /dev/ttyACM* 发 AT 指令)
sudo apt purge -y modemmanager

# 移除 brltty (盲文显示驱动, 会占用串口)
sudo apt purge -y brltty libbrlapi0.8 python3-brlapi

# 确认已移除
dpkg -l | grep -E 'modemmanager|brltty' && echo "还有残留!" || echo "已清理 ✅"
```

---

## 3. 用户权限

```bash
# 将当前用户加入 dialout 组 (访问串口)
sudo usermod -a -G dialout $USER

# 验证
groups $USER | grep dialout && echo "dialout 组成员 ✅"

# ⚠ 如果是首次添加, 需要注销重新登录或重启
```

---

## 4. udev 规则 — MCU 串口固定为 /dev/cod_mcu

> **这是你执行的操作。** 电控组不需要做任何事。

```bash
# 1. 插入 MCU USB, 找到物理端口路径
udevadm info --name=/dev/ttyACM0 --attribute-walk | grep KERNELS
# 输出类似: KERNELS=="1-1.2"

# 2. 编辑项目中的规则文件, 取消注释策略 A 并填入路径值
#    config/99-cod-mcu-serial.rules:
#      SUBSYSTEM=="tty", KERNELS=="1-1.2", SYMLINK+="cod_mcu"

# 3. 安装规则
sudo cp config/99-cod-mcu-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# 4. 验证 (重新插拔 MCU USB 后)
ls -la /dev/cod_mcu
# 期望: lrwxrwxrwx ... /dev/cod_mcu -> ttyACM0
```

> **说明：** 此规则确保无论内核把 MCU 分配成 `ttyACM0` 还是 `ttyACM1`，始终有一个 `/dev/cod_mcu` 符号链接指向它。项目所有 launch 文件已配置为使用 `serial_port: "/dev/cod_mcu"`。

---

## 5. Livox MID-360 雷达网络

```bash
# 1. 确认雷达网线插在哪个接口 (通常 enp5s0)
ip link show | grep -E '^[0-9]+: enp'

# 2. 创建 NetworkManager 静态 IP 连接 (本机 192.168.1.50, 雷达 192.168.1.181)
sudo nmcli connection add \
  type ethernet \
  ifname enp5s0 \
  con-name Livox-MID360 \
  ip4 192.168.1.50/24

# 3. 禁用此接口的默认网关 (避免影响外网)
sudo nmcli connection modify Livox-MID360 ipv4.never-default yes

# 4. 启动连接
sudo nmcli connection up Livox-MID360

# 5. 验证 (雷达上电后)
ping -c 3 192.168.1.181 && echo "雷达连通 ✅"
```

---

## 6. Livox SDK2 + ROS2 驱动

```bash
# --- Livox SDK2 (系统级) ---
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
sudo ldconfig

# --- livox_ros_driver2 (工作空间内) ---
# 进入你的工作空间 src 目录
cd ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/src/
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2

# 配置 MID360 雷达 IP (如果与默认 192.168.1.181 不同)
# 编辑 config/MID360_config.json, 修改 host_net_info 和 lidar_configs[0].host_net_info

# 复制 ROS2 构建文件
cp package_ROS2.xml package.xml
cp -rf launch_ROS2/ launch/

# 构建
cd ../..
source /opt/ros/humble/setup.bash
colcon build --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
```

---

## 7. 项目依赖

```bash
# 系统依赖
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-realsense2-camera \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  python3-colcon-common-extensions \
  python3-rosdep

# 初始化 rosdep (首次)
sudo rosdep init
rosdep update

# 安装工作空间内所有包的依赖
cd ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master
rosdep install --from-paths src --ignore-src -r -y
```

---

## 8. 工作空间构建

```bash
cd ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master
source /opt/ros/humble/setup.bash

# 完整构建 (首次)
colcon build --symlink-install

# 验证: 应该 15/15 包通过
colcon build --symlink-install 2>&1 | grep 'packages finished'
```

---

## 9. 环境变量

```bash
# 在 ~/.bashrc 末尾添加, 或每次手动 source
echo '
# COD 导航项目
source /opt/ros/humble/setup.bash
source ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
# 仿真环境 (如安装了 Gazebo)
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/src/cod_gazebo_simulator/resource
' >> ~/.bashrc

source ~/.bashrc
```

---

## 10. Gazebo 仿真 (可选)

> 仅需在上位机上跑仿真时需要。真机部署可跳过。

```bash
# 安装 Ignition Gazebo Fortress
sudo apt install -y ignition-fortress

# 验证
ign gazebo --version && echo "Gazebo 已安装 ✅"
```

---

## 11. 验证

```bash
# A. 检查 ROS2 环境
ros2 --version && echo "ROS2 ✅"

# B. 检查串口设备
ls -la /dev/cod_mcu && echo "MCU 串口 ✅" || echo "MCU 未连接"

# C. 检查雷达
ping -c 1 192.168.1.181 > /dev/null 2>&1 && echo "雷达网络 ✅" || echo "雷达未连接"

# D. 检查 Livox SDK
ldconfig -p | grep livox_lidar_sdk && echo "Livox SDK2 ✅"

# E. 检查构建
cd ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master
colcon build --symlink-install 2>&1 | grep -E 'packages finished|failed'
```

---

## 快速启动 (所有验证通过后)

```bash
# === 真机导航 ===
# 1. 打开终端 1: 启动雷达驱动
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 2. 打开终端 2: 启动导航 (单点定位模式)
ros2 launch cod_bringup singlenav_launch.py

# === 仿真导航 ===
# 1. 启动 Gazebo SLAM 仿真
ros2 launch cod_gazebo_simulator gazebo_slam.launch.py

# 2. 打开 RViz 查看
ros2 launch cod_bringup rviz_launch.py
```

---

## 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| `/dev/cod_mcu` 不存在 | udev 规则未安装或 MCU 未连接 | 检查 `ls /dev/ttyACM*`，确认 MCU USB 已插 |
| `ros2 launch` 找不到包 | 未 source setup.bash | `source install/setup.bash` |
| `colcon build` 失败 | 缺少系统依赖 | 运行 `rosdep install --from-paths src --ignore-src -r -y` |
| 雷达 ping 不通 | 网线或 IP 配置问题 | 检查 `ip addr show enp5s0`，确认 192.168.1.50/24 |
| `permission denied` 串口 | 不在 dialout 组 | `sudo usermod -a -G dialout $USER` 然后重新登录 |
| Gazebo 找不到 model | 未设置 GZ_SIM_RESOURCE_PATH | 见第 9 节环境变量 |

---

**配套文档：**
- `docs/MCU_SEASKY_PROTOCOL.md` — 串口协议规范
- `docs/mcu_reference/README.md` — MCU 端参考实现指南
- `docs/OPERATING_GUIDE.md` — 完整操作指南 (导航模式、参数、故障排查)
- `config/99-cod-mcu-serial.rules` — udev 规则模板
