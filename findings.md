# Findings: COD导航串口通信迁移

## 两个串口模块的协议差异

### 旧模块 (ros2_simple_serial)
- 15字节裸包: [0xA5][vx:float][vy:float][vz:float][checksum:uint16]
- 校验: 简单累加和，非标准CRC
- 每次消息打开/关闭串口 (高频场景下效率极低)
- send-only，无接收路径
- 端口和波特率硬编码

### 新模块 (RMserial-main / serial_def_sdk)
- Seasky帧协议: [0xA5][len:2B LE][CRC8][msg_id:2B LE][flags:2B][payload:N][CRC16:2B]
- 头部8B + 载荷N + CRC16 2B = 10+N 字节
- fix_control (0x05): 21字节载荷 (yaw/pitch/fire/vx/vy/vz/spin)
- 持久连接 + 500ms自动重连
- 双向通信 (ROS→MCU发送 + MCU→ROS接收)
- 参数化配置

## TF树分析

### 完整链路
```
map → odom → base_link → base_link_fake
                    ├── livox_frame (需用户提供)
                    └── camera_link (Realsense)
```

### 关键发现
1. small_point_lio 广播 odom→base_link, 发布 /Odometry
2. fake_vel_transform 广播 base_link→base_link_fake (-Yaw旋转), 发布 aft_cmd_vel
3. RMserial 的 speed2odom 和 gimble TF 均已注释禁用 — 不影响TF树
4. livox_frame→base_link 必须手动提供，否则 LIO 启动报错
5. map→odom 双重发布 (static + slam_toolbox动态)，TF2规则下动态覆盖静态

### fake_vel_transform 的坐标变换逻辑
- Nav2 在 base_link_fake (航向归零的虚拟坐标系) 中做规划
- fake_vel_transform 将 map系 cmd_vel 旋转回机体系 aft_cmd_vel
- 公式: aft_vx = cmd_vx·cos(Yaw) + cmd_vy·sin(Yaw)
- angular.z 被替换为 spin_speed_ (默认0) — 转向由MCU差速控制

## 代码审查发现的关键问题 (已修复)

### 原始代码缺陷 (RMserial-main自带)
1. uart 全局指针初始化为 nullptr，main()和构造函数直接调用 → UB
2. CRC-16 校验失败后强制 crc_ok=true → 零数据完整性
3. 全局变量跨线程无锁访问 (接收线程写, 发送定时器读, ROS线程读)
4. 双重 thread::detach() 启动同一个空函数
5. SerialDriver::open() 打开失败也返回 true
6. 文件描述符 TOCTOU 竞争

### 协议隐患
7. sentry_state(0x20) 的长度覆盖 (22→20) 破坏 CRC 校验 → 所有哨兵状态帧丢失
8. CRC-8 头部校验发送端有计算但接收端从不验证
9. sentry_state 使用 float 反序列化整数字段 → 数据错误

### 集成缺失
10. livox_frame→base_link 静态TF 未在 launch 文件中提供
11. 500ms 话题超时在50Hz控制环下过长 (25周期的陈旧数据)

## 依赖关系
```
serial_def_sdk
├── def_msg (GimbleControl, GobalInformation, CommonControl, etc.)
├── vision_msg (GimblePosition)
├── pb_rm_interfaces (GameStatus, RobotStatus)
├── std_msgs, geometry_msgs, nav_msgs, sensor_msgs
├── example_interfaces (Float32)
├── tf2_ros, tf2
└── std_srvs
```

## 已知限制
1. MCU 固件必须同步升级才能解析 Seasky 协议帧
2. 旧 ros2_simple_serial 包已在 `safe` 分支从工作空间移除
3. 无单元测试覆盖 (零测试基础设施)
4. speed2odom 功能禁用 — 依赖 small_point_lio 提供里程计
5. Nav2 无法直接控制角速度 (被 fake_vel_transform 替换为 spin_speed_=0)

## Gazebo 仿真环境搭建

### 参考项目结构
- `/home/wangtao/ZZZL_Little_Deng_nav_and_gaze/` — 包含5个独立项目
- 核心仿真包: `rmu_gazebo_simulator-main` (Ignition Gazebo Fortress 6)
- 世界文件: rmul_2024/2025, rmuc_2024/2025 比赛场地
- 机器人模型: xmacro → SDF → URDF 管道 (pb2025_robot_description)
- Nav2仿真参数: `sim_singlenav2_params.yaml` (COD风格, 针对仿真优化)

### 仿真架构差异 (真机 vs 仿真)
| 组件 | 真机 | 仿真 |
|------|------|------|
| 里程计 | small_point_lio (LIO) | Gazebo DiffDrive (ground truth) |
| 串口 | serial_def_sdk → MCU | ros_gz_bridge → Gazebo |
| 雷达 | Livox MID-360 物理设备 | Gazebo GPU LiDAR 传感器 |
| 坐标变换 | fake_vel_transform (机体系) | 直接用 ros_gz_bridge |
| angular.z | MCU差速控制 | Gazebo DiffDrive 处理 |

### 机器人模型接口
用户替换 `cod_robot.urdf.xacro` 时需保持的名称:
- base_link (Nav2 robot_base_frame)
- livox_frame + livox_frame_joint (LiDAR安装位)
- front_mid360_lidar / front_mid360_imu (传感器名称, bridge config依赖)

## 第四轮审查发现 (2026-07-05)

### 已验证的修复 (14/14)
所有前序修复通过验证: xacro处理, robot_description, livox_frame TF, AMCL定位, gz_bridge, 世界文件, auto_save_map, 硬编码路径

### 第四轮新发现 (3个, 已修复)
1. **CRITICAL**: rmuc_2025/model.config 多余 </model> 闭合标签 → Gazebo 拒绝加载
2. **MODERATE**: AMCL 缺少 transform_tolerance → 添加0.2s
3. **LOW**: _sim_common.py 中 write_temp_urdf() 死代码 → 已删除

### gz_bridge.yaml 端到端验证
Odometry: /model/cod_robot/odom ← DiffDrive <odom_topic> ✅
cmd_vel: /model/cod_robot/cmd_vel ← DiffDrive <topic> ✅
LiDAR: .../front_mid360_lidar/scan/points ← xacro sensor name ✅
IMU: .../front_mid360_imu/imu ← xacro sensor name ✅

### 世界文件验证 (5/5)
empty_world.sdf: ogre2 ✅, ground_plane ✅
rmul_2024_world.sdf: ogre2 ✅, static ✅
rmul_2025_world.sdf: ogre2 ✅, static ✅ (无ground_plane, 场地自带地板)
rmuc_2024_world.sdf: ogre2 ✅, static ✅
rmuc_2025_world.sdf: ogre2 ✅, static ✅ (无ground_plane)

## 最终项目统计
- 总包数: 15 (colcon build 通过)
- 新增文件: ~120 (仿真包 + 文档)
- 修改文件: ~15 (launch + config + serial)
- 审查轮次: 4 (GLM5.2)
- 累计修复: 78+ 问题
- 代码行数: ~5000 (新增+修改)

## Gazebo 世界加载验证 (2026-07-05)

### 关键发现: GZ_SIM_RESOURCE_PATH
Ignition Gazebo Fortress 用 `GZ_SIM_RESOURCE_PATH` 解析 `model://` URI，不是 `GAZEBO_MODEL_PATH`。
前者是 Gazebo Classic 的变量名，在 Fortress 中无效。

### 世界加载测试结果
| 世界 | 模型 | 状态 |
|------|------|------|
| empty | ground_plane | ✅ |
| rmul_2024 | rmul_2024 + red_supplier + blue_supplier + ground_plane | ✅ |
| rmul_2025 | rmul_2025 | ✅ |
| rmuc_2024 | rmuc_2024 + ground_plane | ✅ |
| rmuc_2025 | rmuc_2025 | ✅ |

### source 命令
```bash
source /opt/ros/humble/setup.bash
source ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
```

## Livox MID-360 雷达安装 (2026-07-06)

### 已安装组件
| 组件 | 版本/来源 | 安装位置 |
|------|----------|----------|
| Livox SDK2 | GitHub master | /usr/local/lib/liblivox_lidar_sdk_shared.so |
| livox_ros_driver2 | GitHub master | src/livox_ros_driver2 (工作空间内) |
| Livox Viewer 2 | v2.5.9 (2026-04-16) | ~/LivoxViewer2/Viewer2_2.5.9_Linux/ |
| libapr1-dev | apt (1.7.0) | 系统依赖 |

### 网络配置
- 接口: enp5s0
- 主机 IP: 192.168.1.50/24 (静态, NetworkManager 连接名 "Livox-MID360")
- 雷达 IP: 192.168.1.181 (MID360_config.json 中配置)
- 启动连接: sudo nmcli connection up Livox-MID360

### MID360_config.json 关键参数
- lidar_type: 8 (MID360)
- 雷达端口: cmd=56100, push_msg=56200, point_data=56300, imu=56400
- 主机端口: cmd=56101, push_msg=56201, point_data=56301, imu=56401
- pcl_data_type: 1, pattern_mode: 0

### 启动命令
```bash
# 1. 启动雷达网络 (如需)
sudo nmcli connection up Livox-MID360

# 2. 启动雷达驱动
source /opt/ros/humble/setup.bash
source ~/ZZZL_nav2_real-world_application/cod_-rm2026_-navigation-master/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 3. Livox Viewer GUI (可选)
~/bin/livox-viewer
```

### 数据流
MID-360 Radar (192.168.1.181) → Livox SDK2 → livox_ros_driver2 → /livox/lidar (PointCloud2) + /livox/imu → cpp_lidar_filter → small_point_lio

### 注意事项
- launch 文件 xfer_format=1 (custom pointcloud), 如果 LIO 使用 livox_pointcloud2 需改为 0
- 雷达需12V供电, 网线直连 enp5s0 接口
- 首次使用需配置雷达 IP (默认 DHCP 分配 192.168.1.1xx)

## Livox Viewer 2 崩溃问题 (2026-07-06)

### 现象
Livox Viewer 2 v2.5.9 和 v2.3.0 在启动后立即 SIGSEGV 崩溃，null pointer at 0x0。
即使 `-nullrhi` / `-opengl` 也无效 — 不是渲染层问题。

### 根因
UE 4.27.2 (2021年编译) 中 LivoxViewer2 游戏逻辑与 Ubuntu 22.04 + NVIDIA 535.309.01 + Vulkan 不兼容。
崩溃发生在 `ShowFlag.Particles = "0"` 之后，位于 LivoxViewer2 代码而非引擎层。

### 解决方案
使用老版 **Livox Viewer v0.10.0** (Qt原生，非UE)，运行正常。
安装路径: ~/Livox_Viewer_For_Linux_Ubuntu16.04_x64_0.10.0/
启动: ~/bin/livox-viewer

## Livox MID-360 Rosbag 录制 (2026-07-10)

### 数据速率
MID-360 实测数据速率 ~8MB/s (20k pts × 10Hz + 200Hz IMU)。
30s 录制 ≈ 240MB+ 原始数据。

### 常见问题及解决方案

| 问题 | 原因 | 解决 |
|------|------|------|
| rosbag 拆分成 15+ 个 .db3 文件 | rosbag2 默认按文件大小拆分 | `--max-bag-size 0` 禁用大小拆分 |
| `-d 30` 只录了 5s | `ros2 bag record -d` 控制**单文件**时长，非总时长 | 使用 `timeout` 命令控制总时长 |
| CustomMsg type unknown | 未 source 工作空间 setup.bash | `source install/setup.bash` 后再录制 |
| `set -u` 报错 | ROS2 setup.bash 引用未定义变量 | 在 source 前 `set +u`，source 后 `set -u` |

### 录制工具

`scripts/record_bag.sh` — 一键录制脚本，自动化完整流程:
- 自动启动雷达驱动 + 等待话题就绪
- 自动检测雷达连通性 (ping 192.168.1.181)
- `-d SEC` 控制录制时长 (内部用 timeout)
- `--no-driver` 跳过驱动启动 (雷达已运行时)
- Trap 清理，Ctrl+C 安全退出

```bash
# 默认录制 30s
./scripts/record_bag.sh

# 录制 60s 到指定目录
./scripts/record_bag.sh -d 60 -o data/record files/my_scene

# 雷达已运行，只录制
./scripts/record_bag.sh --no-driver -d 10
```

### 已验证录制
- 首次成功: 8s, 30.5 MiB, 单个 .db3, 1647 msgs (1569 IMU + 78 lidar)
- 验证: `ros2 bag info` 确认为有效 sqlite3 存储, 7.84s 时长

## 第五轮审查: MCU 参考代码 (2026-07-11)

### 审查结论
GLM 5.2 审计 docs/mcu_reference/ 全部 11 文件，发现 24 个问题。最严重的是 **参考代码的示例本身就存在协议不兼容问题**。

### 根本问题
- **pack/unpack 缺失**: 示例用 `(uint8_t*)&struct + sizeof(struct)` 发包，ARM 上 mixed float/uint8 结构体有 padding，sizeof ≠ wire bytes
- **CRC16 表线程不安全**: 运行时惰性初始化，多上下文首次调用存在竞态
- **Ring buffer 可移植性不足**: 仅靠 volatile，未定义平台假设
- **发送缓冲区共享**: ctx->tx_buf 全局唯一，多线程调用导致帧互覆盖
- **重同步状态机 bug**: CRC16 错误后用 total_len-_j 代替 rx_pos-_j

### 修复方案
- 添加 12 种消息类型 pack/unpack 函数 + WIRE_LEN_xxx 常量
- CRC16 表改为编译期 static const (256 条目, 0xA001 多项式验证通过)
- ring_buf_put_isr() (ISR专用, 无锁) + ring_buf_get() (主循环, 临界区保护)
- seasky_send_frame() 内部临界区保护, 文档标注不可重入
- 统一 resync 辅助函数, 修复剩余字节计算
- NULL 检查, 溢出统计同步, flags 传递, 超时可配, wrap 文档化
- HeartbeatFromHost/Recv → HeartbeatHostToMcu/McuToHost
- CRC8 表 const, crc_modbus 移除, HAL 契约文档化

### 文件变更清单
| 文件 | 变更 |
|------|------|
| seasky_protocol.h | 重写: +pack/unpack API, +12 WIRE_LEN 常量, +flags 回调, 重命名结构体 |
| seasky_protocol.c | 重写: +pack/unpack 实现, +NULL 检查, +临界区, +统一 resync, +溢出同步 |
| crc8.c | const 表 |
| crc16.h/c | 重写: 移除运行时初始化, 改为编译期静态 const 表, 移除 crc_modbus |
| ring_buffer.h | +_Static_assert, +ISR/主循环 API 分离文档 |
| ring_buffer.c | +ring_buf_put_isr, +临界区, -memset |
| hal_uart.h | +HAL 契约文档 (同步/异步, buffer 生命周期) |
| main_template.c | 重写: 全部用 pack 函数替代 sizeof(struct) 发包 |

## LiDAR 45° Pitch 前倾安装 (2026-07-15)

### 参数分层设计
```
base_link ──(z=0.15m, pitch=45°)──→ livox_frame
                                       ├── LiDAR → /livox/lidar
                                       └── IMU   → /livox/imu

extrinsic_R = I  (LiDAR ↔ IMU 相对位姿不变, MID-360 内部刚性固定)
```

### 为什么不能只改 mid360.yaml 的 extrinsic_R
- `extrinsic_R` 是 LiDAR→IMU 的相对旋转
- MID-360 模块内两个芯片固定, 外部倾斜不改变它们之间的相对关系
- 如果把 45° 写入 extrinsic_R, LIO 将点云旋转 45° 后与未旋转的 IMU 数据融合
  → 重力方向对不上 → 里程计发散

### 修改清单
| 类型 | 文件 | 改动 |
|------|------|------|
| 真机 TF | singlenav_launch.py, multiplenav_launch.py | pitch 0.0→0.7854 |
| 仿真 TF | gazebo_sim.launch.py, sim_standalone.launch.py | 新增 roll+pitch+yaw |
| 仿真 TF | gazebo_slam.launch.py | pitch 0.0→0.7854 |
| 仿真 URDF | cod_robot.urdf.xacro | rpy 0→0.7854 |
| LIO 外参 | mid360.yaml | **不变** — 保持 I + 添加注释 |

### 未改动文件 (确认安全)
| 文件 | 原因 |
|------|------|
| small_point_lio.launch.py | 死代码 — static TF 节点未加入 LaunchDescription |
| MID360_config.json | 驱动外参不涉及坐标系转换 |
| map→odom TF | 定位估计, 与传感器安装角度无关 |

## 参考仓库验证 (2026-07-16)

### 仓库来源
- GitHub: https://github.com/qing199822/ZZZL_nav2_real-world_application
- 克隆位置: ~/ZZZL_nav2_reference/
- 描述: COD 导航系统"确定能跑"的参考实现

### 构建验证
- colcon build 15/15 包通过 ✅
- 构建时间: ~36s

### SLAM 配置验证
- slam_toolbox 独立启动: ✅ 配置加载成功, CeresSolver SCHUR_JACOBI
- 模式: lifelong (持续建图+定位)
- 分辨率: 0.05m, 激光范围: 0.2-10.0m, 回环检测: 开启

### SLAM 启动方式
| 模式 | Launch 文件 | SLAM 支持 |
|------|-----------|-----------|
| 真机多点导航 | multiplenav_launch.py | ✅ async_slam_toolbox_node |
| 真机单点导航 | singlenav_launch.py | ❌ 未集成 slam_toolbox |
| 仿真实时建图 | gazebo_slam.launch.py | ✅ async_slam_toolbox_node |

### 代码缺陷发现
1. singlenav_launch.py: slam_params_file 默认值指向不存在的 mapper_params_async.yaml
2. multiplenav auto_save_map: $(date +%H%M%S) shell语法在Python字符串中不展开
3. 两仓库代码一致, 缺陷也一致

### 启动测试 (multiplenav, 无硬件)
- 17/17 节点启动成功 ✅
- slam_toolbox 加载配置 ✅
- livox_frame TF (pitch=0.7854) ✅
- Nav2 lifecycle 全部创建 ✅
- 预期失败: /dev/cod_mcu 不存在, LiDAR 无数据

## 真机雷达验证 (2026-07-18)

### 验证环境
- 雷达: Livox MID-360 (192.168.1.181, ping <1ms)
- 网络: enp5s0 static 192.168.1.50/24
- 工作空间: cod_-rm2026_-navigation-master
- 构建: 16/16 包通过 (含 Phase 14 导航代码修复)

### 发现并修复的问题

1. **CRITICAL: xfer_format 类型不匹配**
   - livox_ros_driver2 默认 xfer_format=1 发布 CustomMsg
   - 下游 cpp_lidar_filter/small_point_lio/pointcloud_to_laserscan 全部需要 PointCloud2
   - 修复: msg_MID360_launch.py xfer_format=1→0

2. **CRITICAL: launch 文件语法错误**
   - multiplenav_launch.py: "--pitch", "0.7854"--yaw" → 缺逗号导致字符串拼接
   - 导致 livox_frame static_transform_publisher 崩溃
   - 修复: 添加逗号

3. **MEDIUM: 废弃异常类**
   - pb_omni_pid_pursuit_controller 使用 nav2_core::ControllerException → Humble 中不存在
   - 修复: 替换为 std::runtime_error

4. **LOW: 旧串口包编译问题**
   - ros2_simple_serial/cod_serial.cpp: const Clock::now() 和缺失 uart_transporter.cpp
   - 修复: const_cast + 添加源文件到 CMakeLists

5. **LOW: PID 回调函数括号错误**
   - dynamicParametersCallback 多余 } 导致 PID 同步代码被截断
   - 修复: 删除多余的 }

### 验证结论
- 雷达→LIO→SLAM 管线完整可用
- Nav2 lifecycle 全部激活 (controller/planner/behavior/bt/smoother)
- 串口节点正常启动 (MCU 未连接时发零速)
- 待 MCU 连接后可进行导航闭环测试

## SLAM 动态障碍物污染静态地图 (2026-07-20)

### 真机复现
- 用户已在 multiplenav online SLAM 中复现: 暂时静止的人或可移动单元会被融合进 `/map`
- 该问题属于建图输入/occupancy fusion 问题, 与 Nav2 local costmap clearing 是不同层级的问题

### 仓库证据
- commit dfe6fa6 设计文档: docs/superpowers/specs/2026-07-19-slam-dynamic-obstacle-filter-design.md
- untracked plan: docs/superpowers/plans/2026-07-19-slam-dynamic-obstacle-filter-autoplan.md

### 候选调查路径
- cpp_lidar_filter
- pointcloud_to_laserscan
- mapper_params_online_async.yaml
- multiplenav launch

### 根因调查发现
- Existing approved design confirms current `/scan` goes directly into `slam_toolbox` without a dynamic/static temporal classifier; Nav2 STVL decay cannot remove occupancy already fused into `/map`.
- `cpp_lidar_filter` only crops robot-body points and optionally voxel-downsamples; it has no temporal memory or motion classification.
- `mapper_params_online_async.yaml` subscribes `scan_topic` `/scan` in lifelong mode and therefore accepts a temporarily stationary person's ranges like static observations.
- Geometry-only LiDAR cannot identify an object that never moves throughout the session; the reported moves-then-stops case is addressable only through temporal motion tracking plus session memory.
- Approved architecture preserves immediate `/livox/lidar_filtered` for Nav2 STVL and inserts the delayed filter only before `slam_toolbox`.
- Autoplan's unresolved C1 gate requires real-bag evidence for projected MID-360 scan stability and same-source `/Odometry` compensation before enforce mode; Gazebo is smoke-only.
- `multiplenav_launch.py` currently starts `cpp_lidar_filter` to `/livox/lidar_filtered`, but `pointcloud_to_laserscan` independently subscribes raw `/livox/lidar` and publishes `/scan`; therefore the SLAM path bypasses even the existing body-crop filter.
- `slam_toolbox` consumes `/scan` directly.
- `pointcloud_to_laserscan` projects points between z=0.1 and 1.0 m into ~722 angular bins, keeps the nearest range per bin, and sets `LaserScan time_increment=0`. Repeated returns from a stopped person have no temporal identity and look equivalent to static wall returns downstream.
- Existing ROS bag found at `/home/wangtao/livoxviewer/bags/mid360_20260709_222428`; suitability not yet known pending metadata inspection.
- Current root cause is confirmed: the mapping ingress has no temporal dynamic/static classification or session memory. This explains why normal SLAM parameters and STVL decay cannot solve moves-then-stops contamination.
- Official ROS 2 Humble `laser_filters` provides `LaserArrayFilter` temporal median filtering, but this is only a short-window noise baseline: an object stationary longer than the window becomes the median and remains present; it does not provide object identity/session memory.
- `ScanShadowsFilter` removes same-scan geometric shadow artifacts, not dynamic objects.
- Nav2 obstacle-layer observation persistence/decay applies to runtime costmaps and cannot erase occupancy already fused into `slam_toolbox` `/map`.
- No installed `laser_filters` support was found under `/opt/ros/humble` on this machine.
- Therefore the minimal temporal-median baseline cannot meet the reported moves-then-stops requirement; the approved tracker/session-memory architecture is the matching pattern, subject to C1 evidence.
- External references (treat as research data): https://index.ros.org/p/laser_filters/ ; https://docs.ros.org/en/ros2_packages/humble/api/filters/ ; https://docs.nav2.org/configuration/packages/costmap-plugins/obstacle.html ; https://docs.ros.org/en/humble/p/slam_toolbox/
- Existing 7.84s bag is ineligible for C1 because it has only obsolete `CustomMsg` `/livox/lidar` and `/livox/imu`, lacking `PointCloud2`, `/scan_raw`, `/Odometry`, `/tf`, `/tf_static`, and a move-stop sequence.
- User selected evidence-first C1 path on 2026-07-20.
- Stage 0 will publish evidence-only `/scan_raw` from `/livox/lidar_filtered`, while preserving current `/scan -> slam_toolbox`; it must not implement tracker or enforce.

## Phase 18 implementation decision (2026-07-20)
- User explicitly authorized implementation of a pragmatic session-memory filter after accepting that small pre-motion artifacts can be edited manually.
- The valid C1 run is `/home/wangtao/cod_mapping_sessions/20260719T204042Z-static-wall-motion`; it is `BLOCKED C1` only on stationary range MAD P95 (`0.4011 m` versus research threshold `0.08 m`). Scan rate, compensated residual, overlap, TF coverage, CPU and timestamp checks passed.
- This result must not be converted to PASS by changing Stage 0 thresholds. Phase 18 is a coarse practical prototype with independent acceptance tests.
- Required safety boundary: publish a separate `/scan_slam_filtered`; Nav2 costmaps continue consuming the live unfiltered/normal scan so remembered dynamic objects remain collision obstacles.
- Known semantic limit: an object cannot be classified dynamic before its first observed motion; points already fused before motion are not retroactively removed from slam_toolbox and may require map editing or replay.
- Approved-design constraints retained for the prototype: compare segment centroids in `odom`, bound tracks at 32, buffer roughly 1 second, confirm across multiple frames, mask with `NaN`, never emit fake `inf`, and retain dynamic track state through later stationarity/short occlusion.
- A delayed frame must retain segment-to-track IDs so a motion confirmation in a later frame can retroactively mask that same track in buffered scans before publication.
- Wall protection is required because the real projected MID-360 scan is noisy: only object-sized segments are motion candidates; long/wide static structures must not be promoted from centroid jitter alone.
- ROS I/O will remain thin. Segmentation, association, session memory, odometry interpolation and masking will be pure Python modules testable without a live ROS graph.
- Production `multiplenav_launch.py` currently projects raw `/livox/lidar` to `/scan` and lets slam_toolbox subscribe to its default scan topic; the standalone Phase 18 launch must instead project `/livox/lidar_filtered -> /scan_raw` and must not replace this path by default.
- The valid 120-second C1 bag contains no dynamic-object sequence by design. It can validate replay stability/output rate only; move-stop correctness must initially come from deterministic synthetic scans, followed later by a dedicated real move-stop bag for calibration.
- Practical v1 parameters selected for tests/config: 8-frame delay, 3+ point segments, 1.5 m maximum motion-candidate diameter, 0.20 m displacement, 0.12 m/s average speed, directional consistency, 0.75 m association gate, and 32-track hard cap. All remain YAML parameters rather than constants hidden in the algorithm.
- Node rollout contract: `observe` computes masks but publishes the delayed scan unchanged; `enforce` publishes NaN-masked ranges. Neither mode changes the no-delay Nav2 point-cloud branch.
- Implementation review tightened wall protection: segments outside the configured movable-object diameter range should not be tracked at all. This preserves the 32-track budget and ensures a dynamic target merging into a long wall causes no masking rather than erasing the combined wall segment.
- Fail-closed correction: `capacity_exceeded` must suppress `/scan_slam_filtered`, not just set `ready=false`; otherwise slam_toolbox could still ingest an output from an explicitly unhealthy filter.
- First real static-bag replay safely failed closed but exposed severe false promotion: 32/32 tracks became dynamic and no SLAM scans were published. The safety behavior worked; the initial motion classifier was not usable.
- False-promotion forensics showed implausible segment-centroid steps (typically 2-8 m/s) and/or 0.2-0.4 s observation gaps. A conservative existing-parameter profile (`min diameter 0.18 m`, `10 frames`, `0.30 m`, consistency `0.90`, max misses `4`) reduced false dynamics from 32 to 5 without capacity exhaustion.
- All five remaining false promotions are separable by requiring each confirmation step `<=1.2 m/s` and each confirmation gap `<=0.15 s`; four exceeded step speed and the fifth had a 0.4 s gap. These checks match the intended continuous multi-frame motion semantics.
- The pragmatic profile should use a 10-frame buffer so the 10-frame confirmation can still mask all pending frames before publication.
- Final static-bag validation: 1186 scans processed, 1176 outputs after warmup, zero rejects/dynamic tracks/masked beams, max 28 tracks, no capacity event, and about 0.78 ms pure-core time per scan.
- Final live ROS replay at 2x produced 121 messages in 6 seconds (20.145 Hz wall rate), each with 723 beams, finite returns and strictly increasing source timestamps. Diagnostics were healthy/ready with zero input errors, zero dynamic tracks and no capacity event.
- Production mapping now exposes `slam_filter_mode=disabled|observe|enforce`; disabled is the unchanged default. Filter modes remap only slam_toolbox, keep Nav2 STVL direct, and disable unconditional autosave.
- Residual calibration risk: move-stop memory is covered by deterministic synthetic tests, but no dedicated real move-stop bag exists yet. Operator rollout must begin in observe mode on the actual robot before enforce.
- Recorder requires PointCloud2 `/livox/lidar`, `/livox/lidar_filtered`, `/livox/imu`, `/scan_raw`, `/Odometry`, `/tf`, `/tf_static`, with `/scan` optionally recorded as current baseline.
- Evidence scenario is a 120-second low-speed static-wall motion sequence containing stationary, left/right rotation, forward/backward, and lateral motion; analyzer infers motion from odometry.
- Existing multiplenav `/scan` declares scan_time=0.3333 despite measured MID-360 output around 10Hz; evidence projector will use scan_time=0.1 and report observed/declared mismatch rather than changing the live SLAM path.

## Phase 19 real-robot rollout (2026-07-20)
- Session recovery reported no unsynchronized planning context.
- Current host has no residual ROS/Livox/Point-LIO/slam_toolbox/filter processes.
- The runbook requires first rollout in `observe`; `enforce` is gated on healthy diagnostics, ready output, no static false tracks, and a real move-stop target retaining dynamic identity.
- Network inspection from the restricted environment failed at netlink access, so host-scoped inspection is required before launching the robot stack.
