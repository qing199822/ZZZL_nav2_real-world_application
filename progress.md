# Progress Log: COD导航项目完整开发记录

## Session 2026-07-05

### 15:00-15:08 — 代码探索
- 3 并行 Explore 代理: ros2_simple_serial, RMserial-main, launch 集成
- Plan 代理设计迁移方案 → 用户审批

### 15:08-15:20 — 代码迁移
- 复制 4 个包, 修改 3 个 launch 文件, 修复依赖
- colcon build 14/14 包通过, 节点启动验证

### 15:20-15:31 — 第一轮审查+修复
- GLM5.2: 20 个问题 → 全部修复

### 15:31-15:35 — 第二轮审查+修复
- GLM5.2: 6 个残留问题 → 全部修复

### 15:35-15:44 — 规划文件创建
- task_plan.md, findings.md, progress.md

### 15:44-15:57 — Gazebo 仿真环境搭建
- 创建 cod_gazebo_simulator 包, 机器人 URDF, 3 个 launch
- 复制 5 个世界 + 4 个场地模型(32MB), 5 套地图
- ros_gz_bridge 配置, Nav2 仿真参数, RViz 配置, env-hooks

### 15:57-16:05 — SLAM 仿真补充
- 创建 gazebo_slam.launch.py

### 16:05-16:15 — 操作文档
- docs/OPERATING_GUIDE.md (471行, 8章+附录)

### 16:15-16:30 — 第三轮审查+修复
- GLM5.2 审查 80+ 文件: 4 CRITICAL + 5 HIGH + 8 MEDIUM
- 全部修复: xacro处理, TF, AMCL, 桥接, 世界文件, 硬编码路径

### 16:30-16:35 — 第四轮审查+最终修复
- 验证所有前序修复, 发现并修复最后 3 个问题
- 构建通过, 15/15 包

## 最终交付物
| 交付物 | 规模 |
|--------|------|
| cod_gazebo_simulator 包 | ~45 文件, 31MB资源 |
| 修改的 launch 文件 | 3 个仿真 + 2 个真机 + 1 个 auto_save |
| serial_def_sdk 修复 | ~10 处关键修复 |
| OPERATING_GUIDE.md | 471 行 |
| 规划文件 | task_plan + findings + progress |

## 待用户执行 (Phase 10)
- [ ] 安装 Ignition Gazebo + Livox 驱动
- [ ] 配置雷达 IP + 串口权限
- [ ] MCU 固件升级 (Seasky 协议)
- [ ] 仿真实测: ros2 launch cod_gazebo_simulator gazebo_slam.launch.py

## Session 2026-07-06 — Livox MID-360 雷达驱动安装

### 19:22-19:23 — 环境检查
- 确认系统: x86_64, Ubuntu 22.04, ROS2 Humble, cmake 3.22.1, gcc 11.4.0
- 确认 Livox SDK2 未安装, livox_ros_driver2 未在工作空间
- 安装依赖: libapr1-dev

### 19:23-19:26 — Livox SDK2 安装
- 克隆 https://github.com/Livox-SDK/Livox-SDK2 → ~/Livox-SDK2/
- cmake + make -j 编译通过 (仅 rapidjson pragma warning)
- sudo make install → /usr/local/lib/liblivox_lidar_sdk_shared.so + headers

### 19:27-19:28 — livox_ros_driver2 克隆+构建
- 克隆到 src/livox_ros_driver2
- 更新 MID360_config.json: Host IP 192.168.1.50, Radar IP 192.168.1.181
- colcon build --packages-select livox_ros_driver2 通过

### 19:29-19:30 — 雷达网络配置
- 创建 NetworkManager 连接 "Livox-MID360"
- 接口 enp5s0, 静态 IP 192.168.1.50/24, 无默认网关
- 雷达连接后: sudo nmcli connection up Livox-MID360

### 19:31 — Livox Viewer 2 GUI 安装
- 下载 Livox Viewer 2 v2.5.9 (87MB, UE5应用)
- 解压到 ~/LivoxViewer2/, 创建 ~/bin/livox-viewer 快捷方式
- 启动: ~/bin/livox-viewer

## Session 2026-07-09 — 硬件部署推进

### 串口权限配置
- sudo apt purge modemmanager ✅
- sudo apt purge brltty + libbrlapi0.8 + python3-brlapi ✅
- sudo usermod -a -G dialout wangtao ✅
- 验证: groups 确认 dialout 组成员身份

### MCU 固件升级确认
- 确认需要电控组同步修改 MCU 固件，新增 Seasky 帧解析逻辑
- fix_control (msg_id=0x05) 帧结构已提供给用户转交电控组

## Session 2026-07-11 — MCU 协议文档

### MCU Seasky 协议规范文档
- 创建 docs/MCU_SEASKY_PROTOCOL.md (476行)
- 包含: 物理层参数、帧结构、CRC-8/16 算法+查表、12 条消息详细定义
- 包含: 接收/发送伪代码、优先级分级、旧协议对照
- 交付电控组用于固件升级

## Session 2026-07-11 — MCU 参考实现 + udev 规则

### 创建文件 (14 new)

**config/**
- `config/99-cod-mcu-serial.rules` (53行) — 3 种匹配策略 + dialout 权限

**docs/mcu_reference/ (11 files)**
- `README.md` (290行) — 移植指南, STM32/ESP32 HAL 示例
- `seasky_protocol.h` (244行) — 协议接口, 12 消息结构体
- `seasky_protocol.c` (223行) — 解帧状态机 + 组帧, 与上位机一致
- `crc8.h/c` — 从上游复制 (SHT75 查表)
- `crc16.h/c` — 从上游复制 (CRC-16-IBM)
- `ring_buffer.h/c` (70+64行) — ISR-safe, 无锁, 2的幂
- `hal_uart.h` (104行) — 3 核心 + 3 可选 HAL 接口
- `main_template.c` (318行) — 完整示例: 收 fix_control + 周期发送 5 类消息

### 修改文件 (4 existing)
- `serial.launch.py`: /dev/ttyUSB0 → /dev/cod_mcu
- `singlenav_launch.py`: /dev/ttyACM0 → /dev/cod_mcu
- `multiplenav_launch.py`: /dev/ttyACM0 → /dev/cod_mcu
- `docs/MCU_SEASKY_PROTOCOL.md`: +91行 (§9 USB虚拟串口+udev)

### 验证
- gcc -fsyntax-only: 所有 .c 文件通过 ✅
- CRC-8 查表: 与上游一致 ✅
- CRC-16 文件: 完全相同 ✅
- colcon build: 2/2 包通过 ✅
- udevadm: 可用 ✅

## Session 2026-07-11 — 审查修复 + 迁移清单

### GLM 5.2 代码审查修复
- 17/17 问题已修复 (2 CRITICAL + 3 HIGH + 5 MEDIUM + 5 LOW + 2 INFO)
- 全部 MCU C 代码编译通过, colcon build 2/2 包通过

### 新上位机迁移清单
- 创建 docs/HOST_SETUP_CHECKLIST.md (294行, 11 步骤)
- 覆盖: ROS2 安装、服务清理、权限、udev、Livox 网络、雷达驱动、依赖、构建、仿真、验证

### 第五轮审查: MCU 参考代码 (GLM 5.2) — 2026-07-11
- 审查 docs/mcu_reference/ 全部 11 文件
- 发现 24 问题 (5 CRITICAL + 5 HIGH + 9 MEDIUM + 5 LOW)
- 24/24 修复完成
- gcc -fsyntax-only: 5/5 .c 文件通过 ✅
- CRC-16 表验证: 与多项式 0xA001 计算一致 ✅

### 第五轮审查修复 (GLM 5.2) — 2026-07-11
- GLM 5.2 发现 12 问题 (1 CRITICAL + 2 HIGH + 4 MEDIUM + 5 LOW)
- 12/12 修复完成 ✅
- gcc -fsyntax-only: 5/5 .c 文件 0 errors ✅

## Session 2026-07-15 — LiDAR 45° 前倾安装参数适配

### 方案讨论
- 用户反馈: MID-360 雷达安装倾斜了 45° (前倾 Pitch)
- 分析需要修改的文件: 6 个 launch/URDF + 确认 mid360.yaml 不改
- 设计原则: Pitch 写入 base_link→livox_frame TF, extrinsic_R 保持 I
  - MID-360 内部 LiDAR 和 IMU 芯片刚性固定, 倾斜整个模块不改变相对旋转
  - 如果写入 extrinsic_R, LIO 重力方向对不上, 里程计发散

### 文件修改 (7 files)
- 真机: singlenav_launch.py + multiplenav_launch.py (pitch 0.0→0.7854)
- 仿真: gazebo_sim + gazebo_slam + sim_standalone launch (新增/修改 pitch)
- 仿真: cod_robot.urdf.xacro (rpy 0 0 0 → 0 0.7854 0)
- mid360.yaml: extrinsic_R 保持不变, 添加注释说明

### 审查
- reviewer agent 审查: ✅ 通过, 无功能性问题
- 发现 4 cosmetic 问题 (注释格式不一致) → 全部修复

### 交付物
- 迁移文档: ~/程序改动迁移文件夹/LiDAR_45度前倾安装_参数改动记录.md (172行)

## Session 2026-07-16 — 参考仓库验证 + SLAM 测试

### 22:38-22:44 — 仓库克隆+构建
- git clone https://github.com/qing199822/ZZZL_nav2_real-world_application → ~/ZZZL_nav2_reference
- rosdep install (部分失败, sudo 不可用) → colcon build --symlink
- 15/15 包构建通过 ✅

### 22:44-22:50 — SLAM 配置验证
- slam_toolbox 独立启动: 配置加载成功 ✅
- syntax check: 6/6 launch 文件语法正确 ✅
- multiplenav_launch.py 启动测试: 17/17 节点启动成功 ✅
- 验证: livox_frame TF pitch=0.7854, Nav2 lifecycle 全部激活

### 关键发现
- SLAM 仅在 multiplenav (多点导航) 和 gazebo_slam (仿真) 中集成
- singlenav (单点导航) 是纯定位模式, 需要预建地图
- 参考仓库代码与我们项目完全一致

## Session 2026-07-17 — 导航代码安全审查+修复

### 21:24-21:40 — 代码审查
- ask_gateway 全面审查 9 个导航源文件 + Nav2 配置文件
- 发现 8 CRITICAL + 多 HIGH/MEDIUM 问题
- 综合评分: 4.7/10 (原型可用，高速前需修复)

### 21:40-22:00 — 安全修复实施
- PID: 积分顺序修复, derivative kick防护, NaN保护, reset/setGains
- 碰撞检测: fail-open→fail-safe(costmap越界→不安全), 密集采样, NO_INFORMATION检查
- GoalApproachController: TF变换goal, 保留angular.z, 上限限速, TF失败→透传
- fake_vel_transform: angular.z透传, TF补平移, watchdog, enable_vel_rotation开关
- cod_serial: 持久连接, 修正include, 日志降级, 断线重连退避
- intensity_voxel_layer: 丢弃超Z范围点(不再钳位)
- filter_node: 参数缓存+动态回调, voxel滤波参数化
- 配置: controller_frequency 50→30Hz, progress_checker 999→30s

### 22:00-22:10 — 坐标系对齐(首测)
- Nav2 robot_base_frame: base_link_fake→base_link (全部5处)
- fake_vel_transform enable_vel_rotation 默认 false (直通模式)
- 验证速度方向链: Nav2 vx→base_link+x→LiDAR前倾方向→电控vx 三者一致

### 22:10-22:20 — 复查+文档
- ask_gateway复查: 7/7项通过, 修复残留的TF fallback问题
- gpt55复查: 确认修复有效
- 生成 docs/nav2_code_fixes.md (640行完整修复记录)

### 22:20-22:40 — 高速自转方案讨论
- 确认 base_link_fake 在自转1.5rad/s下是合理的field-centric control设计
- 确认圆形footprint + Omni MPPI适配自转场景
- 确认首测用直通模式, 自转时切回旋转模式

### 修改文件统计
| 文件 | 行数变化 |
|------|---------|
| pid.hpp | 重构 (+integral_min/max, reset, setGains, getIntegral) |
| pid.cpp | 重写 (积分顺序, 防kick, NaN保护, dt验证) |
| omni_pid_pursuit_controller.cpp | ~800行中修复6处关键逻辑 |
| goal_approach_controller.cpp | 重写 (TF变换, 上限限速, 透传) |
| fake_vel_transform.hpp/cpp | 重写 (透传angular.z, watchdog, 旋转开关) |
| cod_serial.cpp | 重写 (持久连接, 日志降级, 重连) |
| intensity_voxel_layer.cpp | 1处逻辑修复 (Z范围) |
| filter_node.cpp | 重写 (参数缓存, 动态回调) |
| back_up_free_space.cpp | 1处修复 (去重) |
| singlenav2_params.yaml | 5处参数调整 |

## Session 2026-07-18 — 真机雷达验证

### 02:36-02:40 — 构建修复
- livox_ros_driver2 符号链接冲突 → 清理重建
- cod_serial_ul26 const Clock + 缺失 uart_transporter → 修复
- pb_omni_pid_pursuit_controller 括号错误 + ControllerException → 修复
- 16/16 包构建通过 (首次包含 Phase 14 全部修复)

### 02:41-02:46 — 雷达驱动测试
- livox_ros_driver2 启动成功, LiDAR 10Hz + IMU 200Hz
- 发现 xfer_format=1 导致 CustomMsg/PointCloud2 类型冲突
- 修复 xfer_format=0 → PointCloud2

### 02:45-02:50 — 导航系统启动
- 发现 launch 文件 --pitch 0.7854--yaw 语法错误 → 修复缺逗号
- 注释 realsense2_camera (未安装)
- 全系统启动: 22 节点全部存活

### 02:48 — 管线验证通过
- /Odometry: nav_msgs/Odometry, odom→base_link ✅
- /map: OccupancyGrid, 0.05m分辨率, SLAM建图 ✅  
- /scan: LaserScan, slam_toolbox订阅 ✅
- 修复3个bug, 雷达感知-定位-建图管线完整可用

### 待完成
- [ ] MCU 串口连接 (/dev/cod_mcu)
- [ ] 导航闭环测试 (设置goal验证运动)
- [ ] 恢复 realsense2_camera (安装驱动后)
