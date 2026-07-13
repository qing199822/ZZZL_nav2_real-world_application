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
