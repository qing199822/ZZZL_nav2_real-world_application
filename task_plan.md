# Task Plan: COD导航串口通信迁移 + 仿真环境搭建

## Goal
1. 将串口通信从 `ros2_simple_serial` 替换为 `RMserial-main`(Seasky协议)，修复所有审查问题
2. 搭建自包含 Gazebo 仿真环境，共享场地模型和地图
3. 创建完整操作文档，覆盖真机和仿真两种模式

## Current Phase
所有阶段完成 ✅ — 交付物就绪，硬件部署待用户执行

## Phases

### Phase 1: 代码探索与分析 ✅
- [x] 深入阅读 ros2_simple_serial 全部源码
- [x] 深入阅读 RMserial-main 全部源码 (6层架构)
- [x] 分析协议差异 (15字节裸包 vs Seasky帧协议)
- [x] 分析数据流和TF树完整性
- **Status:** complete

### Phase 2: 方案设计 ✅
- [x] 确定话题接口适配 (保留 fake_vel_transform, 重映射→aft_cmd_vel)
- [x] 确定包迁移方案 (复制4个包到 src/)
- [x] 用户审批
- **Status:** complete

### Phase 3: 代码迁移 ✅
- [x] 复制 def_msg, vision_msg, pb_rm_interfaces, serial_def_sdk
- [x] 修改 serial.launch.py + multiplenav/singlenav launch
- [x] 修复依赖 (std_mags, builtin_interfaces, X11)
- [x] colcon build 14/14 包通过
- **Status:** complete

### Phase 4: 第一轮审查+修复 ✅
- [x] 4 CRITICAL: uart空指针, 双重线程, CRC绕过, 数据竞争(标注)
- [x] 5 HIGH: open()返回值, fd TOCTOU, 端口三重配置, TF不一致, 协议魔数
- [x] 6 MEDIUM: CRC-8校验, 静态变量, spin_speed, 超时配置, livox_frame TF
- **Status:** complete

### Phase 5: 第二轮审查+修复 ✅
- [x] 验证修复质量, 发现 sentry_state 长度覆盖 + 超时参数未生效
- [x] 修复 topic_timeout_ms_ 实际应用 + abs→std::abs
- **Status:** complete

### Phase 6: Gazebo 仿真环境搭建 ✅
- [x] 创建 cod_gazebo_simulator 包 (自包含: worlds+models+maps+configs)
- [x] 创建占位机器人 URDF (LiDAR+IMU+DiffDrive, 用户可替换)
- [x] 创建 3 个 launch 文件 (SLAM/定位/空世界)
- [x] 复制 4 个比赛场地模型 + 5 个世界 SDF
- [x] 创建仿真 Nav2 参数 + ros_gz_bridge 配置
- [x] 创建 RViz 配置 + env-hooks
- **Status:** complete

### Phase 7: 第三轮审查+仿真修复 ✅
- [x] 审查整个项目 (80+文件)
- [x] 4 CRITICAL: robot_description缺失, livox_frame TF, 无AMCL, 桥接类型错误
- [x] 5 HIGH: model名称, 时钟重复, 硬编码路径, static=False, ogre渲染
- [x] 8 MEDIUM: xacro传参, 地图不匹配, 孤立文件, 类型命名不一致等
- [x] 全部修复 + 构建验证
- **Status:** complete

### Phase 8: 操作文档 ✅
- [x] 创建 docs/OPERATING_GUIDE.md (471行, 8章节+附录)
- [x] 覆盖: 项目结构、真机2种模式、仿真3种模式、缓存清理、参数参考(10表)、TF树、调试命令、故障排查
- **Status:** complete

### Phase 9: 第四轮审查+最终修复 ✅
- [x] 4 轮审查完成, 累计发现并修复 24 CRITICAL + 20 HIGH + 21 MEDIUM + 13 LOW
- [x] 最后修复: model.config XML, AMCL transform_tolerance, 死代码清理
- [x] 15/15 包构建通过
- **Status:** complete

### Phase 10: 硬件部署 (进行中) 🚧
- [x] 安装 Livox SDK2 + livox_ros_driver2 ✅
- [x] 配置雷达 IP (MID360_config.json + Netplan) ✅
- [x] 安装 Livox Viewer 2 GUI ✅
- [x] 串口权限 (purge modemmanager/brltty, dialout组) ✅
- [x] 网络配置 (enp5s0, 192.168.1.50/24, nmcli Livox-MID360) ✅
- [ ] MCU 固件同步升级 (解析 Seasky 协议帧)
- [ ] 硬件在环测试
- [ ] 仿真环境实测 (需 Ignition Gazebo 安装)
- **Status:** pending

## 四轮审查趋势
| 轮次 | CRITICAL | HIGH | MEDIUM | LOW |
|------|----------|------|--------|-----|
| 第一轮 (serial_def_sdk) | 4 | 5 | 6 | 5 |
| 第二轮 (修复验证) | 1 | 1 | 1 | 3 |
| 第三轮 (仿真包) | 4 | 5 | 8 | 4 |
| 第四轮 (最终验证) | 1 | 0 | 1 | 1 |

## Key Architectural Decisions
| 决策 | 选择 | 理由 |
|------|------|------|
| 话题桥接 | 保留 fake_vel_transform, 重映射→aft_cmd_vel | MCU 需要机体坐标系速度 |
| CRC 策略 | #ifdef NO_CRC_MODE, 默认严格 | 安全优先, 兼容未就绪MCU |
| 超时参数 | 100ms, launch可配 | 50Hz控制环, 5周期 |
| TF 补充 | livox_frame→base_link (z=0.15) | LIO/pointcloud_to_laserscan 依赖 |
| 仿真里程计 | Gazebo DiffDrive (替代LIO) | 真机用LIO, 仿真用ground truth |
| 仿真驱动 | ros_gz_bridge (替代serial) | cmd_vel→Gazebo, Odometry/LiDAR/IMU→ROS |

## Deliverables
| 交付物 | 位置 |
|--------|------|
| 串口迁移 | src/serial_def_sdk/ (+ def_msg, vision_msg, pb_rm_interfaces) |
| 仿真包 | src/cod_gazebo_simulator/ (自包含, 3 launch, 5 worlds, 4 venue models) |
| 操作文档 | docs/OPERATING_GUIDE.md (471行, 中英文混合) |
| 规划文件 | task_plan.md, findings.md, progress.md (项目根目录) |

### Phase 11: Gazebo 世界加载验证 + env hook 修复 ✅
- [x] 测试空世界: 加载成功 (ground_plane)
- [x] 测试 rmul_2025: 失败 — model://rmul_2025 无法解析
- [x] 根因: Ignition Gazebo Fortress 使用 GZ_SIM_RESOURCE_PATH (非 GAZEBO_MODEL_PATH) 解析 model://
- [x] 修复 env-hooks: gazebo_model_path.dsv.in → GZ_SIM_RESOURCE_PATH
- [x] 重测全部 5 个世界: 全部加载成功
- **Status:** complete

## Errors Encountered (追加)
| Error | Phase | Resolution |
|-------|-------|------------|
| model://rmul_2025 无法找到 | 11 | env hook 改用 GZ_SIM_RESOURCE_PATH |
