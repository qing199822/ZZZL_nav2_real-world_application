# Task Plan: COD导航串口通信迁移 + 仿真环境搭建

## Goal
1. 将串口通信从 `ros2_simple_serial` 替换为 `RMserial-main`(Seasky协议)，修复所有审查问题
2. 搭建自包含 Gazebo 仿真环境，共享场地模型和地图
3. 创建完整操作文档，覆盖真机和仿真两种模式

## Current Phase
Phase 19 进行中 🚧 — 真机 observe 模式验证移动后停止目标与静态场景误报

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

| TF 倾斜 (v2) | base_link→livox_frame TF 写入 pitch | extrinsic_R (IMU↔LiDAR内部) 保持不变, 重力方向正确 |

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

### Phase 12: LiDAR 45° 前倾安装参数适配 ✅
- [x] 确定修改方案: Pitch 写入 base_link→livox_frame TF，不改 extrinsic_R
- [x] 真机: singlenav_launch.py + multiplenav_launch.py (pitch 0.0→0.7854)
- [x] 仿真: gazebo_sim + gazebo_slam + sim_standalone launch + cod_robot.urdf.xacro
- [x] mid360.yaml extrinsic_R 保持单位矩阵 (IMU/LiDAR 芯片相对旋转不变)
- [x] reviewer 审查通过 (无功能性问题, 4 cosmetic 修复)
- [x] 生成迁移文档: ~/程序改动迁移文件夹/LiDAR_45度前倾安装_参数改动记录.md
- **Status:** complete

### Phase 14: 导航代码安全修复 + 坐标系对齐 ✅
- [x] ask_gateway 全面审查导航代码 (6个源文件 + 配置文件)
- [x] 修复 PID: 积分限幅顺序, derivative kick, NaN保护, reset/setGains
- [x] 修复碰撞检测: fail-open→fail-safe, 密集采样, NO_INFORMATION检查
- [x] 修复 GoalApproachController: TF变换goal, 保留angular.z, 上限限速
- [x] 修复 fake_vel_transform: angular.z透传, TF补平移, watchdog, enable_vel_rotation开关
- [x] 重构 cod_serial: 持久连接, 修正include, 日志降级, 断线重连
- [x] 修复 intensity_voxel_layer: 丢弃超Z范围点(不再钳位)
- [x] 优化 filter_node: 参数缓存+动态回调, voxel滤波参数化
- [x] 配置调整: controller_frequency 50→30Hz, progress_checker 999→30s
- [x] 坐标系对齐: Nav2 robot_base_frame→base_link, enable_vel_rotation→false(首测)
- [x] 生成修复文档: docs/nav2_code_fixes.md (640行)
- [x] 复查确认 7/7 修复项通过 (ask_gateway + gpt55)
- **Status:** complete


## Errors Encountered (追加)
| Error | Phase | Resolution |
|-------|-------|------------|
| model://rmul_2025 无法找到 | 11 | env hook 改用 GZ_SIM_RESOURCE_PATH |
| direct Edit 被 Composer hook 阻止 | 17 | 使用 composer_code_cli 完成精确的 planning-only 更新 |
| old ROS bag rejected because it lacks PointCloud2/current scan/odom/TF and move-stop scenario | 17 | resolution is a new complete 120-second evidence capture |
| `test_session_filter.py` cannot import `slam_dynamic_filter.session_filter` | 18 | expected TDD red; implement the pure algorithm module next |
| ROS launch test cannot create `~/.ros/log` in managed sandbox | 18 | point `ROS_LOG_DIR` to `/tmp` for sandboxed tests; production behavior unchanged |
| ROS message sequence assertions compared `array('f')` directly to list | 18 | compare `list(message.ranges/intensities)` in tests |
| source node wrapper chmod denied by managed sandbox | 18 | request scoped permission for file-mode update; CMake install remains declared |
| `colcon test-result` attempted to create a new workspace log under sandbox | 18 | rely on the completed 12/12 CTest output; rerun result summary only with writable log scope if needed |
| sandboxed Fast DDS cannot create local UDP socket (`Operation not permitted`) | 18 | stop the restricted node and rerun scoped ROS replay commands with local DDS permission |
| first direct replay used relative `odometry` instead of bag topic `/Odometry` | 18 | restart the node with the same remaps declared by `mapping_filter.launch.py` |
| Ctrl+C caused a second `rclpy.shutdown()` and RCLError | 18 | catch KeyboardInterrupt and call shutdown only while `rclpy.ok()` |
| compressed bag reader needed write access beside immutable evidence | 18 | copy exact bag/metadata to `/tmp`, analyze there, then remove the 1.28 GB temporary copy |
| cod_bringup package-wide flake8 reports 41 pre-existing issues | 18 | clean all findings in touched `multiplenav_launch.py`; leave unrelated singlenav/navigation legacy formatting unchanged |
| sandboxed `py_compile` tried to create source `__pycache__` | 18 | rerun with `PYTHONPYCACHEPREFIX=/tmp/codex_pycache`, then remove that temporary cache |
| final `ament_flake8` invocation omitted ROS environment | 18 | source `/opt/ros/humble/setup.bash` and rerun the same touched-file check |
| `git add -A` cannot create `.git/index.lock` because managed Git metadata is read-only | 19 | rerun the repository-scoped Git write with explicit approval; working files were unaffected |
| first `git push origin improve` rejected as non-fast-forward | 19 | fetch and integrate the concurrently updated remote branch; never force-push |
| two SSH fetch attempts disconnected (`early EOF`) | 19 | fetch the same public `improve` ref over HTTPS; completed successfully |

### Phase 13: 参考仓库验证 + SLAM 测试 ✅
- [x] 克隆参考仓库 (github.com/qing199822/ZZZL_nav2_real-world_application)
- [x] colcon build 15/15 包通过
- [x] slam_toolbox 配置加载验证 (CeresSolver, lifelong 模式)
- [x] multiplenav_launch.py 17 节点启动验证
- [x] 确认两仓库代码一致
- **Status:** complete

### Phase 15: 真机雷达验证 ✅
- [x] 构建修复: const Clock, uart_transporter链接, PID括号, ControllerException
- [x] xfer_format=1→0 (CustomMsg→PointCloud2)
- [x] launch文件语法修复 (livox_frame TF缺逗号)
- [x] 注释realsense2_camera (未安装)
- [x] 雷达→LIO→SLAM全管线验证通过
- [x] Odometry+Map+Scan 数据确认
- **Status:** complete

### Phase 16: MCU连接+导航闭环测试 (待MCU就绪)
- [ ] 连接MCU (/dev/cod_mcu)
- [ ] 设置导航目标点验证运动控制
- [ ] 恢复realsense2_camera (安装驱动后)
- **Status:** pending

### Phase 17: SLAM 动态障碍物污染静态地图调查与修复 ✅
- [x] trace multiplenav LiDAR→point-cloud filter→LaserScan→slam_toolbox data flow
- [x] review existing dynamic-obstacle design against real config
- [x] 实现 evidence-only `/scan_raw` 投影、完整真机 recorder 和冻结 metadata（不改变 `/scan -> slam_toolbox`）
- [x] 实现 scan/odom/TF/R5 指标和机器可读 C1 gate
- [x] 录制 120 秒 static-wall-motion 真机证据并执行分析
- [x] 记录有效 `BLOCKED C1`：仅静止量测噪声未满足研究级 8 cm 门限，其他检查通过
- **Status:** complete

### Phase 18: 建图专用 session-memory 动态过滤器 ✅
- [x] 复核已批准设计、Stage 0 包结构和生产 launch 数据流
- [x] 定义粗粒度聚类/关联/运动确认/会话记忆的纯算法接口与参数
- [x] 实现 `/scan_raw -> /scan_slam_filtered` ROS 节点；只影响 SLAM，不影响 Nav2 costmap
- [x] 增加单元测试和合成 move-stop 场景，覆盖静态墙、移动后停止、跟踪丢失及保守失败行为
- [x] 使用现有真机 bag 做回放烟雾测试并检查话题/频率/非空输出
- [x] 验证通过后提供 `disabled|observe|enforce` opt-in；生产默认仍为 `disabled`
- **Status:** complete

### Phase 19: 真机 observe 模式标定与准入验证 🚧
- [ ] 核对雷达直连网络、驱动和工作空间构建产物
- [ ] 启动 `multiplenav_launch.py slam_filter_mode:=observe`
- [ ] 验证过滤器健康状态、输出频率、静态环境零误报
- [ ] 让代表性移动单位完成“移动后停止”，确认产生并保留动态轨迹
- [ ] 记录专用 move-stop bag 和诊断结果
- [ ] 根据证据决定是否进入受控 `enforce` 试验
- **Status:** in_progress
