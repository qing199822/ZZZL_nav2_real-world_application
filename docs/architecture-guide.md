# COD RM2026 导航系统 — 架构与实现详解

> 面向 C++/ROS2 初学者，从零理解项目架构、核心模块及实现原理。
> 与 [OPERATING_GUIDE.md](../cod_-rm2026_-navigation-master/docs/OPERATING_GUIDE.md) 互补——后者侧重操作，本文档侧重原理。

## 目录

1. [项目概述](#1-项目概述)
2. [前置概念（C++/ROS2 小白必读）](#2-前置概念cros2-小白必读)
3. [整体架构](#3-整体架构)
4. [模块详解](#4-模块详解)
   - 4.1 [cpp_lidar_filter — 点云裁剪](#41-cpp_lidar_filter--激光雷达点云裁剪)
   - 4.2 [small_point_lio — 激光惯性里程计](#42-small_point_lio--激光惯性里程计)
   - 4.3 [fake_vel_transform — 坐标变换](#43-fake_vel_transform--坐标变换)
   - 4.4 [Nav2 导航核心栈](#44-nav2-导航核心栈)
   - 4.5 [全局规划器 — planner_server](#45-全局规划器--planner_server)
   - 4.6 [局部控制器 — GoalApproachController + MPPI](#46-局部控制器--goalapproachcontroller--mppi)
   - 4.7 [pb_omni_pid_pursuit_controller — PID 纯追踪（备用）](#47-pb_omni_pid_pursuit_controller--pid-纯追踪控制器备用方案)
   - 4.8 [代价地图 — Costmap](#48-代价地图--哪里能走哪里不能走)
   - 4.9 [行为树 — 应急预案](#49-行为树--导航的应急预案)
   - 4.10 [pb_nav2_plugins — 自定义插件](#410-pb_nav2_plugins--自定义导航插件)
   - 4.11 [串口通信](#411-serial_def_sdk--串口通信)
   - 4.12 [waypoint_editor — 航点编辑器](#412-waypoint_editor--可视化航点编辑器)
   - 4.13 [pointcloud_to_laserscan — 3D转2D](#413-pointcloud_to_laserscan--3d-点云转-2d-激光)
5. [TF 坐标变换树](#5-tf-坐标变换树)
6. [两种运行模式对比](#6-两种运行模式)
7. [完整数据流](#7-完整数据流总结)
8. [关键设计决策](#8-为什么不直接用-nav2-原生配置)
9. [参数调优速查](#9-参数调优速查)

---

## 1. 项目概述

本项目是 **RoboMaster 2026 比赛机器人**的自主导航系统，基于 **ROS2 Humble + Nav2** 框架构建，运行在 **Ubuntu 22.04** 上，主传感器为 **Livox MID-360 激光雷达**。

机器人底盘为**全向（Omni）运动模型**——可以在不旋转的情况下向任意方向平移，导航系统需要充分利用这一特性。

| 关键信息 | 内容 |
|---------|------|
| 操作系统 | Ubuntu 22.04 |
| ROS2 版本 | Humble |
| 主传感器 | Livox MID-360 (非重复扫描 LiDAR) |
| 障碍观测源 | Livox MID-360（真机未安装 RealSense） |
| 底盘类型 | 全向 (Omni-directional) |
| 串口 | /dev/ttyACM0, 115200bps |
| 控制器 | MPPI (模型预测路径积分) |
| 规划器 | SmacPlanner2D (Hybrid-A*) |
| 里程计 | Point-LIO (激光惯性融合) |

MID-360 光心实测离地 `0.46m`，安装 pitch 为 `+0.7854rad`。导航以 `base_link` 的 z=0 作为地面基准，所有点云裁剪、高度门限和碰撞监控都在该几何关系下工作。

---

## 2. 前置概念（C++/ROS2 小白必读）

### 2.1 ROS2 的核心通信模型

ROS2 的核心是一个**消息传递系统**。每个程序被称为**节点（Node）**，节点之间通过**话题（Topic）**传递消息：

```
雷达节点 ──/livox/lidar (点云)──→ 导航节点 ──/cmd_vel (速度指令)──→ 底盘控制节点
```

这就像一个公司内部的邮件系统：每个部门独立工作，通过邮件通信。你可以单独开发、测试、替换任何一个节点，只要消息格式不变。

#### 关键术语

| 术语 | 含义 | 类比 |
|------|------|------|
| Node (节点) | 一个独立的可执行程序 | 一个部门 |
| Topic (话题) | 命名的消息通道 | 邮件主题 |
| Publisher (发布者) | 向话题发送消息的节点 | 发件人 |
| Subscriber (订阅者) | 从话题接收消息的节点 | 收件人 |
| TF (坐标变换) | 坐标系之间的位置关系 | GPS 告诉你 "你的位置在地图上的哪里" |
| Launch File | 一次性启动多个节点的脚本 | 开机自启动脚本 |

### 2.2 Nav2 是什么？

Nav2 是 ROS2 官方的**导航框架**，可以看作一个"导航大脑"——输入传感器数据和目标位置，输出速度指令。它由多个模块组成：

```
传感器数据 ──→ 代价地图 (哪里能走、哪里不能走)
                    ↓
目标位置 ──→ 全局规划器 (规划大致路径) ──→ 局部控制器 (实时生成速度指令) ──→ 底盘
                    ↑                         ↑
              定位系统                 行为树 (协调整个流程:
            (我在哪里？)              规划→追踪→失败→恢复→重试)
```

### 2.3 行为树（Behavior Tree）

行为树是一棵**倒立的逻辑树**，每个节点代表一个动作或条件判断。导航流程被编码为：

```
如果迷路了 → 清除代价地图 → 重新规划
如果被挡住了 → 尝试后退 → 还不行就转圈找路
正常情况 → 规划路径 → 跟踪路径 → 到达目标
```

使用行为树的好处是可以**可视化编辑**（用 Groot 工具），不需要修改 C++ 代码就能调整导航策略。

### 2.4 代价地图（Costmap）

代价地图将世界划分为**栅格**（每个格子 5cm × 5cm），每个格子有一个"代价"值（0-255）：

| 代价值 | 含义 |
|--------|------|
| 0 | 自由空间，可以走 |
| 1-127 | 离障碍物逐渐靠近，代价值递增 |
| 253 | 膨胀区域边缘（机器人外壳会碰到） |
| 254 | 有障碍物 |
| 255 | 致命障碍物，绝对不可通过 |

### 2.5 C++ 中的关键模式：插件机制（Pluginlib）

ROS2/Nav2 大量使用**插件机制**——你可以自定义一个控制器或规划器，只需继承标准接口并注册为插件：

```cpp
// 继承 nav2_core::Controller 接口
class GoalApproachController : public nav2_core::Controller {
    void configure(...) override;   // 加载时调用，读取参数
    void activate() override;       // 启动时调用
    TwistStamped computeVelocityCommands(...) override;  // 核心：计算速度
    void setPlan(...) override;     // 接收新路径
};

// 注册为 Nav2 可用的插件
PLUGINLIB_EXPORT_CLASS(GoalApproachController, nav2_core::Controller)
```

然后在 YAML 配置中指定使用哪个插件：

```yaml
controller_plugins: ["FollowPath"]
FollowPath:
  plugin: "goal_approach_controller::GoalApproachController"
```

---

## 3. 整体架构

### 3.1 节点启动图

运行 `ros2 launch cod_bringup singlenav_launch.py` 时，以下节点同时启动：

```
┌──────────────────────────────────────────────────────────────────────┐
│                         信息流全景图                                   │
│                                                                      │
│  Livox雷达驱动      cpp_lidar_filter      small_point_lio             │
│  /livox/lidar  →   剪掉车身点云      →    估计里程计                   │
│                     /livox/lidar_filtered  /Odometry                  │
│                          ↓                     ↓                      │
│                          ├─────────────────────┤                      │
│                          ↓                     ↓                      │
│                     代价地图层           fake_vel_transform            │
│                   (障碍物感知)           (坐标旋转适配)                  │
│                          ↓                     ↓                      │
│                          └─────────┬───────────┘                      │
│                                    ↓                                  │
│                            Nav2 导航栈                                 │
│                     ┌──────────────┴──────────────┐                   │
│                     │ 全局规划器 → 局部控制器(MPPI) │                   │
│                     │ Hybrid-A*    套娃限速       │                   │
│                     └──────────────┬──────────────┘                   │
│                                    ↓                                  │
│             /cmd_vel_nav → smoother → collision_monitor              │
│                    → lidar_cmd_watchdog → /cmd_vel                    │
│                                    ↓                                  │
│                   fake_vel_transform → serial_def_sdk                 │
│                         (串口 115200bps)                               │
│                                    ↓                                  │
│                       STM32 → 全向轮电机                               │
└──────────────────────────────────────────────────────────────────────┘
```

### 3.2 源码目录结构

```
cod_-rm2026_-navigation-master/src/
├── cod_bringup/               # ★ 导航启动文件、参数、地图、航点
│   ├── launch/                #   真机启动文件
│   ├── params/                #   Nav2/SLAM 参数
│   ├── maps/                  #   预建地图 (.pgm + .yaml)
│   ├── behavior_trees/        #   行为树 XML
│   └── wps/                   #   多点航点 CSV
│
├── small_point_lio/           #   Point-LIO 里程计 (odom→base_link)
├── fake_vel_transform/        #   速度坐标系变换
├── goal_approach_controller/  #   MPPI 目标接近控制器 (wrapper 模式)
├── pb_omni_pid_pursuit_controller/ # PID 纯追踪控制器 (备用方案)
├── pb_nav2_plugins/           #   自定义 Nav2 插件 (后退/体素层)
├── serial_def_sdk/            #   串口通信 (Seasky 协议)
├── slam_dynamic_filter/       #   SLAM 动态障碍过滤
├── cpp_lidar_filter/          #   点云裁剪 (去车身)
├── pointcloud_to_laserscan/   #   3D点云→2D激光 (给 slam_toolbox 用)
├── waypoint_editor/           #   Rviz 2 航点编辑插件
├── cod_gazebo_simulator/      #   Gazebo 仿真包
├── def_msg/                   #   自定义 ROS 消息
├── vision_msg/                #   视觉消息
└── pb_rm_interfaces/          #   裁判系统消息接口
```

---

## 4. 模块详解

### 4.1 `cpp_lidar_filter` — 激光雷达点云裁剪

**作用：** 去掉激光雷达照到**机器人自身**的点云。

激光雷达装在机器人上，发出的激光束会扫到机器人自己的身体。如果不删除这些点，代价地图会认为"机器人面前永远有一堵墙"，导航将完全无法工作。

**输入：** `/livox/lidar` (Raw LiDAR 点云)  
**输出：** `/livox/lidar_filtered` (裁剪后点云)

**参数配置（在 launch 文件中）：**
```python
Node(
    package='cpp_lidar_filter',
    executable='lidar_filter_node',
    parameters=[{
        'input_topic': '/livox/lidar',
        'output_topic': '/livox/lidar_filtered',
        'min_x': -0.2, 'max_x': 0.2,   # 机器人底盘的X范围
        'min_y': -0.2, 'max_y': 0.4,   # 机器人底盘的Y范围
        'min_z': -0.1, 'max_z': 0.2,   # 机器人底盘的高度范围
        'negative': True,   # True = 挖掉这块区域; False = 只保留这块
        'leaf_size': 0.05   # 降采样: 5cm 体素滤波, 减少50%+计算量
    }]
),
```

**技术细节：**
- 使用 PCL (Point Cloud Library) 的 CropBox 和 VoxelGrid 滤镜
- `negative: True` 意味着把车身范围内的点**删除**，保留外部环境点
- 降采样（`leaf_size: 0.05`）很重要——原始雷达每秒数万点，导航不需要这么高精度

---

### 4.2 `small_point_lio` — 激光惯性里程计

**作用：** 整个导航系统的定位基础。告诉你"机器人现在在哪、朝向哪、移动多快"。

**输入：** 激光雷达点云 + IMU 数据  
**输出：** `/Odometry` 话题 + `odom → base_link` TF 变换（~100Hz）

#### 工作原理（简化版）

1. **接收**一帧激光点云和一段时间的 IMU 数据
2. **预测**：根据 IMU（加速度+角速度）先用物理模型推算位移
3. **匹配**：用当前点云去匹配之前的局部地图（ICP 类算法）
4. **优化**：融合预测和匹配结果，得到最优位姿估计
5. **发布** `odom → base_link` 的 TF 变换

```cpp
// small_point_lio_node.cpp 核心逻辑
small_point_lio->set_odometry_callback([this](const Odometry &odometry) {
    // 1. 从 tf_buffer 查 base_link→livox_frame 的安装关系
    auto base_to_lidar = tf_buffer->lookupTransform("base_link", lidar_frame, time_msg);

    // 2. 将 LIO 算出的雷达位姿 转为 base_link 的位姿
    //    odom_to_base = base_to_lidar⁻¹ * lidar_odom_to_lidar * base_to_lidar
    auto tf_odom_to_base = base_to_lidar.inverse() * lidar_odom * base_to_lidar;

    // 3. 发布 TF 变换
    tf_broadcaster->sendTransform(transform_stamped);

    // 4. 发布里程计消息
    odometry_publisher->publish(odometry_msg);
});
```

**注意：** LIO 本体计算的是 `雷达坐标系` 下的位姿。节点查询 `base_link → livox_frame` 的静态 TF，把位姿转换到 `base_link` 坐标系再发布。这就是为什么**必须先发布 livox_frame 到 base_link 的静态 TF**，否则 LIO 无法输出正确里程计。

#### 支持的 LiDAR 适配器

| 适配器 | 适用场景 |
|--------|---------|
| `livox_custom_msg` | Livox 自定义消息格式 (高效) |
| `livox_pointcloud2` | 标准 PointCloud2 消息 |
| `custom_mid360_driver` | MID-360 专用驱动 |
| `unilidar` | 宇树激光雷达 |

---

### 4.3 `fake_vel_transform` — 坐标变换

**作用：** 把安全门控后的速度送往串口，提供 0.5s 命令超时清零，并保留可选的场地坐标速度旋转模式。

#### 问题

当前 Nav2 的 `robot_base_frame` 为真实 `base_link`，所以控制器输出默认已经是机体系速度，不需要再次旋转。

#### 解决方案

默认 `enable_vel_rotation=false`，节点直接透传线速度与上游角速度。它仍发布同原点、仅抵消 yaw 的 `base_link_fake` 辅助帧，供以后显式启用场地坐标速度模式时使用；当前 Nav2 不使用该帧。

```cpp
// fake_vel_transform.cpp 核心逻辑

// 步骤1: 每收到一次里程计数据, 发布 base_link → base_link_fake 的旋转
void FakeVelTransform::odomCallback(const Odometry::SharedPtr msg) {
    current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);

    TransformStamped t;
    t.header.frame_id = "base_link";
    t.child_frame_id = "base_link_fake";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    // 施加反向旋转：让 base_link_fake 的朝向永远为 0
    q.setRPY(0, 0, -current_robot_base_angle_);
    tf_broadcaster_->sendTransform(t);
}

// 步骤2: 默认透传；仅 enable_vel_rotation=true 时执行场地系到机体系旋转
void FakeVelTransform::cmdVelCallback(const Twist::SharedPtr msg) {
    Twist aft_tf_vel;
    aft_tf_vel.angular.z = (spin_speed_ != 0.0) ? spin_speed_ : msg->angular.z;
    if (enable_vel_rotation_) {
        aft_tf_vel.linear.x = msg->linear.x * cos(yaw) + msg->linear.y * sin(yaw);
        aft_tf_vel.linear.y = -msg->linear.x * sin(yaw) + msg->linear.y * cos(yaw);
    } else {
        aft_tf_vel.linear.x = msg->linear.x;
        aft_tf_vel.linear.y = msg->linear.y;
    }

    cmd_vel_chassis_pub_->publish(aft_tf_vel);  // → /aft_cmd_vel
}
```

节点每 50ms 检查一次上游命令；超过 `cmd_vel_timeout=0.5s` 后持续发布零速度。

---

### 4.4 Nav2 导航核心栈

由 `navigation_launch.py` 启动，包含 8 个生命周期节点、一个独立 LiDAR 命令看门狗和生命周期管理器：

| 节点 | 包 | 功能 |
|------|------|------|
| `controller_server` | nav2_controller | 控制指令计算 (MPPI) |
| `smoother_server` | nav2_smoother | 路径平滑 (Savitzky-Golay) |
| `planner_server` | nav2_planner | 全局路径规划 (A*) |
| `behavior_server` | nav2_behaviors | 行为动作 (后退/转圈/等待) |
| `bt_navigator` | nav2_bt_navigator | 行为树引擎 |
| `waypoint_follower` | nav2_waypoint_follower | 多点导航调度 |
| `velocity_smoother` | nav2_velocity_smoother | 速度平滑/限幅 |
| `collision_monitor` | nav2_collision_monitor | 0.55m StopZone 独立停车检查 |
| `lidar_cmd_watchdog` | cod_bringup | 点云超过 0.3s 未更新时持续发布零速度 |
| `lifecycle_manager` | nav2_lifecycle_manager | 管理以上节点的 启动/配置/激活/关闭 |

#### 话题重映射（重要）

```python
# 控制器的输出话题被重映射
remappings=[('cmd_vel', 'cmd_vel_nav')]

# 速度平滑器接收 cmd_vel_nav，默认输出 cmd_vel_smoothed
remappings=[('cmd_vel', 'cmd_vel_nav')]
```

数据流：`controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed → collision_monitor → /cmd_vel_collision_safe → lidar_cmd_watchdog → /cmd_vel → fake_vel_transform → /aft_cmd_vel → 串口`

---

### 4.5 全局规划器 — `planner_server`

**作用：** 给定目标位置，在代价地图上搜索一条避开障碍物的最优路径。

**当前配置：** SmacPlanner2D (Hybrid-A* 算法)

```yaml
# singlenav2_params.yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.5                    # 与目标的容差: 0.5m
      allow_unknown: true               # 允许穿越未知区域
      motion_model_for_search: "DUBIN"  # Dubin 运动学 (考虑转弯半径)
      minimum_turning_radius: 0.05      # 最小转弯半径 (全向底盘很小)
      max_iterations: 1000000           # A* 最大搜索迭代数
      max_planning_time: 4.5            # 最大规划时间(秒)
      cost_travel_multiplier: 4.0       # 代价乘数, 越大越偏向安全通道中心
      reverse_penalty: 1.0              # 倒车惩罚 (值>1 不鼓励倒车)
      analytic_expansion_ratio: 3.5     # 解析扩展比例 (加速最终接近)
      analytic_expansion_max_length: 3.0 # 解析扩展最大长度
      cache_obstacle_heuristic: True    # 缓存启发式 (重新规划加速40x)
```

**算法通俗解释：** A\* 算法就像走迷宫——每次往前走一步，选择"离目标最近 + 走过的路最短"的方向。Hybrid-A\* 额外考虑了"不能原地转向"的运动约束，生成的路径物理上可行。

**Hybrid-A\* vs 普通 A\*：**
- 普通 A\* 假设可以原地转向 （适用于小型差速机器人）
- Hybrid-A\* 考虑最小转弯半径，路径更平滑
- Dubin 模型适合全向底盘 (转弯半径极小 ≈ 0.05m)

---

### 4.6 局部控制器 — `GoalApproachController` + MPPI

本项目最核心的控制设计，采用**套娃（Wrapper）模式**：

```
GoalApproachController (外层，限速 wrapper)
    └── MPPIController (内层，采样择优)
```

#### 4.6.1 MPPI — 模型预测路径积分控制器

**核心思想：** "试 1500 次，选最好的"。不需要推导复杂的控制公式，只需要定义"什么轨迹好"。

**工作流程：**

1. 从当前速度出发，随机生成 1500 条轨迹（不同的速度、角速度组合）
2. 每条轨迹用**6 个评分器**打分
3. 选分数最高的轨迹，执行第一步的速度指令
4. 每隔 0.02 秒重复一次

```yaml
# MPPI 核心参数
FollowPath:
  time_steps: 80          # 每条轨迹预测 80 步 = 1.6秒
  model_dt: 0.02          # 每步时长 20ms
  batch_size: 1500        # 每轮生成 1500 条候选轨迹
  temperature: 0.25       # 决策果断度 (越低越果断, 不再次优间摇摆)
  motion_model: "Omni"    # 全向运动模型
  vx_max: 2.5             # X 方向最大速度 (m/s)
  vy_max: 2.5             # Y 方向最大速度
  wz_max: 1.5             # 最大角速度 (rad/s)
  ax_max: 1.5             # 最大加速度
  ay_max: 1.5
  ax_min: -3.5            # ★关键: 必须对称制动, 高速下刹得住
  ay_min: -3.5

  # ★ 六个评分器 (critics), 决定每条轨迹的分数
  critics: [
      "ConstraintCritic",   # 1. 运动约束 (速度/加速度不能超限)
      "CostCritic",         # 2. 代价地图评分 (远离高代价区域)
      "GoalCritic",         # 3. 目标吸引 (靠近目标加分, ★必须开启)
      "PathFollowCritic",   # 4. 路径跟踪 (远离全局路径惩罚)
      "PathAlignCritic",    # 5. 路径对齐 (轨迹方向与路径一致)
      "ObstaclesCritic"     # 6. 障碍物排斥 (推开近距离障碍物)
  ]
```

**六个评分器详解：**

| 评分器 | 权重 | 作用 | 关键参数 |
|--------|------|------|---------|
| ConstraintCritic | 4.0 | 限制速度/加速度不超限 | 内置, 无需额外配置 |
| CostCritic | 3.8 | 按代价地图梯度引导避障 | `critical_cost: 253` — 只有 inscribed/lethal 才算碰撞 |
| GoalCritic | 25.0 | 提供到达目标的动力 | `threshold_to_consider: 4.0` — 4m 内开始评分 |
| PathFollowCritic | 15.0 | 防止脱离全局路径 | `threshold_to_consider: 1.0` — 1m 内让 GoalCritic 主导 |
| PathAlignCritic | 15.0 | 轨迹方向与路径对齐 | `use_path_orientations: false` — 全向机器人用运动方向 |
| ObstaclesCritic | 1.5 | 障碍物排斥力 | `critical_weight: 20.0` — 近距离强力推开 |

**参数调试经验（来自实际比赛调参）：**
- `temperature: 0.25` 让 MPPI 果断决策，不再在次优轨迹间摇摆
- `cost_scaling_factor: 5.0` 配合 `critical_cost: 253` 修正了 costmap 的梯度，让 MPPI 能区分"近障碍物"和"碰撞"
- `prune_distance: 1.7` 在高速下提前剪除已通过的路径点
- 制动加速度（`ax_min: -3.5`）比加速（`ax_max: 1.5`）大得多——这是有意为之：宁可走得慢，不能刹不住

#### 4.6.2 GoalApproachController — 接近目标时的特殊处理

**问题：** MPPI 在距目标很近时（< 2.5m），评分函数可能没有足够梯度让它直线冲过终点——它可能绕圈、画弧、甚至冲过头。

**解决方案：** 在 MPPI 外面包一层 wrapper，只按目标距离限制 MPPI 已经碰撞检查过的线速度幅值：

```cpp
// goal_approach_controller.cpp
geometry_msgs::msg::TwistStamped computeVelocityCommands(...) {
    auto cmd = inner_controller_->computeVelocityCommands(...);  // 1. 先让 MPPI 算

    double dist = 到目标的距离;  // hypot(dx, dy)

    if (dist < approach_distance_) {         // < 2.5m: 限速模式
        double allowed = min(approach_velocity_, dist * approach_kp_);
        double speed = hypot(cmd.twist.linear.x, cmd.twist.linear.y);
        if (speed > allowed) {
            // 等比缩放线速度，不覆盖 MPPI 选择的安全方向
            double scale = allowed / speed;
            cmd.twist.linear.x *= scale;
            cmd.twist.linear.y *= scale;
        }
    }

    return cmd;
}
```

两层策略：

| 距目标距离 | 策略 | 说明 |
|-----------|------|------|
| > 2.5m | 纯 MPPI | 正常运行 |
| < 2.5m | MPPI 方向保持 + 距离相关限速 | 最大 0.2m/s，接近终点继续下降 |

**设计理念：** Wrapper 对内部控制器透明——你可以把 MPPI 换成 PID 控制器，外层限速逻辑不变。这就是 "开闭原则"（对扩展开放，对修改关闭）。

---

### 4.7 `pb_omni_pid_pursuit_controller` — PID 纯追踪控制器（备用方案）

这是一个完整的**独立控制器**，基于纯追踪（Pure Pursuit）算法，适合全向底盘。

#### 纯追踪算法

在路径前方放一个"胡萝卜"（前视点 / lookahead point），机器人追着胡萝卜走，胡萝卜随机器人前进而前移：

```
            路径 ●──●──●──●──●──🟠 目标
                  ↑              ↑
                  │         前视点 (胡萝卜)
                  │    ← lookahead_dist →
                  🤖
```

#### 核心流程

```cpp
geometry_msgs::msg::TwistStamped computeVelocityCommands(...) {
    // 1. 将全局路径转换到机器人坐标系 (base_link)
    auto transformed_plan = transformGlobalPlan(pose);

    // 2. 根据当前速度动态决定 "看多远"
    //    跑得快时看远点 (max 1.0m), 跑得慢看近点 (min 0.2m)
    double lookahead_dist = getLookAheadDistance(velocity);

    // 3. 找到路径上前视距离处的目标点
    auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);

    // 4. PID 分别控制平移和旋转
    double lin_dist = hypot(carrot_pose.x, carrot_pose.y);       // 到前视点的距离
    double theta_dist = atan2(carrot_pose.y, carrot_pose.x);     // 到前视点的方向角
    auto lin_vel = move_pid_->calculate(lin_dist, 0);            // 平移PID: 距离→速度
    auto angular_vel = heading_pid_->calculate(angle_to_goal, 0); // 旋转PID: 角度→角速度

    // 5. 弯道减速: 分析前视点附近路径的曲率, 弯越急速度越低
    applyCurvatureLimitation(transformed_plan, carrot_pose, lin_vel);

    // 6. 接近目标减速: 剩余距离越短, 速度越低
    applyApproachVelocityScaling(transformed_plan, lin_vel);

    // 7. 碰撞检测: 沿路径采样 10 个点检查是否碰撞
    if (!isCollisionDetected(costmap_frame_local_plan)) {
        cmd_vel.twist.linear.x = lin_vel * cos(theta_dist);   // 全向: 朝前视点方向
        cmd_vel.twist.linear.y = lin_vel * sin(theta_dist);   // 不需要先转向
        cmd_vel.twist.angular.z = angular_vel;
    } else {
        throw PlannerException("Collision detected!");  // 抛异常→行为树进入恢复
    }
}
```

#### 弯道减速实现

```cpp
void applyCurvatureLimitation(path, lookahead_pose, linear_vel) {
    double curvature = calculateCurvature(backward_pose, carrot_pose, forward_pose);
    // 取前向点和后向点, 三点确定一个圆, 曲率 = 1/半径

    if (curvature > curvature_min_) {  // 曲率 > 阈值→开始减速
        double reduction_ratio = 1.0;
        if (curvature > curvature_max_) {
            reduction_ratio = 0.5;  // 急弯→降速50%
        } else {
            // 中等弯→线性插值
            reduction_ratio = 1.0 - (curvature - min) / (max - min) * 0.5;
        }
        linear_vel *= reduction_ratio;
    }
}
```

#### MPPI vs PID Pure Pursuit 对比

| 特性 | MPPI | PID Pure Pursuit |
|------|------|-----------------|
| 原理 | 采样 + 评分 (1500 条轨迹) | 解析公式 (前视点 + PID) |
| CPU 开销 | 高 | 低 |
| 弯道处理 | 自动 (评分函数隐含) | 显式曲率计算 + 限速 |
| 调参难度 | 多个 critics 权重 | PID 三个参数 |
| 复杂环境表现 | 优秀 (考虑多因素) | 一般 |
| 当前状态 | **主用** | 备用 |

---

### 4.8 代价地图 — "哪里能走、哪里不能走"

代价地图是导航的**环境感知层**，分为两层：

#### 全局代价地图 (Global Costmap)

- 坐标系：`map`
- 大小：25m × 25m rolling window
- 分辨率：0.04m/格
- 作用：给全局规划器提供"大图"

#### 局部代价地图 (Local Costmap)

- 坐标系：`odom`
- 大小：10m × 10m rolling window
- 分辨率：0.05m/格
- 作用：给局部控制器提供实时障碍物信息

#### 三层叠加结构

```yaml
plugins: ["static_layer", "stvl_voxel_layer", "inflation_layer"]
```

1. **static_layer** — 预建地图中的墙壁、障碍物（静态）
2. **3D 障碍层** — 单点模式使用 STVL，多点模式使用 Nav2 VoxelLayer；两者都只消费 MID-360 过滤点云
3. **inflation_layer**（膨胀层）— 在障碍物周围"吹气球"，留出安全缓冲

#### 膨胀层的关键参数

```yaml
inflation_layer:
  cost_scaling_factor: 5.0   # ★ 代价随距离的衰减速率
  inflation_radius: 0.55     # 膨胀半径 0.55m
```

**`cost_scaling_factor: 5.0` 的含义：**

```
代价
255 |██
    |  ██
    |    ████          ← cost_scaling_factor=0.3 (默认): 代价衰减很慢
    |        ██████       整个膨胀区代价都很高, MPPI 看不出梯度
    |              ██████████
  0 |___________________________→ 距离
    障碍物   0.25m   0.5m

代价
255 |██
    |█
    |█                        ← cost_scaling_factor=5.0 (本项目): 代价衰减快
    | █                          形成清晰的"远离障碍物"梯度
    |  █
  0 |___█_______________________→ 距离
    障碍物   0.25m   0.5m
```

这是比赛中的关键调参成果：0.3 的默认值时，整个膨胀区代价都接近 255，MPPI 的 CostCritic 看不出"往哪个方向走更安全"。

#### LiDAR 作为障碍物源

```yaml
livox_source:
  data_type: PointCloud2
  topic: /livox/lidar_filtered       # 来自 cpp_lidar_filter 的已裁剪点云
  obstacle_range: 8.0                 # 8m 内点云标记为障碍物
  raytrace_range: 9.0                # 9m 内清除之前的标记 (raytracing)
  min_obstacle_height: 0.05          # 覆盖最低离地10cm障碍，保留5cm裕量
  max_obstacle_height: 1.0           # 高于 1m 的点忽略 (横梁等)
  expected_update_rate: 0.3          # 10Hz点云的新鲜度期望（秒）
  filter: "voxel"                    # 体素滤波降采样
  model_type: 1                      # 1 = 3D LiDAR (圆顶投影)
  vertical_fov_angle: 1.03           # MID-360 垂直视场约 -7°至+52°
  vertical_fov_offset: 0.3927
  horizontal_fov_angle: 6.28         # 水平视场角 360°
```

当前真机没有 RealSense，配置中也没有相机观测源。`expected_update_rate` 只用于代价地图观测状态；真正的雷达掉线停车由独立 `lidar_cmd_watchdog` 执行。

---

### 4.9 行为树 — 导航的应急预案

```xml
<!-- navigate_to_pose_w_replanning_and_recovery.xml -->
<root BTCPP_format="3" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">

    <!-- 外层: 恢复节点, 最多重试10次 -->
    <RecoveryNode name="NavigateRecovery" number_of_retries="10">

      <!-- 正常流程: 串行执行 -->
      <PipelineSequence name="NavigateWithReplanning">

        <!-- 步骤1: 规划路径, 如果失败→清除全局代价地图→重试 -->
        <RateController hz="3.0">  <!-- 每3秒重新规划一次 -->
          <RecoveryNode name="ComputePathToPose" number_of_retries="1">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context"
                service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>

        <!-- 步骤2: 跟踪路径, 如果卡住→清除局部代价地图→重试(最多10次) -->
        <RecoveryNode name="FollowPath" number_of_retries="10">
          <ReactiveFallback name="FollowPathWithStuckDetection">
            <FollowPath path="{path}" controller_id="FollowPath"/>
          </ReactiveFallback>
          <ClearEntireCostmap name="ClearLocalCostmap-Context"
              service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>

      </PipelineSequence>

      <!-- 恢复行为: 以上都失败后执行 -->
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>  <!-- 如果目标更新了, 直接重新规划 -->
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree"
                service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree"
                service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
        </RoundRobin>
      </ReactiveFallback>

    </RecoveryNode>
  </BehaviorTree>
</root>
```

**行为树工作流程：**

```
开始
  │
  ▼
规划路径 ──失败──→ 清除全局地图 ──→ 重试规划
  │成功
  ▼
跟踪路径 ──卡住──→ 清除局部地图 ──→ 重试追踪 (最多10次)
  │                     │
  │                     └─全部失败──→ 清除两个地图 ──→ 回到规划
  │成功
  ▼
到达目标 ✓
```

**关键设计决策：**
- 规划重试只给 1 次——如果规划失败通常是地图问题，清理一次就够了
- 追踪重试给 10 次——传感器噪声可能产生假障碍物，清理代价地图通常能解决
- `RateController hz="3.0"` — 不每次都重新规划，节省 CPU

---

### 4.10 `pb_nav2_plugins` — 自定义导航插件

#### BackUpFreeSpace — 智能后退

Nav2 原生的后退行为是"朝机器人后方盲退"——如果后面 0.2m 就是墙，它也照退不误。

本项目自定义的 `BackUpFreeSpace` 会分析全局代价地图，找到最空闲的方向再后退：

```xml
<!-- 注册为 Nav2 行为插件 -->
<library path="pb_back_up_frees_space_behavior">
  <class name="pb_nav2_behaviors/BackUpFreeSpace"
         type="pb_nav2_behaviors::BackUpFreeSpace"
         base_class_type="nav2_core::Behavior">
    <description>An enhanced back_up action that move toward free space.</description>
  </class>
</library>
```

```yaml
# 参数配置
backup:
  plugin: "pb_nav2_behaviors/BackUpFreeSpace"  # 替代原生 nav2_behaviors/BackUp
robot_radius: 0.1
max_radius: 3.5         # 搜索空闲区域的半径
free_threshold: 5        # 代价 < 5 视为空闲
visualize: True          # RViz 可视化
```

#### IntensityVoxelLayer — 强度体素层

利用 Livox 雷达的**回波强度信息**过滤噪点，减少玻璃/镜面反射造成的假障碍物。

---

### 4.11 `serial_def_sdk` — 串口通信

**作用：** Nav2 计算的数字速度指令，通过**串口**以二进制协议发给底盘 STM32 单片机，由单片机控制电机。

#### 数据流

```
/aft_cmd_vel (ROS 消息)  →  serial_def_sdk  →  /dev/cod_mcu  →  STM32  →  电机
  float vx, vy, vz            打包成二进制           115200bps
```

该模块使用 Seasky 协议，提供 CRC 校验、话题超时清零、串口自动重连和双向状态通信。完整帧格式见 `docs/MCU_SEASKY_PROTOCOL.md`。

---

### 4.12 `waypoint_editor` — 可视化航点编辑器

一个 **Rviz 2 插件**，允许在 RViz 界面中直接点击地图设置导航目标：

- **waypoint_editor_tool** — RViz 工具栏工具，点击地图添加/编辑航点
- **waypoint_to_nav2** — 将航点列表转换为 Nav2 可执行的多点导航任务
- **waypoint_through_nav2** — 将航点发送给 `navigate_through_poses` 动作

在比赛中用于快速设置巡逻路线，导出 CSV 文件供多点导航使用。

---

### 4.13 `pointcloud_to_laserscan` — 3D 点云转 2D 激光

**仅在 SLAM 模式下使用。**

`slam_toolbox` 需要 2D 激光扫描数据 `/scan`，但 Livox MID-360 输出的是 3D 点云。此节点将一定高度范围内的 3D 点云**投影到 2D 平面**，模拟传统 2D 激光雷达的输出。

```
/livox/lidar_filtered (3D 点云) → pointcloud_to_laserscan → /scan (2D)
```

投影在 `base_link` 中使用 `0.05m..1.00m` 高度范围，确保最低离地 `0.10m` 的障碍不会被下限过滤。

---

## 5. TF 坐标变换树

TF（Transform）是 ROS2 的核心基础设施，用**树状结构**表示各坐标系之间的关系：

### 5.1 真机 TF 树

```
map ──→ odom ──→ base_link ──→ livox_frame (z=0.46m, pitch=+45°)
  ↑        ↑         │
  │        │         └──→ base_link_fake (可选辅助帧，Nav2不使用)
  │        │
  │        └── small_point_lio 发布 (~100Hz)
  │             激光里程计实时计算
  │
  └── 两种真机模式都使用静态 TF
```

### 5.2 各变换的发布者

| 变换 | 发布者 | 频率 | 含义 |
|------|--------|------|------|
| `map → odom` | static_transform_publisher | 静态 | 地图到里程计的固定偏移 |
| `odom → base_link` | small_point_lio | ~100Hz | 机器人的实时位姿 |
| `base_link → base_link_fake` | fake_vel_transform | 随 /Odometry | 去旋转后的虚拟坐标系 |
| `base_link → livox_frame` | singlenav/multiplenav launch | 静态 | 雷达实测安装位 |

### 5.3 当前定位与基础帧约束

Nav2 当前统一使用真实 `base_link`。单点模式只启动地图服务、不启动 AMCL；多点模式设置 `slam_toolbox.transform_publish_period: 0.0`，也不发布 `map→odom` 校正。因此两种真机模式都依赖固定 `map→odom`，不会持续修正 LIO 漂移。这是短时间、短距离应用中明确接受的限制，任务开始前必须确认固定地图对齐正确。

---

## 6. 两种运行模式

### 6.1 模式对比

| 特性 | 单点导航 (singlenav) | 多点 SLAM 导航 (multiplenav) |
|------|---------------------|---------------------------|
| 启动命令 | `ros2 launch cod_bringup singlenav_launch.py` | `ros2 launch cod_bringup multiplenav_launch.py` |
| 地图来源 | 预建 `.pgm` 地图 | `slam_toolbox` 实时建图 |
| 地图对齐 | 固定 map→odom，无 AMCL | 固定 map→odom；SLAM 不发布校正 TF |
| 导航目标 | 单个目标点 (Rviz 2D Goal) | CSV 中的多个航点 |
| 需要 `/scan` | 否 | 是 (pointcloud_to_laserscan) |
| 参数文件 | `singlenav2_params.yaml` | `multiplenav2_params.yaml` |
| 建图节点 | 不启动 | 启动 slam_toolbox + auto_save_map |

### 6.2 SLAM 模式关键参数

```yaml
# mapper_params_online_async.yaml
slam_toolbox:
  ros__parameters:
    mode: lifelong                    # 终身 SLAM 模式 (持续优化)
    resolution: 0.05                 # 地图分辨率 5cm
    max_laser_range: 10.0            # 激光最大有效距离
    minimum_travel_distance: 1.0     # 移动超过 1m 才更新地图
    do_loop_closing: true            # 启用回环检测
    map_update_interval: 1.0         # 每秒更新一次地图
    transform_publish_period: 0.0    # 不发布 map→odom 校正
```

---

## 7. 完整数据流总结

```
┌──────────┐   ┌──────────────┐   ┌──────────────┐
│ Livox雷达 │──→│cpp_lidar_filter│─→│small_point_lio│──→ /Odometry
│ MID-360  │   │  去掉车身点云  │   │  激光惯性里程计 │──→ odom→base_link TF
└──────────┘   └──────┬───────┘   └──────────────┘
                      │
                      ↓ /livox/lidar_filtered (3D 点云)
               ┌──────────────┐
               │  代价地图层   │ ← 仅接收 MID-360 过滤点云
               │  (局部+全局) │
               └──────┬───────┘
                      ↓ costmap (2D 障碍物代价图)
┌──────────┐   ┌──────────────┐   ┌──────────────────┐
│ 全局规划器 │──→│  路径平滑器   │──→│ GoalApproachCtrl │
│ Hybrid-A* │   │ SavitzkyGolay│   │  └── MPPI 控制器 │
└──────────┘   └──────────────┘   └────────┬─────────┘
                                           ↓ /cmd_vel_nav
                                    ┌──────────────┐
                                    │velocity_smth │  速度平滑+限幅
                                    └──────┬───────┘
                                           ↓ /cmd_vel_smoothed
                                    ┌──────────────┐
                                    │collision_mon │  0.55m StopZone
                                    └──────┬───────┘
                                           ↓ /cmd_vel_collision_safe
                                    ┌──────────────┐
                                    │lidar_watchdog│  点云超时0.3s强制零速
                                    └──────┬───────┘
                                           ↓ /cmd_vel
                                    ┌──────────────┐
                                    │fake_vel_xform │  旋转到真实机体系
                                    └──────┬───────┘
                                           ↓ /aft_cmd_vel
                                    ┌──────────────┐
                                    │serial_def_sdk │  串口 115200bps
                                    └──────┬───────┘
                                           ↓ 二进制数据包
                                    ┌──────────────┐
                                    │  STM32 底盘   │ → 全向轮电机
                                    └──────────────┘
```

---

## 8. 为什么不直接用 Nav2 原生配置

| Nav2 默认 | 本项目定制 | 原因 |
|-----------|-----------|------|
| 原生 MPPI | GoalApproachController 套娃 | 接近目标时 MPPI 会画弧或冲过头 |
| 直接发控制指令 | Collision Monitor + LiDAR watchdog | 障碍入侵或雷达掉线时在串口前强制零速 |
| 原生 BackUp 行为 | BackUpFreeSpace | 普通后退是盲退，需找空闲方向 |
| 默认 critics 权重 | 大量调试后的值 | 比赛场景需要更快的速度和更果断的决策 |
| `cost_scaling_factor: 0.3` | `5.0` | 默认值使整个膨胀区代价饱和，MPPI 无梯度 |
| 差速运动模型 | `Omni` | 全向底盘可独立控制 X/Y 方向 |
| `temperature: 0.7` | `0.25` | 更小的温度让 MPPI 决策更果断 |

---

## 9. 参数调优速查

| 想要的效果 | 文件 | 参数 | 调整方向 |
|-----------|------|------|---------|
| 机器人跑太快/太慢 | `singlenav2_params.yaml` | `vx_max` / `vy_max` | 减小/增大 |
| 到目标停不到位 | `singlenav2_params.yaml` | `xy_goal_tolerance` | 调小 = 更精确 |
| 接近目标时减速太早/太晚 | `singlenav2_params.yaml` | `approach_distance` | 调小/调大 |
| 避障太激进/太保守 | `singlenav2_params.yaml` | `inflation_radius` | 调大 = 离障碍物更远 |
| MPPI 太犹豫/太激进 | `singlenav2_params.yaml` | `temperature` | 调低 = 更果断 |
| CPU 占用太高 | `singlenav2_params.yaml` | `batch_size` | 调小 = 减少采样数 |
| 弯道速度太快 | `singlenav2_params.yaml` | `curvature_min` / `reduction_ratio` | 调整曲率减速参数 |
| 发送速度太陡 | `singlenav2_params.yaml` | `max_accel` / `max_decel` | 调小 = 更平滑 |
| 雷达数据量太大 | launch 文件 | `leaf_size` | 调大 = 更多降采样 |
| 后退找不到路 | `singlenav2_params.yaml` | `backup.max_radius` | 调大 = 搜索更大范围 |

---

## 附录：学习资源

- [Nav2 官方文档](https://docs.nav2.org/)
- [MPPI 控制器论文](https://arxiv.org/abs/2103.04532)
- [Behavior Tree CPP v3 文档](https://www.behaviortree.dev/)
- [Smac Planner 论文](https://arxiv.org/abs/2105.02178)
- [Point-LIO 论文](https://arxiv.org/abs/2302.05938)
- [RM COD 导航分享](https://www.bilibili.com/video/BV1XSXZBUEYL/) — 项目作者的 B 站教程

---

> **文档维护：** 本文档与 [OPERATING_GUIDE.md](../cod_-rm2026_-navigation-master/docs/OPERATING_GUIDE.md) 互补。操作变更更新 OPERATING_GUIDE，架构变更更新本文档。
