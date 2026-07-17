# 导航代码全部修复记录

> 日期：2026-07-17
> 基于 ask_gateway 代码审查报告，修复 8 个严重问题 + 多个中等问题 + 坐标系对齐

---

## 修改文件总览

| # | 文件 | 修改类型 |
|---|------|---------|
| 1 | `pid.hpp` | 重构 |
| 2 | `pid.cpp` | 重写 |
| 3 | `omni_pid_pursuit_controller.cpp` | 安全修复 |
| 4 | `goal_approach_controller.cpp` | 重写 |
| 5 | `fake_vel_transform.hpp` | 新增成员 |
| 6 | `fake_vel_transform.cpp` | 重写 |
| 7 | `cod_serial.cpp` | 重写 |
| 8 | `intensity_voxel_layer.cpp` | 逻辑修复 |
| 9 | `filter_node.cpp` | 重写 |
| 10 | `back_up_free_space.cpp` | 小修复 |
| 11 | `singlenav2_params.yaml` | 参数调整 |

---

## 1. pid.hpp — PID 控制器头文件

**路径**: `src/pb_omni_pid_pursuit_controller/include/pb_omni_pid_pursuit_controller/pid.hpp`

**原问题**:
- 积分限幅硬编码 ±1，没有可配置的积分上下限
- 缺少 `reset()` 方法（切换路径/控制器停用无法清空积分）
- 缺少 `setGains()` 方法（动态调参无法同步到 PID 对象）
- 缺少 `getIntegral()` 调试接口

**修改内容**:

| 变更 | 说明 |
|------|------|
| 构造参数新增 `integral_min`, `integral_max` | 默认值 -1.0, 1.0，保持向后兼容 |
| 新增 `void reset()` | 清空 integral_、pre_error_、initialized_ |
| 新增 `void setGains(kp, ki, kd)` | 线程安全更新 PID 增益 |
| 新增 `double getIntegral() const` | 内联函数，返回当前积分值 |
| 新增私有成员 `integral_min_`, `integral_max_`, `initialized_` | 支持新功能 |

**构造签名变更**:
```cpp
// 旧
PID(double dt, double max, double min, double kp, double kd, double ki);

// 新（后两个参数有默认值，不破坏现有调用）
PID(double dt, double max, double min, double kp, double kd, double ki,
    double integral_min = -1.0, double integral_max = 1.0);
```

---

## 2. pid.cpp — PID 控制器实现

**路径**: `src/pb_omni_pid_pursuit_controller/src/pid.cpp`

**原问题**:
1. 积分先计算 `i_out = ki_ * integral_`，再限幅 `integral_`。本周期积分输出可能出现越界尖峰
2. `pre_error_` 初始化为 0，首次调用产生 derivative kick
3. 积分限幅硬编码 ±1，不可配置
4. 没有 NaN/Inf 输入输出保护
5. 没有 `dt_` 合法性验证

**修改内容**:

### 构造函数
```cpp
// 新增验证
if (!std::isfinite(dt_) || dt_ <= 0.0) {
  throw std::invalid_argument("PID: dt must be positive and finite");
}
if (!std::isfinite(max_) || !std::isfinite(min_) || min_ > max_) {
  throw std::invalid_argument("PID: max/min must be finite and max >= min");
}
if (!std::isfinite(kp_) || !std::isfinite(ki_) || !std::isfinite(kd_)) {
  throw std::invalid_argument("PID: gains must be finite");
}
```

### calculate() 核心逻辑
```cpp
// 旧版（有 bug）:
integral_ += error * dt_;
double i_out = ki_ * integral_;  // 先算输出
if (integral_ > 1) integral_ = 1;  // 后限幅 ← 本周期 i_out 已越界

// 新版（修复后）:
integral_ += error * dt_;
integral_ = std::clamp(integral_, integral_min_, integral_max_);  // 先限幅
double i_out = ki_ * integral_;  // 再算输出 ← 安全

// 旧版（derivative kick）:
double derivative = (error - pre_error_) / dt_;  // pre_error_=0，error大 → D项尖峰

// 新版（防 kick）:
if (!initialized_) {
  pre_error_ = error;   // 首帧把 pre_error_ 设为当前误差
  initialized_ = true;  // 首帧 d_out = 0
}
// ... 后续正常计算 derivative
```

### 新增方法
```cpp
void PID::reset() {
  integral_ = 0.0;
  pre_error_ = 0.0;
  initialized_ = false;
}

void PID::setGains(double kp, double ki, double kd) {
  kp_ = kp; ki_ = ki; kd_ = kd;
}
```

### NaN/Inf 保护
```cpp
// 输入保护
if (!std::isfinite(set_point) || !std::isfinite(pv)) { return 0.0; }

// 输出保护
if (!std::isfinite(output)) { output = 0.0; }
```

---

## 3. omni_pid_pursuit_controller.cpp — 全向 PID 纯追踪控制器

**路径**: `src/pb_omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp`

**原问题**:
1. **isCollisionDetected()** costmap 越界返回 `false`（fail-open），路径点超出 costmap 时放行
2. **isCollisionDetected()** 只采样固定 10 个点，稀疏
3. **isCollisionDetected()** 不检查 `NO_INFORMATION` 未知区域
4. 碰撞检测失败抛出 `PlannerException`（语义错误）
5. 动态参数更新 kp/ki/kd 后未同步到 PID 对象
6. 没有 NaN/Inf 速度输出保护

**修改内容**:

### isCollisionDetected() — 完全重写
```cpp
// 旧版（fail-open）:
if (costmap->worldToMap(...)) {
  if (cost >= INSCRIBED_INFLATED_OBSTACLE) return true;
} else {
  return false;  // ← 越界 = 放行！安全缺陷
}

// 新版（fail-safe）:
if (!costmap->worldToMap(...)) {
  // 越界 → 视为碰撞风险
  RCLCPP_WARN_THROTTLE(..., "outside costmap bounds. Treating as collision risk.");
  return true;  // ← 保守：越界 = 不安全
}
unsigned char cost = costmap->getCost(mx, my);
if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) return true;
// 新增：NO_INFORMATION 也视为不安全
if (cost == nav2_costmap_2d::NO_INFORMATION) {
  RCLCPP_WARN_THROTTLE(..., "unknown costmap area. Collision risk.");
  return true;
}
```

### 碰撞检测采样密度
```cpp
// 旧版：固定 10 点
int sample_points = 10;

// 新版：基于 costmap 分辨率密集采样，最少 20 点
const double costmap_res = costmap->getResolution();
int sample_points = std::max(20, static_cast<int>(plan_size / std::max(costmap_res, 0.01)));
sample_points = std::min(sample_points, plan_size);
```

### 异常类型
```cpp
// 旧：PlannerException（语义错误，这是控制器不是规划器）
throw nav2_core::PlannerException("...");

// 新：ControllerException
throw nav2_core::ControllerException("...");

// 全部 4 处 PlannerException → ControllerException:
// 1. "Collision detected in the trajectory"
// 2. "Received plan with zero length"
// 3. "Unable to transform robot pose into global plan's frame"
// 4. "Resulting plan has 0 poses in it"
```

### NaN/Inf 速度保护
```cpp
// computeVelocityCommands() 中 return 前新增：
if (!std::isfinite(cmd_vel.twist.linear.x) ||
    !std::isfinite(cmd_vel.twist.linear.y) ||
    !std::isfinite(cmd_vel.twist.angular.z)) {
  RCLCPP_ERROR(logger_, "NaN/Inf detected in velocity command. Publishing zero velocity.");
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.linear.y = 0.0;
  cmd_vel.twist.angular.z = 0.0;
}
```

### 动态参数同步 PID
```cpp
// dynamicParametersCallback() 末尾新增：
if (move_pid_) {
  move_pid_->setGains(translation_kp_, translation_ki_, translation_kd_);
}
if (heading_pid_) {
  heading_pid_->setGains(rotation_kp_, rotation_ki_, rotation_kd_);
}
```

### 新增 include
```cpp
#include <cmath>  // std::isfinite
```

---

## 4. goal_approach_controller.cpp — 目标接近控制器

**路径**: `src/goal_approach_controller/src/goal_approach_controller.cpp`

**原问题**:
1. `goal_` 的坐标系与 `pose` 可能不一致，直接用欧氏距离比较
2. 直接驱动模式（dist < direct_approach_distance_）强制 `angular.z = 0`
3. 速度钳位逻辑可能把低速反向放大
4. 直接模式绕过 MPPI 避障
5. TF 失败时 fallback 到未变换坐标（残留风险）

**修改内容**:

### goal_ TF 变换
```cpp
// 旧版：直接用 goal_.pose 和 pose.pose 做欧氏距离
double dx = goal_.pose.position.x - pose.pose.position.x;
double dy = goal_.pose.position.y - pose.pose.position.y;

// 新版：通过 TF 将 goal 变换到 base_frame
if (tf_ && !goal_.header.frame_id.empty()) {
  try {
    goal_in_base = tf_->transform(
      goal_, costmap_ros_->getBaseFrameID(), tf2::durationFromSec(0.1));
    dx = goal_in_base.pose.position.x;
    dy = goal_in_base.pose.position.y;
    dist = std::hypot(dx, dy);
  } catch (const tf2::TransformException & ex) {
    // TF 失败：跳过接近控制，直接透传内部控制器输出
    RCLCPP_WARN_THROTTLE(..., "TF failed. Skipping approach control.");
    return cmd;
  }
} else {
  // 无 TF 或 frame_id 为空：跳过接近控制
  RCLCPP_WARN_THROTTLE(..., "TF unavailable. Skipping approach control.");
  return cmd;
}
```

### 直接驱动模式
```cpp
// 旧版：强制 angular.z = 0
cmd.twist.angular.z = 0.0;

// 新版：保留内部控制器输出的 angular.z
// (不强制设 0，goal checker 可能需要最终 yaw 对齐)
```

### 速度钳位
```cpp
// 旧版：可能把低速反向放大到 approach_velocity_
// （如果实现为"将所有非零速度归一化到固定模长"）

// 新版：只在 speed > approach_velocity_ 时缩小（上限限制）
if (speed > approach_velocity_) {
  double scale = approach_velocity_ / speed;
  cmd.twist.linear.x *= scale;
  cmd.twist.linear.y *= scale;
  cmd.twist.angular.z *= scale;
}
```

### NaN/Inf 保护
```cpp
if (!std::isfinite(cmd.twist.linear.x) || !std::isfinite(cmd.twist.linear.y) ||
    !std::isfinite(cmd.twist.angular.z)) {
  RCLCPP_ERROR(logger_, "NaN/Inf detected, zeroing velocity.");
  cmd.twist.linear.x = 0.0; cmd.twist.linear.y = 0.0; cmd.twist.angular.z = 0.0;
}
```

### 新增成员
```cpp
std::shared_ptr<tf2_ros::Buffer> tf_;                    // TF 缓冲区
std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;  // costmap
rclcpp::Clock::SharedPtr clock_;                         // 时钟（日志节流）
```

---

## 5. fake_vel_transform.hpp — 假速度变换头文件

**路径**: `src/fake_vel_transform/include/fake_vel_transform/fake_vel_transform.hpp`

**修改内容**:

| 新增成员 | 类型 | 说明 |
|---------|------|------|
| `watchdogCallback()` | 方法 | 超时输出零速度 |
| `watchdog_timer_` | `TimerBase::SharedPtr` | 看门狗定时器 |
| `cmd_vel_timeout_` | `double` | 超时阈值，默认 0.5s |
| `last_cmd_time_` | `rclcpp::Time` | 最后收到 cmd_vel 时间 |
| `enable_vel_rotation_` | `bool` | 速度旋转开关，默认 false |

---

## 6. fake_vel_transform.cpp — 假速度变换实现

**路径**: `src/fake_vel_transform/src/fake_vel_transform.cpp`

**原问题**:
1. 无条件覆盖 `angular.z = spin_speed_`
2. odom TF 只有旋转没有平移
3. 没有 cmd_vel 超时保护 (watchdog)
4. 速度旋转一直开启，无法切换直通模式（首测不需要旋转）

**修改内容**:

### angular.z 处理
```cpp
// 旧版：无条件覆盖
cmd.angular.z = spin_speed_;

// 新版：默认透明转发，仅 spin_speed_ 非零时覆盖
aft_tf_vel.angular.z = (spin_speed_ != 0.0f) ? spin_speed_ : msg->angular.z;
```

### odom TF — 补全平移
```cpp
// 旧版：只有旋转，没有平移
// (平移固定为 0)

// 新版：完整复制 odometry 位姿
t.transform.translation.x = msg->pose.pose.position.x;
t.transform.translation.y = msg->pose.pose.position.y;
t.transform.translation.z = msg->pose.pose.position.z;
```

### 新增参数
```cpp
this->declare_parameter<bool>("enable_vel_rotation", false);  // 默认关闭（首测直通模式）
this->declare_parameter<double>("cmd_vel_timeout", 0.5);      // watchdog 超时
```

### 速度旋转开关
```cpp
if (enable_vel_rotation_) {
  // Field-centric 模式：世界坐标 → 底盘坐标
  aft_tf_vel.linear.x = msg->linear.x * cos(angle_diff) + msg->linear.y * sin(angle_diff);
  aft_tf_vel.linear.y = -msg->linear.x * sin(angle_diff) + msg->linear.y * cos(angle_diff);
} else {
  // 直通模式：Nav2 输出直接给电控
  aft_tf_vel.linear.x = msg->linear.x;
  aft_tf_vel.linear.y = msg->linear.y;
}
```

### watchdog 回调
```cpp
void FakeVelTransform::watchdogCallback() {
  auto now = this->now();
  auto elapsed = now - last_cmd_time_;
  if (elapsed.seconds() > cmd_vel_timeout_) {
    RCLCPP_WARN_THROTTLE(..., "cmd_vel timeout. Publishing zero velocity.");
    geometry_msgs::msg::Twist zero_vel;
    cmd_vel_chassis_pub_->publish(zero_vel);
  }
}
```

---

## 7. cod_serial.cpp — 串口通信

**路径**: `src/ros2_simple_serial/src/cod_serial.cpp`

**原问题**:
1. 每条消息创建→打开→写入→关闭串口，极度低效
2. `#include "uart_transporter.cpp"`（C++ 反模式）
3. 每条消息打印 15 行 INFO 日志

**修改内容**:

### 串口改为持久连接
```cpp
// 旧版（topic_callback 内）:
UartTransporter uart("/dev/ttyACM0", 115200);
uart.open();
uart.writeBuffer(packet, sizeof(packet));
uart.close();

// 新版（构造函数）:
uart_ = std::make_unique<UartTransporter>("/dev/ttyACM0", 115200);
uart_->open();

// 新版（topic_callback 内）:
uart_->writeBuffer(packet, sizeof(packet));  // 直接写入已打开的串口

// 新版（析构函数）:
uart_->close();
```

### 修正 include
```cpp
// 旧版：
#include "uart_transporter.cpp"  // ← 错误

// 新版：
#include "uart_transporter.hpp"  // ← 正确
```

### 日志降级
```cpp
// 旧版：每条消息 15 行 RCLCPP_INFO（高频刷屏）
// 新版：数据包用 RCLCPP_DEBUG，错误用 RCLCPP_WARN_THROTTLE
RCLCPP_DEBUG(..., "Sent cmd_vel packet: vx=%.3f vy=%.3f vz=%.3f", vx, vy, vz);
RCLCPP_WARN_THROTTLE(..., "Serial write incomplete: %d/%zu bytes", written, sizeof(packet));
```

### 断线重连
```cpp
void tryReconnect() {
  if (reconnect_in_progress_) return;
  reconnect_in_progress_ = true;
  if (uart_->open()) {
    reconnect_count_ = 0;
  } else {
    reconnect_count_++;
    auto delay = std::min(5.0, 0.5 * reconnect_count_);  // 指数退避，上限 5s
  }
  reconnect_in_progress_ = false;
}
```

### 新增成员
```cpp
std::unique_ptr<UartTransporter> uart_;
mutable int reconnect_count_{0};
mutable bool reconnect_in_progress_{false};
```

---

## 8. intensity_voxel_layer.cpp — 强度体素层

**路径**: `src/pb_nav2_plugins/src/layers/intensity_voxel_layer.cpp`

**原问题**: `pz < origin_z_` 时把点钳位到 `origin_z_`，将所有低于体素层底部的点投影到最低层，产生虚假障碍

**修改内容**:

```cpp
// 旧版（制造虚假障碍）:
if (pz < origin_z_) {
  if (!worldToMap3D(px, py, origin_z_, mx, my, mz)) {  // ← 钳位
    continue;
  }
} else if (!worldToMap3D(px, py, pz, mx, my, mz)) {
  continue;
}

// 新版（直接丢弃）:
double max_z = origin_z_ + size_z_ * z_resolution_;
if (pz < origin_z_ || pz >= max_z) {
  continue;  // ← 直接丢弃超范围点
}
if (!worldToMap3D(px, py, pz, mx, my, mz)) {
  continue;
}
```

---

## 9. filter_node.cpp — 激光雷达过滤节点

**路径**: `src/cpp_lidar_filter/src/filter_node.cpp`

**原问题**:
1. `cloud_callback` 中每帧调用 `get_parameter()`，高频点云下性能开销大
2. 降采样代码被注释掉，无法配置开关

**修改内容**:

### 参数缓存
```cpp
// 旧版（cloud_callback 内每帧读取）:
crop.setMin(this->get_parameter("min_x").as_double());

// 新版（初始化时缓存为成员变量）:
// 构造函数中调用 updateCachedParams() 一次性读取
// callback 中使用缓存值 min_x_, max_x_ 等
```

### 动态参数回调
```cpp
rcl_interfaces::msg::SetParametersResult dynamicParamsCallback(
  std::vector<rclcpp::Parameter> parameters) {
  updateCachedParams();  // 参数更新时刷新缓存
  result.successful = true;
  return result;
}
```

### 体素滤波参数化
```cpp
// 新增参数
this->declare_parameter("enable_voxel_filter", false);

// 条件启用
if (enable_voxel_filter_) {
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud_cropped);
  sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  sor.filter(*cloud_filtered);
}
```

### 新增成员
```cpp
// 缓存参数
double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
bool negative_;
double leaf_size_;
bool enable_voxel_filter_;
std::string input_topic_, output_topic_;
// 动态参数句柄
rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
```

---

## 10. back_up_free_space.cpp — 后退自由空间行为

**路径**: `src/pb_nav2_plugins/src/behaviors/back_up_free_space.cpp`

**原问题**: `onRun()` 中获取了两次当前位姿（分别在检查 costmap 之前和计算后退方向之后），第二次覆盖第一次，第一次查询浪费且两次位姿时间不同

**修改内容**:

```cpp
// 旧版（重复获取）:
// 第一次 getCurrentPose（约第 75 行）
// ... 计算 findBestDirection ...
// 第二次 getCurrentPose（约第 98 行） ← 重复，覆盖 initial_pose_

// 新版：删除第二次 getCurrentPose，只保留第一次
end_time_ = clock_->now() + command_time_allowance_;
// 直接进入日志输出
RCLCPP_WARN(logger_, "backing up %f meters towards free space at angle %f", ...);
```

---

## 11. singlenav2_params.yaml — Nav2 参数配置

**路径**: `src/cod_bringup/params/singlenav2_params.yaml`

**修改内容**:

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `controller_frequency` | 50.0 | 30.0 | 给 MPPI (batch_size=1500, time_steps=80) 更多计算时间 |
| `progress_checker.movement_time_allowance` | 999.0 | 30.0 | 卡死时 30s 触发恢复，而非 17 分钟 |
| `robot_base_frame` (全部 5 处) | `base_link_fake` | `base_link` | 首测使用真实机身坐标系 |
| `vx_max` 注释 | 无 | 添加对角速度警告 | 提醒 vx/vy 各自 ±2.5 时对角合速度可达 3.54m/s |
| `smoothing_frequency` 注释 | 无 | 添加匹配说明 | 标注与 controller_frequency 一致 |

**robot_base_frame 修改位置**:
1. `bt_navigator.ros__parameters.robot_base_frame`
2. `controller_server.ros__parameters` (如有)
3. `local_costmap.local_costmap.ros__parameters.robot_base_frame`
4. `global_costmap.global_costmap.ros__parameters.robot_base_frame`
5. `behavior_server.ros__parameters.robot_base_frame`

---

## 坐标系最终确认

### 速度方向链

```
Nav2 vx(+) → base_link +x → LiDAR前倾方向(水平投影) → 电控 vx(+)
```

三者一致的条件（已验证）:
- `singlenav2_params.yaml`: `robot_base_frame: base_link`
- `fake_vel_transform`: `enable_vel_rotation: false`（直通模式）
- URDF: `livox_frame` 相对 `base_link` 仅俯仰 45°，yaw=0

### 自转模式恢复

以后需要高速自转时，改两处：
```yaml
# singlenav2_params.yaml: base_link → base_link_fake
# fake_vel_transform: enable_vel_rotation: true
```

---

## 附录：MCP 工具修复（非导航代码）

### openai-gateway-server.py

**路径**: `~/.claude/mcp-servers/openai-gateway-server.py`

**问题**: SOCKS 代理环境变量 `ALL_PROXY=socks://127.0.0.1:7897/` 导致 httpx 连接失败

**修复**: 在创建 httpx client 前清除 socks 代理环境变量
```python
for key in ("ALL_PROXY", "all_proxy"):
    os.environ.pop(key, None)
```

### settings.json

**路径**: `~/.claude/settings.json`

**修复**: openai-gateway MCP 的 env 中覆盖代理变量
```json
"env": {
  "ALL_PROXY": "",
  "all_proxy": ""
}
```
