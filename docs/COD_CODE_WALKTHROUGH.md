# 🤖 COD 导航项目 — 小白级逐行讲解

> 生成日期: 2026-07-10
> 项目路径: `cod_-rm2026_-navigation-master/`

---

## 目录

1. [项目概述](#第一章项目概述)
2. [项目入口 cod_bringup](#第二章项目入口--cod_bringup)
3. [串口通信 serial_def_sdk](#第三章串口通信--serial_def_sdk最核心的包)
4. [坐标变换 fake_vel_transform](#第四章坐标变换-fake_vel_transform)
5. [控制器层](#第五章控制器层)
6. [完整数据流](#第六章完整数据流)
7. [辅助组件](#第七章辅助组件)
8. [新旧协议对比](#第八章新旧串口协议对比)
9. [LIO 算法 small_point_lio](#第九章lio-算法--small_point_lio)
10. [Gazebo 仿真环境](#第十章gazebo-仿真环境--cod_gazebo_simulator)
11. [Nav2 配置参数详解](#第十一章nav2-配置参数详解)
12. [常见问题与排查](#第十二章常见问题与排查)
13. [总结](#总结项目的精妙之处)

---

## 第一章：项目概述

### 1.1 一句话概括

这是一个 **RoboMaster 比赛机器人的自动导航系统**。它让机器人能够：
- 在比赛场地上**知道自己在哪**（定位）
- **规划路线**去目标点（路径规划）
- **控制底盘**沿路线行驶（运动控制）
- 通过**串口**和底层单片机(MCU)通信，发送速度指令

### 1.2 硬件组成

```
[Livox MID-360 激光雷达] → 感知环境，获取点云
[MCU (单片机)]          → 接收速度指令，驱动电机
[工控机 (你的电脑)]      → 运行本项目的所有代码
```

当前真机未安装 RealSense，导航障碍物观测源只有 MID-360。雷达光心离地 `0.46m`、前倾 `+0.7854rad`，软件以 `base_link` 的 z=0 为地面基准。

### 1.3 软件架构总览（16 个包）

```
┌───────────────────────────────────────────────┐
│                  cod_bringup                    │  ← 启动入口（launch文件+配置）
├───────────────────────────────────────────────┤
│  small_point_lio    │  pointcloud_to_laserscan │  ← 感知层
│  cpp_lidar_filter   │                          │
├───────────────────────────────────────────────┤
│  Nav2 (navigation_launch.py)                   │  ← 导航框架
│  ├─ planner_server   (全局路径规划)            │
│  ├─ controller_server (局部运动控制)           │
│  │   ├─ goal_approach_controller (wrapper)     │
│  │   └─ pb_omni_pid_pursuit_controller (PID)   │
│  ├─ behavior_server  (行为树)                  │
│  ├─ smoother_server  (路径平滑)                │
│  └─ bt_navigator     (行为树导航器)            │
├───────────────────────────────────────────────┤
│  fake_vel_transform  │  serial_def_sdk         │  ← 坐标变换+串口通信
├───────────────────────────────────────────────┤
│  waypoint_editor     │  pb_nav2_plugins        │  ← 辅助工具
└───────────────────────────────────────────────┘
```

### 1.4 16 个包功能速查

| 包名 | 功能 | 类型 |
|------|------|------|
| `cod_bringup` | 启动入口，launch 文件 + 参数配置 | Python/CMake |
| `serial_def_sdk` | 串口通信 (Seasky协议) | C++ |
| `fake_vel_transform` | 坐标系速度变换 | C++ |
| `small_point_lio` | 激光惯性里程计 (LIO) | C++ |
| `pointcloud_to_laserscan` | 3D点云转2D激光扫描 | C++ |
| `cpp_lidar_filter` | 车身点云过滤 (CropBox) | C++ |
| `goal_approach_controller` | Nav2控制器wrapper，目标接近减速 | C++ |
| `pb_omni_pid_pursuit_controller` | 全向底盘PID路径追踪 | C++ |
| `pb_nav2_plugins` | 后退行为 + 强度体素层 | C++ |
| `slam_dynamic_filter` | SLAM 动态障碍过滤 | Python |
| `def_msg` | 自定义ROS消息定义 | CMake |
| `vision_msg` | 视觉消息定义 | CMake |
| `pb_rm_interfaces` | 裁判系统接口定义 | CMake |
| `waypoint_editor` | RViz航点编辑器 | C++/Qt |
| `cod_gazebo_simulator` | Gazebo仿真环境 | CMake/Python |
| `livox_ros_driver2` | Livox雷达驱动 | C++ |

---

## 第二章：项目入口 — `cod_bringup`

### 2.1 什么是 launch 文件？

ROS2 的 `.launch.py` 文件就像是"启动脚本"，它一次性启动多个程序（ROS2 中叫"节点"）。

`cod_bringup` 是纯 CMake 包，没有 C++ 源码，只包含配置文件和 launch 脚本。它的 `CMakeLists.txt` 仅做安装：

```cmake
install(
  DIRECTORY launch params rviz maps wps behavior_trees
  DESTINATION share/${PROJECT_NAME}
)
```

### 2.2 单点导航启动文件：`singlenav_launch.py`

这个文件启动的是**单点导航模式**（加载已有地图，以 LIO 提供 `odom→base_link`，固定 `map→odom`；不启动 AMCL）。

```python
# 关键导入
from launch import LaunchDescription               # launch的核心类
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node                 # Node: 启动一个ROS2节点
from ament_index_python.packages import get_package_share_directory
```

**最核心的概念：`Node`**。一个 Node = 一个运行中的程序。比如 `/livox/lidar` 发布者是一个 Node，导航控制器是另一个 Node。

```python
# 声明启动参数
declare_use_sim_time = DeclareLaunchArgument(
    'use_sim_time', default_value='false',           # 默认不用仿真时间
    description='Use simulation (Gazebo) clock if true')
```

ROS2 有两种时间来源：真实系统时间和 Gazebo 仿真时间。`use_sim_time=false` 表示用真实时间。

```python
# cpp_lidar_filter 节点 — 裁剪车身点云
Node(
    package='cpp_lidar_filter',
    executable='lidar_filter_node',
    name='my_lidar_filter',
    output='screen',
    parameters=[{
        'input_topic': '/livox/lidar',       # 输入: 原始雷达点云
        'output_topic': '/livox/lidar_filtered', # 输出: 过滤后的点云
        'crop_frame': 'base_link',           # 先变换到机体/地面坐标系再裁剪
        'min_x': -0.2, 'max_x': 0.2,         # 裁剪盒子参数
        'min_y': -0.2, 'max_y': 0.4,
        'min_z': -0.1, 'max_z': 0.2,
        'negative': True,                    # True=挖掉盒子内部的点(去除车身)
        'leaf_size': 0.05                    # 降采样粒度 5cm
    }]
),
```

**为什么需要这个节点？** 激光雷达是 360° 的，它会"看到"自己的身体。如果不把车身点云挖掉，机器人会把自己当成障碍物！

```python
# small_point_lio 节点 — 激光惯性里程计
Node(
    package="small_point_lio",
    executable="small_point_lio_node",
    name="small_point_lio",
    output="screen",
    parameters=[
        PathJoinSubstitution([
            FindPackageShare("small_point_lio"), "config", "mid360.yaml",
        ])
    ],
),
```

**什么是 LIO？** LIO = LiDAR-Inertial Odometry。通过匹配连续的激光雷达帧来估算机器人移动了多远。GPS 在室内不能用，LIO 就是室内的"GPS"。

```python
# map→odom 静态TF发布 (单点导航特有)
Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="map_to_odom",
    arguments=[
        "--x", "0.0", "--y", "0.0", "--z", "0.05",
        "--yaw", "-0.5",
        "--frame-id", "map", "--child-frame-id", "odom",
    ],
),
```

```python
# base_link→livox_frame 静态TF
Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="livox_to_base_link",
    arguments=[
        "--x", "0.0", "--y", "0.0", "--z", "0.46",  # 雷达光心离地 46cm
        "--pitch", "0.7854",                         # 前倾 45°
        "--frame-id", "base_link", "--child-frame-id", "livox_frame",
    ],
),
```

```python
# fake_vel_transform 节点 — 坐标系速度变换 (COD最关键的创新之一！)
Node(
    package="fake_vel_transform",
    executable="fake_vel_transform_node",
    output="screen",
    parameters=[{"use_sim_time": use_sim_time}],
),
```

```python
# serial_def_sdk 节点 — 串口通信
Node(
    package='serial_def_sdk',
    executable='uart',
    name='hardware_serial',
    output='screen',
    parameters=[{'serial_port': "/dev/ttyACM0"}],
    remappings=[
        ('/hardware/cmd_vel_api', '/aft_cmd_vel'),  # ★ 话题重映射
    ],
),
```

**`/hardware/cmd_vel_api` → `/aft_cmd_vel` 是数据流的核心链路！**
- Nav2 输出依次经过速度平滑、Collision Monitor 和 LiDAR 新鲜度看门狗 → `cmd_vel`
- `fake_vel_transform` 接收安全门控后的 `cmd_vel` → 输出 `aft_cmd_vel`
- `serial_def_sdk` 接收 `aft_cmd_vel` → 发送给 MCU

### 2.3 多点导航 vs 单点导航的区别

| 特性 | singlenav (单点) | multiplenav (多点) |
|------|-----------------|-------------------|
| 地图对齐 | 固定 map→odom，无 AMCL | 固定 map→odom；slam_toolbox 不发布校正 TF |
| 参数文件 | singlenav2_params.yaml | multiplenav2_params.yaml |
| pointcloud_to_laserscan | 不需要 | 需要 (SLAM用2D scan) |
| auto_save_map | 不需要 | 每30秒自动保存地图 |
| localization_launch | ✓ 包含 | ✗ 不需要 |

### 2.4 Nav2 导航启动文件 `navigation_launch.py`

启动了 8 个生命周期节点、一个独立 LiDAR 命令看门狗和生命周期管理器：

| 服务器 | 作用 |
|--------|------|
| `controller_server` | 局部运动控制（MPPI/PID 等） |
| `smoother_server` | 路径平滑（Savitzky-Golay滤波器） |
| `planner_server` | 全局路径规划（SmacPlanner2D） |
| `behavior_server` | 行为管理（spin/backup/drive_on_heading） |
| `bt_navigator` | 行为树导航器 |
| `waypoint_follower` | 多点巡逻 |
| `velocity_smoother` | 速度平滑 |
| `collision_monitor` | 0.55m StopZone 独立停车检查 |
| `lidar_cmd_watchdog` | 点云超过 0.3s 未更新时持续发布零速度 |

**关键的重映射：**
- `controller_server` 输出 `cmd_vel_nav` (而非默认的 `cmd_vel`)
- `velocity_smoother` 接收 `cmd_vel_nav` → 输出 `cmd_vel_smoothed`
- `collision_monitor` 输出 `cmd_vel_collision_safe`
- `lidar_cmd_watchdog` 仅在点云新鲜时转发为 `cmd_vel`
- `fake_vel_transform` 从 `cmd_vel` 接收 → 输出 `aft_cmd_vel`

---

## 第三章：串口通信 — `serial_def_sdk`（最核心的包）

### 3.1 六层架构

```
┌──────────────────────────────────────┐
│ 6. base_controller.cpp  ←  ROS2 节点  │  "把 ROS 消息写到全局变量"
│ 5. uart_interface.cpp   ←  C 接口     │  "全局变量 → 协议发送"
│ 4. SeaskyProtocol.cpp   ←  协议层     │  "把结构体打包成字节流"
│ 3. DataType.h           ←  数据定义   │  "定义结构体：速度、云台、心跳"
│ 2. SerialDriver.cpp     ←  驱动层     │  "读写 /dev/ttyACM0"
│ 1. crc16.h + crc8.h     ←  校验层     │  "算 CRC 校验和"
└──────────────────────────────────────┘
```

### 3.2 第 1 层：CRC 校验

```c
#define CRC_START_16 0xFFFF       // CRC-16 初始值
#define CRC_POLY_16 0xA001        // CRC-16 多项式（与Modbus相同）
uint16_t crc_16(const uint8_t *input_str, uint16_t num_bytes);

#define CRC_START_8 0x00
uint8_t crc_8(const uint8_t *input_str, uint16_t num_bytes);
```

**什么是 CRC？** CRC = Cyclic Redundancy Check（循环冗余校验）。发送方算校验码附在数据末尾，接收方重算一遍。对不上说明数据出错。类似快递封条——封条破了说明包裹在路上被动过。

### 3.3 第 2 层：数据定义 — `DataType.h`

```c
// 消息 ID 枚举
enum msg_id {
    heartbeat   = 0x01,   // 心跳包: 双方确认"我还活着"
    gimbal      = 0x02,   // 云台数据: yaw, pitch, fire
    control     = 0x03,   // 控制数据: 视觉→电控
    additional  = 0x04,   // 附加状态: 发射机构、摩擦轮
    fix_control = 0x05,   // ★ 组合控制包: 云台+底盘合并
    imu2        = 0x06,   // IMU 数据
    yaw         = 0x07,   // Yaw 轴角度
    game_status = 0x08,   // 比赛状态
    chassis     = 0x10,   // 底盘速度
    sentry_state= 0x20,   // 哨兵状态（电控→上位机）
};
```

**★ 最核心的结构体：FixControlData（上位机→MCU，msg_id=0x05）**
```c
typedef struct {
    // 云台部分
    float yaw;       // 偏航角 (rad)
    float pitch;     // 俯仰角 (rad)
    uint8_t fire;    // 开火指令 (0/1)
    
    // 底盘部分
    float vx;        // X方向线速度 (m/s)
    float vy;        // Y方向线速度 (m/s)
    float vz;        // Z方向角速度 (rad/s)
    float spin;      // 小陀螺速度 (rad/s)
} FixControlData;
// 序列化后精确 21 字节
```

**其他重要结构体：**
```c
typedef struct { float vx, vy, vz; } ChassisSpeed;     // 底盘速度
typedef struct { float yaw, pitch; uint8_t fire; } Gimbal;  // 云台
typedef struct { float battery, life, color, bullet, fault_flag; } SentryState;  // 哨兵状态
```

### 3.4 第 3 层：串口驱动 — `SerialDriver.cpp`

```cpp
class SerialDriver {
public:
    using ReceiveCallback = std::function<void(const uint8_t*, size_t)>;

    bool open(const std::string& device, int baudrate = 115200);
    void close();
    int write(const uint8_t* data, size_t len);
    void setReceiverCallback(ReceiveCallback callback);

private:
    void readLoop();                              // 后台线程，不停读串口
    bool configureTermios(int baudrate);          // 配置串口为 8N1 原始模式

    std::string device_name_;
    std::atomic<int> fd_{-1};                     // ★ 原子类型，防多线程竞争
    std::atomic<bool> is_running_;
    std::thread read_thread_;
    ReceiveCallback callback_;
    std::mutex write_mutex_;
};
```

**`open()` 详解：**
```cpp
int new_fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
// O_RDWR: 读写模式
// O_NOCTTY: 不成为控制终端
// O_NDELAY: 非阻塞模式
```

**`configureTermios()` — 配置为 8N1 原始模式：**
```cpp
options.c_cflag |= (CLOCAL | CREAD);  // 本地连接 + 启用接收
options.c_cflag |= CS8;                // 8位数据
options.c_cflag &= ~PARENB;            // 无校验
options.c_cflag &= ~CSTOPB;            // 1停止位
options.c_cflag &= ~CRTSCTS;           // 无流控
options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // 原始模式
options.c_cc[VMIN] = 0;               // 最少读0字符
options.c_cc[VTIME] = 1;              // 等0.1秒
```

**`readLoop()` — 持久连接 + 自动重连（核心能力）：**
```cpp
while (is_running_) {
    int current_fd = fd_.load();          // 原子读取

    if (current_fd == -1) {
        // 串口断了！等500ms后重试打开
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        int new_fd = ::open(device_name_.c_str(), ...);
        if (new_fd != -1) {
            fd_.store(new_fd);            // 原子写入
            configureTermios(...);
            std::cout << "SUCCESS: Device reconnected!" << std::endl;
        }
        continue;
    }

    int n = ::read(current_fd, buffer, sizeof(buffer));
    if (n > 0) {
        if (callback_) callback_(buffer, n);  // → 上层处理
    } else if (n == 0 || (n < 0 && errno != EAGAIN)) {
        // 物理断开 → 原子关fd
        int expected = current_fd;
        fd_.compare_exchange_strong(expected, -1);
        ::close(current_fd);
    }
}
```

**为什么用 `std::atomic`？** `readLoop` 在后台线程运行，`write()` 在主线程调用。不用原子操作会导致数据竞争。

**`write()` — 线程安全发送：**
```cpp
int SerialDriver::write(const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(write_mutex_);  // 防并发写
    int current_fd = fd_.load();  // 锁内重新读fd，避免TOCTOU
    if (current_fd == -1) return -1;
    
    int total_written = 0;
    while (remaining > 0) {
        int n = ::write(current_fd, data + total_written, remaining);
        if (n <= 0) {
            if (errno != EAGAIN) {
                // 写失败 → 原子关fd
                int expected = current_fd;
                if (fd_.compare_exchange_strong(expected, -1)) ::close(current_fd);
            }
            return -1;
        }
        total_written += n;
        remaining -= n;
    }
    return total_written;
}
```

### 3.5 第 4 层：协议层 — `SeaskyProtocol.cpp`

#### 3.5.1 发送：`send()` 函数

```cpp
bool SeaskyProtocol::send(int msg_id, void* data) {
    std::lock_guard<std::mutex> lock(send_mutex_);  // 防并发发送
    
    uint8_t payload[128];
    uint16_t len = 0;

    switch (msg_id) {
        case fix_control: {  // ★ 最常用
            FixControlData* d = (FixControlData*)data;
            memcpy(payload + len, &d->yaw, 4);   len += 4;
            memcpy(payload + len, &d->pitch, 4); len += 4;
            payload[len++] = d->fire;
            memcpy(payload + len, &d->vx, 4);    len += 4;
            memcpy(payload + len, &d->vy, 4);    len += 4;
            memcpy(payload + len, &d->vz, 4);    len += 4;
            memcpy(payload + len, &d->spin, 4);  len += 4;
            break;
        }
        case chassis: { /* 3 float = 12 字节 */ break; }
        case gimbal:  { /* 2 float + 1 u8 = 9 字节 */ break; }
        case heartbeat: { /* 7 字节 */ break; }
    }
    sendPacket((uint16_t)msg_id, payload, len);
}
```

#### 3.5.2 Seasky 帧格式：`sendPacket()`

```cpp
void SeaskyProtocol::sendPacket(uint16_t msg_id, const uint8_t* payload, uint16_t payload_len) {
    uint8_t frame[256];
    int pos = 0;

    frame[pos++] = PROTOCOL_HEADER_ID;    // 0xA5 — 帧头
    frame[pos++] = payload_len & 0xFF;    // 数据长度低字节
    frame[pos++] = (payload_len >> 8) & 0xFF; // 数据长度高字节
    frame[pos++] = crc_8(frame, 3);       // CRC-8 校验前3字节
    frame[pos++] = msg_id & 0xFF;         // 消息ID
    frame[pos++] = (msg_id >> 8) & 0xFF;
    frame[pos++] = flags & 0xFF;          // 标志位
    frame[pos++] = (flags >> 8) & 0xFF;
    memcpy(frame + pos, payload, payload_len);
    pos += payload_len;
    
    uint16_t crc = crc_16(frame, pos);    // CRC-16 校验全帧
    frame[pos++] = crc & 0xFF;
    frame[pos++] = (crc >> 8) & 0xFF;
    
    driver_.write(frame, pos);            // → SerialDriver::write()
}
```

**完整帧结构：**

```
Byte:  0    1    2    3    4    5    6    7    8..N+7  N+8  N+9
     ┌────┬────┬────┬────┬────┬────┬────┬────┬───────┬────┬────┐
     │0xA5│LenL│LenH│CRC8│ID_L│ID_H│FL_L│FL_H│ Data  │CRCL│CRCH│
     └────┴────┴────┴────┴────┴────┴────┴────┴───────┴────┴────┘
     帧头  长度(LE) CRC8  ID(LE)  Flags   payload   CRC16(LE)
     
总长度 = 8(头部) + N(数据) + 2(CRC16) = 10+N 字节
```

#### 3.5.3 接收：`processBuffer()` — 流式解析

```cpp
void SeaskyProtocol::processBuffer() {
    while (rx_buffer_.size() >= 8) {
        // 1. 找帧头 0xA5
        if (rx_buffer_[0] != PROTOCOL_HEADER_ID) {
            rx_buffer_.erase(rx_buffer_.begin());
            continue;
        }

        // 2. 解析数据长度
        uint16_t payload_len = rx_buffer_[1] | (rx_buffer_[2] << 8);
        if (payload_len > 200) { rx_buffer_.erase(rx_buffer_.begin()); continue; }

        // 3. 检查完整性
        size_t total_frame_len = payload_len + 10;
        if (rx_buffer_.size() < total_frame_len) return;  // 数据不够，等下次

        // 4. CRC-8 头部校验
        uint8_t header_crc8 = crc_8(rx_buffer_.data(), 3);
        if (header_crc8 != rx_buffer_[3]) {
            rx_buffer_.erase(rx_buffer_.begin());
            continue;
        }

        // 5. CRC-16 全帧校验 (支持 NO_CRC_MODE 编译开关)
        uint16_t calculated = crc_16(rx_buffer_.data(), total_frame_len - 2);
        uint16_t received = ...;
        bool crc_ok = (calculated == received);
#ifdef NO_CRC_MODE
        crc_ok = true;  // MCU固件CRC未就绪时的兼容模式
#endif

        if (crc_ok) {
            uint16_t msg_id = rx_buffer_[4] | (rx_buffer_[5] << 8);
            dispatchMessage(msg_id, rx_buffer_.data() + 8, payload_len);
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + total_frame_len);
        } else {
            rx_buffer_.erase(rx_buffer_.begin());  // CRC失败→丢一字节继续找
        }
    }
}
```

**为什么需要缓冲区？** 串口数据是流式的，一帧可能分多次收到。缓冲区攒够一帧才解析。

**`dispatchMessage()` — 将字节填充到全局结构体：**
```cpp
void dispatchMessage(uint16_t msg_id, const uint8_t* payload, uint16_t len) {
    switch (msg_id) {
        case heartbeat:
            memcpy(&heartbeat_receive.timestamp, payload, 2);
            heartbeat_receive.battery = payload[2];
            // ...
            break;
        case gimbal:
            memcpy(&gimbal_receive.yaw, payload, 4);
            memcpy(&gimbal_receive.pitch, payload + 4, 4);
            break;
        case chassis:
            memcpy(&chassis_receive.vx, payload, 4);
            memcpy(&chassis_receive.vy, payload + 4, 4);
            memcpy(&chassis_receive.vz, payload + 8, 4);
            break;
        case sentry_state:
            memcpy(&sentry_state_receive, payload, 20);  // 5×float
            break;
    }
}
```

### 3.6 第 5 层：C 接口 — `uart_interface.cpp`

连接 C 风格全局变量和 C++ SeaskyProtocol：

```cpp
static std::unique_ptr<SeaskyProtocol> g_protocol = nullptr;

extern "C" {
    bool defUartSend(int id) {
        switch (id) {
            case fix_control: return g_protocol->send(id, &fix_control_send);
            case gimbal:      return g_protocol->send(id, &gimbal_send);
            case control:     return g_protocol->send(id, &control_data);
            case chassis:     return g_protocol->send(id, &chassis_send);
            // ...
        }
    }
}
```

### 3.7 第 6 层：ROS2 节点 — `base_controller.cpp`

#### 3.7.1 构造函数

```cpp
BaseController(string name) : Node(name) {
    // 参数
    this->declare_parameter("serial_port", "/dev/ttyUSB0");
    this->declare_parameter("topic_timeout_ms", 100);

    // ★ 订阅速度指令 (Nav2→MCU入口)
    cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "hardware/cmd_vel_api", 10,              // 被重映射为 /aft_cmd_vel
        std::bind(&BaseController::cmd2serial, this, _1));

    // ★ 订阅云台控制
    gimble_control_sub = this->create_subscription<def_msg::msg::GimbleControl>(
        "vision/gimble_control", rclcpp::SensorDataQoS(),
        std::bind(&BaseController::gimble2serial, this, _1));

    // 发布 MCU 回传数据
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    gobal_information_pub = this->create_publisher<def_msg::msg::GobalInformation>(
        "hardware/gobal_information", 10);
    game_status_pub_ = this->create_publisher<pb_rm_interfaces::msg::GameStatus>(
        "/robot/referee/game_status", 10);
    robot_status_pub_ = this->create_publisher<pb_rm_interfaces::msg::RobotStatus>(
        "/robot/referee/robot_status", 10);

    // ★ 定时器 50Hz 合并发送
    fix_send_timer_ = this->create_wall_timer(20ms,
        [this]() { this->send_merged_control(); });
    
    // 话题超时参数
    this->declare_parameter("topic_timeout_ms", 100);
}
```

#### 3.7.2 数据接收回调

```cpp
// 速度指令到全局变量 (不立即发送!)
void BaseController::cmd2serial(const geometry_msgs::msg::Twist::UniquePtr twist_aux) {
    fix_control_send.vx = twist_aux->linear.x;
    fix_control_send.vy = twist_aux->linear.y;
    fix_control_send.vz = twist_aux->angular.z;
    last_cmd_time_ = this->get_clock()->now();   // 记录时间戳
}

// 云台指令到全局变量
void BaseController::gimble2serial(const def_msg::msg::GimbleControl::UniquePtr msg) {
    fix_control_send.yaw = msg->yaw;
    fix_control_send.pitch = msg->pitch;
    fix_control_send.fire = msg->fire_advise;
    last_gimbal_time_ = this->get_clock()->now();
}
```

**为什么不立即发送？** Nav2 各话题频率不同，来一个发一个会把串口打爆。定时合并发送更可靠。

#### 3.7.3 核心方法：`send_merged_control()`

```cpp
void BaseController::send_merged_control() {
    // ★ 1. 初始拦截锁：收到第一条消息前绝对静默
    if (!has_ever_received_) {
        if (last_cmd_time_ != init_cmd_time_ || 
            last_gimbal_time_ != init_gimbal_time_) {
            has_ever_received_ = true;  // 解除！
        } else {
            return;  // 还没收到过消息，不发数据
        }
    }

    // ★ 2. 话题超时检测 (默认100ms)
    double timeout_sec = topic_timeout_ms_ / 1000.0;
    bool cmd_alive = (now - last_cmd_time_).seconds() <= timeout_sec;
    bool gimbal_alive = (now - last_gimbal_time_).seconds() <= timeout_sec;

    // 全部超时→不发
    if (!cmd_alive && !gimbal_alive && !spin_alive) return;

    // ★ 3. 超时话题数据清零
    if (!cmd_alive) { fix_control_send.vx = 0.0; fix_control_send.vy = 0.0; }
    if (!gimbal_alive || override_gimbal_) { 
        fix_control_send.yaw = 0.0; fix_control_send.pitch = 0.0; 
    }

    // ★ 4. 小陀螺互斥逻辑
    fix_control_send.spin = (std::abs(current_spin_speed) > 0.001) ? current_spin_speed : 0.0;

    // ★ 5. 发送！
    defUartSend(fix_control);  // → SeaskyProtocol::send() → SerialDriver::write()
}
```

#### 3.7.4 裁判系统数据发布 (`serial2global()`)

```cpp
void BaseController::serial2global() {
    // 发布 GlobalInformation（来自 sentry_state_receive）
    def_msg::msg::GobalInformation status{};
    status.battery = sentry_state_receive.battery;
    status.life_extra = sentry_state_receive.life;
    // ...
    gobal_information_pub->publish(status);

    // 发布 GameStatus
    pb_rm_interfaces::msg::GameStatus game_msg;
    game_msg.game_progress = (sentry_state_receive.fault_flag == 1) ? 4 : 0;  // RUNNING or NOT_START
    game_status_pub_->publish(game_msg);

    // 发布 RobotStatus（血量等）
    pb_rm_interfaces::msg::RobotStatus robot_msg;
    robot_msg.current_hp = sentry_state_receive.life;
    robot_msg.maximum_hp = 500;
    robot_msg.robot_id = 1;
    // ...
    robot_status_pub_->publish(robot_msg);
}
```

#### 3.7.5 main() 函数

```cpp
int main(int argc, char* argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);  // 关闭stdout缓冲
    rclcpp::init(argc, argv);

    uart = std::make_shared<UartLinux>();
    uart->startReading();  // 启动后台读取（实际由SerialDriver处理）

    auto base_control_node = std::make_shared<BaseController>("hardware_serial");
    rclcpp::spin(base_control_node);
    rclcpp::shutdown();
    return 0;
}
```

## 第四章：坐标变换 `fake_vel_transform`

该节点位于安全门控与串口之间，负责可选的场地坐标速度旋转和命令超时清零。

### 4.1 问题背景

当前 Nav2 的 `robot_base_frame` 是真实 `base_link`，控制器输出本来就是机体系速度，因此默认无需额外旋转。

- Map 系：X指北、Y指东，固定
- 机体系：X指前方、Y指左方，随机器人旋转

### 4.2 当前默认行为

`enable_vel_rotation=false` 时，线速度和上游角速度直接透传。节点仍发布同原点、仅抵消 yaw 的 `base_link_fake`，供以后显式启用场地坐标速度模式时使用；Nav2 当前不使用该帧。

```cpp
// odomCallback: 发布 base_link → base_link_fake 的TF
void FakeVelTransform::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);  // 机器人朝向
    
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = robot_base_frame_;       // 父
    t.child_frame_id = fake_robot_base_frame_;    // 子
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, -current_robot_base_angle_);  // ★ 转负朝向角
    t.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(t);
}
```

### 4.3 速度与超时保护

```cpp
void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    geometry_msgs::msg::Twist aft_tf_vel;
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

50ms 定时器检查 `cmd_vel_timeout`（默认 0.5s）；上游命令消失后持续发布零速度。

---

## 第五章：控制器层

### 5.1 GoalApproachController — 接近目标减速

Nav2 控制器的 **wrapper**，透明代理内部控制器（如 MPPI），仅在接近目标时限速。

```cpp
geometry_msgs::msg::TwistStamped computeVelocityCommands(...) {
    auto cmd = inner_controller_->computeVelocityCommands(...);  // MPPI输出

    double dist = std::hypot(dx, dy);  // 到目标距离

    if (dist < approach_distance_) {              // < 2.5m
        // 仅缩放 MPPI 已检查命令的线速度幅值，不覆盖方向
        double allowed = std::min(approach_velocity_, dist * approach_kp_);
        double speed = std::hypot(cmd.twist.linear.x, cmd.twist.linear.y);
        if (speed > allowed) {
            double scale = allowed / speed;
            cmd.twist.linear.x *= scale;
            cmd.twist.linear.y *= scale;
        }
    }
    return cmd;
}
```

**三个距离区间：**
- `< 0.5m`: P控制器直接朝目标走
- `< 2.5m`: 速度钳位 ≤ 0.2m/s
- `> 2.5m`: 正常MPPI控制

### 5.2 OmniPidPursuitController — 全向底盘PID追踪

**核心流程：**
1. 找**前瞻点**（lookahead point）
2. PID 算速度追那个点
3. **曲率减速**过弯
4. 碰撞检测

```cpp
double lin_dist = hypot(carrot_pose.x, carrot_pose.y);        // 到前瞻点距离
auto lin_vel = move_pid_->calculate(lin_dist, 0);              // PID算线速度
auto angular_vel = heading_pid_->calculate(angle_to_goal, 0);  // PID算角速度

// 分解到X/Y（全向底盘直接朝那个方向走）
cmd_vel.twist.linear.x = lin_vel * cos(theta_dist);
cmd_vel.twist.linear.y = lin_vel * sin(theta_dist);
```

**曲率减速：**
```cpp
if (curvature > curvature_min_) {
    reduction_ratio = 1.0 - (curvature - curvature_min_) / 
                     (curvature_max_ - curvature_min_) * (1.0 - reduction_ratio_at_high_curvature_);
    scaled_linear_vel = linear_vel * reduction_ratio;
}
```

---

## 第六章：完整数据流

### 6.1 控制命令流（上位机→MCU）

```
用户点击
    │
    ▼
┌──────────────────────────────────────────────────────────┐
│                  Nav2 导航框架                             │
│                                                          │
│  planner_server(Smac2D) → smoother_server(SavitzkyGolay) │
│        → controller_server(GoalApproach/Maxq2             │
│                                                          │
│  输出: /cmd_vel_nav (Twist)                              │
└──────────────────────┬───────────────────────────────────┘
                       │
                       ▼
            [velocity_smoother]
              输出: /cmd_vel_smoothed
                       │
                       ▼
            [collision_monitor]
              0.55m StopZone
              输出: /cmd_vel_collision_safe
                       │
                       ▼
            [lidar_cmd_watchdog]
              点云超时0.3s持续零速
              输出: /cmd_vel
                       │
                       ▼
         ┌─────────────────────────────┐
         │    fake_vel_transform       │
         │  默认透传 + 0.5s命令超时      │
         │  输出: /aft_cmd_vel         │
         └──────────────┬──────────────┘
                        │
                        ▼
         ┌─────────────────────────────┐
         │    base_controller.cpp      │
         │  send_merged_control() 50Hz │
         │  打包 Seasky 帧             │
         │  [0xA5|Len|CRC8|ID|...|CRC] │
         │  driver_.write(frame)       │
         └──────────────┬──────────────┘
                        │ /dev/ttyACM0
                        ▼
                 ┌──────────────┐
                 │     MCU      │
                 │  驱动电机     │
                 └──────────────┘
```

### 6.2 感知数据流（传感器→上位机）

```
MID-360 雷达 (192.168.1.181)
    │ UDP
    ▼
[livox_ros_driver2]
    │ /livox/lidar + /livox/imu
    ├──────────────────────────┐
    ▼                          ▼
[cpp_lidar_filter]      [small_point_lio]
    │ 挖车身              │ 匹配点云→位姿
    ▼                          │ /Odometry
/livox/lidar_filtered          ▼
    │                    [Nav2 定位]
    ├──────────────────┐
    ▼                  ▼
[STVL/Collision Monitor]  [pointcloud_to_laserscan]
    │ 障碍物          │ 3D→2D
    ▼                  ▼
[local_costmap]    [slam_toolbox]
    │                  │ /scan
    ▼                  ▼
[MPPI controller]  [建图]
```

---

## 第七章：辅助组件

### 7.1 `cpp_lidar_filter` — 车身点云过滤

```cpp
pcl::CropBox<pcl::PCLPointCloud2> crop;
crop.setInputCloud(cloud_in);

Eigen::Vector4f min_pt(-0.2, -0.2, -0.1, 1.0);
Eigen::Vector4f max_pt( 0.2,  0.4,  0.2, 1.0);

crop.setMin(min_pt);
crop.setMax(max_pt);
crop.setNegative(true);  // ★ 挖掉盒内=车身
crop.filter(*cloud_cropped);
```

实现会先通过 TF 把输入点云变换到 `crop_frame=base_link`，再应用 CropBox；输出点云也标记为 `base_link`。这样前倾 45° 的雷达不会把雷达坐标轴误当成车身坐标轴。

### 7.2 `pointcloud_to_laserscan` — 3D→2D 转换

遍历点云，按高度/距离/角度过滤，将每个角度上最近的点距离填入 LaserScan：

生产多点模式输入为 `/livox/lidar_filtered`，目标帧为 `base_link`，高度范围 `0.05m..1.00m`。`0.05m` 下限用于覆盖最低离地 `0.10m` 的障碍，并留出地面/标定噪声裕量。

```cpp
for (iter_x, iter_y, iter_z) {
    if (*iter_z > max_height_ || *iter_z < min_height_) continue;  // 高度过滤
    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_ || range > range_max_) continue;        // 距离过滤
    double angle = atan2(*iter_y, *iter_x);
    int index = (angle - angle_min_) / angle_increment_;
    if (range < scan_msg->ranges[index])
        scan_msg->ranges[index] = range;  // 取最近距离
}
```

### 7.3 `waypoint_editor` — RViz 航点编辑器

RViz2 插件功能：
- 点击地图添加航点（绿色球体+ID标签）
- 拖拽移动航点
- 右键菜单：删除/改ID/编辑功能命令
- 保存/加载CSV
- 实时距离显示

### 7.4 航点执行器

- `waypoint_to_nav2`: `follow_waypoints` action（每个航点停下）
- `waypoint_through_nav2`: `navigate_through_poses` action（连续通过）

---

## 第八章：新旧串口协议对比

| 特性 | 已移除的旧 `ros2_simple_serial` | 当前 `serial_def_sdk` |
|------|------------------------|---------------------|
| 帧格式 | 15 字节裸包 | Seasky 帧 (10+N 字节) |
| 帧头 | 0xA5 | 0xA5 |
| 校验 | 简单累加和 | CRC-8 + CRC-16 |
| 连接 | 每次开/关 | 持久+自动重连 |
| 方向 | 发送only | 双向 |
| 数据 | 仅 vx/vy/vz | 云台+底盘+心跳+状态 |
| 安全 | 无超时保护 | 超时自动清零+拦截锁 |

---

## 第九章：LIO 算法 — `small_point_lio`

### 9.1 概述

基于 Point-LIO 的高效激光惯性里程计，ESKF (Error-State Kalman Filter) 驱动。

### 9.2 状态向量（30维）

```cpp
struct state {
    Eigen::Vector3d position;      // 位置
    Eigen::Matrix3d rotation;      // 旋转 (SO3)
    Eigen::Vector3d offset_R_L_I;  // LiDAR→IMU外参旋转
    Eigen::Vector3d offset_T_L_I;  // LiDAR→IMU外参平移
    Eigen::Vector3d velocity;      // 线速度
    Eigen::Vector3d omg;           // 当前角速度
    Eigen::Vector3d acceleration;  // 当前加速度
    Eigen::Vector3d gravity;       // 重力方向
    Eigen::Vector3d bg;            // 陀螺仪零偏
    Eigen::Vector3d ba;            // 加速度计零偏
};
```

### 9.3 算法流程

```
1. lidar_callback → 体素降采样 → point_deque
2. imu_callback → imu_deque
3. handle_once():
   ├─ 未初始化 → 积累地图+估计重力
   └─ 已初始化:
       ├─ IMU先到 → predict_state + update_imu
       ├─ 点云先到 → predict_state + update_point (PCA平面匹配)
       └─ 发布 Odometry + 注册点云
```

### 9.4 关键参数

| 参数 | 值 | 说明 |
|------|------|------|
| point_filter_num | 1 | 不降采样 |
| space_downsample_leaf_size | 0.5m | 体素大小 |
| extrinsic_T | [-0.011, -0.023, 0.044] | LiDAR相对IMU位置 |
| laser_point_cov | 0.01 | 激光点协方差 |
| plane_threshold | 0.1 | 平面拟合阈值 |
| match_sqaured | 81.0 | 匹配质量阈值 |

---

## 第十章：Gazebo 仿真环境 — `cod_gazebo_simulator`

### 10.1 包结构

```
cod_gazebo_simulator/
├── launch/
│   ├── _sim_common.py              # xacro处理
│   ├── sim_standalone.launch.py    # 空世界（无Nav2）
│   ├── gazebo_sim.launch.py        # AMCL定位仿真
│   └── gazebo_slam.launch.py       # SLAM建图仿真
├── config/
│   ├── gz_bridge.yaml              # ros_gz_bridge配置
│   └── sim_nav2_params.yaml        # 仿真Nav2参数
├── resource/
│   ├── cod_robot.urdf.xacro        # 机器人模型
│   ├── worlds/                     # 5个世界
│   └── models/                     # 4个场地模型
├── env-hooks/
│   ├── gazebo_model_path.dsv.in    # GZ_SIM_RESOURCE_PATH
│   └── gazebo_resource_path.dsv.in
└── maps/
```

### 10.2 真机 vs 仿真

| 组件 | 真机 | 仿真 |
|------|------|------|
| 里程计 | LIO | DiffDrive ground truth |
| 串口 | serial→MCU | ros_gz_bridge→Gazebo |
| 雷达 | MID-360物理 | GPU LiDAR传感器 |
| angular.z | MCU差速 | DiffDrive处理 |

### 10.3 gz_bridge

| 方向 | ROS | Gazebo |
|------|-----|--------|
| GZ→ROS | /Odometry | /model/cod_robot/odom |
| ROS→GZ | /cmd_vel | /model/cod_robot/cmd_vel |
| GZ→ROS | /livox/lidar | lidar/scan/points |
| GZ→ROS | /livox/imu | imu/imu |

### 10.4 重要细节

**env-hooks 使用 `GZ_SIM_RESOURCE_PATH`**（Ignition Fortress），不是 Classic Gazebo 的 `GAZEBO_MODEL_PATH`。

---

## 第十一章：Nav2 配置参数详解

### 11.1 MPPI 控制器

| 参数 | 值 | 说明 |
|------|------|------|
| controller_frequency | 50Hz | 控制频率 |
| motion_model | Omni | 全向模型 |
| time_steps | 80 | 预测步数 |
| batch_size | 1500 | 采样轨迹数 |
| temperature | 0.25 | 越小越果断 |
| vx_max/vy_max | 2.5 | 最大线速度 |
| xy_goal_tolerance | 0.2~0.3m | 目标精度 |

### 11.2 MPPI Critics

| Critic | 作用 | 权重 |
|--------|------|------|
| ConstraintCritic | 约束惩罚 | 4.0 |
| GoalCritic | 目标吸引 | 25.0 |
| PathFollowCritic | 路径跟随 | 15.0 |
| PathAlignCritic | 方向对齐 | 15.0 |
| ObstaclesCritic | 障碍排斥 | 1.5 |
| CostCritic | 代价梯度 | 3.8 |

**关键调优：** `critical_cost: 253.0` — 只处罚 inscribed/lethal，不把整个膨胀区当碰撞。

### 11.3 SmacPlanner2D

- `motion_model_for_search: DUBIN` — Dubin曲线搜索（允许倒车）
- `tolerance: 0.5m` — 规划容差
- `allow_unknown: true` — 可穿越未知区域
- `cost_travel_multiplier: 4.0` — 代价乘数（越高越走通道中心）

### 11.4 代价地图

```yaml
local_costmap:
  robot_base_frame: base_link         # 当前使用真实机体坐标系
  width: 10m, height: 10m
  resolution: 0.05m
  plugins: [static_layer, stvl_voxel_layer, inflation_layer]
  inflation:
    cost_scaling_factor: 5.0         # 代价快速衰减
    inflation_radius: 0.55m
  livox_source:
    min_obstacle_height: 0.05m       # 覆盖最低离地0.10m目标，保留5cm裕量
    expected_update_rate: 0.3s
```

### 11.5 STVL 观测源

| 源 | 话题 | 范围 | FOV |
|------|------|------|-----|
| livox | /livox/lidar_filtered | 8m | 水平360°，垂直约-7°至+52° |

当前没有 RealSense 观测源。Collision Monitor 同样消费过滤点云，并在 `base_link` 周围设置 0.55m StopZone；独立看门狗负责点云掉线时强制零速。

---

## 第十二章：常见问题与排查

### 12.1 雷达

**LIO 报 "livox_frame not found"** → 需发布 `base_link→livox_frame` 静态TF

**Livox Viewer 2 崩溃** → UE4.27+Ubuntu22.04不兼容，用老版 v0.10.0

### 12.2 串口

**打不开 /dev/ttyACM0** →
```bash
sudo apt purge modemmanager brltty
sudo usermod -a -G dialout $USER && newgrp dialout
```

**MCU收到数据但不动** → 确认MCU固件支持 Seasky 协议 (fix_control msg_id=0x05)

### 12.3 导航

**接近目标速度异常** → 检查 GoalApproachController 的 `approach_distance`、`approach_velocity`、`approach_kp`

**把自己当障碍物** → 检查 cpp_lidar_filter 的 CropBox 参数

### 12.4 仿真

**model://xxx 找不到** → source setup.bash (env-hook 设置 GZ_SIM_RESOURCE_PATH)

---

## 总结：项目的精妙之处

1. **Collision Monitor + LiDAR watchdog** — 障碍入侵和雷达掉线都能在串口前强制零速
2. **`send_merged_control()`** — 话题超时明确发送底盘、云台和发射机构零指令
3. **Seasky 双层 CRC** — CRC-8+CRC-16 双重数据完整性
4. **SerialDriver 原子 fd + 自动重连** — 支持串口热插拔
5. **GoalApproachController wrapper** — 不修改Nav2源码的增加接近减速
6. **`fake_vel_transform`** — 默认透传，可选场地坐标旋转，并提供独立命令超时清零
7. **Gazebo + ros_gz_bridge** — 真机代码 100% 复用的仿真环境

---

## 附录：项目统计

- 总包数: 15 (colcon build 通过)
- 审查轮次: 4
- 累计修复: 78+ 问题 (24 CRITICAL + 20 HIGH + 21 MEDIUM + 13 LOW)

## 参考资源

- [RM COD导航分享一](https://www.bilibili.com/video/BV1XSXZBUEYL/)
- [RM COD导航分享二](https://www.bilibili.com/video/BV1r79cBME6Y/)
- Nav2 官方文档: https://docs.nav2.org/
- Livox SDK2: https://github.com/Livox-SDK/Livox-SDK2
