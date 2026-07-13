# MCU 串口通信协议规范 (Seasky 协议)

> **目标读者：** 电控组 / MCU 固件开发人员  
> **上位机端：** ROS2 Humble, serial_def_sdk (SeaskyProtocol)  
> **更新日期：** 2026-07-11

---

## 1. 串口物理层

| 参数 | 值 |
|------|-----|
| 接口 | `/dev/ttyACM0`（可通过 launch 文件配置） |
| 波特率 | **115200** |
| 数据位 | 8 |
| 校验位 | 无 (None) |
| 停止位 | 1 |
| 流控 | 无 |

---

## 2. 帧结构 (Seasky 协议)

每帧 = **10 字节头部 + N 字节载荷 + 2 字节 CRC16**

```
┌────────┬──────────────┬───────┬──────────┬──────────┬─────────────┬──────────┐
│ Header │ Payload Len  │ CRC-8 │  Msg ID  │  Flags   │  Payload    │  CRC-16  │
│  1B    │    2B LE     │  1B   │  2B LE   │  2B LE   │   N bytes   │  2B LE   │
│ 0xA5   │              │       │          │          │             │          │
└────────┴──────────────┴───────┴──────────┴──────────┴─────────────┴──────────┘
  Byte 0   Byte 1-2      Byte 3  Byte 4-5   Byte 6-7   Byte 8..N+7   N+8..N+9
```

| 字段 | 偏移 | 大小 | 说明 |
|------|------|------|------|
| Header | 0 | 1 | 固定 `0xA5` |
| Payload Len | 1 | 2 | 载荷长度 N (不含帧头尾)，小端序 |
| CRC-8 | 3 | 1 | CRC-8 校验 Byte 0~2 |
| Msg ID | 4 | 2 | 消息类型 ID，小端序 |
| Flags | 6 | 2 | 标志位，当前固定 `0x0000` |
| Payload | 8 | N | 实际数据（各消息结构见下） |
| CRC-16 | 8+N | 2 | CRC-16 校验 Byte 0 ~ (7+N)，小端序 |

**帧总长 = 10 + N 字节**

---

## 3. CRC 校验算法

### CRC-8（头部校验）

- 校验范围：帧头前 3 字节 (Header + Payload Len)
- 算法：SHT75 CRC-8 查表法
- 初始值：`0x00`
- 查找表（256 字节）：

```c
// C 代码等价实现
static const uint8_t crc8_table[256] = {
    0,   49,  98,  83,  196, 245, 166, 151, 185, 136, 219, 234, 125, 76,  31,  46,
    67,  114, 33,  16,  135, 182, 229, 212, 250, 203, 152, 169, 62,  15,  92,  109,
    134, 183, 228, 213, 66,  115, 32,  17,  63,  14,  93,  108, 251, 202, 153, 168,
    197, 244, 167, 150, 1,   48,  99,  82,  124, 77,  30,  47,  184, 137, 218, 235,
    61,  12,  95,  110, 249, 200, 155, 170, 132, 181, 230, 215, 64,  113, 34,  19,
    126, 79,  28,  45,  186, 139, 216, 233, 199, 246, 165, 148, 3,   50,  97,  80,
    187, 138, 217, 232, 127, 78,  29,  44,  2,   51,  96,  81,  198, 247, 164, 149,
    248, 201, 154, 171, 60,  13,  94,  111, 65,  112, 35,  18,  133, 180, 231, 214,
    122, 75,  24,  41,  190, 143, 220, 237, 195, 242, 161, 144, 7,   54,  101, 84,
    57,  8,   91,  106, 253, 204, 159, 174, 128, 177, 226, 211, 68,  117, 38,  23,
    252, 205, 158, 175, 56,  9,   90,  107, 69,  116, 39,  22,  129, 176, 227, 210,
    191, 142, 221, 236, 123, 74,  25,  40,  6,   55,  100, 85,  194, 243, 160, 145,
    71,  118, 37,  20,  131, 178, 225, 208, 254, 207, 156, 173, 58,  11,  88,  105,
    4,   53,  102, 87,  192, 241, 162, 147, 189, 140, 223, 238, 121, 72,  27,  42,
    193, 240, 163, 146, 5,   52,  103, 86,  120, 73,  26,  43,  188, 141, 222, 239,
    130, 179, 224, 209, 70,  119, 36,  21,  59,  10,  89,  104, 255, 206, 157, 172
};

uint8_t crc_8(const uint8_t *data, uint16_t len) {
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++) {
        crc = crc8_table[data[i] ^ crc];
    }
    return crc;
}
```

### CRC-16（帧尾校验）

- 校验范围：从帧头到载荷末尾（不含 CRC-16 自身）
- 算法：CRC-16-IBM (Modbus)，查表法
- 多项式：`0xA001`
- 初始值：`0xFFFF`

```c
// C 代码等价实现
#define CRC_POLY_16 0xA001
#define CRC_START_16 0xFFFF

// 查找表需在首次调用前初始化
static uint16_t crc16_table[256];

void init_crc16_table(void) {
    for (uint16_t i = 0; i < 256; i++) {
        uint16_t crc = 0;
        uint16_t c = i;
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc ^ c) & 0x0001)
                crc = (crc >> 1) ^ CRC_POLY_16;
            else
                crc = crc >> 1;
            c = c >> 1;
        }
        crc16_table[i] = crc;
    }
}

uint16_t crc_16(const uint8_t *data, uint16_t len) {
    uint16_t crc = CRC_START_16;
    for (uint16_t i = 0; i < len; i++) {
        crc = (crc >> 8) ^ crc16_table[(crc ^ (uint16_t)data[i]) & 0x00FF];
    }
    return crc;
}
```

### 校验步骤（MCU 接收端）

```
1. 在字节流中找到 0xA5 (帧头同步)
2. 读 Payload Len (字节1-2, LE), 计算总帧长 = 10 + Len
3. 用 CRC-8 验证字节 0-2 (Header + Len)
   如果失败 → 丢弃字节0, 回到步骤1
4. 等待接收完整帧 (总帧长字节)
5. 用 CRC-16 验证字节 0 到 (总帧长-3)
   如果失败 → 丢弃整帧
6. 提取 Msg ID 和 Payload, 分发处理
```

---

## 4. 消息 ID 汇总

| Msg ID | 名称 | 方向 | 载荷长度 | 说明 |
|--------|------|------|----------|------|
| `0x01` | heartbeat | 双向 | 7B(发)/5B(收) | 心跳包 |
| `0x02` | gimbal | 双向 | 9B(发)/8B(收) | 云台控制/回传 |
| `0x03` | control | 发→收 | 5B | 视觉控制 |
| `0x04` | additional | 收→发 | 5B | 附加状态 |
| **`0x05`** | **fix_control** | **发→收** | **21B** | **导航+云台合并包 (核心)** |
| `0x06` | imu2 | 收→发 | 24B | IMU 数据 |
| `0x07` | yaw | 收→发 | 4B | Yaw 角度 |
| `0x08` | game_status | 收→发 | 3B | 比赛状态 |
| `0x10` | chassis | 双向 | 12B | 底盘速度 |
| `0x20` | sentry_state | 收→发 | 20B | 哨兵状态 |

> **方向说明：** "发→收" = ROS2 上位机 发送, MCU 接收  
> "收→发" = MCU 发送, ROS2 上位机 接收

---

## 5. 逐消息详细定义

### 5.1 fix_control (0x05) — 导航控制包（最重要）

**方向：** ROS2 → MCU  
**发送频率：** 50Hz（每 20ms）  
**载荷长度：** 21 字节  

```
┌──────────┬──────────┬──────────┬──────────┬──────────┬──────────┬──────────┐
│   yaw    │  pitch   │   fire   │    vx    │    vy    │    vz    │   spin   │
│ float LE │ float LE │  uint8   │ float LE │ float LE │ float LE │ float LE │
│  4B      │  4B      │  1B      │  4B      │  4B      │  4B      │  4B      │
└──────────┴──────────┴──────────┴──────────┴──────────┴──────────┴──────────┘
 Byte 0-3   Byte 4-7   Byte 8     Byte 9-12  Byte 13-16 Byte 17-20 Byte 21-24
                                                                  (载荷内偏移)
```

| 字段 | 偏移 | 类型 | 单位 | 说明 |
|------|------|------|------|------|
| yaw | 0 | float32 | rad | 云台偏航角目标值 |
| pitch | 4 | float32 | rad | 云台俯仰角目标值 |
| fire | 8 | uint8 | — | 开火指令 (0=停火, 1=开火) |
| vx | 9 | float32 | m/s | 底盘 X 轴线速度 (前向为正) |
| vy | 13 | float32 | m/s | 底盘 Y 轴线速度 (左向为正) |
| vz | 17 | float32 | m/s | 底盘 Z 轴线速度（导航不使用，填 0） |
| spin | 21 | float32 | rad/s | 小陀螺自旋角速度 (不使用则 MCU 忽略) |

> **注意：** vz 字段目前导航系统发送 0（转向由 MCU 差速控制）。  
> **字节序：** 所有多字节值均为小端序 (Little Endian)。  
> **浮点格式：** IEEE 754 单精度。

**完整帧示例（21字节载荷）：**
```
A5 15 00 XX 05 00 00 00  [yaw:4B] [pitch:4B] [fire:1B] [vx:4B] [vy:4B] [vz:4B] [spin:4B]  [CRC16:2B]
│   │     │  │     │      └──────────────── 21 bytes payload ────────────────────┘
│   │     │  │     └─ Flags = 0x0000
│   │     │  └─ Msg ID = 0x0005
│   │     └─ CRC-8
│   └─ Payload Len = 0x0015 = 21
└─ Header = 0xA5
```

### 5.2 heartbeat (0x01) — 发送方向 (ROS2 → MCU)

**载荷长度：** 7 字节  

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | timestamp | uint16 LE | 时间戳 (秒) |
| 2 | battery | uint8 | 电池电量 |
| 3 | life | uint8 | 剩余生命值 |
| 4 | color | uint8 | 队伍颜色 (0=未知, 1=蓝, 2=紫, 3=红) |
| 5 | bullet | uint8 | 剩余弹量 |
| 6 | fault_flag | uint8 | 故障标志 |

### 5.3 heartbeat (0x01) — 接收方向 (MCU → ROS2)

**载荷长度：** 5 字节  

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | timestamp | uint16 LE | 时间戳 |
| 2 | set_launch | uint8 | 发射机构已启用 |
| 3 | set_arm | uint8 | 摩擦轮已启动 |
| 4 | fault_flag | uint8 | 故障标志位 |

### 5.4 gimbal (0x02) — 发送方向 (ROS2 → MCU)

**载荷长度：** 9 字节  

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | yaw | float32 LE | 偏航角 (rad) |
| 4 | pitch | float32 LE | 俯仰角 (rad) |
| 8 | fire | uint8 | 开火 (0/1) |

### 5.5 gimbal (0x02) — 接收方向 (MCU → ROS2)

**载荷长度：** 8 字节  

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | yaw | float32 LE | 当前偏航角 (rad) |
| 4 | pitch | float32 LE | 当前俯仰角 (rad) |

> **注意：** 接收方向只有 8 字节（无 fire 字段），发送方向是 9 字节（有 fire）。

### 5.6 chassis (0x10) — 双向

**载荷长度：** 12 字节（双向相同）

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | vx | float32 LE | X 轴线速度 (m/s) |
| 4 | vy | float32 LE | Y 轴线速度 (m/s) |
| 8 | vz | float32 LE | Z 轴角速度 (rad/s) |

> 发送方向 (ROS2 → MCU): 单独发送底盘速度（一般不使用，优先用 fix_control）  
> 接收方向 (MCU → ROS2): 回传当前底盘速度  

### 5.7 sentry_state (0x20) — 接收方向 (MCU → ROS2)

**载荷长度：** 20 字节  
**发送频率：** ≥5Hz（ROS2 端以 5Hz 读取）

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | battery | float32 LE | 电池电量百分比 |
| 4 | life | float32 LE | 当前血量 |
| 8 | color | float32 LE | 队伍颜色 |
| 12 | bullet | float32 LE | 剩余弹量 |
| 16 | fault_flag | float32 LE | 故障标志 |

> **重要：** 载荷长度必须严格为 20 字节。之前 MCU 固件曾错误写入 22 字节，导致 CRC 校验失败，上位机丢弃所有哨兵状态帧。

### 5.8 imu2 (0x06) — 接收方向 (MCU → ROS2)

**载荷长度：** 24 字节  

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | vx | float32 LE | 角速度 X |
| 4 | vy | float32 LE | 角速度 Y |
| 8 | vz | float32 LE | 角速度 Z |
| 12 | ax | float32 LE | 线加速度 X |
| 16 | ay | float32 LE | 线加速度 Y |
| 20 | az | float32 LE | 线加速度 Z |

### 5.9 yaw (0x07) — 接收方向 (MCU → ROS2)

**载荷长度：** 4 字节  

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | yaw | float32 LE | 云台偏航角度 (rad) |

### 5.10 control (0x03) — 发送方向 (ROS2 → MCU)

**载荷长度：** 5 字节  

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | velocity_top | float32 LE | 摩擦轮线速度上限 |
| 4 | shoot | uint8 | 射击指令 |

### 5.11 additional (0x04) — 接收方向 (MCU → ROS2)

**载荷长度：** 5 字节  

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | launch | uint8 | 发射机构是否就绪 |
| 1 | arm | uint8 | 摩擦轮是否启动 |
| 2 | base_hp_our | uint8 | 我方基地血量 (%) |
| 3 | base_hp_enemy | uint8 | 敌方基地血量 (%) |
| 4 | judge_warning_level | uint8 | 裁判系统警告等级 |

### 5.12 game_status (0x08) — 接收方向 (MCU → ROS2)

**载荷长度：** 3 字节  

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | stage_remain_time | uint16 LE | 阶段剩余时间 (s) |
| 2 | game_progress | uint8 | 比赛进度 |

---

## 6. MCU 固件实现要求

### 6.1 必须实现的消息

| 优先级 | Msg ID | 方向 | 说明 |
|--------|-------|------|------|
| **P0** | `0x05` fix_control | 接收 | 导航控制核心包，50Hz，必须正确解析 |
| **P0** | `0x20` sentry_state | 发送 | 哨兵状态回传，≥5Hz，长度必须严格 20 字节 |
| P1 | `0x01` heartbeat | 双向 | 心跳保活 |
| P1 | `0x02` gimbal | 双向 | 云台控制与回传 |
| P2 | `0x07` yaw | 发送 | 云台角度回传 |
| P3 | `0x06` imu2 | 发送 | IMU 数据回传 (可选) |
| P3 | `0x04` additional | 发送 | 附加状态 |
| P3 | `0x08` game_status | 发送 | 比赛状态 |
| P3 | `0x10` chassis | 发送 | 底盘速度回传 |
| P3 | `0x03` control | 接收 | 视觉控制指令 |

### 6.2 接收处理逻辑

```
串口接收中断/线程:
  while (有数据):
    字节放入接收缓冲区
    
    主循环:
      if 缓冲区空: 等待
      if 缓冲区[0] != 0xA5: 丢弃1字节, continue
      if 缓冲区长度 < 3: 等待更多数据
      
      payload_len = 缓冲区[1] | (缓冲区[2] << 8)
      if payload_len > 200: 丢弃1字节, continue   // 异常长度
      
      if CRC8(缓冲区[0..2]) != 缓冲区[3]: 丢弃1字节, continue
      
      总帧长 = 10 + payload_len
      if 缓冲区长度 < 总帧长: 等待更多数据
      
      if CRC16(缓冲区[0..总帧长-3]) != 最后2字节: 丢弃整帧, continue
      
      msg_id = 缓冲区[4] | (缓冲区[5] << 8)
      
      switch(msg_id):
        case 0x05:  // fix_control
          读取 payload(21B) → 更新电机目标速度 + 云台角度 + 开火
        case 0x01:  // heartbeat
          读取 payload(7B) → 更新心跳状态
        case 0x02:  // gimbal
          读取 payload(9B) → 更新云台目标
        case 0x03:  // control
          读取 payload(5B) → 更新摩擦轮速度
        case 0x10:  // chassis
          读取 payload(12B) → 更新底盘速度 (备用)
      
      移除已处理帧, 继续处理下一帧
```

### 6.3 发送要求 (MCU → ROS2)

| 消息 | 频率 | 说明 |
|------|------|------|
| sentry_state (0x20) | ≥ 5Hz | 必须严格 20 字节载荷 |
| yaw (0x07) | ≥ 10Hz | 推荐 100Hz |
| gimbal (0x02) | ≥ 10Hz | 云台当前角度 |
| heartbeat (0x01) | ≥ 2Hz | 心跳 |
| imu2 (0x06) | 50-200Hz | IMU 原始数据 |
| chassis (0x10) | 50-100Hz | 底盘实际速度 |

### 6.4 数据字节序速查

| 类型 | 字节数 | 字节序 |
|------|--------|--------|
| uint8 | 1 | — |
| uint16 | 2 | 小端 (LSB 先) |
| float32 | 4 | IEEE 754, 小端 |

### 6.5 帧发送函数伪代码

```c
// MCU 发送端伪代码
void send_sentry_state(float battery, float life, float color,
                       float bullet, float fault_flag) {
    uint8_t payload[20];
    memcpy(payload + 0,  &battery,    4);  // float32 LE
    memcpy(payload + 4,  &life,       4);
    memcpy(payload + 8,  &color,      4);
    memcpy(payload + 12, &bullet,     4);
    memcpy(payload + 16, &fault_flag, 4);

    seasky_send_frame(0x20, payload, 20);  // 内部完成组帧+CRC+发送
}

void seasky_send_frame(uint16_t msg_id, uint8_t *payload, uint16_t len) {
    uint8_t frame[256];
    int pos = 0;

    frame[pos++] = 0xA5;                    // Header
    frame[pos++] = len & 0xFF;              // Len LSB
    frame[pos++] = (len >> 8) & 0xFF;       // Len MSB
    frame[pos++] = crc_8(frame, 3);         // CRC-8 of first 3 bytes
    frame[pos++] = msg_id & 0xFF;           // Msg ID LSB
    frame[pos++] = (msg_id >> 8) & 0xFF;    // Msg ID MSB
    frame[pos++] = 0x00;                    // Flags LSB
    frame[pos++] = 0x00;                    // Flags MSB
    memcpy(frame + pos, payload, len);      // Payload
    pos += len;
    uint16_t crc = crc_16(frame, pos);      // CRC-16 of everything before it
    frame[pos++] = crc & 0xFF;
    frame[pos++] = (crc >> 8) & 0xFF;

    uart_send(frame, pos);                  // 写入串口
}
```

---

## 7. 旧协议对照 (ros2_simple_serial，已废弃)

> 仅供过渡期参考，新固件请忽略。

旧协议：15 字节裸包，无帧头帧尾，无 CRC。

```
[A5][vx:float LE 4B][vy:float LE 4B][vz:float LE 4B][checksum:uint16 LE 2B]
```

校验方式：简单累加和（非标准 CRC）。

---

## 8. 联系与调试

- 上位机日志话题：`/hardware_serial` 节点的 ROS2 日志
- 串口调试工具：`minicom -D /dev/ttyACM0 -b 115200`（需先停 ROS2 节点）
- 数据流路径：`/cmd_vel → fake_vel_transform → /aft_cmd_vel → serial_def_sdk → /dev/ttyACM0 → MCU`

---

**上位机协议实现参考文件：**

| 文件 | 内容 |
|------|------|
| `src/serial_def_sdk/uart/SeaskyProtocol.cpp` | 组帧/解帧、CRC 校验 |
| `src/serial_def_sdk/uart/DataType.h` | 消息 ID 枚举、结构体定义 |
| `src/serial_def_sdk/uart/crc8.c` | CRC-8 查表实现 |
| `src/serial_def_sdk/uart/crc16.c` | CRC-16 查表实现 |
| `src/serial_def_sdk/uart/SerialDriver.cpp` | 串口打开/配置/读写 |

---

## 9. USB 虚拟串口与 udev 设备名固定

### 9.1 USB CDC ACM 说明

当前使用 USB CDC ACM 虚拟串口（`/dev/ttyACM0`），而非传统物理 UART。USB CDC 的特点：

| 特性 | USB CDC ACM | 物理 UART (ttyS/USB转串口) |
|------|-------------|---------------------------|
| 理论带宽 | 12Mbps (FS) / 480Mbps (HS) | 115200-921600 bps |
| 波特率 | 忽略 — USB 总线调度决定实际速度 | 必须双方一致 |
| 接线 | 一根 USB 线 (供电+通信) | TX/RX/GND 至少 3 根 |
| 热插拔 | 支持, 但设备号可能漂移 | 不适用 |
| 延迟 | 受 USB 1ms 帧轮询间隔影响 | 稳定, 与波特率相关 |
| 驱动 | 内核 cdc_acm 模块, 免驱 | FTDI/CH340 可能需装驱动 |

**风险:** USB 重新枚举时设备号可能从 `ttyACM0` 漂移到 `ttyACM1`, `ttyACM2`...  
**解决:** 使用 udev 规则创建固定符号链接 `/dev/cod_mcu`

### 9.2 安装 udev 规则

项目提供了 udev 规则文件 `config/99-cod-mcu-serial.rules`，安装方法：

```bash
# 1. 复制规则文件
sudo cp config/99-cod-mcu-serial.rules /etc/udev/rules.d/

# 2. 重新加载规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 3. 验证 (插拔 MCU USB 后)
ls -la /dev/cod_mcu
# 输出: lrwxrwxrwx ... /dev/cod_mcu -> ttyACM0
```

### 9.3 配置规则（三选一）

编辑 `/etc/udev/rules.d/99-cod-mcu-serial.rules`，根据实际情况选择合适的匹配策略：

**策略 A — 按 USB 端口路径（推荐，无需知道 VID/PID）:**
```bash
# 1. 找到 MCU 的物理端口路径
udevadm info --name=/dev/ttyACM0 --attribute-walk | grep KERNELS
# 输出示例: KERNELS=="1-1.2"

# 2. 取消注释并修改规则文件中的对应行:
SUBSYSTEM=="tty", KERNELS=="1-1.2", SYMLINK+="cod_mcu", MODE="0666"
```

**策略 B — 按 VID/PID:**
```bash
# 1. 查找 MCU 的 USB ID
lsusb | grep -i stm  # 或其他 MCU 厂商
# 输出示例: Bus 001 Device 005: ID 0483:5740 STMicroelectronics ...

# 2. 取消注释并填写:
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="cod_mcu", MODE="0666"
```

**策略 C — 按设备序列号:**
```bash
# 1. 查找序列号
udevadm info --name=/dev/ttyACM0 --attribute-walk | grep serial

# 2. 取消注释并填写:
SUBSYSTEM=="tty", ATTRS{serial}=="XXXXXXXXXXXX", SYMLINK+="cod_mcu", MODE="0666"
```

### 9.4 权限说明

udev 规则同时设置了 `GROUP="dialout"`, `MODE="0660"`，确保 `dialout` 组成员可读写串口。运行导航前确认当前用户在 dialout 组中：

```bash
groups $USER  # 应包含 "dialout"
# 如果没有, 执行: sudo usermod -a -G dialout $USER  然后重新登录
```

### 9.5 上位机配置变更

所有 launch 文件的默认串口已从 `/dev/ttyUSB0` 改为 `/dev/cod_mcu`：

| 文件 | 变更 |
|------|------|
| `serial_def_sdk/launch/serial.launch.py` | `serial_port: "/dev/cod_mcu"` |
| `cod_bringup/launch/singlenav_launch.py` | `serial_port: "/dev/cod_mcu"` |
| `cod_bringup/launch/multiplenav_launch.py` | `serial_port: "/dev/cod_mcu"` |

如需覆盖，在 launch 命令行指定参数：`serial_port:=/dev/ttyACM1`
