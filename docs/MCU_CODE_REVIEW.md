# 电控 Seasky 协议代码审查报告

> **审查文件:** `seasky.h`, `seasky.c`
> **对照基准:** ROS2 上位机 `SeaskyProtocol.cpp` + 协议规范 `MCU_SEASKY_PROTOCOL.md`
> **日期:** 2026-07-13

---

## 总览

| 级别 | 数量 | 阻塞? |
|------|------|-------|
| CRITICAL | 3 | 是 — 修复前导航控制帧全部丢失 |
| HIGH | 3 | — |
| MEDIUM | 4 | — |

---

## 🔴 CRITICAL（必须修复，否则串口通信不工作）

### C1. fix_control 载荷长度错误：21 应改为 25

**文件:** `seasky.c` 第 100-106 行

**当前代码:**
```c
case MSG_ID_FIX_CONTROL:
    if (payload_len == 21)          // ❌ 错误！ROS2 发送的是 25 字节
    {
        memcpy(&latest_fix_ctrl, payload, 21);  // ❌ 只拷贝了 21 字节，spin 字段丢失
```

**问题:** ROS2 上位机实际发送 **25 字节**，MCU 期望 21 字节，导致所有 fix_control 帧 CRC-16 校验通过后被 `frame_dispatch` 静默丢弃。**机器人永远不会收到导航指令。**

**ROS2 端证据** (`SeaskyProtocol.cpp` 第 89-103 行):
```
yaw(4B) + pitch(4B) + fire(1B) + vx(4B) + vy(4B) + vz(4B) + spin(4B) = 25 字节
```

**修复后代码:**
```c
case MSG_ID_FIX_CONTROL:
    if (payload_len == 25)          // 7 个字段: 6×float32 + 1×uint8 = 25 bytes
    {
        memcpy(&latest_fix_ctrl, payload, 25);
```

> 附注: 协议文档 §5.1 中文字写"21 字节"是笔误，ASCII 表格中 Byte 编号从 0 到 24 共 25 字节是正确的。以 ROS2 实际发送的 25 字节为准。

---

### C2. CRC-16 查找表惰性初始化存在竞态条件

**文件:** `seasky.c` 第 40-42, 61-68 行

```c
static uint16_t crc16_table[256];
static uint8_t  crc16_ready = 0;

static void crc16_init(void) {
    // 运行时计算 256 个表项...
    crc16_ready = 1;   // ❌ 非原子操作，无内存屏障
}

static uint16_t crc_16(const uint8_t *data, uint16_t len) {
    if (!crc16_ready) crc16_init();  // ❌ 竞态窗口
    ...
}
```

**问题:** 若接收 ISR 和主循环发送线程同时触发首次 `crc_16()` 调用，一个线程正在写表，另一个线程同时读取半初始化的表，CRC 计算结果不可预测。正确帧可能被误判为损坏帧丢弃。

**修复方案:** 将 CRC-16 查找表改为编译期静态常量（消除运行时初始化）。完整 256 项静态表见本文末尾 **附件 A**。

---

### C3. 缺少 IMU 和 Yaw 发送函数（陀螺仪数据上报）

**文件:** `seasky.c` / `seasky.h`

**需求:** 设备陀螺仪缺失部分数据直接输出 0。

当前代码中 `MSG_ID_IMU2 (0x06)` 和 `MSG_ID_YAW (0x07)` 只有宏定义，**没有对应的发送函数**。ROS2 上位机 `dispatchMessage()` 已准备好接收这两种消息。

**需要新增的代码:**

`seasky.h` 添加声明:
```c
/* 发送 IMU 数据 (陀螺仪+加速度计, 缺失轴填 0.0f) */
void Seasky_SendImu(float vx, float vy, float vz,
                     float ax, float ay, float az);

/* 发送云台 Yaw 角度 (rad) */
void Seasky_SendYaw(float yaw);
```

`seasky.c` 添加实现:
```c
void Seasky_SendImu(float vx, float vy, float vz,
                     float ax, float ay, float az)
{
    uint8_t p[24];
    memcpy(p + 0,  &vx, 4);   /* 角速度 X, rad/s */
    memcpy(p + 4,  &vy, 4);   /* 角速度 Y, rad/s */
    memcpy(p + 8,  &vz, 4);   /* 角速度 Z, rad/s */
    memcpy(p + 12, &ax, 4);   /* 线加速度 X, m/s² (无加速度计则填 0.0f) */
    memcpy(p + 16, &ay, 4);   /* 线加速度 Y, m/s² (无加速度计则填 0.0f) */
    memcpy(p + 20, &az, 4);   /* 线加速度 Z, m/s² (无加速度计则填 0.0f) */
    seasky_send_frame(MSG_ID_IMU2, p, 24);
}

void Seasky_SendYaw(float yaw)
{
    uint8_t p[4];
    memcpy(p, &yaw, 4);
    seasky_send_frame(MSG_ID_YAW, p, 4);
}
```

**主循环调用示例:**
```c
// 推荐 50-100Hz 周期发送
Seasky_SendImu(gyro_x, gyro_y, gyro_z,    // 陀螺仪角速度 (rad/s)
               0.0f,   0.0f,   0.0f);      // 加速度计: 没有就填 0
Seasky_SendYaw(current_yaw);               // 云台当前偏航角 (rad)
```

---

## 🟡 HIGH

### H1. 缺少 ROS2→MCU 方向其他消息的接收处理

**文件:** `seasky.c` 第 97-109 行 `frame_dispatch()`

当前只处理 `MSG_ID_FIX_CONTROL (0x05)`。ROS2 还可能发送以下消息:

| MsgID | 名称 | 载荷长度 | 用途 |
|-------|------|----------|------|
| 0x01 | heartbeat | 7B | 心跳包 (timestamp+电池+血量+颜色+弹量+故障) |
| 0x02 | gimbal | 9B | 云台目标角度 (yaw:4B + pitch:4B + fire:1B) |
| 0x03 | control | 5B | 摩擦轮速度上限 + 射击指令 |
| 0x10 | chassis | 12B | 备用底盘速度 (vx/vy/vz, 各 float32) |

**修复建议:** 至少添加 heartbeat (0x01) 的处理，确保心跳链路能闭环。其余按需实现。

---

### H2. MCU→ROS2 发送函数清单（部分缺失）

| 消息 | MsgID | 载荷 | 状态 |
|------|-------|------|------|
| heartbeat (心跳响应) | 0x01 | 5B | ✅ 已实现 `Seasky_SendHeartbeat` |
| chassis (底盘速度) | 0x10 | 12B | ✅ 已实现 `Seasky_SendChassis` |
| sentry_state (哨兵) | 0x20 | 20B | ✅ 已实现 `Seasky_SendSentry` |
| **imu2 (IMU)** | **0x06** | **24B** | ❌ 缺失 → 见 C3 |
| **yaw (云台角度)** | **0x07** | **4B** | ❌ 缺失 → 见 C3 |
| gimbal (云台回传) | 0x02 | 8B | ❌ 缺失 |
| additional (附加状态) | 0x04 | 5B | ❌ 缺失 |
| game_status (比赛) | 0x08 | 3B | ❌ 缺失 |

最少需要补充 **IMU** 和 **Yaw**。其余四项目前 ROS2 端有接收逻辑但非必须，后续按需补充。

---

### H3. memcpy 按结构体布局序列化 — 依赖平台约定

**当前做法:**
```c
memcpy(&latest_fix_ctrl, payload, 25);   // 假设线格式 == 内存布局
memcpy(p + 0, &vx, 4);                    // 假设 float 布局 == IEEE 754 LE
```

在 ARM Cortex-M (little-endian) + `__attribute__((packed))` 下可以正常工作，但参考代码 `docs/mcu_reference/seasky_protocol.h` 提供了显式的 `seasky_read_f32_le()` / `seasky_write_f32_le()` 序列化函数，消除隐式平台假设。短期可不改，若后续移植到其他平台需注意。

---

## 🔵 MEDIUM

### M1. HAL_GetTick() 溢出导致 fix_control 超时误判

**文件:** `seasky.c` 第 202 行

```c
if (fix_ctrl_valid && (HAL_GetTick() - fix_ctrl_tick) > 200)
```

32-bit ms 计数器约 49.7 天回绕到 0。回绕瞬间差值变为大正数，`fix_ctrl_valid` 被错误清零（持续 200ms 后自动恢复）。对单场比赛无影响，但若设备长期不断电运行会有一次短暂误判。

**修复（可选）:** 将减法结果强制转为 `uint32_t` 即可天然处理溢出，无需额外逻辑。

---

### M2. 无 payload 最小长度检查

`seasky.c` 第 149 行只检查 `plen > SEASKY_PAYLOAD_MAX` (200)，不检查下限。理论上可能接受 payload_len=0 的帧（无实际危害但无意义）。防御性编程建议加 `plen == 0` 检查。

---

### M3. CRC-8 校验失败后重同步不彻底

`seasky.c` 第 135-143 行的重同步逻辑仅在 `rx_buf[1..3]` 中搜索 `0xA5`。若 payload_len 低字节恰好等于 0xA5 (165)，会误触发同步，导致丢掉一帧。这是小概率事件（1/256），可后续优化。

---

### M4. `frame_dispatch` 未用 `total_len` 做一致性校验

函数签名已传入 `total_len`，但内部又自行从 `frame` 重新解析 `payload_len`，未校验 `total_len == 10 + payload_len`。调用者已保证一致性，防御性编程建议添加断言。

---

## 📋 修复优先级

```
P0 — C1  fix_control 25 字节           ← 阻塞级，改 1 行
P0 — C3  新增 Seasky_SendImu / Yaw     ← 用户需求，新增 ~30 行
P1 — C2  CRC-16 改为静态表             ← 稳定性，替换表定义
P1 — H1  补充接收 case                 ← 功能性，加骨架
P2 — H2  补充其他发送函数               ← 按需
P3 — H3 / M1-M4  加固                  ← 可后续迭代
```

---

## 📎 附件 A: CRC-16 静态查找表

将 `seasky.c` 中的 `crc16_table[256]`、`crc16_ready`、`crc16_init()` 全部删除，替换为以下编译期常量:

```c
/* CRC-16-IBM 查找表 (poly=0xA001, 编译期常量 — 无需运行时初始化) */
static const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/* crc_16 函数保持不变，删除 if (!crc16_ready) 检查 */
static uint16_t crc_16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
        crc = (crc >> 8) ^ crc16_table[(crc ^ (uint16_t)data[i]) & 0x00FF];
    return crc;
}
```

同时删除 `Seasky_Init()` 中的 `if (!crc16_ready) crc16_init();` 行（第 185 行）。

---

## 📎 附件 B: fix_control (0x05) 载荷完整格式

| 偏移 | 字段 | 类型 | 大小 | 说明 |
|------|------|------|------|------|
| 0 | yaw | float32 LE | 4B | 云台偏航角目标值 (rad) |
| 4 | pitch | float32 LE | 4B | 云台俯仰角目标值 (rad) |
| 8 | fire | uint8 | 1B | 开火指令 (0=停火, 1=开火) |
| 9 | vx | float32 LE | 4B | 底盘 X 轴线速度 (m/s, 前向为正) |
| 13 | vy | float32 LE | 4B | 底盘 Y 轴线速度 (m/s, 左向为正) |
| 17 | vz | float32 LE | 4B | 底盘 Z 轴线速度 (导航填 0, MCU 忽略) |
| 21 | spin | float32 LE | 4B | 小陀螺自旋角速度 (rad/s, 导航填 0) |
| **合计** | | | **25B** | |

---

## 📎 附件 C: 完整消息 ID 速查表

```
帧格式: 0xA5 + PayloadLen(2B LE) + CRC8(1B) + MsgID(2B LE) + Flags(2B LE) + Payload(N B) + CRC16(2B LE)
总帧长 = 10 + N 字节
```

| MsgID | 名称 | 方向 | 载荷长度 |
|-------|------|------|----------|
| 0x01 | heartbeat | 双向 | 上位机→MCU: 7B / MCU→上位机: 5B |
| 0x02 | gimbal | 双向 | 上位机→MCU: 9B / MCU→上位机: 8B |
| 0x03 | control | 上位机→MCU | 5B |
| 0x04 | additional | MCU→上位机 | 5B |
| **0x05** | **fix_control** | **上位机→MCU** | **25B** |
| 0x06 | imu2 | MCU→上位机 | 24B |
| 0x07 | yaw | MCU→上位机 | 4B |
| 0x08 | game_status | MCU→上位机 | 3B |
| 0x10 | chassis | 双向 | 12B |
| 0x20 | sentry_state | MCU→上位机 | 20B |
