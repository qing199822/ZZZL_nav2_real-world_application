# MCU Seasky 协议参考实现

> **目标读者：** 电控组 / MCU 固件开发人员  
> **配套文档：** `docs/MCU_SEASKY_PROTOCOL.md`（协议规范，先读那个）  
> **上位机代码：** `src/serial_def_sdk/uart/`（ROS2 端实现，可交叉参考）

---

## 目录结构

```
docs/mcu_reference/
├── README.md              ← 本文档 (移植指南)
├── seasky_protocol.h      ← 协议核心接口 + 数据结构定义
├── seasky_protocol.c      ← 解帧状态机 + 组帧发送 (平台无关, 不要改)
├── crc8.h / crc8.c        ← CRC-8 查表 (平台无关, 不要改)
├── crc16.h / crc16.c      ← CRC-16 查表 (平台无关, 不要改)
├── ring_buffer.h / ring_buffer.c  ← ISR-safe 环形缓冲区 (平台无关)
├── hal_uart.h             ← ★ 硬件抽象层 (需要你实现)
└── main_template.c        ← ★ 主程序模板 (从这里开始)
```

**★ = 需要电控组修改的文件**

---

## 快速起步

### 第 1 步：理解协议

先阅读 `docs/MCU_SEASKY_PROTOCOL.md`，了解：
- 帧结构：`0xA5 + Len(2B LE) + CRC8 + MsgID(2B) + Flags(2B) + Payload + CRC16(2B)`
- 消息类型：12 种，最核心的是 `fix_control (0x05)` 和 `sentry_state (0x20)`

### 第 2 步：实现 HAL 接口

创建 `hal_uart_stm32.c`（或其他 MCU 平台），实现以下 3 个函数：

```c
bool hal_uart_init(void);              // 初始化 UART/USB CDC (115200, 8N1, 启用 RX 中断)
void hal_uart_send(uint8_t *data, uint16_t len);  // 发送字节数组
uint32_t hal_get_tick_ms(void);        // 获取系统毫秒滴答
```

**STM32 HAL 示例见 `main_template.c` 末尾注释。**

### 第 3 步：配置 UART RX 中断

在 MCU 的 UART RX 中断服务函数中：

```c
void UART_RX_ISR(void) {
    uint8_t byte = USARTx->DR;           // 读取接收到的字节
    ring_buf_put_isr(&g_rx_rb, byte);       // 写入环形缓冲区
}
```

### 第 4 步：修改 main_template.c

1. 将 `main_template.c` 复制到你的工程
2. 实现所有 `★ 实现` 标记的函数（电池电量、血量、编码器读取等）
3. 实现 `apply_motor_control()` — 将接收到的速度指令转换为电机控制信号

### 第 5 步：编译和烧录

```bash
# STM32 GCC 示例
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O2 \
  main_template.c seasky_protocol.c ring_buffer.c crc8.c crc16.c \
  hal_uart_stm32.c your_other_files.c \
  -o firmware.elf

# 或直接在 STM32CubeIDE / Keil / IAR 中添加源文件
```

---

## HAL 实现指南

### STM32 (HAL 库)

```c
// hal_uart_stm32.c
#include "stm32f4xx_hal.h"
#include "hal_uart.h"
#include "ring_buffer.h"

extern UART_HandleTypeDef huart1;   // CubeMX 生成的句柄
extern RingBuffer g_rx_rb;          // 来自 main_template.c
static uint8_t rx_byte;

bool hal_uart_init(void) {
    // CubeMX 已配置 huart1 为 115200-8-N-1, 启用 UART 全局中断
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    return true;
}

void hal_uart_send(const uint8_t *data, uint16_t len) {
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 100);
}

uint32_t hal_get_tick_ms(void) {
    return HAL_GetTick();
}

// RX 中断回调 (在 stm32f4xx_it.c 中配置)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        ring_buf_put_isr(&g_rx_rb, rx_byte);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);  // 重新使能中断
    }
}
```

### STM32 USB CDC

```c
// 如果用 USB CDC 虚拟串口而不是硬件 UART:
bool hal_uart_init(void) {
    MX_USB_DEVICE_Init();  // CubeMX 生成的 USB 初始化
    return true;
}

void hal_uart_send(const uint8_t *data, uint16_t len) {
    CDC_Transmit_FS((uint8_t *)data, len);
}

// CDC 接收回调 (在 usbd_cdc_if.c 中)
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len) {
    for (uint32_t i = 0; i < *Len; i++) {
        ring_buf_put_isr(&g_rx_rb, Buf[i]);
    }
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return USBD_OK;
}
```

### ESP32 (ESP-IDF)

```c
#include "driver/uart.h"
#include "esp_timer.h"

bool hal_uart_init(void) {
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &cfg);
    uart_set_pin(UART_NUM_1, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    return true;
}

void hal_uart_send(const uint8_t *data, uint16_t len) {
    uart_write_bytes(UART_NUM_1, data, len);
}

uint32_t hal_get_tick_ms(void) {
    return esp_timer_get_time() / 1000;
}

// ESP32 RX 任务 (在 FreeRTOS 任务中, 非 ISR!)
void uart_rx_task(void *arg) {
    uint8_t byte;
    while (1) {
        if (uart_read_bytes(UART_NUM_1, &byte, 1, portMAX_DELAY) > 0) {
            // FreeRTOS 任务上下文需临界区保护, 防止与主循环并发
            taskENTER_CRITICAL();
            ring_buf_put_isr(&g_rx_rb, byte);
            taskEXIT_CRITICAL();
        }
    }
}
```

---

## 关键注意事项

### 1. 端序 (Endianness)
所有多字节值都是**小端序** (LSB 先)。ARM Cortex-M 默认是小端，如果你的 MCU 是大端需要额外处理。

### 2. 浮点格式
`float` = IEEE 754 单精度 (32-bit)。ARM Cortex-M4/M7 有硬件 FPU，确保编译器启用。

### 3. ★ 必须使用 pack/unpack 函数，禁止直接 sizeof(struct) 发包

结构体内存布局 ≠ 线格式 (ARM 上 mixed float/uint8 有 padding)。
**所有发送必须通过 pack 函数序列化。**

```c
// ✅ 正确: 使用 pack 函数
SentryState ss = { .battery=100.0f, .life=500.0f, .color=1.0f, .bullet=50.0f, .fault_flag=0.0f };
uint8_t payload[WIRE_LEN_SENTRY_STATE];           // 精确 20 字节
uint16_t len = seasky_pack_sentry_state(payload, &ss);
seasky_send_frame(&ctx, MSG_SENTRY_STATE, payload, len);

// ❌ 错误: 直接强转结构体 (ARM 上 sizeof != wire bytes!)
seasky_send_frame(&ctx, MSG_SENTRY_STATE, (uint8_t*)&ss, sizeof(ss));
```

所有消息类型都有对应的 pack/unpack 函数和 WIRE_LEN_xxx 常量，见 `seasky_protocol.h`。

### 4. 中断安全
- `ring_buf_put_isr()` 只能在 ISR（或单个任务）中调用
- `ring_buf_get()` 只能在主循环 (内部临界区保护)中调用
- 不要在 ISR 中调用 `seasky_process()`、`seasky_send_frame()` 或 `hal_uart_send()`

### 5. 性能要求
- `seasky_process()` 在主循环中的调用频率应 ≥ 1kHz（USB CDC 可能以 1Mbps+ 涌入数据）
- `hal_uart_send()` 如果使用阻塞发送，确保比帧间隔快（50Hz = 20ms/帧，31 字节/帧，115200bps 发送只需 ~2.7ms）
- ★ `periodic_send_tasks()` 总执行时间必须 < 10ms (50Hz chassis 的半周期), 否则会丢帧

### 6. 看门狗
建议在主循环中喂狗。如果 `seasky_process()` 或 `periodic_send_tasks()` 执行时间超过看门狗超时，考虑拆分为状态机。

---

## 与上位机代码的对应关系

| MCU 端 (docs/mcu_reference/) | 上位机端 (src/serial_def_sdk/) |
|------------------------------|-------------------------------|
| `seasky_protocol.c:seasky_process()` | `SeaskyProtocol.cpp:processBuffer()` |
| `seasky_protocol.c:seasky_send_frame()` | `SeaskyProtocol.cpp:sendPacket()` |
| `seasky_protocol.h:SeaskyMsgId` | `DataType.h:msg_id` |
| `seasky_protocol.h:FixControlData` | `DataType.h:FixControlData` |
| `seasky_protocol.h:SentryState` | `DataType.h:SentryState` |
| `crc8.c:crc_8()` | `crc8.c:crc_8()` (完全相同) |
| `crc16.c:crc_16()` | `crc16.c:crc_16()` (完全相同) |
| `ring_buffer.h` | (无直接对应, 上位机用 `std::vector`) |
| `hal_uart.h` | `SerialDriver.cpp` (POSIX 串口) |

---

## 调试验证

### 1. 回环测试 (MCU 独立验证)
将 MCU TX 接 RX，发送一帧后检查 CRC 校验是否通过：

```c
// 测试代码
uint8_t test_payload[] = {0x01, 0x02, 0x03};
seasky_send_frame(&g_seasky, 0x05, test_payload, 3);
// 期望: 发送的帧被自身接收并触发 on_frame_received(0x05, ...)
```

### 2. 与上位机联调
1. 插上 MCU USB, 运行 `sudo udevadm trigger` 确认 `/dev/cod_mcu` 存在
2. 启动 ROS2 导航: `ros2 launch cod_bringup singlenav_launch.py`
3. 上位机终端查看: `ros2 topic echo /hardware/gobal_information` — 应收到 sentry_state 数据
4. 发送测试指令: `ros2 topic pub /aft_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"`

### 3. 串口监视
```bash
# 在上位机端用十六进制查看原始数据
xxd /dev/cod_mcu | head -20
# 或在串口监视器中查看
minicom -D /dev/cod_mcu -b 115200 -H
```

---

## 文件依赖图

```
main_template.c
  ├── seasky_protocol.h/c       (核心协议)
  │     ├── crc8.h/c            (CRC-8)
  │     ├── crc16.h/c           (CRC-16)
  │     ├── ring_buffer.h/c     (接收缓冲)
  │     └── hal_uart.h          (发送 → hal_uart_send)
  └── hal_uart.h                (你的实现)
```

---

## 常见问题

**Q: 发送的帧上位机收不到？**
A: 检查 CRC 是否正确。用逻辑分析仪抓串口波形，对比上位机 `SeaskyProtocol::sendPacket()` 的输出。

**Q: 结构体大小不对？**
A: ★ 线格式长度 ≠ sizeof(struct)! ARM 上 mixed float/uint8 结构体有 padding。
   正确的做法是使用 WIRE_LEN_xxx 常量 + pack/unpack 函数，不要依赖 sizeof。

   如果确实需要编译期验证线格式常量与 pack 函数输出一致:
```c
// 线格式长度常量 (见 seasky_protocol.h)
_Static_assert(WIRE_LEN_HEARTBEAT_HOST ==  7, "HeartbeatHostToMcu wire len");
_Static_assert(WIRE_LEN_HEARTBEAT_MCU  ==  5, "HeartbeatMcuToHost wire len");
_Static_assert(WIRE_LEN_FIX_CONTROL    == 25, "FixControlData wire len");
_Static_assert(WIRE_LEN_SENTRY_STATE   == 20, "SentryState wire len");
_Static_assert(WIRE_LEN_GIMBAL_SEND    ==  9, "GimbalSend wire len");
_Static_assert(WIRE_LEN_GIMBAL_RECV    ==  8, "GimbalRecv wire len");

// 注意: sizeof(FixControlData) 在 ARM 上 ≈28 (6×float + uint8 + padding),
// 与线格式 25 字节不同. 因此必须使用 seasky_unpack_fix_control().
```

**Q: 缓冲区溢出导致丢帧？**
A: 增大 `ring_buffer.h` 中的 `RING_BUF_SIZE`（当前 1024 字节）。1024 足够了：50Hz × 31 字节/帧 ≈ 1.5KB/s，但如果是 IMU 数据 (200Hz × 34B = 6.8KB/s)，需要 2048+。

**Q: USB CDC 和 UART 的区别？**
A: 参考 `docs/MCU_SEASKY_PROTOCOL.md` 第 9 节。简单说 USB CDC 不需要波特率匹配，速率更高。但驱动配置更复杂。
