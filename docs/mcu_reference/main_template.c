/**
 * @file    main_template.c
 * @brief   MCU 主程序模板 — 完整的使用示例
 *
 * 演示如何使用 Seasky 协议栈 (v2 — 使用 pack/unpack API):
 *   1. 接收上位机的 fix_control (0x05) — 驱动电机
 *   2. 周期性发送 sentry_state (0x20) — 回传机器人状态
 *   3. 周期性发送 yaw (0x07)            — 回传云台角度
 *   4. 周期性发送 gimbal (0x02)          — 回传云台状态
 *   5. 周期性发送 chassis (0x10)         — 回传底盘速度
 *   6. 周期性发送 heartbeat (0x01)       — 心跳
 *
 * ★ 所有发送均使用 seasky_pack_xxx() 序列化为精确线格式,
 *   不再直接 memcpy 结构体内存 (避免 ARM padding 问题).
 *
 * 电控组将此文件作为起点, 根据实际 MCU 平台修改 HAL 实现和业务逻辑。
 *
 * 硬件依赖:
 *   - UART/USB CDC 外设 (115200, 8N1)
 *   - 系统滴答定时器 (SysTick 或等效, 用于定时)
 *   - 电机控制接口 (PWM/CAN/...)
 *   - 传感器接口 (IMU, 云台编码器, ...)
 *
 * 编译说明 (GCC + STM32 示例):
 *   arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O2 \
 *     main_template.c seasky_protocol.c ring_buffer.c crc8.c crc16.c \
 *     hal_uart_stm32.c -o firmware.elf
 */

#include "seasky_protocol.h"
#include "hal_uart.h"
#include <string.h>
#include <stdbool.h>

/* ========================================================================
 * 全局对象
 * ======================================================================== */

static RingBuffer       g_rx_rb;         /* 接收环形缓冲区 */
static SeaskyContext    g_seasky;        /* 协议上下文 */

/* 定时状态 */
static uint32_t g_last_sentry_time;      /* 上次发送 sentry_state 的时间戳 */
static uint32_t g_last_yaw_time;         /* 上次发送 yaw 的时间戳 */
static uint32_t g_last_gimbal_time;      /* 上次发送 gimbal 的时间戳 */
static uint32_t g_last_chassis_time;     /* 上次发送 chassis 的时间戳 */
static uint32_t g_last_heartbeat_time;   /* 上次发送 heartbeat 的时间戳 */

/* 接收到的控制数据 (只在主循环中更新, ISR 安全) */
static FixControlData g_cmd = {0};       /* 最新的 fix_control */
static bool           g_cmd_updated = false;

/* ========================================================================
 * 帧接收回调 — 上位机 → MCU
 * ======================================================================== */

/**
 * @brief 收到完整帧时被 seasky_process() 调用
 *
 * 根据 msg_id 使用 unpack 函数解析载荷, 更新对应数据结构.
 * 注意: payload 指向 ctx 内部缓冲区, 回调返回后即失效 —
 *       如需持久保存, 需要在此函数内 memcpy.
 */
static void on_frame_received(uint16_t msg_id, uint16_t flags,
                              const uint8_t *payload, uint16_t len) {
    (void)flags;  /* 当前未使用, 预留扩展 */

    switch (msg_id) {

    case MSG_FIX_CONTROL:  /* ★ 导航+云台合并包 */
        if (seasky_unpack_fix_control(&g_cmd, payload, len)) {
            g_cmd_updated = true;

            /* ★ 在这里将 vx, vy, vz, spin 写入电机控制寄存器 ★ */
            /* set_motor_target(g_cmd.vx, g_cmd.vy, g_cmd.vz, g_cmd.spin); */
            /* set_gimbal_target(g_cmd.yaw, g_cmd.pitch); */
            /* if (g_cmd.fire) trigger_shoot(); */
        }
        break;

    case MSG_GIMBAL:  /* 单独的云台控制 */
        {
            GimbalSend gs;
            if (seasky_unpack_gimbal_send(&gs, payload, len)) {
                /* set_gimbal_target(gs.yaw, gs.pitch); */
            }
        }
        break;

    case MSG_CONTROL:  /* 视觉控制 (摩擦轮速度) */
        {
            ControlData cd;
            if (seasky_unpack_control(&cd, payload, len)) {
                /* set_friction_wheel_speed(cd.velocity_top); */
            }
        }
        break;

    case MSG_HEARTBEAT:  /* 心跳 (上位机→MCU) */
        {
            HeartbeatHostToMcu hb;
            if (seasky_unpack_heartbeat_host(&hb, payload, len)) {
                /* 更新看门狗状态 */
                /* watchdog_feed(); */
            }
        }
        break;

    default:
        /* 未知消息 ID — 忽略 */
        break;
    }
}

/* ========================================================================
 * 定时发送 — MCU → 上位机
 * ======================================================================== */

/**
 * @brief 周期性发送任务
 *
 * 在主循环中调用, 按各自频率发送.
 * 所有发送使用 seasky_pack_xxx() 序列化为精确线格式.
 *
 * 频率参考:
 *   sentry_state:  5Hz   (每 200ms)
 *   yaw:          10Hz   (每 100ms)
 *   gimbal:       10Hz   (每 100ms)
 *   chassis:      50Hz   (每  20ms)
 *   heartbeat:     2Hz   (每 500ms)
 */
static void periodic_send_tasks(uint32_t now_ms) {

    /* --- sentry_state (5Hz, 20 bytes wire) --- */
    if (now_ms - g_last_sentry_time >= 200U) {
        g_last_sentry_time = now_ms;

        SentryState ss;
        ss.battery    = get_battery_percent();   /* ★ 实现: 读取电池电量 */
        ss.life       = get_current_hp();        /* ★ 实现: 读取当前血量 */
        ss.color      = get_team_color();        /* ★ 实现: 读取队伍颜色 */
        ss.bullet     = get_remaining_bullets(); /* ★ 实现: 读取剩余弹量 */
        ss.fault_flag = get_fault_flags();       /* ★ 实现: 读取故障标志 */

        uint8_t payload[WIRE_LEN_SENTRY_STATE];
        uint16_t plen = seasky_pack_sentry_state(payload, &ss);
        seasky_send_frame(&g_seasky, MSG_SENTRY_STATE, payload, plen);
    }

    /* --- yaw (10Hz, 4 bytes wire) --- */
    if (now_ms - g_last_yaw_time >= 100U) {
        g_last_yaw_time = now_ms;

        YawData yd;
        yd.yaw = get_gimbal_yaw();  /* ★ 实现: 读取云台编码器 */

        uint8_t payload[WIRE_LEN_YAW];
        uint16_t plen = seasky_pack_yaw(payload, &yd);
        seasky_send_frame(&g_seasky, MSG_YAW, payload, plen);
    }

    /* --- gimbal (10Hz, 8 bytes wire) — 云台当前角度回传 --- */
    if (now_ms - g_last_gimbal_time >= 100U) {
        g_last_gimbal_time = now_ms;

        GimbalRecv gr;
        gr.yaw   = get_gimbal_yaw();    /* ★ 实现 */
        gr.pitch = get_gimbal_pitch();  /* ★ 实现 */

        uint8_t payload[WIRE_LEN_GIMBAL_RECV];
        uint16_t plen = seasky_pack_gimbal_recv(payload, &gr);
        seasky_send_frame(&g_seasky, MSG_GIMBAL, payload, plen);
    }

    /* --- chassis (50Hz, 12 bytes wire) — 底盘实际速度回传 --- */
    if (now_ms - g_last_chassis_time >= 20U) {
        g_last_chassis_time = now_ms;

        ChassisSpeed cs;
        cs.vx = get_chassis_vx();  /* ★ 实现: 读取编码器/里程计 */
        cs.vy = get_chassis_vy();
        cs.vz = get_chassis_vz();

        uint8_t payload[WIRE_LEN_CHASSIS];
        uint16_t plen = seasky_pack_chassis(payload, &cs);
        seasky_send_frame(&g_seasky, MSG_CHASSIS, payload, plen);
    }

    /* --- heartbeat (2Hz, 5 bytes wire) --- */
    if (now_ms - g_last_heartbeat_time >= 500U) {
        g_last_heartbeat_time = now_ms;

        HeartbeatMcuToHost hb;
        hb.timestamp  = (uint16_t)(now_ms / 1000U);  /* 秒级时间戳 */
        hb.set_launch = is_launcher_ready();   /* ★ 实现 */
        hb.set_arm    = is_friction_on();      /* ★ 实现 */
        hb.fault_flag = get_fault_flags();     /* ★ 实现 */

        uint8_t payload[WIRE_LEN_HEARTBEAT_MCU];
        uint16_t plen = seasky_pack_heartbeat_mcu(payload, &hb);
        seasky_send_frame(&g_seasky, MSG_HEARTBEAT, payload, plen);
    }
}

/* ========================================================================
 * 主函数
 * ======================================================================== */

int main(void) {
    /* ---- 1. 硬件初始化 ---- */
    hal_uart_init();                     /* UART/USB CDC 初始化, 启用 RX 中断 */
    ring_buf_init(&g_rx_rb);             /* 环形缓冲区初始化 */

    /* ---- 2. 协议栈初始化 ---- */
    seasky_init(&g_seasky, &g_rx_rb, on_frame_received);

    /* ---- 3. 定时器初始化 (SysTick 1ms) ---- */
    /* SysTick_Config(SystemCoreClock / 1000); */

    /* ---- 4. 主循环 ---- */
    while (1) {
        uint32_t now_ms = hal_get_tick_ms();

        /* 4a. 处理接收数据 (从 ring_buffer 取字节, 尝试解帧) */
        seasky_process(&g_seasky);

        /* 4b. 应用最新控制指令 (收到新 fix_control 后) */
        if (g_cmd_updated) {
            g_cmd_updated = false;
            apply_motor_control(&g_cmd);  /* ★ 实现: 写入电机驱动 */
        }

        /* 4c. 周期性发送任务 */
        periodic_send_tasks(now_ms);

        /* 4d. 其他 MCU 任务 (传感器读取, 状态机, 看门狗...) */
        /* watchdog_feed(); */
    }

    return 0; /* unreachable */
}

/* ========================================================================
 * ★ 以下函数需要电控组根据实际硬件实现 ★
 * ======================================================================== */

/*
  需要实现的传感器/执行器接口 (示例, 按实际硬件修改):

  float get_battery_percent(void)      { return adc_read(BAT_ADC_CH) * 100.0f / 4096.0f; }
  float get_current_hp(void)           { return referee_uart.robot_hp; }
  float get_team_color(void)           { return referee_uart.team == BLUE ? 1.0f : 3.0f; }
  float get_remaining_bullets(void)    { return referee_uart.bullet_remain; }
  float get_fault_flags(void)          { return fault_register; }
  float get_gimbal_yaw(void)           { return encoder_read(YAW_ENC); }
  float get_gimbal_pitch(void)         { return encoder_read(PITCH_ENC); }
  float get_chassis_vx(void)           { return odometry.vx; }
  float get_chassis_vy(void)           { return odometry.vy; }
  float get_chassis_vz(void)           { return gyro.z; }

  bool  is_launcher_ready(void)        { return launcher_state == READY; }
  bool  is_friction_on(void)           { return friction_motor.speed > 0; }

  void  apply_motor_control(FixControlData *cmd) {
      // 将 cmd->vx, cmd->vy, cmd->vz 转为各电机 PWM/CAN 指令
      // 如果 cmd->spin != 0, 启用小陀螺模式
      // 将 cmd->yaw, cmd->pitch 写入云台电机
      // 如果 cmd->fire, 触发电磁阀
  }

  注意: 以上函数在 periodic_send_tasks() 中被周期性调用,
        确保它们执行速度足够快 (不阻塞), 或者在内部做缓存.

  发送频率检查: 如果主循环周期 > 20ms, chassis (50Hz) 将降级.
  建议主循环频率 ≥ 1kHz, 或使用定时器中断驱动高频发送.
*/

/* ========================================================================
 * HAL 实现示例 (STM32 HAL 库风格, 仅供参考)
 * ========================================================================
 *
 * // ---- hal_uart_stm32.c ----
 * #include "hal_uart.h"
 * #include "stm32f4xx_hal.h"
 * #include "ring_buffer.h"
 *
 * extern UART_HandleTypeDef huart1;  // 或 USB CDC 句柄
 * extern RingBuffer g_rx_rb;         // 来自 main_template.c
 * static uint8_t rx_byte;
 *
 * bool hal_uart_init(void) {
 *     // STM32 CubeMX 已初始化 huart1 (115200, 8N1)
 *     HAL_UART_Receive_IT(&huart1, &rx_byte, 1);  // 启动中断接收
 *     return true;
 * }
 *
 * void hal_uart_send(const uint8_t *data, uint16_t len) {
 *     HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 100);  // 阻塞发送
 *     // 或使用 DMA:  HAL_UART_Transmit_DMA(&huart1, data, len);
 *     // ★ 若用 DMA: 需在上层实现发送完成等待, 防止帧覆盖
 * }
 *
 * uint32_t hal_get_tick_ms(void) {
 *     return HAL_GetTick();  // STM32 HAL SysTick
 * }
 *
 * void hal_enter_critical(void) { __disable_irq(); }
 * void hal_exit_critical(void) { __enable_irq(); }
 *
 * // STM32 UART RX 中断回调
 * void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 *     if (huart->Instance == USART1) {
 *         ring_buf_put_isr(&g_rx_rb, rx_byte);       // ISR-safe 写入
 *         HAL_UART_Receive_IT(&huart1, &rx_byte, 1); // 重新启动中断
 *     }
 * }
 *
 * // ---- 对于 USB CDC (STM32) ----
 * // 将 hal_uart_send 改为:
 * //   CDC_Transmit_FS((uint8_t*)data, len);
 * // 接收在 CDC_Receive_FS 回调中调用 ring_buf_put_isr()
 */
