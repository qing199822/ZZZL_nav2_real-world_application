/**
 * @file    seasky_protocol.h
 * @brief   Seasky 串口协议 — 平台无关核心
 *
 * 与上位机 serial_def_sdk/uart/SeaskyProtocol.hpp 完全兼容。
 *
 * 帧格式: Header(0xA5) + Len(2B LE) + CRC8(1B) + MsgID(2B LE) + Flags(2B LE)
 *         + Payload(N) + CRC16(2B LE)
 * 总帧长 = 10 + N 字节
 *
 * 线格式约定 (Wire Format):
 *   - 所有多字节值: 小端序 (Little-Endian, LSB first)
 *   - float 类型: IEEE-754 binary32, 小端序
 *   - 结构体内存布局 ≠ 线格式 (ARM 上 mixed float/uint8 有 padding)
 *   - ★ 必须使用 pack/unpack 函数, 禁止直接 memcpy struct → 线格式
 *
 * 使用流程:
 *   1. 初始化: seasky_init(&ctx, &rx_rb)
 *   2. ISR 中:  ring_buf_put_isr(&rx_rb, received_byte)
 *   3. 主循环:  seasky_process(&ctx) → 自动回调 on_frame()
 *   4. 发送:    seasky_pack_xxx() + seasky_send_frame(&ctx, msg_id, payload, len)
 */

#ifndef SEASKY_PROTOCOL_H
#define SEASKY_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "ring_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 协议常量 (与上位机 PROTOCOL_HEADER_ID / MAX_PACKET_SIZE 一致)
 * ======================================================================== */

#define SEASKY_HEADER         0xA5U
#define SEASKY_MAX_PAYLOAD    200U      /* payload_len 安全上限 */
#define SEASKY_FRAME_OVERHEAD 10U       /* Header(1)+Len(2)+CRC8(1)+ID(2)+Flags(2)+CRC16(2) */
#define SEASKY_TX_BUF_SIZE    (SEASKY_MAX_PAYLOAD + SEASKY_FRAME_OVERHEAD)  /* 210, 组帧缓冲区 */

/* 帧超时: 接收半帧超过此时间(ms)则重置状态机.
   115200bps 下最坏帧 ~23ms, 100ms 留有足够余量.
   若移植到更低波特率或更大 payload, 按需调大. */
#ifndef SEASKY_RX_TIMEOUT_MS
#define SEASKY_RX_TIMEOUT_MS  100U
#endif

/* ========================================================================
 * 消息 ID 枚举 (与上位机 DataType.h:msg_id 一致)
 * ======================================================================== */

typedef enum {
    MSG_HEARTBEAT      = 0x01,   /* 心跳 (双向)           */
    MSG_GIMBAL         = 0x02,   /* 云台控制/回传 (双向)  */
    MSG_CONTROL        = 0x03,   /* 视觉控制 (上位机→MCU) */
    MSG_ADDITIONAL     = 0x04,   /* 附加状态 (MCU→上位机) */
    MSG_FIX_CONTROL    = 0x05,   /* 导航+云台合并包 ★     */
    MSG_IMU2           = 0x06,   /* IMU 数据 (MCU→上位机) */
    MSG_YAW            = 0x07,   /* Yaw 角度 (MCU→上位机) */
    MSG_GAME_STATUS    = 0x08,   /* 比赛状态 (MCU→上位机) */
    MSG_CHASSIS        = 0x10,   /* 底盘速度 (双向)       */
    MSG_SENTRY_STATE   = 0x20,   /* 哨兵状态 (MCU→上位机) */
} SeaskyMsgId;

/* ========================================================================
 * 线格式长度常量 (Wire Lengths)
 *
 * ★ 这些是协议规定的精确字节数, 发送时必须严格遵守.
 *    sizeof(struct) 可能不同 (ARM 上 mixed float/uint8 有 padding),
 *    因此禁止用 sizeof 作为发送长度.
 * ======================================================================== */

#define WIRE_LEN_HEARTBEAT_HOST   7U    /* HeartbeatHostToMcu */
#define WIRE_LEN_HEARTBEAT_MCU    5U    /* HeartbeatMcuToHost */
#define WIRE_LEN_GIMBAL_SEND      9U    /* GimbalSend */
#define WIRE_LEN_GIMBAL_RECV      8U    /* GimbalRecv */
#define WIRE_LEN_FIX_CONTROL      25U   /* FixControlData ★ */
#define WIRE_LEN_CONTROL          5U    /* ControlData */
#define WIRE_LEN_CHASSIS          12U   /* ChassisSpeed */
#define WIRE_LEN_ADDITIONAL       5U    /* AdditionalData */
#define WIRE_LEN_GAME_STATUS      3U    /* GameStatus */
#define WIRE_LEN_SENTRY_STATE     20U   /* SentryState */
#define WIRE_LEN_IMU              24U   /* ImuData */
#define WIRE_LEN_YAW              4U    /* YawData */

/* ========================================================================
 * 载荷数据结构体 (语义结构, 非线格式!)
 *
 * ╔══════════════════════════════════════════════════════════════╗
 * ║  ★★★ 关键约束 ★★★                                          ║
 * ║                                                            ║
 * ║  这些结构体是"语义容器", 内存布局 ≠ 线格式 (wire format).   ║
 * ║  ARM 上 mixed float/uint8 有 padding, sizeof() ≠ 线字节数.  ║
 * ║                                                            ║
 * ║  ★ 发送: 使用 pack 函数 (seasky_pack_xxx)                  ║
 * ║  ★ 接收: 使用 unpack 函数 (seasky_unpack_xxx)              ║
 * ║  ★ 禁止: (uint8_t*)&struct + sizeof(struct) 直接发送       ║
 * ║                                                            ║
 * ║  若编译器支持, 可加 _Static_assert 验证 sizeof:             ║
 * ║  _Static_assert(sizeof(FixControlData) >= WIRE_LEN_FIX_CONTROL, ║
 * ║                 "FixControlData too small");                ║
 * ╚══════════════════════════════════════════════════════════════╝
 * ======================================================================== */

/* 心跳 — 上位机→MCU: 7 bytes wire */
typedef struct {
    uint16_t timestamp;        /* 时间戳 (秒) */
    uint8_t  battery;          /* 电池电量 */
    uint8_t  life;             /* 剩余生命值 */
    uint8_t  color;            /* 队伍颜色 (0=未知, 1=蓝, 2=紫, 3=红) */
    uint8_t  bullet;           /* 剩余弹量 */
    uint8_t  fault_flag;       /* 故障标志 */
} HeartbeatHostToMcu;

/* 心跳 — MCU→上位机: 5 bytes wire */
typedef struct {
    uint16_t timestamp;        /* 时间戳 (秒) */
    uint8_t  set_launch;       /* 发射机构已启用 */
    uint8_t  set_arm;          /* 摩擦轮已启动 */
    uint8_t  fault_flag;       /* 故障标志 */
} HeartbeatMcuToHost;

/* 云台 — 上位机→MCU: 9 bytes wire */
typedef struct {
    float    yaw;              /* 目标偏航角 (rad) */
    float    pitch;            /* 目标俯仰角 (rad) */
    uint8_t  fire;             /* 开火 (0/1) */
} GimbalSend;

/* 云台 — MCU→上位机: 8 bytes wire */
typedef struct {
    float    yaw;              /* 当前偏航角 (rad) */
    float    pitch;            /* 当前俯仰角 (rad) */
} GimbalRecv;

/* ★ fix_control — 上位机→MCU: 25 bytes wire ★
 * 线序: [yaw:4][pitch:4][fire:1][vx:4][vy:4][vz:4][spin:4] */
typedef struct {
    float    yaw;              /* 目标偏航角 (rad) */
    float    pitch;            /* 目标俯仰角 (rad) */
    uint8_t  fire;             /* 开火 */
    float    vx;               /* X 轴线速度 (m/s, 前向+) */
    float    vy;               /* Y 轴线速度 (m/s, 左向+) */
    float    vz;               /* Z 轴线速度 (导航填0, MCU忽略) */
    float    spin;             /* 小陀螺角速度 (rad/s) */
} FixControlData;

/* 视觉控制 — 上位机→MCU: 5 bytes wire */
typedef struct {
    float    velocity_top;     /* 摩擦轮线速度上限 */
    uint8_t  shoot;            /* 射击指令 */
} ControlData;

/* 底盘速度 — 双向: 12 bytes wire (all-float, naturally aligned) */
typedef struct {
    float    vx, vy, vz;
} ChassisSpeed;

/* 附加状态 — MCU→上位机: 5 bytes wire (all-uint8, naturally packed) */
typedef struct {
    uint8_t  launch;           /* 发射机构就绪 */
    uint8_t  arm;              /* 摩擦轮启动 */
    uint8_t  base_hp_our;      /* 我方基地血量 (%) */
    uint8_t  base_hp_enemy;    /* 敌方基地血量 (%) */
    uint8_t  judge_warning;    /* 裁判系统警告等级 */
} AdditionalData;

/* 比赛状态 — MCU→上位机: 3 bytes wire */
typedef struct {
    uint16_t  stage_remain_time;  /* 阶段剩余时间 (s) */
    uint8_t   game_progress;      /* 比赛进度 */
} GameStatus;

/* 哨兵状态 — MCU→上位机: 20 bytes wire (all-float, naturally aligned) */
typedef struct {
    float    battery;          /* 电池百分比 */
    float    life;             /* 当前血量 */
    float    color;            /* 队伍颜色 */
    float    bullet;           /* 剩余弹量 */
    float    fault_flag;       /* 故障标志 */
} SentryState;

/* IMU — MCU→上位机: 24 bytes wire (all-float) */
typedef struct {
    float    vx, vy, vz;       /* 角速度 (rad/s) */
    float    ax, ay, az;       /* 线加速度 (m/s²) */
} ImuData;

/* Yaw — MCU→上位机: 4 bytes wire */
typedef struct {
    float    yaw;
} YawData;

/* ========================================================================
 * 解帧状态机
 * ======================================================================== */

typedef enum {
    STATE_IDLE = 0,             /* 等待 0xA5 */
    STATE_GOT_HEADER,           /* 收到帧头, 等待 2B 长度 + CRC8 */
    STATE_GOT_CRC8,             /* CRC-8 校验通过, 等待剩余帧 */
} RxState;

/* ========================================================================
 * 帧接收回调
 * ======================================================================== */

/**
 * @brief 收到完整帧时的回调函数
 *
 * @param msg_id      消息 ID (SeaskyMsgId)
 * @param flags       帧标志 (当前固定 0x0000, 预留扩展)
 * @param payload     载荷数据指针 (指向上下文内部缓冲区, 回调返回后失效!)
 * @param payload_len 载荷长度
 *
 * ╔══════════════════════════════════════════════════════════╗
 * ║  WARNING: payload → ctx 内部缓冲区!                     ║
 * ║  回调返回后立即失效. 如需持久保存, 请 memcpy().         ║
 * ╚══════════════════════════════════════════════════════════╝
 *
 * 在 seasky_process() 中被调用, 运行在主循环上下文.
 * 电控组在此函数中实现业务逻辑: 解析 fix_control, 更新电机目标值等.
 */
typedef void (*SeaskyFrameCallback)(uint16_t msg_id, uint16_t flags,
                                    const uint8_t *payload, uint16_t payload_len);

/* ========================================================================
 * 协议上下文
 * ======================================================================== */

typedef struct {
    RingBuffer          *rx_rb;             /* 接收环形缓冲区指针 (外部提供) */
    SeaskyFrameCallback  on_frame;          /* 帧接收回调 */

    /* 解帧状态机 (内部使用) */
    uint8_t             rx_buf[SEASKY_MAX_PAYLOAD + SEASKY_FRAME_OVERHEAD];
    uint16_t            rx_pos;
    uint32_t            last_byte_ms;       /* 帧超时追踪, uint32 回绕安全 */
    uint16_t            expected_len;       /* 期望的 payload 长度 */
    RxState             state;

    /* 发送缓冲区 (单线程使用; 若多线程需外部互斥) */
    uint8_t             tx_buf[SEASKY_TX_BUF_SIZE];

    /* 统计 (调试用) */
    uint32_t            stat_rx_frames;     /* 成功接收帧数 */
    uint32_t            stat_rx_crc8_err;   /* CRC-8 错误数 */
    uint32_t            stat_rx_crc16_err;  /* CRC-16 错误数 */
    uint32_t            stat_rx_overflow;   /* RX 缓冲区溢出次数 */
    uint32_t            stat_tx_frames;     /* 成功发送帧数 */
} SeaskyContext;

/* ========================================================================
 * 协议核心 API
 * ======================================================================== */

/**
 * @brief 初始化协议上下文
 * @param ctx      未初始化的上下文指针 (不可为 NULL)
 * @param rx_rb    接收环形缓冲区指针 (需在 ISR 中通过 ring_buf_put_isr 写入)
 * @param callback 帧接收回调函数
 */
void seasky_init(SeaskyContext *ctx, RingBuffer *rx_rb, SeaskyFrameCallback callback);

/**
 * @brief 喂数据并尝试解析帧 (在主循环中反复调用)
 *
 * 每次调用从 ring_buffer 取出所有可用字节, 推进解帧状态机.
 * 当成功解析一帧时, 调用 ctx->on_frame().
 *
 * 调用频率建议 ≥ 1kHz, 确保不积压.
 *
 * @param ctx  已初始化的上下文
 */
void seasky_process(SeaskyContext *ctx);

/**
 * @brief 组帧并发送 (主循环上下文, 不可重入)
 *
 * ╔══════════════════════════════════════════════════════════╗
 * ║  ★ 此函数不可在 ISR 中调用! ★                           ║
 * ║  使用 ctx->tx_buf 共享缓冲区, 非重入安全.                ║
 * ║  如需多线程发送, 上层自行实现互斥或发送队列.             ║
 * ╚══════════════════════════════════════════════════════════╝
 *
 * @param ctx         已初始化的上下文
 * @param msg_id      消息 ID
 * @param payload     载荷数据 (payload_len==0 时可为 NULL)
 * @param payload_len 载荷长度 (必须 ≤ SEASKY_MAX_PAYLOAD)
 * @return true 发送成功, false 参数无效
 */
bool seasky_send_frame(SeaskyContext *ctx, uint16_t msg_id,
                       const uint8_t *payload, uint16_t payload_len);

/**
 * @brief 获取统计信息 (调试用)
 */
void seasky_get_stats(const SeaskyContext *ctx, uint32_t *rx_frames,
                      uint32_t *rx_crc8_err, uint32_t *rx_crc16_err,
                      uint32_t *rx_overflow, uint32_t *tx_frames);

/* ========================================================================
 * Pack/Unpack API — 结构体 ⇄ 线格式 (小端序)
 *
 * 每个消息类型提供一对序列化/反序列化函数.
 * out 缓冲区必须至少 WIRE_LEN_xxx 字节.
 * 返回实际写入的字节数 (应等于对应的 WIRE_LEN_xxx).
 * ======================================================================== */

/* 小端序读写辅助 (inline, 零开销) */
static inline uint16_t seasky_read_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline void seasky_write_u16_le(uint8_t *p, uint16_t val) {
    p[0] = (uint8_t)(val & 0xFFU);
    p[1] = (uint8_t)((val >> 8) & 0xFFU);
}

static inline void seasky_write_f32_le(uint8_t *p, float val) {
    uint32_t u;
    /* memcpy 是 C 标准中唯一的 type-punning 安全方式 */
    { const uint8_t *src = (const uint8_t *)&val; uint8_t *dst = (uint8_t *)&u;
      dst[0]=src[0]; dst[1]=src[1]; dst[2]=src[2]; dst[3]=src[3]; }
    p[0] = (uint8_t)(u & 0xFFU);
    p[1] = (uint8_t)((u >> 8) & 0xFFU);
    p[2] = (uint8_t)((u >> 16) & 0xFFU);
    p[3] = (uint8_t)((u >> 24) & 0xFFU);
}

static inline float seasky_read_f32_le(const uint8_t *p) {
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8)
               | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    float val;
    { uint8_t *dst = (uint8_t *)&val; const uint8_t *src = (const uint8_t *)&u;
      dst[0]=src[0]; dst[1]=src[1]; dst[2]=src[2]; dst[3]=src[3]; }
    return val;
}

/* ---- HeartbeatMcuToHost (5 bytes) ---- */
uint16_t seasky_pack_heartbeat_mcu(uint8_t *out, const HeartbeatMcuToHost *hb);
bool     seasky_unpack_heartbeat_host(HeartbeatHostToMcu *out, const uint8_t *payload, uint16_t len);

/* ---- GimbalSend (9 bytes) ---- */
uint16_t seasky_pack_gimbal_send(uint8_t *out, const GimbalSend *gs);

/* ---- GimbalRecv (8 bytes) ---- */
uint16_t seasky_pack_gimbal_recv(uint8_t *out, const GimbalRecv *gr);
bool     seasky_unpack_gimbal_send(GimbalSend *out, const uint8_t *payload, uint16_t len);

/* ---- FixControlData (25 bytes) ★ ---- */
bool     seasky_unpack_fix_control(FixControlData *out, const uint8_t *payload, uint16_t len);

/* ---- ControlData (5 bytes) ---- */
bool     seasky_unpack_control(ControlData *out, const uint8_t *payload, uint16_t len);

/* ---- SentryState (20 bytes) ---- */
uint16_t seasky_pack_sentry_state(uint8_t *out, const SentryState *ss);

/* ---- YawData (4 bytes) ---- */
uint16_t seasky_pack_yaw(uint8_t *out, const YawData *yd);

/* ---- ChassisSpeed (12 bytes) ---- */
uint16_t seasky_pack_chassis(uint8_t *out, const ChassisSpeed *cs);

/* ---- AdditionalData (5 bytes, all-uint8) ---- */
uint16_t seasky_pack_additional(uint8_t *out, const AdditionalData *ad);

/* ---- GameStatus (3 bytes) ---- */
uint16_t seasky_pack_game_status(uint8_t *out, const GameStatus *gs);

/* ---- ImuData (24 bytes) ---- */
uint16_t seasky_pack_imu(uint8_t *out, const ImuData *imu);

#ifdef __cplusplus
}
#endif

#endif /* SEASKY_PROTOCOL_H */
