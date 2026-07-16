/**
 * @file    seasky_protocol.c
 * @brief   Seasky 协议 — 解帧状态机 + 组帧发送 + pack/unpack
 *
 * 与上位机 serial_def_sdk/uart/SeaskyProtocol.cpp 逻辑完全对应:
 *   - seasky_process()     ←→ SeaskyProtocol::processBuffer()
 *   - seasky_send_frame()  ←→ SeaskyProtocol::sendPacket()
 *   - CRC-8 校验头部       ←→ crc_8(frame, 3)
 *   - CRC-16 校验全帧      ←→ crc_16(frame, pos)
 */

#include "seasky_protocol.h"
#include "crc8.h"
#include "crc16.h"
#include "hal_uart.h"
#include <string.h>

/* ========================================================================
 * 初始化
 * ======================================================================== */

void seasky_init(SeaskyContext *ctx, RingBuffer *rx_rb, SeaskyFrameCallback callback) {
    if (ctx == NULL) return;
    memset(ctx, 0, sizeof(SeaskyContext));
    ctx->rx_rb    = rx_rb;
    ctx->on_frame = callback;
    ctx->state    = STATE_IDLE;
    ctx->rx_pos   = 0;
}

/* ========================================================================
 * 内部辅助 — 重同步逻辑
 *
 * 关键设计: 重同步后若缓存中已有 ≥4 字节 (完整 header+len+crc8),
 * 必须立即校验头部, 不能等下一轮. 否则 rx_pos 已经 ≥4,
 * GOT_HEADER 中的 `if (rx_pos == 4)` 永远无法触达.
 * ======================================================================== */

/**
 * @brief 重同步后尝试立即完成头部校验
 *
 * 当重同步后 rx_pos >= 4 时, 缓冲区内已有完整的 header+len+crc8.
 * 直接校验并决定状态: 通过 → STATE_GOT_CRC8, 失败 → 继续搜索或重置.
 *
 * @param ctx  协议上下文
 * @return true  头部校验通过, 已转入 STATE_GOT_CRC8
 * @return false 校验失败, 已重置或继续搜索
 */
static bool seasky_try_complete_header(SeaskyContext *ctx) {
    /* 迭代而非递归 — 避免全 0xA5 输入下 ~210 层递归栈风险 */
    while (1) {
        if (ctx->rx_pos < 4) {
            ctx->state = STATE_GOT_HEADER;
            return false;
        }

        /* 校验 payload 长度上限 */
        ctx->expected_len = seasky_read_u16_le(&ctx->rx_buf[1]);
        bool header_bad = (ctx->expected_len > SEASKY_MAX_PAYLOAD);

        /* 校验 CRC-8 */
        if (!header_bad) {
            uint8_t calc_crc8 = crc_8(ctx->rx_buf, 3);
            if (calc_crc8 != ctx->rx_buf[3]) {
                ctx->stat_rx_crc8_err++;
                header_bad = true;
            }
        }

        if (header_bad) {
            /* 头部校验失败 — 从字节 1 搜索下一个 0xA5 */
            uint16_t search_idx;
            for (search_idx = 1; search_idx < ctx->rx_pos; search_idx++) {
                if (ctx->rx_buf[search_idx] == SEASKY_HEADER) {
                    uint16_t remaining = ctx->rx_pos - search_idx;
                    memmove(ctx->rx_buf, &ctx->rx_buf[search_idx], remaining);
                    ctx->rx_pos = remaining;
                    continue;  /* 回到 while(1) 顶部, 重新校验新候选帧 */
                }
            }
            /* 未找到 0xA5 — 完全重置 */
            ctx->rx_pos = 0;
            ctx->state  = STATE_IDLE;
            return false;
        }

        /* 头部校验通过 → 直接进入等待帧体阶段 */
        ctx->state = STATE_GOT_CRC8;
        return true;

        /* continue 跳回这里, 进入下一次迭代 */
    }
}

/**
 * @brief 在当前缓冲区内搜索帧头 0xA5, 尝试恢复解析
 *
 * 从 pos=1 开始搜索 (pos=0 已是当前帧头, 跳过).
 * 找到后移动数据, 调用 try_complete_header 智能恢复状态.
 *
 * @param ctx  协议上下文
 * @return true  找到 0xA5, 状态机已恢复
 * @return false 未找到, 状态机已重置为 IDLE
 */
static bool seasky_resync_from_buffer(SeaskyContext *ctx) {
    uint16_t search_idx;

    for (search_idx = 1; search_idx < ctx->rx_pos; search_idx++) {
        if (ctx->rx_buf[search_idx] == SEASKY_HEADER) {
            uint16_t remaining = ctx->rx_pos - search_idx;
            memmove(ctx->rx_buf, &ctx->rx_buf[search_idx], remaining);
            ctx->rx_pos = remaining;
            /* 根据已缓存字节数智能恢复状态 (处理 remaining>=4 的情况) */
            seasky_try_complete_header(ctx);
            return true;
        }
    }

    /* 未找到帧头 → 完全重置 */
    ctx->rx_pos = 0;
    ctx->state  = STATE_IDLE;
    return false;
}

/* ========================================================================
 * 解帧状态机 (主循环中反复调用)
 * ======================================================================== */

void seasky_process(SeaskyContext *ctx) {
    uint8_t byte;

    if (ctx == NULL || ctx->rx_rb == NULL) return;

    /* 同步 ring buffer 溢出统计 */
    ctx->stat_rx_overflow = ctx->rx_rb->overflow_count;

    uint32_t now = hal_get_tick_ms();

    /* 帧超时: 若在非 IDLE 状态停滞超过 SEASKY_RX_TIMEOUT_MS, 重置状态机.
       uint32_t 无符号差值天然抗回绕 (约 49.7 天). */
    if (ctx->state != STATE_IDLE && ctx->last_byte_ms != 0U) {
        if ((uint32_t)(now - ctx->last_byte_ms) > SEASKY_RX_TIMEOUT_MS) {
            ctx->rx_pos = 0;
            ctx->state  = STATE_IDLE;
        }
    }

    /* 批量读取 ring_buffer 中的所有可用字节 */
    while (ring_buf_get(ctx->rx_rb, &byte)) {
        /* 每次取到字节都刷新时间戳, 使超时判断更准确.
           并在 while 循环内也做超时检查 (防止积压大量字节时超时被延迟). */
        now = hal_get_tick_ms();
        if (ctx->state != STATE_IDLE && ctx->last_byte_ms != 0U) {
            if ((uint32_t)(now - ctx->last_byte_ms) > SEASKY_RX_TIMEOUT_MS) {
                ctx->rx_pos = 0;
                ctx->state  = STATE_IDLE;
            }
        }
        ctx->last_byte_ms = now;

        switch (ctx->state) {

        /* ------------------------------------------------------------
         * STATE_IDLE: 搜索帧头 0xA5
         * ------------------------------------------------------------ */
        case STATE_IDLE:
            if (byte == SEASKY_HEADER) {
                ctx->rx_buf[0] = byte;
                ctx->rx_pos    = 1;
                ctx->state     = STATE_GOT_HEADER;
            }
            /* 不是 0xA5 → 丢弃, 继续搜索 */
            break;

        /* ------------------------------------------------------------
         * STATE_GOT_HEADER: 读取 2 字节长度 + CRC-8, 共 4 字节就绪
         * ------------------------------------------------------------ */
        case STATE_GOT_HEADER:
            ctx->rx_buf[ctx->rx_pos++] = byte;
            if (ctx->rx_pos == 4) {
                /* 现在有 Byte 0-3: [Header][Len_L][Len_H][CRC8] */

                ctx->expected_len = seasky_read_u16_le(&ctx->rx_buf[1]);

                /* 安全上限检查 (与上位机 processBuffer() 一致) */
                if (ctx->expected_len > SEASKY_MAX_PAYLOAD) {
                    /* 长度异常, 在当前缓冲中搜索 0xA5 重新同步 */
                    seasky_resync_from_buffer(ctx);
                    break;
                }

                /* CRC-8 校验头部 (Byte 0-2) */
                uint8_t calc_crc8 = crc_8(ctx->rx_buf, 3);
                if (calc_crc8 != ctx->rx_buf[3]) {
                    ctx->stat_rx_crc8_err++;
                    /* 从字节 1 开始搜索帧头 重新同步 (与上位机一致) */
                    seasky_resync_from_buffer(ctx);
                    break;
                }

                ctx->state = STATE_GOT_CRC8;
            }
            break;

        /* ------------------------------------------------------------
         * STATE_GOT_CRC8: 等待帧剩余部分
         * ------------------------------------------------------------
         * 帧总长 = 10 + expected_len
         * 已有 4 字节, 还需: expected_len + 6 字节
         *   (ID(2)+Flags(2)+Payload(N)+CRC16(2))
         */
        case STATE_GOT_CRC8: {
            uint16_t total_len = ctx->expected_len + SEASKY_FRAME_OVERHEAD;

            if (ctx->rx_pos < total_len) {
                ctx->rx_buf[ctx->rx_pos++] = byte;
            }

            if (ctx->rx_pos >= total_len) {
                /* 帧收全, CRC-16 校验 */
                uint16_t calc_crc16 = crc_16(ctx->rx_buf, total_len - 2);
                uint16_t recv_crc16 = seasky_read_u16_le(&ctx->rx_buf[total_len - 2]);

                if (calc_crc16 == recv_crc16) {
                    ctx->stat_rx_frames++;

                    uint16_t msg_id       = seasky_read_u16_le(&ctx->rx_buf[4]);
                    uint16_t flags        = seasky_read_u16_le(&ctx->rx_buf[6]);
                    const uint8_t *payload = &ctx->rx_buf[8];
                    uint16_t payload_len   = ctx->expected_len;

                    if (ctx->on_frame) {
                        ctx->on_frame(msg_id, flags, payload, payload_len);
                    }

                    /* 成功 — 重置状态机 */
                    ctx->rx_pos = 0;
                    ctx->state  = STATE_IDLE;

                } else {
                    ctx->stat_rx_crc16_err++;

                    /* CRC-16 失败: 在已缓冲的帧内搜索 0xA5 重新同步
                     * (与上位机仅丢弃1字节不同, 但嵌入式无法回退 ring_buffer)
                     * 复用 seasky_resync_from_buffer() 避免代码重复 */
                    seasky_resync_from_buffer(ctx);
                    /* 状态机已恢复 (STATE_GOT_HEADER / STATE_GOT_CRC8 / STATE_IDLE),
                       继续 while 循环处理 ring_buffer 中的后续字节 */
                }
            }
            break;
        }

        } /* switch(state) */

    } /* while ring_buf_get */
}

/* ========================================================================
 * 组帧发送 (与上位机 SeaskyProtocol::sendPacket 完全一致)
 *
 * ★ 不可重入: 使用 ctx->tx_buf 共享缓冲, 禁止在 ISR 中调用.
 *   若需多线程发送, 调用者负责外部互斥.
 *   若 hal_uart_send 是异步 DMA, 上层需实现发送队列防止帧覆盖.
 *
 * ★ 临界区仅保护 tx_buf 组帧, hal_uart_send 在临界区外执行 —
 *   避免阻塞发送期间关中断导致 RX 丢字节.
 * ======================================================================== */

bool seasky_send_frame(SeaskyContext *ctx, uint16_t msg_id,
                       const uint8_t *payload, uint16_t payload_len) {

    /* 参数校验 */
    if (ctx == NULL) return false;
    if (payload_len > SEASKY_MAX_PAYLOAD) return false;
    if ((payload_len > 0U) && (payload == NULL)) return false;

    uint8_t *frame = ctx->tx_buf;
    uint16_t pos   = 0;
    uint16_t send_len;

    /* 临界区仅保护组帧过程 (tx_buf 共享缓冲区写入) */
    hal_enter_critical();

    /* Byte 0: Header */
    frame[pos++] = SEASKY_HEADER;

    /* Byte 1-2: Payload Len (LE) */
    seasky_write_u16_le(&frame[pos], payload_len);
    pos += 2;

    /* Byte 3: CRC-8 of bytes 0-2 */
    frame[pos++] = crc_8(frame, 3);

    /* Byte 4-5: Msg ID (LE) */
    seasky_write_u16_le(&frame[pos], msg_id);
    pos += 2;

    /* Byte 6-7: Flags (目前固定 0x0000) */
    seasky_write_u16_le(&frame[pos], 0);
    pos += 2;

    /* Byte 8..: Payload */
    if (payload_len > 0U) {
        memcpy(&frame[pos], payload, payload_len);
        pos += payload_len;
    }

    /* 最后 2 字节: CRC-16 of bytes 0..pos-1 */
    uint16_t crc = crc_16(frame, pos);
    seasky_write_u16_le(&frame[pos], crc);
    pos += 2;

    /* 记录发送长度并退出临界区 — 帧已完整组装在 tx_buf 中 */
    send_len = pos;
    ctx->stat_tx_frames++;

    hal_exit_critical();

    /* 在临界区外发送, 避免阻塞发送期间关中断导致 RX 丢字节 */
    hal_uart_send(frame, send_len);
    return true;
}

/* ========================================================================
 * 统计查询
 * ======================================================================== */

void seasky_get_stats(const SeaskyContext *ctx, uint32_t *rx_frames,
                      uint32_t *rx_crc8_err, uint32_t *rx_crc16_err,
                      uint32_t *rx_overflow, uint32_t *tx_frames) {
    if (ctx == NULL) return;
    if (rx_frames)     *rx_frames     = ctx->stat_rx_frames;
    if (rx_crc8_err)   *rx_crc8_err   = ctx->stat_rx_crc8_err;
    if (rx_crc16_err)  *rx_crc16_err  = ctx->stat_rx_crc16_err;
    if (rx_overflow)   *rx_overflow   = ctx->stat_rx_overflow;
    if (tx_frames)     *tx_frames     = ctx->stat_tx_frames;
}

/* ========================================================================
 * Pack/Unpack 函数 — 结构体 ⇄ 线格式 (小端序)
 *
 * 每个函数将语义结构体序列化为精确线格式字节, 或反向解析.
 * 所有多字节字段按 LE 写入/读取.
 * ======================================================================== */

/* ---- HeartbeatMcuToHost (5 bytes) ---- */
uint16_t seasky_pack_heartbeat_mcu(uint8_t *out, const HeartbeatMcuToHost *hb) {
    if (out == NULL || hb == NULL) return 0;
    seasky_write_u16_le(&out[0], hb->timestamp);
    out[2] = hb->set_launch;
    out[3] = hb->set_arm;
    out[4] = hb->fault_flag;
    return WIRE_LEN_HEARTBEAT_MCU;
}

bool seasky_unpack_heartbeat_host(HeartbeatHostToMcu *out, const uint8_t *payload, uint16_t len) {
    if (out == NULL || payload == NULL || len < WIRE_LEN_HEARTBEAT_HOST) return false;
    out->timestamp  = seasky_read_u16_le(&payload[0]);
    out->battery    = payload[2];
    out->life       = payload[3];
    out->color      = payload[4];
    out->bullet     = payload[5];
    out->fault_flag = payload[6];
    return true;
}

/* ---- GimbalSend (9 bytes) ---- */
uint16_t seasky_pack_gimbal_send(uint8_t *out, const GimbalSend *gs) {
    if (out == NULL || gs == NULL) return 0;
    seasky_write_f32_le(&out[0], gs->yaw);
    seasky_write_f32_le(&out[4], gs->pitch);
    out[8] = gs->fire;
    return WIRE_LEN_GIMBAL_SEND;
}

/* ---- GimbalRecv (8 bytes) ---- */
uint16_t seasky_pack_gimbal_recv(uint8_t *out, const GimbalRecv *gr) {
    if (out == NULL || gr == NULL) return 0;
    seasky_write_f32_le(&out[0], gr->yaw);
    seasky_write_f32_le(&out[4], gr->pitch);
    return WIRE_LEN_GIMBAL_RECV;
}

bool seasky_unpack_gimbal_send(GimbalSend *out, const uint8_t *payload, uint16_t len) {
    if (out == NULL || payload == NULL || len < WIRE_LEN_GIMBAL_SEND) return false;
    out->yaw   = seasky_read_f32_le(&payload[0]);
    out->pitch = seasky_read_f32_le(&payload[4]);
    out->fire  = payload[8];
    return true;
}

/* ---- FixControlData (25 bytes) ★ ---- */
bool seasky_unpack_fix_control(FixControlData *out, const uint8_t *payload, uint16_t len) {
    if (out == NULL || payload == NULL || len < WIRE_LEN_FIX_CONTROL) return false;
    uint16_t pos = 0;
    out->yaw   = seasky_read_f32_le(&payload[pos]); pos += 4;
    out->pitch = seasky_read_f32_le(&payload[pos]); pos += 4;
    out->fire  = payload[pos]; pos += 1;
    out->vx    = seasky_read_f32_le(&payload[pos]); pos += 4;
    out->vy    = seasky_read_f32_le(&payload[pos]); pos += 4;
    out->vz    = seasky_read_f32_le(&payload[pos]); pos += 4;
    out->spin  = seasky_read_f32_le(&payload[pos]); /* pos += 4; */
    return true;
}

/* ---- ControlData (5 bytes) ---- */
bool seasky_unpack_control(ControlData *out, const uint8_t *payload, uint16_t len) {
    if (out == NULL || payload == NULL || len < WIRE_LEN_CONTROL) return false;
    out->velocity_top = seasky_read_f32_le(&payload[0]);
    out->shoot        = payload[4];
    return true;
}

/* ---- SentryState (20 bytes, all-float) ---- */
uint16_t seasky_pack_sentry_state(uint8_t *out, const SentryState *ss) {
    if (out == NULL || ss == NULL) return 0;
    seasky_write_f32_le(&out[0],  ss->battery);
    seasky_write_f32_le(&out[4],  ss->life);
    seasky_write_f32_le(&out[8],  ss->color);
    seasky_write_f32_le(&out[12], ss->bullet);
    seasky_write_f32_le(&out[16], ss->fault_flag);
    return WIRE_LEN_SENTRY_STATE;
}

/* ---- YawData (4 bytes) ---- */
uint16_t seasky_pack_yaw(uint8_t *out, const YawData *yd) {
    if (out == NULL || yd == NULL) return 0;
    seasky_write_f32_le(&out[0], yd->yaw);
    return WIRE_LEN_YAW;
}

/* ---- ChassisSpeed (12 bytes, all-float) ---- */
uint16_t seasky_pack_chassis(uint8_t *out, const ChassisSpeed *cs) {
    if (out == NULL || cs == NULL) return 0;
    seasky_write_f32_le(&out[0], cs->vx);
    seasky_write_f32_le(&out[4], cs->vy);
    seasky_write_f32_le(&out[8], cs->vz);
    return WIRE_LEN_CHASSIS;
}

/* ---- AdditionalData (5 bytes, all-uint8) ---- */
uint16_t seasky_pack_additional(uint8_t *out, const AdditionalData *ad) {
    if (out == NULL || ad == NULL) return 0;
    out[0] = ad->launch;
    out[1] = ad->arm;
    out[2] = ad->base_hp_our;
    out[3] = ad->base_hp_enemy;
    out[4] = ad->judge_warning;
    return WIRE_LEN_ADDITIONAL;
}

/* ---- GameStatus (3 bytes) ---- */
uint16_t seasky_pack_game_status(uint8_t *out, const GameStatus *gs) {
    if (out == NULL || gs == NULL) return 0;
    seasky_write_u16_le(&out[0], gs->stage_remain_time);
    out[2] = gs->game_progress;
    return WIRE_LEN_GAME_STATUS;
}

/* ---- ImuData (24 bytes, all-float) ---- */
uint16_t seasky_pack_imu(uint8_t *out, const ImuData *imu) {
    if (out == NULL || imu == NULL) return 0;
    seasky_write_f32_le(&out[0],  imu->vx);
    seasky_write_f32_le(&out[4],  imu->vy);
    seasky_write_f32_le(&out[8],  imu->vz);
    seasky_write_f32_le(&out[12], imu->ax);
    seasky_write_f32_le(&out[16], imu->ay);
    seasky_write_f32_le(&out[20], imu->az);
    return WIRE_LEN_IMU;
}
