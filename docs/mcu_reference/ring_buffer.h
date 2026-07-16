/**
 * @file    ring_buffer.h
 * @brief   ISR-safe 环形字节缓冲区 (SPSC: 单生产者/单消费者)
 *
 * 设计约束:
 *   - 单生产者(ISR) / 单消费者(主循环) — 无锁设计
 *   - 大小必须为 2 的幂 (用位掩码代替取模运算)
 *   - 静态分配，零 malloc
 *   - 编译期断言确保大小是 2 的幂
 *
 * 线程模型 (Cortex-M / ESP32 等 32-bit 平台):
 *   - ring_buf_put_isr():  仅 ISR 中调用 (生产者), 无临界区
 *   - ring_buf_get():      仅主循环中调用 (消费者), 内部使用临界区保护
 *   - ring_buf_get_batch(): 仅主循环中调用, 内部使用临界区保护
 *   - ring_buf_flush():     仅主循环中调用, 内部使用临界区保护 (无需调用者额外关中断)
 *   - ring_buf_available/free(): 返回瞬时快照, 仅用于调试/监控, 不应用于控制逻辑
 *
 * 移植注意:
 *   - 若平台不支持 16-bit 原子读写, 需要在 ring_buf_put_isr 中也加临界区
 *   - 若 ISR 可嵌套且多个 ISR 都写入同一 ring buffer, 需额外保护
 *
 * 使用方式:
 *   1. 在 ISR 中调用 ring_buf_put_isr() 写入接收到的字节
 *   2. 在主循环中调用 ring_buf_get() 读取字节进行处理
 */

#ifndef SEASKY_RING_BUFFER_H
#define SEASKY_RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* 缓冲区大小必须是 2 的幂 (256, 512, 1024, ...) */
#define SEASKY_RING_BUF_SIZE   1024U
#define SEASKY_RING_BUF_MASK   (SEASKY_RING_BUF_SIZE - 1U)

/* 编译期断言: 大小必须是 2 的幂 */
#if ((SEASKY_RING_BUF_SIZE == 0) || ((SEASKY_RING_BUF_SIZE & (SEASKY_RING_BUF_SIZE - 1U)) != 0))
#error "SEASKY_RING_BUF_SIZE must be a power of two (e.g. 256, 512, 1024, 2048)"
#endif

typedef struct {
    uint8_t  buffer[SEASKY_RING_BUF_SIZE];
    volatile uint16_t head;            /* ISR writes this, main loop reads */
    volatile uint16_t tail;            /* main loop writes, ISR reads */
    volatile uint32_t overflow_count;  /* bytes dropped when buffer full */
} RingBuffer;

/**
 * @brief 初始化环形缓冲区
 */
void ring_buf_init(RingBuffer *rb);

/**
 * @brief 写入一个字节 (ISR 中调用, 无临界区)
 *
 * 仅从 ISR/单个生产者上下文调用.
 * 若需从非 ISR 上下文 (如 FreeRTOS 任务) 写入, 调用者需自行加临界区保护.
 *
 * @return true 成功, false 缓冲区满 (此时 overflow_count++)
 */
bool ring_buf_put_isr(RingBuffer *rb, uint8_t byte);

/**
 * @brief 读取一个字节 (主循环中调用, 内部使用临界区保护)
 * @return true 成功, false 缓冲区空
 */
bool ring_buf_get(RingBuffer *rb, uint8_t *byte);

/**
 * @brief 批量读取多个字节 (主循环中调用, 内部使用临界区保护)
 * @return 实际读取的字节数 (0 = 缓冲区空)
 */
uint16_t ring_buf_get_batch(RingBuffer *rb, uint8_t *out, uint16_t max_len);

/**
 * @brief 返回缓冲区中待读取的字节数 (瞬时快照, 仅用于调试/监控)
 *
 * ⚠ 返回值为非精确快照: head/tail 可能在读取期间被 ISR 修改.
 *   不应用于关键控制逻辑 (如"available > 0 才读取").
 *   直接使用 ring_buf_get() 即可 — 它内部有空检查.
 */
uint16_t ring_buf_available(const RingBuffer *rb);

/**
 * @brief 返回缓冲区剩余空闲字节数 (瞬时快照, 仅用于调试/监控)
 *
 * ⚠ 同 ring_buf_available — 返回值为非精确快照.
 */
uint16_t ring_buf_free(const RingBuffer *rb);

/**
 * @brief 清空缓冲区 (主循环中调用, 内部使用临界区保护)
 *
 * 已内置临界区保护, 调用者无需额外关中断.
 */
void ring_buf_flush(RingBuffer *rb);

#endif /* SEASKY_RING_BUFFER_H */
