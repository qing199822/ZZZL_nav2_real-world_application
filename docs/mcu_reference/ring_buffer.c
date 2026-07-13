/**
 * @file    ring_buffer.c
 * @brief   ISR-safe 环形字节缓冲区实现 (SPSC, 临界区保护)
 *
 * 线程安全分析:
 *   - ring_buf_put_isr():  仅 ISR 写入 head, 仅读 tail → 单生产者安全
 *   - ring_buf_get():      使用临界区保护 tail 写入, 防止重入
 *   - ring_buf_available/free/flush(): 快照读 head/tail, 临界区保护
 *
 * volatile 确保编译器不优化掉 ISR 侧的内存访问.
 * 在 Cortex-M3/4/7 和 ESP32 上, 16-bit 对齐读写是原子的.
 */

#include "ring_buffer.h"
#include "hal_uart.h"

void ring_buf_init(RingBuffer *rb) {
    if (rb == NULL) return;
    rb->head = 0;
    rb->tail = 0;
    rb->overflow_count = 0;
}

/* ISR-only: 无临界区, 依赖单生产者 + volatile */
bool ring_buf_put_isr(RingBuffer *rb, uint8_t byte) {
    uint16_t next_head = (uint16_t)((rb->head + 1U) & SEASKY_RING_BUF_MASK);
    if (next_head == rb->tail) {
        rb->overflow_count++;
        return false;
    }
    rb->buffer[rb->head] = byte;
    rb->head = next_head;
    return true;
}

/* 主循环调用, 临界区保护 tail 修改 */
bool ring_buf_get(RingBuffer *rb, uint8_t *byte) {
    if (rb == NULL || byte == NULL) return false;

    bool ok = false;
    hal_enter_critical();
    if (rb->tail != rb->head) {
        *byte = rb->buffer[rb->tail];
        rb->tail = (uint16_t)((rb->tail + 1U) & SEASKY_RING_BUF_MASK);
        ok = true;
    }
    hal_exit_critical();
    return ok;
}

/* 主循环调用, 临界区保护 */
uint16_t ring_buf_get_batch(RingBuffer *rb, uint8_t *out, uint16_t max_len) {
    if (rb == NULL || out == NULL) return 0;

    uint16_t count = 0;
    hal_enter_critical();
    while (count < max_len && rb->tail != rb->head) {
        out[count++] = rb->buffer[rb->tail];
        rb->tail = (uint16_t)((rb->tail + 1U) & SEASKY_RING_BUF_MASK);
    }
    hal_exit_critical();
    return count;
}

/* 瞬时快照: 读到局部变量防止 ISR 中途修改 */
uint16_t ring_buf_available(const RingBuffer *rb) {
    if (rb == NULL) return 0;
    uint16_t h = rb->head;
    uint16_t t = rb->tail;
    return (uint16_t)((h - t) & SEASKY_RING_BUF_MASK);
}

uint16_t ring_buf_free(const RingBuffer *rb) {
    if (rb == NULL) return 0;
    return SEASKY_RING_BUF_SIZE - 1U - ring_buf_available(rb);
}

/* 内部使用临界区保护, 调用者无需额外关中断 */
void ring_buf_flush(RingBuffer *rb) {
    if (rb == NULL) return;
    hal_enter_critical();
    rb->head = 0;
    rb->tail = 0;
    hal_exit_critical();
}
