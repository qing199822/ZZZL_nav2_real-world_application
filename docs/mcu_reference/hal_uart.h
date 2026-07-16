/**
 * @file    hal_uart.h
 * @brief   硬件抽象层 — UART/USB CDC 接口
 *
 * ╔══════════════════════════════════════════════════════════════╗
 * ║  ★ 此文件是 MCU 相关代码中唯一需要移植的部分 ★              ║
 * ║                                                            ║
 * ║  电控组只需实现下面 3 个核心函数，其余协议代码无需修改。    ║
 * ║  参考实现: STM32 HAL / ESP32 / USB CDC 见 README.md         ║
 * ╚══════════════════════════════════════════════════════════════╝
 */

#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 串口配置 (与上位机 SerialDriver::configureTermios 一致)
 * ======================================================================== */
#define HAL_UART_BAUDRATE   115200U
#define HAL_UART_DATA_BITS  8U
#define HAL_UART_PARITY     0U    /* 0=None, 1=Odd, 2=Even */
#define HAL_UART_STOP_BITS  1U

/* ========================================================================
 * 需要电控组实现的核心函数 (3 个必需 + 3 个可选)
 * ======================================================================== */

/**
 * @brief 初始化 UART/USB CDC 外设
 *
 * 配置:
 *   - 波特率 115200, 8N1, 无流控
 *   - 启用接收中断 (每收到 1 字节触发 RXNE/接收中断)
 *   - 若为 USB CDC, 确保 USB 时钟和端点已配置
 *
 * @return true 初始化成功, false 失败
 */
bool hal_uart_init(void);

/**
 * @brief 发送原始字节数组
 *
 * ★ 关键契约 (必须遵守, 否则协议栈无法保证正确性):
 *
 *   本函数必须满足以下条件之一:
 *     a) 阻塞发送: 函数返回前已完成所有字节的实际发送, 或
 *     b) 内部拷贝: 函数返回前已将数据拷贝到内部 DMA 缓冲区,
 *        调用者的 buffer 在函数返回后可安全释放/重用.
 *
 *   禁止行为:
 *     - 保存 data 指针供以后使用 (DMA 环形缓冲区除外, 但需确保已拷贝)
 *     - 在 ISR 中调用阻塞版本 (HAL_UART_Transmit 在 ISR 中会死锁)
 *
 *   传输大小: 通常 10~54 字节, 最大 256 字节 (SEASKY_TX_BUF_SIZE)
 *
 *   若使用 DMA: 需要发送完成标志或队列机制, 防止上一帧未发完
 *   就被下一帧覆盖. 建议用双缓冲或发送队列.
 *
 * @param data   待发送数据 (函数返回后调用者可释放)
 * @param len    数据长度 (通常 10~54 字节)
 */
void hal_uart_send(const uint8_t *data, uint16_t len);

/**
 * @brief 获取系统滴答 (毫秒)
 *
 * 必须满足:
 *   - 单位: 毫秒
 *   - 类型: uint32_t, 回绕安全 (约 49.7 天)
 *   - 单调递增 (不受系统时间调整影响)
 *
 * 典型实现: STM32 HAL_GetTick(), ESP32 esp_timer_get_time()/1000
 */
uint32_t hal_get_tick_ms(void);

/* ========================================================================
 * 可选接口
 * ======================================================================== */

/**
 * @brief 进入/退出临界区 (禁用/恢复全局中断)
 *
 * 用于保护非 ISR 上下文中的共享数据访问.
 *
 * 实现要求:
 *   - hal_enter_critical(): 禁用全局中断, 保存先前状态
 *   - hal_exit_critical():  恢复先前中断状态 (不是无条件开中断)
 *   - 可嵌套调用 (需保存/恢复 PRIMASK 或使用计数)
 *
 * 典型实现 (Cortex-M):
 *   hal_enter:  __disable_irq()  // 或保存 PRIMASK
 *   hal_exit:   __enable_irq()   // 或恢复 PRIMASK
 *
 * 如果 MCU 不支持中断嵌套, 且 hal_uart_send 是同步阻塞的,
 * 这两个函数可以实现为空.
 */
void hal_enter_critical(void);
void hal_exit_critical(void);

/**
 * @brief 从 UART 接收寄存器读取一个字节 (可选)
 *
 * 推荐方案: 在 ISR 中直接调用 ring_buf_put_isr(), 跳过此函数.
 * 此函数仅用于需要额外抽象的场合.
 *
 * @param byte   输出: 接收到的字节
 * @return true 读取成功, false 无数据
 */
bool hal_uart_recv_byte(uint8_t *byte);

#ifdef __cplusplus
}
#endif

#endif /* HAL_UART_H */
