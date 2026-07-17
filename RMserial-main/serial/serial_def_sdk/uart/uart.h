#ifndef ASSERT_UART__H
#define ASSERT_UART__H

#ifdef __cplusplus
extern "C" {
#endif

#include "DataType.h"
#include <stdint.h>
#include <stdbool.h>

// 保留这些宏，防止编译报错
#define MAX_PACKET_SIZE 256 
#define MAX_PAYLOAD_LEN 13 
#define CACHE_LENGTH 100
#define TIME_PER 500

// 全局变量声明 (兼容旧代码引用)
extern float error_lost;
extern uint8_t readbuff[255];

// 核心函数声明 (保持不变)
extern void defUartInit();
extern void defUartDeinit();
extern bool defUartSend(int id);
extern void defUartRead(uint8_t* readbuff, int nread); // 兼容保留

// 辅助函数声明
extern void cntLossConnection();
extern void print_frame(const char *desc,uint8_t *buf,int size);

#ifdef __cplusplus
}
#endif

#endif // ASSERT_UART__H