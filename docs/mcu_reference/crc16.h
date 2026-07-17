/**
 * @file    crc16.h
 * @brief   CRC-16-IBM 校验 (与上位机 serial_def_sdk/uart/crc16.h 完全一致)
 *
 * 多项式: 0xA001 (CRC-16-IBM, 又名 CRC-16-ANSI / Modbus CRC)
 * 初始值: 0xFFFF
 * 查表法实现, 表在编译期静态生成, 存放于只读存储区
 *
 * 验证向量 (与上位机交叉验证):
 *   输入:  {0xA5, 0x19, 0x00, 0x95}  (Header + Len=25 + CRC8)
 *   输出:  需与上位机 crc_16() 完全一致
 */

#ifndef SEASKY_CRC16_H
#define SEASKY_CRC16_H

#include <stdint.h>
#include <stddef.h>

#define CRC_START_16    0xFFFFU
#define CRC_POLY_16     0xA001U

uint16_t crc_16(const uint8_t *input_str, uint16_t num_bytes);
uint16_t update_crc_16(uint16_t crc, uint8_t c);

#endif /* SEASKY_CRC16_H */
