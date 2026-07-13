#ifndef __SEASKY_H__
#define __SEASKY_H__

#include "main.h"


/*  Seasky 串口协议 — 上位机  MCU 通信 (USB CDC)                    
                                                                   
 帧结构: 0xA5 + PayloadLen(2B LE) + CRC8(1B) + MsgID(2B LE)      
        + Flags(2B LE) + Payload(NB) + CRC16(2B LE)              
  总帧长 = 10 + N 字节                                             */


/* ---- 帧头 ---- */
#define SEASKY_HEADER       0xA5
#define SEASKY_FRAME_MAX    256
#define SEASKY_PAYLOAD_MAX  200

/* ---- 消息 ID ---- */
#define MSG_ID_HEARTBEAT    0x01
#define MSG_ID_GIMBAL       0x02
#define MSG_ID_CONTROL      0x03
#define MSG_ID_ADDITIONAL   0x04
#define MSG_ID_FIX_CONTROL  0x05
#define MSG_ID_IMU2         0x06
#define MSG_ID_YAW          0x07
#define MSG_ID_GAME_STATUS  0x08
#define MSG_ID_CHASSIS      0x10
#define MSG_ID_SENTRY       0x20


/*  发送回调类型: 指向 CDC_Transmit_FS 或其他发送函数                 */

typedef uint8_t (*seasky_send_fn_t)(uint8_t *data, uint16_t len);


/*  数据结构                                                         */

typedef struct {
    float yaw;
    float pitch;
    uint8_t fire;
    float vx;       /* m/s */
    float vy;       /* m/s */
    float vz;
    float spin;     /* rad/s */
} __attribute__((packed)) fix_control_t;


/*  API                                                              */


/* 初始化: 传入发送函数指针 (USB CDC 用 CDC_Transmit_FS) */
void Seasky_Init(seasky_send_fn_t send_fn);

/* USB CDC收到数据后调用 — 喂字节到协议解析器 */
void Seasky_FeedBytes(const uint8_t *data, uint16_t len);

/* 获取最新 fix_control 数据 (NULL = 未收到过) */
const fix_control_t *Seasky_GetFixControl(void);

/* 发送底盘速度回传 */
void Seasky_SendChassis(float vx, float vy, float wz);

/* 发送哨兵状态 */
void Seasky_SendSentry(float battery, float life, float color,
                       float bullet, float fault_flag);

/* 发送心跳响应 */
void Seasky_SendHeartbeat(uint16_t timestamp, uint8_t set_launch,
                          uint8_t set_arm, uint8_t fault_flag);

#endif /* __SEASKY_H__ */
